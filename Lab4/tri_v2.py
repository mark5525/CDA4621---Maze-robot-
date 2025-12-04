import time
import itertools
from dataclasses import dataclass

import numpy as np
from robot_systems.robot import HamBot

#CONFIG 

CELL_SIZE_M = 0.60   
LANDMARK_RADIUS_M = 0.25  

LIDAR_MM_TO_M = 1.0 / 1000.0

# LIDAR indices: 0=back, 90=left, 180=front, 270=right
FRONT_SECTOR = range(170, 191)  # ~20° fan around front

# Camera detection tuning (far landmarks need smaller min area)
COLOR_TOL = 0.18
MIN_BOX_AREA = 120           
REQUIRED_CONSEC_FRAMES = 3   
CENTER_TOL_PX = 130          # base centering tolerance (adaptive below)

# Rotation / centering parameters
BASE_SPIN_RPM = 15
Kp_pix = 0.05                # servo gain turning blob to center
MAX_SPIN_TIME_PER_LM = 18.0
MAX_SPIN_DEG_PER_LM = 720.0  # ~2 full rotations if IMU works

#MAP / LANDMARK DEFINITIONS 

@dataclass
class MapLandmark:
    name: str
    color_rgb: tuple[int, int, int]
    x: float
    y: float


ORANGE_RGB = (242, 151, 0)
RED_RGB    = (255, 0, 73)
GREEN_RGB  = (93, 240, 114)
BLUE_RGB   = (81, 184, 210)

LANDMARKS = [
    MapLandmark("orange", ORANGE_RGB, -2 * CELL_SIZE_M,  2 * CELL_SIZE_M),
    MapLandmark("red",    RED_RGB,     2 * CELL_SIZE_M,  2 * CELL_SIZE_M),
    MapLandmark("green",  GREEN_RGB,  -2 * CELL_SIZE_M, -2 * CELL_SIZE_M),
    MapLandmark("blue",   BLUE_RGB,    2 * CELL_SIZE_M, -2 * CELL_SIZE_M),
]

#LIDAR HELPERS

def measure_front_distance_m(robot, num_keep=7):
    """
    Robust distance using median of the closest front rays.
    Returns distance to the FIRST SURFACE HIT in meters.
    """
    scan = robot.get_range_image()
    if scan == -1:
        raise RuntimeError("LIDAR is not enabled on this HamBot instance.")

    vals_mm = [scan[i] for i in FRONT_SECTOR if scan[i] > 0]
    if not vals_mm:
        raise RuntimeError("No valid LIDAR readings in front sector.")

    vals_mm.sort()
    vals_mm = vals_mm[:min(num_keep, len(vals_mm))]
    return float(np.median(vals_mm)) * LIDAR_MM_TO_M


def robust_front_distance_m(robot, samples=5):
    """
    Take multiple front distance samples and return median.
    """
    ds = []
    for _ in range(samples):
        ds.append(measure_front_distance_m(robot))
        time.sleep(0.03)
    return float(np.median(ds))

#CAMERA + IMU DRIVEN LANDMARK SEARCH

def _angle_diff_deg(a, b):
    return (a - b + 540.0) % 360.0 - 180.0


def spin_search_and_measure(robot, lm: MapLandmark):
    """
    Rotate until landmark centered, then read LIDAR.
    Returns distance to LANDMARK CENTER in meters, or None if not found.
    """
    cam = robot.camera
    if cam is None:
        print("No camera present; cannot search for landmarks.")
        return None

    print(f"\n--- Searching for {lm.name.upper()} landmark ---")
    cam.set_target_colors([lm.color_rgb], tolerance=COLOR_TOL)
    time.sleep(0.5)

    # start spinning
    robot.set_left_motor_speed(BASE_SPIN_RPM)
    robot.set_right_motor_speed(-BASE_SPIN_RPM)

    start_time = time.time()
    last_print = start_time

    last_heading = robot.get_heading(fresh_within=1.0, blocking=False)
    accumulated_spin = 0.0

    consec = 0
    best_det = None
    detection_distance = None

    try:
        while True:
            now = time.time()
            if now - start_time > MAX_SPIN_TIME_PER_LM:
                print("  Timeout while searching for", lm.name)
                break

            # IMU angle limit safety
            h = robot.get_heading(fresh_within=0.5, blocking=False)
            if h is not None and last_heading is not None:
                dtheta = abs(_angle_diff_deg(h, last_heading))
                accumulated_spin += dtheta
                last_heading = h
                if accumulated_spin >= MAX_SPIN_DEG_PER_LM:
                    print("  Reached spin angle limit while searching for", lm.name)
                    break
            elif h is not None:
                last_heading = h

            frame = cam.get_frame(copy=False)
            if frame is None:
                time.sleep(0.05)
                continue

            h_img, w_img, _ = frame.shape
            center_x = w_img // 2

            detections = cam.find_landmarks(min_area=MIN_BOX_AREA)

            if detections:
                best_det = max(detections, key=lambda d: d.width * d.height)
                consec += 1
            else:
                best_det = None
                consec = 0

            if best_det:
                # signed pixel error (positive => blob right of center)
                offset_px = best_det.x - center_x
                area = best_det.width * best_det.height

                # adaptive center tolerance: smaller blobs get more leniency
                adaptive_tol = CENTER_TOL_PX + int(800 / max(area, 1))
                adaptive_tol = min(adaptive_tol, 220)

                # visual servoing (steer while spinning)
                steer = np.clip(Kp_pix * offset_px, -8, 8)
                left = BASE_SPIN_RPM - steer
                right = -(BASE_SPIN_RPM + steer)
                robot.set_left_motor_speed(left)
                robot.set_right_motor_speed(right)

                if abs(offset_px) <= adaptive_tol and consec >= REQUIRED_CONSEC_FRAMES:
                    d_surface = robust_front_distance_m(robot)
                    d_center = d_surface + LANDMARK_RADIUS_M  # <<< radius correction
                    detection_distance = d_center
                    print(f"  Found {lm.name} at ~{d_center:.3f} m (center distance)")
                    break
            else:
                # no detection: spin at base speed
                robot.set_left_motor_speed(BASE_SPIN_RPM)
                robot.set_right_motor_speed(-BASE_SPIN_RPM)

            if now - last_print > 2.0:
                last_print = now
                print(f"  Spinning... still looking for {lm.name}.")

            time.sleep(0.05)

    finally:
        robot.stop_motors()

    if detection_distance is None:
        print(f" -> FAILED to find {lm.name} within limits.")
    return detection_distance

#TRILATERATION CORE

def solve_trilateration_xy(landmarks_xy, distances):
    """
    Least-squares trilateration for >=3 landmarks.
    """
    if len(landmarks_xy) < 3:
        raise ValueError("Need at least 3 landmarks for trilateration.")

    x1, y1 = landmarks_xy[0]
    d1 = distances[0]

    A_rows, b_rows = [], []

    for (xi, yi), di in zip(landmarks_xy[1:], distances[1:]):
        A_rows.append([2.0 * (xi - x1), 2.0 * (yi - y1)])
        rhs = d1**2 - di**2 - x1**2 - y1**2 + xi**2 + yi**2
        b_rows.append(rhs)

    A = np.array(A_rows, dtype=float)
    b = np.array(b_rows, dtype=float)

    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(sol[0]), float(sol[1])


def solve_best_of_triplets(landmark_xy, distances):
    """
    If 4 landmarks available, solve all 3-landmark triplets
    and pick the solution with smallest residual.
    """
    if len(landmark_xy) == 3:
        return solve_trilateration_xy(landmark_xy, distances)

    best_xy = None
    best_err = 1e9

    for idxs in itertools.combinations(range(len(landmark_xy)), 3):
        pts = [landmark_xy[i] for i in idxs]
        ds = [distances[i] for i in idxs]
        x, y = solve_trilateration_xy(pts, ds)

        err = 0.0
        for (xi, yi), di in zip(landmark_xy, distances):
            err += (np.hypot(x - xi, y - yi) - di) ** 2

        if err < best_err:
            best_err = err
            best_xy = (x, y)

    return best_xy


def coord_to_cell_index(x, y, cell_size=CELL_SIZE_M):
    """
    Region-based mapping (less off-by-one than rounding to centers).
    """
    col_f = (x / cell_size) + 2.5
    row_f = 2.5 - (y / cell_size)

    col = int(np.floor(col_f))
    row = int(np.floor(row_f))

    col = int(np.clip(col, 0, 4))
    row = int(np.clip(row, 0, 4))

    return row * 5 + col + 1, row, col

#TASK 1 PIPELINE

def auto_localize_with_trilateration(min_landmarks=3):
    print("Starting TRILATERATION (Task 1)")
    robot = HamBot(lidar_enabled=True, camera_enabled=True)

    try:
        time.sleep(2.0)  # sensor warm-up

        start_heading = robot.get_heading(fresh_within=1.0, blocking=True, wait_timeout=2.0)
        print(f"Initial IMU: {start_heading}")

        measured_lms = []
        distances = []

        for lm in LANDMARKS:
            d = spin_search_and_measure(robot, lm)
            if d is not None:
                measured_lms.append(lm)
                distances.append(d)

        if len(measured_lms) < min_landmarks:
            print(f"\nERROR: Only saw {len(measured_lms)} landmarks (need {min_landmarks}).")
            return

        landmark_xy = [(lm.x, lm.y) for lm in measured_lms]
        x_hat, y_hat = solve_best_of_triplets(landmark_xy, distances)

        cell_index, row, col = coord_to_cell_index(x_hat, y_hat)

        print("\n=== TRILATERATION RESULTS ===")
        print("Landmarks used (name, distance to center [m]):")
        for lm, d in zip(measured_lms, distances):
            print(f"  {lm.name:6s} : {d:.3f}")
        print(f"Estimated position: x = {x_hat:.3f} m, y = {y_hat:.3f} m")
        print(f"Estimated cell index: {cell_index}  (row {row}, col {col})")

    finally:
        print("\nShutting down HamBot...")
        robot.disconnect_robot()


if __name__ == "__main__":
    auto_localize_with_trilateration()
