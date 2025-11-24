from HamBot.src.robot_systems.robot import HamBot
import math
import time

GRID_SIZE = 5            # 5 x 5 grid
CELL_SIZE = 1.0          # 1 m per cell

LANDMARK_POSITIONS = {
    "yellow": (-2.0,  2.0),   # top-left (cell 1)
    "red":    ( 2.0,  2.0),   # top-right (cell 5)
    "green":  (-2.0, -2.0),   # bottom-left (cell 21)
    "blue":   ( 2.0, -2.0),   # bottom-right (cell 25)
}

def normalize_color(color):

    r, g, b = color
    max_c = max(r, g, b)
    if max_c > 1.5:   # likely 0..255
        return r / 255.0, g / 255.0, b / 255.0
    return r, g, b


def classify_landmark_by_color(color):

    r, g, b = normalize_color(color)

    if r > 0.7 and g > 0.7 and b < 0.4:
        return "yellow"
    if r > 0.7 and g < 0.4 and b < 0.4:
        return "red"
    if r < 0.4 and g > 0.7 and b < 0.4:
        return "green"
    if r < 0.4 and g < 0.4 and b > 0.7:
        return "blue"

    return None


def measure_landmark_distances(robot):

    rec_objects = robot.rgb_camera.getRecognitionObjects()
    distances = {}

    for obj in rec_objects:
        name = classify_landmark_by_color(obj.getColors())
        if name is None:
            continue

        x, y, z = obj.getPosition()
        d = math.sqrt(x * x + y * y + z * z)

        if name not in distances or d < distances[name]:
            distances[name] = d

    return distances

def trilaterate_position(landmark_positions, measurements):

    names = [n for n in measurements.keys() if n in landmark_positions]
    if len(names) < 3:
        raise ValueError("Need at least 3 landmarks for trilateration")

    ref = names[0]
    x0, y0 = landmark_positions[ref]
    r0 = measurements[ref]

    A11 = A12 = A22 = b1 = b2 = 0.0

    for name in names[1:]:
        xi, yi = landmark_positions[name]
        ri = measurements[name]

        a_i = 2.0 * (xi - x0)
        b_i = 2.0 * (yi - y0)
        c_i = (xi * xi + yi * yi - ri * ri) - (x0 * x0 + y0 * y0 - r0 * r0)

        A11 += a_i * a_i
        A12 += a_i * b_i
        A22 += b_i * b_i
        b1  += a_i * c_i
        b2  += b_i * c_i

    det = A11 * A22 - A12 * A12
    if abs(det) < 1e-8:
        raise ValueError("Degenerate landmark configuration (det ~ 0)")

    x_est = ( b1 * A22 - b2 * A12) / det
    y_est = (-b1 * A12 + b2 * A11) / det

    return x_est, y_est

def position_to_cell_index(x, y, grid_size=GRID_SIZE, cell_size=CELL_SIZE):

    grid_center = (grid_size - 1) / 2.0  # e.g., 2 for 5x5

    col = int(round(x / cell_size + grid_center))      # 0..4
    row = int(round(grid_center - y / cell_size))      # 0..4 (y up)

    col = max(0, min(grid_size - 1, col))
    row = max(0, min(grid_size - 1, row))

    cell_index = row * grid_size + col + 1
    return cell_index, row, col

def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=True)

    time.sleep(1.0)

    start_time = time.time()
    measurements = {}

    while time.time() - start_time < 5.0 and len(measurements) < 3:
        new_meas = measure_landmark_distances(bot)
        # Merge (keep any new landmarks we see)
        for name, dist in new_meas.items():
            # Keep the closest distance we have for each landmark
            if name not in measurements or dist < measurements[name]:
                measurements[name] = dist

        # Small delay so we don't hammer the camera
        time.sleep(0.1)

    if len(measurements) < 3:
        print("ERROR: saw fewer than 3 landmarks, cannot trilaterate.")
        print("Visible landmarks:", measurements)
        return

    print("Measured landmark distances (m):")
    for name, d in measurements.items():
        print(f"  {name}: {d:.3f}")

    # Trilateration: compute (x, y)
    x_est, y_est = trilaterate_position(LANDMARK_POSITIONS, measurements)
    cell_index, row, col = position_to_cell_index(x_est, y_est)

    print("\n=== Trilateration Result ===")
    print(f"Estimated position: x = {x_est:.3f} m, y = {y_est:.3f} m")
    print(f"Estimated grid cell index: {cell_index} (row={row}, col={col})")

    # Stop the robot (we didn't move, but this is safe)
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


if __name__ == "__main__":
    main()

