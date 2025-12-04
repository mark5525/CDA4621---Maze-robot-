"""
Task 2 — Particle Filter in the 4x4 grid.

Enhanced with features from localize.py:
 - Encoder-based distance tracking
 - Drift/side correction
 - Diagonal obstacle detection
 - Majority voting for observations
 - Autonomous navigation
 - Stable convergence requirement
"""

import random
import time
from collections import defaultdict, Counter
from typing import Dict, Tuple
import numpy as np

from HamBot.src.robot_systems.robot import HamBot

# ============== GRID CONFIG ==============
GRID_SIZE = 4          # 4x4 grid → 16 cells
NUM_PARTICLES = 160    # 10 particles per cell

# ============== LIDAR CONFIG ==============
LIDAR_MM_TO_M = 1.0 / 1000.0
SECTOR_HALF_WIDTH = 8  # degrees around center ray
WALL_DETECT_THRESH_M = 0.50  # meters for wall observation

# ============== DRIVING SAFETY THRESHOLDS ==============
FRONT_STOP_M = 0.22
FRONT_DIAG_ENTER_M = 0.35
FRONT_DIAG_EXIT_M = 0.40
FRONT_DIAG_STOP_M = 0.18
FRONT_LEFT_DIAG_DEG = 135
FRONT_RIGHT_DIAG_DEG = 225

# ============== MOTION TUNING ==============
FWD_RPM = 35
MIN_RAMP_RPM = 18
RAMP_TIME_S = 0.8
CTRL_DT = 0.05
TURN_RPM = 25
RAD_PER_CELL = 6.0    # encoder radians per cell
MAX_FWD_TIME_S = 6.0

# Drift correction (encoders)
ENC_KP = 8.0
# Side-centering correction
SIDE_KP = 1.2

# ============== SENSOR MODEL (matching localize.py) ==============
P_Z1_S1 = 0.8
P_Z0_S1 = 0.2
P_Z1_S0 = 0.3
P_Z0_S0 = 0.7

# Stable convergence requirement
STABLE_CONV_STEPS = 2


# ============== MAZE MAP ==============
def build_4x4_map() -> Dict[int, Tuple[int, int, int, int]]:
    """
    Build wall map for 4x4 grid based on Image 2.
    Grid layout (cell numbers):
     1  2  3  4
     5  6  7  8
     9 10 11 12
    13 14 15 16
    
    Each cell is 60cm x 60cm.
    Walls: Outer boundary + internal walls on left and right sides.
    
    Looking at Image 2:
    - Full outer boundary
    - Left vertical wall between col 0-1 for rows 1-2 (cells 5,9 have E wall; cells 6,10 have W wall)
    - Right vertical wall between col 2-3 for rows 1-2 (cells 7,11 have E wall; cells 8,12 have W wall)
    """
    walls: Dict[int, Tuple[int, int, int, int]] = {}
    n = GRID_SIZE  # 4
    
    for r in range(n):
        for c in range(n):
            cell_id = r * n + c + 1
            # Start with outer boundary
            N = 1 if r == 0 else 0         # Top row has North wall
            S = 1 if r == n - 1 else 0     # Bottom row has South wall
            W = 1 if c == 0 else 0         # Left column has West wall
            E = 1 if c == n - 1 else 0     # Right column has East wall
            walls[cell_id] = [N, E, S, W]  # Use list for mutability
    
    # Add internal walls based on Image 2
    # Left vertical wall (between col 0 and col 1, rows 1-2)
    walls[5][1] = 1   # Cell 5: add East wall
    walls[6][3] = 1   # Cell 6: add West wall
    walls[9][1] = 1   # Cell 9: add East wall
    walls[10][3] = 1  # Cell 10: add West wall
    
    # Right vertical wall (between col 2 and col 3, rows 1-2)
    walls[7][1] = 1   # Cell 7: add East wall
    walls[8][3] = 1   # Cell 8: add West wall
    walls[11][1] = 1  # Cell 11: add East wall
    walls[12][3] = 1  # Cell 12: add West wall
    
    # Convert to tuples
    for cell_id in walls:
        walls[cell_id] = tuple(walls[cell_id])
    
    return walls


MAZE_MAP = build_4x4_map()


# ============== CELL/ORIENTATION UTILITIES ==============
ORIENTATIONS = ['N', 'E', 'S', 'W']
ORIENT_INDEX = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
ORIENT_TO_VEC = {
    "N": (0, -1),  # (dc, dr)
    "E": (1, 0),
    "S": (0, 1),
    "W": (-1, 0),
}
LEFT_TURN = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
RIGHT_TURN = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}


def rc_to_cell(r: int, c: int, n: int = GRID_SIZE) -> int:
    return r * n + c + 1


def cell_to_rc(cell: int, n: int = GRID_SIZE) -> Tuple[int, int]:
    cell0 = cell - 1
    return divmod(cell0, n)


def in_bounds(r: int, c: int) -> bool:
    return 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE


def wall_in_direction(cell: int, direction: str) -> int:
    N, E, S, W = MAZE_MAP[cell]
    return {'N': N, 'E': E, 'S': S, 'W': W}[direction]


# ============== PARTICLE CLASS ==============
class Particle:
    def __init__(self, cell: int, orient: str):
        self.cell = cell
        self.orient = orient


def init_particles_even() -> list:
    """Initialize particles evenly across all cells."""
    particles = []
    per_cell = NUM_PARTICLES // (GRID_SIZE * GRID_SIZE)
    for cell in range(1, GRID_SIZE * GRID_SIZE + 1):
        for _ in range(per_cell):
            particles.append(Particle(cell, random.choice(ORIENTATIONS)))
    while len(particles) < NUM_PARTICLES:
        particles.append(Particle(random.randint(1, GRID_SIZE * GRID_SIZE),
                                  random.choice(ORIENTATIONS)))
    return particles


# ============== MOTION UPDATE ==============
def motion_update(particles: list, command: str) -> list:
    """
    Perfect deterministic model:
      F: move one cell forward if no wall
      L/R: rotate in place
      S: stay (used when forward aborts early)
    """
    new_parts = []
    for p in particles:
        if command == "S":
            new_parts.append(Particle(p.cell, p.orient))
        elif command == "L":
            new_parts.append(Particle(p.cell, LEFT_TURN[p.orient]))
        elif command == "R":
            new_parts.append(Particle(p.cell, RIGHT_TURN[p.orient]))
        elif command == "F":
            if wall_in_direction(p.cell, p.orient) == 1:
                new_parts.append(Particle(p.cell, p.orient))
            else:
                r, c = cell_to_rc(p.cell)
                dc, dr = ORIENT_TO_VEC[p.orient]
                nr, nc = r + dr, c + dc
                if in_bounds(nr, nc):
                    new_parts.append(Particle(rc_to_cell(nr, nc), p.orient))
                else:
                    new_parts.append(Particle(p.cell, p.orient))
        else:
            raise ValueError("command must be F/L/R/S")
    return new_parts


# ============== SENSOR UTILITIES ==============
def median_sector(scan, center_deg: int):
    """Get median distance in a sector around center_deg."""
    idxs = [(center_deg + d) % 360 for d in range(-SECTOR_HALF_WIDTH, SECTOR_HALF_WIDTH + 1)]
    vals = [scan[i] for i in idxs if 0 <= i < len(scan) and scan[i] > 0]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2] * LIDAR_MM_TO_M


def heading_to_orient(heading_deg) -> str:
    """Convert IMU heading to cardinal orientation."""
    if heading_deg is None:
        return None
    cardinals = [(0, "E"), (90, "N"), (180, "W"), (270, "S"), (360, "E")]
    best = min(cardinals, key=lambda x: abs((heading_deg - x[0] + 180) % 360 - 180))
    return best[1]


def get_wall_observation(robot: HamBot, debug: bool = True):
    """Get wall observations in global N/E/S/W coordinates."""
    scan = robot.get_range_image()
    if scan == -1:
        raise RuntimeError("LIDAR not enabled.")

    heading = robot.get_heading()
    orient = heading_to_orient(heading)
    if orient is None:
        orient = "N"  # Default fallback

    # HamBot LIDAR: front=180, left=90, right=270, back=0
    d_front = median_sector(scan, 180)
    d_right = median_sector(scan, 270)
    d_left = median_sector(scan, 90)
    d_back = median_sector(scan, 0)

    if debug:
        def fmt(v):
            return f"{v:.2f}" if v is not None else "N/A"
        print(f"  [DEBUG] Heading: {heading:.1f}° → Orient: {orient}")
        print(f"  [DEBUG] Distances (m): front={fmt(d_front)}, "
              f"left={fmt(d_left)}, right={fmt(d_right)}, back={fmt(d_back)}")

    def is_wall(d):
        return 1 if (d is not None and d <= WALL_DETECT_THRESH_M) else 0

    z_robot = {
        "front": is_wall(d_front),
        "right": is_wall(d_right),
        "back": is_wall(d_back),
        "left": is_wall(d_left),
    }

    # Map robot-relative to global N/E/S/W based on orientation
    if orient == "N":
        z_global = {"N": z_robot["front"], "E": z_robot["right"], "S": z_robot["back"], "W": z_robot["left"]}
    elif orient == "E":
        z_global = {"E": z_robot["front"], "S": z_robot["right"], "W": z_robot["back"], "N": z_robot["left"]}
    elif orient == "S":
        z_global = {"S": z_robot["front"], "W": z_robot["right"], "N": z_robot["back"], "E": z_robot["left"]}
    else:  # "W"
        z_global = {"W": z_robot["front"], "N": z_robot["right"], "E": z_robot["back"], "S": z_robot["left"]}

    if debug:
        print(f"  [DEBUG] Robot obs: {z_robot}")
        print(f"  [DEBUG] Global obs (N,E,S,W): {z_global['N']},{z_global['E']},{z_global['S']},{z_global['W']}")
        # Show which cells match this observation
        matching = []
        for cell in sorted(MAZE_MAP.keys()):
            N, E, S, W = MAZE_MAP[cell]
            if N == z_global['N'] and E == z_global['E'] and S == z_global['S'] and W == z_global['W']:
                matching.append(cell)
        print(f"  [DEBUG] Cells matching this obs: {matching}")

    return (z_global["N"], z_global["E"], z_global["S"], z_global["W"]), z_robot


def get_wall_observation_majority(robot: HamBot, samples: int = 3, dt: float = 0.12):
    """Get majority-voted wall observations over multiple samples."""
    zs = []
    for i in range(samples):
        z, _ = get_wall_observation(robot, debug=(i == 0))  # Only debug first sample
        zs.append(z)
        time.sleep(dt)
    arr = np.array(zs, dtype=int)
    maj = (arr.sum(axis=0) >= (samples / 2)).astype(int)
    return tuple(int(x) for x in maj)


def likelihood(z: int, s: int) -> float:
    """Calculate P(z | s) for a single side."""
    if s == 1 and z == 1:
        return P_Z1_S1
    if s == 1 and z == 0:
        return P_Z0_S1
    if s == 0 and z == 1:
        return P_Z1_S0
    return P_Z0_S0


def sensor_update(particles: list, observation: tuple) -> np.ndarray:
    """Update particle weights based on observations."""
    zN, zE, zS, zW = observation
    w = np.zeros(len(particles), dtype=float)
    for i, p in enumerate(particles):
        sN, sE, sS, sW = MAZE_MAP[p.cell]
        w[i] = (likelihood(zN, sN) * likelihood(zE, sE) *
                likelihood(zS, sS) * likelihood(zW, sW))
    total = w.sum()
    if total == 0:
        w[:] = 1.0 / len(w)
    else:
        w /= total
    return w


# ============== RESAMPLING ==============
def resample_particles(particles: list, weights: np.ndarray) -> list:
    """Resample particles using multinomial resampling."""
    idxs = np.random.choice(len(particles), size=len(particles), p=weights)
    new_parts = [Particle(particles[j].cell, random.choice(ORIENTATIONS)) for j in idxs]
    return new_parts


# ============== PRINTING / ESTIMATION ==============
def particle_counts_grid(particles: list):
    """Get particle counts as grid and counter."""
    counts = Counter(p.cell for p in particles)
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    for cell, k in counts.items():
        r, c = cell_to_rc(cell)
        grid[r, c] = k
    return grid, counts


def print_grid(grid):
    """Print particle distribution grid."""
    print("\nParticle count grid (row 0 = top):")
    for r in range(GRID_SIZE):
        print(" ".join(f"{grid[r, c]:3d}" for c in range(GRID_SIZE)))


def mode_cell_and_fraction(counts):
    """Get mode cell and fraction of particles."""
    mode_cell = max(counts.keys(), key=lambda c: counts[c])
    mode_count = counts[mode_cell]
    frac = mode_count / NUM_PARTICLES
    return mode_cell, mode_count, frac


def print_maze_walls():
    """Print the wall configuration for each cell."""
    print("Wall configuration (N,E,S,W for each cell):")
    for r in range(GRID_SIZE):
        row_vals = []
        for c in range(GRID_SIZE):
            cell = rc_to_cell(r, c)
            N, E, S, W = MAZE_MAP[cell]
            row_vals.append(f"{N}{E}{S}{W}")
        print(" | ".join(row_vals))
    print()


def print_maze_visual():
    """Print a visual representation of the maze."""
    print("\nMaze Layout (visual):")
    print("Cell numbers:")
    for r in range(GRID_SIZE):
        print("  " + " ".join(f"{rc_to_cell(r, c):2d}" for c in range(GRID_SIZE)))
    print()
    
    # Draw maze with walls
    for r in range(GRID_SIZE):
        # Top walls
        top_line = ""
        for c in range(GRID_SIZE):
            cell = rc_to_cell(r, c)
            N, E, S, W = MAZE_MAP[cell]
            top_line += "+--" if N else "+  "
        top_line += "+"
        print(top_line)
        
        # Left/right walls and cell numbers
        mid_line = ""
        for c in range(GRID_SIZE):
            cell = rc_to_cell(r, c)
            N, E, S, W = MAZE_MAP[cell]
            mid_line += "|" if W else " "
            mid_line += f"{cell:2d}"
        # Right edge
        last_cell = rc_to_cell(r, GRID_SIZE - 1)
        N, E, S, W = MAZE_MAP[last_cell]
        mid_line += "|" if E else " "
        print(mid_line)
    
    # Bottom walls of last row
    bottom_line = ""
    for c in range(GRID_SIZE):
        cell = rc_to_cell(GRID_SIZE - 1, c)
        N, E, S, W = MAZE_MAP[cell]
        bottom_line += "+--" if S else "+  "
    bottom_line += "+"
    print(bottom_line)
    print()


# ============== AUTONOMOUS DRIVING ==============
def execute_turn(robot: HamBot, cmd: str):
    """Execute a 90-degree turn."""
    if cmd == "R":
        robot.set_left_motor_speed(TURN_RPM)
        robot.set_right_motor_speed(-TURN_RPM)
    else:
        robot.set_left_motor_speed(-TURN_RPM)
        robot.set_right_motor_speed(TURN_RPM)
    time.sleep(1.25)  # Tune for 90 degrees
    robot.stop_motors()
    time.sleep(0.2)
    robot.reset_encoders()
    time.sleep(0.1)


def do_forward_one_cell(robot: HamBot):
    """
    Move forward one cell using encoder feedback, LIDAR safety, and diagonal detection.
    """
    # Persistent state across calls (like localize.py)
    if not hasattr(do_forward_one_cell, "slowing"):
        do_forward_one_cell.slowing = False
    if not hasattr(do_forward_one_cell, "fld_hits"):
        do_forward_one_cell.fld_hits = 0
    if not hasattr(do_forward_one_cell, "frd_hits"):
        do_forward_one_cell.frd_hits = 0

    robot.reset_encoders()
    start_time = time.time()
    aborted_reason = None
    avg_rad = 0.0

    while True:
        l_rad, r_rad = robot.get_encoder_readings()
        avg_rad = (abs(l_rad) + abs(r_rad)) / 2.0

        if avg_rad >= RAD_PER_CELL:
            break

        if time.time() - start_time > MAX_FWD_TIME_S:
            print("WARN: forward move timeout hit.")
            aborted_reason = "timeout"
            break

        scan = robot.get_range_image()
        if scan == -1:
            print("WARN: LIDAR unavailable during forward.")
            aborted_reason = "lidar"
            break

        d_front = median_sector(scan, 180)
        d_left = median_sector(scan, 90)
        d_right = median_sector(scan, 270)
        d_fld = median_sector(scan, FRONT_LEFT_DIAG_DEG)
        d_frd = median_sector(scan, FRONT_RIGHT_DIAG_DEG)

        # Emergency stop: front wall
        if d_front is not None and d_front <= FRONT_STOP_M:
            print("EMERGENCY STOP: front wall too close.")
            aborted_reason = "front"
            break

        # Consecutive diagonal hits detection
        if d_fld is not None and d_fld <= FRONT_DIAG_STOP_M:
            do_forward_one_cell.fld_hits += 1
        else:
            do_forward_one_cell.fld_hits = 0

        if d_frd is not None and d_frd <= FRONT_DIAG_STOP_M:
            do_forward_one_cell.frd_hits += 1
        else:
            do_forward_one_cell.frd_hits = 0

        if do_forward_one_cell.fld_hits >= 2:
            print("EMERGENCY STOP: front-left diag too close (persistent).")
            aborted_reason = "diagL"
            break
        if do_forward_one_cell.frd_hits >= 2:
            print("EMERGENCY STOP: front-right diag too close (persistent).")
            aborted_reason = "diagR"
            break

        # Speed ramp
        t = time.time() - start_time
        ramp_frac = min(1.0, t / RAMP_TIME_S)
        base = max(MIN_RAMP_RPM, FWD_RPM * ramp_frac)

        # Drift correction using encoders
        drift_err = (l_rad - r_rad)
        drift_corr = ENC_KP * drift_err

        # Side centering correction
        side_corr = 0.0
        if d_left is not None and d_right is not None:
            side_err = d_left - d_right
            side_corr = SIDE_KP * side_err

        # Diagonal slowdown + steering with hysteresis
        diag_corr = 0.0
        diag_slow = 1.0
        if d_fld is not None and d_frd is not None:
            min_diag = min(d_fld, d_frd)

            if do_forward_one_cell.slowing:
                if min_diag > FRONT_DIAG_EXIT_M:
                    do_forward_one_cell.slowing = False
            else:
                if min_diag < FRONT_DIAG_ENTER_M:
                    do_forward_one_cell.slowing = True

            if do_forward_one_cell.slowing:
                diag_slow = max(0.7, min_diag / FRONT_DIAG_ENTER_M)

            diag_err = d_fld - d_frd
            diag_corr = (SIDE_KP * 0.8) * diag_err

        total_corr = drift_corr + side_corr + diag_corr
        base *= diag_slow

        robot.set_left_motor_speed(base - total_corr)
        robot.set_right_motor_speed(base + total_corr)

        time.sleep(CTRL_DT)

    robot.stop_motors()
    time.sleep(0.2)

    # Reset hit counters after any abort
    if aborted_reason is not None:
        do_forward_one_cell.fld_hits = 0
        do_forward_one_cell.frd_hits = 0
        do_forward_one_cell.slowing = False

    # If aborted before 90% of cell, treat as NO MOVE
    if aborted_reason is not None and avg_rad < 0.90 * RAD_PER_CELL:
        return False, aborted_reason

    return True, "ok"


def choose_autonomous_command(z_robot: dict, last_cmd: str = None) -> str:
    """Choose next command based on wall observations."""
    front_clear = (z_robot["front"] == 0)
    left_clear = (z_robot["left"] == 0)
    right_clear = (z_robot["right"] == 0)

    if front_clear:
        return "F"

    opts = []
    if left_clear:
        opts.append("L")
    if right_clear:
        opts.append("R")

    if opts:
        if last_cmd in opts and len(opts) > 1:
            opts.remove(last_cmd)
        return random.choice(opts)

    return random.choice(["L", "R"])


# ============== MAIN LOOP ==============
def run_particle_filter():
    print("Initializing HamBot...")
    robot = HamBot(lidar_enabled=True, camera_enabled=False)
    robot.max_motor_speed = 60
    time.sleep(2.0)

    print(f"\nInitialized particle filter:")
    print(f"  Grid size: {GRID_SIZE}x{GRID_SIZE} = {GRID_SIZE ** 2} cells")
    print(f"  Total particles: {NUM_PARTICLES}")
    print(f"  Particles per cell: {NUM_PARTICLES // (GRID_SIZE ** 2)}")
    print(f"  Wall threshold: {WALL_DETECT_THRESH_M}m")
    print()
    print_maze_walls()
    print_maze_visual()
    print("VERIFY: Does this maze match your physical maze?")

    particles = init_particles_even()
    last_cmd = None
    stable = 0

    try:
        step = 0
        while True:
            step += 1
            print(f"\n{'=' * 20} FILTER STEP {step} {'=' * 20}")

            # Observe for policy
            _, z_robot = get_wall_observation(robot)
            print(f"Robot observation: {z_robot}")

            # Autonomous command
            cmd = choose_autonomous_command(z_robot, last_cmd=last_cmd)
            print(f"Autonomous command chosen: {cmd}")
            last_cmd = cmd

            # Execute physically and get ACTUAL cmd (with recovery)
            if cmd == "F":
                moved_full, reason = do_forward_one_cell(robot)

                if moved_full:
                    actual_cmd = "F"
                else:
                    # Recovery turn based on what blocked us
                    if reason == "diagR":
                        recovery_turn = "L"
                    elif reason == "diagL":
                        recovery_turn = "R"
                    elif reason == "front":
                        recovery_turn = random.choice(["L", "R"])
                    else:
                        recovery_turn = None

                    if recovery_turn is not None:
                        execute_turn(robot, recovery_turn)
                        actual_cmd = recovery_turn
                        last_cmd = recovery_turn
                    else:
                        actual_cmd = "S"
            else:
                execute_turn(robot, cmd)
                actual_cmd = cmd

            print(f"Actual command executed: {actual_cmd}")

            # PF prediction with ACTUAL cmd
            particles = motion_update(particles, actual_cmd)

            # PF correction with majority voting
            obs = get_wall_observation_majority(robot, samples=3)
            print(f"Observation (majority): N={obs[0]}, E={obs[1]}, S={obs[2]}, W={obs[3]}")
            weights = sensor_update(particles, obs)

            # Resample
            particles = resample_particles(particles, weights)

            # Report
            grid, counts = particle_counts_grid(particles)
            print_grid(grid)

            mode_cell, mode_count, frac = mode_cell_and_fraction(counts)
            print(f"\nMode cell estimate: {mode_cell} with {mode_count}/{NUM_PARTICLES} "
                  f"particles ({frac * 100:.1f}%)")
            print(f"Converged? {'YES' if frac >= 0.80 else 'NO'}")

            # Stable convergence check
            if frac >= 0.80:
                stable += 1
            else:
                stable = 0

            if stable >= STABLE_CONV_STEPS:
                print(f"\n{'*' * 50}")
                print(f"SUCCESS: ≥80% for {STABLE_CONV_STEPS} consecutive steps!")
                print(f"Robot localized in cell {mode_cell}!")
                print(f"{'*' * 50}")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("\nShutting down HamBot...")
        robot.set_left_motor_speed(0.0)
        robot.set_right_motor_speed(0.0)


if __name__ == "__main__":
    run_particle_filter()
