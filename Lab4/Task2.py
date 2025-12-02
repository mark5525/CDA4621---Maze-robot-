"""
Task 2 — Particle Filter in the 4x4 grid.

Modified for:
 - 4x4 grid (16 cells)
 - 160 particles total (10 particles per cell)
 - Evenly distributed initialization
"""

import random
import time
from collections import defaultdict
from typing import Dict, Tuple

from HamBot.src.robot_systems.robot import HamBot

GRID_SIZE = 4          # 4x4 grid → 16 cells
NUM_PARTICLES = 160    # 10 particles per cell
WALL_DETECT_THRESH_M = 0.50  # meters; tweak based on your maze spacing


def rc_to_cell(r: int, c: int, n: int = GRID_SIZE) -> int:
    return r * n + c + 1


def cell_to_rc(cell: int, n: int = GRID_SIZE) -> Tuple[int, int]:
    cell0 = cell - 1
    return divmod(cell0, n)


def build_outer_walls(n: int = GRID_SIZE) -> Dict[int, list[int]]:
    """Build outer boundary walls for the grid."""
    cell_walls: Dict[int, list[int]] = {}
    for r in range(n):
        for c in range(n):
            N = 1 if r == 0     else 0
            S = 1 if r == n - 1 else 0
            W = 1 if c == 0     else 0
            E = 1 if c == n - 1 else 0
            cell_id = rc_to_cell(r, c, n)
            cell_walls[cell_id] = [N, E, S, W]  # mutable: [N, E, S, W]
    return cell_walls


def add_horizontal_wall(cell_walls: Dict[int, list[int]], cell_above: int, cell_below: int):
    """Add a horizontal wall between two vertically adjacent cells."""
    cell_walls[cell_above][2] = 1  # S
    cell_walls[cell_below][0] = 1  # N


def add_vertical_wall(cell_walls: Dict[int, list[int]], cell_left: int, cell_right: int):
    """Add a vertical wall between two horizontally adjacent cells."""
    cell_walls[cell_left][1] = 1  # E
    cell_walls[cell_right][3] = 1  # W


def build_4x4_map() -> Dict[int, Tuple[int, int, int, int]]:
    """
    Build wall map for 4x4 grid matching Physical Robot Maze 2.
    Grid layout (cell numbers):
     1  2  3  4
     5  6  7  8
     9 10 11 12
    13 14 15 16
    
    Landmarks:
    - Orange: top-left (-1.2, 1.2)
    - Blue: top-right (1.2, 1.2)
    - Green: bottom-left (-1.2, -1.2)
    - Pink: bottom-right (1.2, -1.2)
    """
    walls = build_outer_walls(GRID_SIZE)

    # Internal walls based on the maze diagram
    # Upper horizontal wall section (between row 0 and row 1, columns 1-2)
    add_horizontal_wall(walls, rc_to_cell(0, 1), rc_to_cell(1, 1))
    add_horizontal_wall(walls, rc_to_cell(0, 2), rc_to_cell(1, 2))
    
    # Middle horizontal wall section (between row 1 and row 2, columns 1-3)
    add_horizontal_wall(walls, rc_to_cell(1, 1), rc_to_cell(2, 1))
    add_horizontal_wall(walls, rc_to_cell(1, 2), rc_to_cell(2, 2))
    add_horizontal_wall(walls, rc_to_cell(1, 3), rc_to_cell(2, 3))
    
    # Lower horizontal wall section (between row 2 and row 3, columns 1-2)
    add_horizontal_wall(walls, rc_to_cell(2, 1), rc_to_cell(3, 1))
    add_horizontal_wall(walls, rc_to_cell(2, 2), rc_to_cell(3, 2))
    
    # Left vertical wall section (between columns 0 and 1, rows 1-2)
    add_vertical_wall(walls, rc_to_cell(1, 0), rc_to_cell(1, 1))
    add_vertical_wall(walls, rc_to_cell(2, 0), rc_to_cell(2, 1))
    
    # Right vertical wall section (between columns 2 and 3, rows 1-2)
    add_vertical_wall(walls, rc_to_cell(1, 2), rc_to_cell(1, 3))
    add_vertical_wall(walls, rc_to_cell(2, 2), rc_to_cell(2, 3))

    # Freeze to tuples
    for cid in walls:
        walls[cid] = tuple(walls[cid])
    return walls


ORIENTS = ['N', 'E', 'S', 'W']
ORIENT_INDEX = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

LEFT_TURN = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
RIGHT_TURN = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}

P_Z1_GIVEN_S1 = 0.8  # P(z=1 | s=1) observed wall when wall truly present
P_Z1_GIVEN_S0 = 0.3  # P(z=1 | s=0) false positive


def side_likelihood(true_side: int, z: int) -> float:
    """Calculate P(z | s) for a single side."""
    if z not in (0, 1):
        raise ValueError("Observation z must be 0 or 1")
    if true_side not in (0, 1):
        raise ValueError("true_side must be 0 or 1")
    if z == 1:
        return P_Z1_GIVEN_S1 if true_side == 1 else P_Z1_GIVEN_S0
    return (1.0 - P_Z1_GIVEN_S1) if true_side == 1 else (1.0 - P_Z1_GIVEN_S0)


class ParticleFilterGrid:
    def __init__(self, cell_walls: Dict[int, Tuple[int, int, int, int]],
                 grid_size: int = GRID_SIZE,
                 n_particles: int = NUM_PARTICLES):
        self.cell_walls = cell_walls
        self.grid_size = grid_size
        self.n_particles = n_particles
        self.particles = self._init_particles()
        self.weights = [1.0 / n_particles] * n_particles

    def _init_particles(self):
        """Initialize particles evenly across all cells."""
        cells = sorted(self.cell_walls.keys())
        n_cells = len(cells)
        particles_per_cell = self.n_particles // n_cells
        
        particles = []
        for cell in cells:
            for _ in range(particles_per_cell):
                orient = random.choice(ORIENTS)
                particles.append({'cell': cell, 'orient': orient})
        
        # Handle any remainder particles
        remainder = self.n_particles - len(particles)
        for i in range(remainder):
            cell = cells[i % n_cells]
            orient = random.choice(ORIENTS)
            particles.append({'cell': cell, 'orient': orient})
        
        return particles

    def _forward_cell(self, cell: int, orient: str) -> int:
        """Calculate the cell forward from current cell in given orientation."""
        r, c = cell_to_rc(cell, self.grid_size)
        if orient == 'N':
            r -= 1
        elif orient == 'S':
            r += 1
        elif orient == 'E':
            c += 1
        elif orient == 'W':
            c -= 1

        if not (0 <= r < self.grid_size and 0 <= c < self.grid_size):
            return cell
        return rc_to_cell(r, c, self.grid_size)

    def motion_update(self, action: str):
        """Update particles based on action (left, right, forward)."""
        a = action.lower()
        for p in self.particles:
            cell, orient = p['cell'], p['orient']
            if a in ('l', 'left'):
                p['orient'] = LEFT_TURN[orient]
            elif a in ('r', 'right'):
                p['orient'] = RIGHT_TURN[orient]
            elif a in ('f', 'forward'):
                side_idx = ORIENT_INDEX[orient]
                N, E, S, W = self.cell_walls[cell]
                front_wall = (N, E, S, W)[side_idx]
                if front_wall == 0:
                    p['cell'] = self._forward_cell(cell, orient)
            else:
                raise ValueError(f"Unknown action: {action}")

    def observation_likelihood(self, cell: int, obs: Dict[str, int]) -> float:
        """Calculate P(observation | cell) for all four sides."""
        N, E, S, W = self.cell_walls[cell]
        true_sides = {'N': N, 'E': E, 'S': S, 'W': W}
        prob = 1.0
        for side, z in obs.items():
            prob *= side_likelihood(true_sides[side], z)
        return prob

    def sensor_update(self, obs: Dict[str, int]):
        """Update particle weights based on observations."""
        new_weights = []
        for p in self.particles:
            w = self.observation_likelihood(p['cell'], obs)
            new_weights.append(w)

        total = sum(new_weights)
        if total == 0:
            self.weights = [1.0 / self.n_particles] * self.n_particles
        else:
            self.weights = [w / total for w in new_weights]

    def systematic_resample(self):
        """Resample particles using systematic resampling."""
        N = self.n_particles
        cumulative = []
        cumsum = 0.0
        for w in self.weights:
            cumsum += w
            cumulative.append(cumsum)

        step = 1.0 / N
        start = random.random() * step

        new_particles = []
        i = 0
        for m in range(N):
            u = start + m * step
            while u > cumulative[i]:
                i += 1
            base = self.particles[i]
            new_particles.append({
                'cell': base['cell'],
                'orient': random.choice(ORIENTS)
            })

        self.particles = new_particles
        self.weights = [1.0 / N] * N

    def particle_histogram(self) -> Dict[int, int]:
        """Count particles per cell."""
        counts = defaultdict(int)
        for p in self.particles:
            counts[p['cell']] += 1
        return counts

    def estimate(self):
        """Estimate current cell and check if localized."""
        counts = self.particle_histogram()
        if not counts:
            return {}, None, 0, False
        mode_cell, mode_count = max(counts.items(), key=lambda kv: kv[1])
        localized = (mode_count >= 0.8 * self.n_particles)
        return counts, mode_cell, mode_count, localized

    def print_distribution(self, counts: Dict[int, int]):
        """Print particle distribution as a grid."""
        print("Particle counts per cell (row-wise):")
        for r in range(self.grid_size):
            row_vals = []
            for c in range(self.grid_size):
                cell = rc_to_cell(r, c, self.grid_size)
                row_vals.append(f"{counts.get(cell, 0):3d}")
            print(" ".join(row_vals))
        print()
    
    def print_maze_walls(self):
        """Print the wall configuration for each cell."""
        print("Wall configuration (N,E,S,W for each cell):")
        for r in range(self.grid_size):
            row_vals = []
            for c in range(self.grid_size):
                cell = rc_to_cell(r, c, self.grid_size)
                N, E, S, W = self.cell_walls[cell]
                row_vals.append(f"{N}{E}{S}{W}")
            print(" | ".join(row_vals))
        print()

    def step(self, action: str, obs: Dict[str, int]):
        """Execute one full particle filter step."""
        self.motion_update(action)
        self.sensor_update(obs)
        self.systematic_resample()

        counts, mode_cell, mode_count, localized = self.estimate()
        self.print_distribution(counts)
        print(f"Mode cell: {mode_cell} with {mode_count}/{self.n_particles} "
              f"({mode_count / self.n_particles:.1%})")
        print(f"Localized (>=80% in one cell)? {localized}")
        print("-" * 50)
        return mode_cell, localized


def observe_walls_from_lidar(bot: HamBot,
                             wall_thresh_m: float = WALL_DETECT_THRESH_M) -> Dict[str, int]:
    """
    Convert lidar scan to binary wall observations on N, E, S, W.
    Robot-frame angles when heading=0 (East): front=270, left=180, right=0, back=90.
    Rotate into global N/E/S/W using IMU heading (deg from East).
    """
    scan = bot.get_range_image()
    if scan == -1:
        return {'N': 0, 'E': 0, 'S': 0, 'W': 0}
    heading = bot.get_heading()
    heading_idx = 0 if heading is None else int(round(heading / 90.0)) % 4

    def is_wall_at(angle_deg: int) -> int:
        d_mm = scan[angle_deg] if 0 <= angle_deg < len(scan) else -1
        return 1 if (d_mm > 0 and d_mm / 1000.0 <= wall_thresh_m) else 0

    front = is_wall_at(270)
    left = is_wall_at(180)
    right = is_wall_at(0)
    back = is_wall_at(90)

    if heading_idx == 0:   # facing East
        obs = {'N': left, 'E': front, 'S': right, 'W': back}
    elif heading_idx == 1:  # facing North
        obs = {'N': front, 'E': right, 'S': back, 'W': left}
    elif heading_idx == 2:  # facing West
        obs = {'N': right, 'E': back, 'S': left, 'W': front}
    else:                   # facing South
        obs = {'N': back, 'E': left, 'S': front, 'W': right}
    return obs


def rotate_90(bot: HamBot, direction: str = "left"):
    """
    Rotate the robot by ±90° using IMU feedback.
    direction: "left" (CCW) or "right" (CW)
    """
    ROTATE_RPM = 30.0
    ROTATE_MIN_RPM = 8.0
    DT_ROTATE = 0.05
    target_deg = 90.0
    sign = -1 if direction == "left" else +1  # left turn = CCW
    start = bot.get_heading(blocking=True, wait_timeout=0.5)
    if start is None:
        return
    total_rotated = 0.0
    last_heading = start
    while total_rotated < target_deg - 2.0:
        cur = bot.get_heading()
        if cur is not None:
            delta = (cur - last_heading + 180) % 360 - 180
            total_rotated += abs(delta)
            last_heading = cur
        remaining = max(0.0, target_deg - total_rotated)
        scale = max(ROTATE_MIN_RPM / ROTATE_RPM, min(1.0, remaining / target_deg))
        rpm = ROTATE_RPM * scale
        bot.set_left_motor_speed(sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT_ROTATE)
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


if __name__ == "__main__":
    # Build the 4x4 maze map and PF
    maze_map = build_4x4_map()
    pf = ParticleFilterGrid(maze_map, grid_size=GRID_SIZE, n_particles=NUM_PARTICLES)

    print(f"Initialized particle filter:")
    print(f"  Grid size: {GRID_SIZE}x{GRID_SIZE} = {GRID_SIZE**2} cells")
    print(f"  Total particles: {NUM_PARTICLES}")
    print(f"  Particles per cell: {NUM_PARTICLES // (GRID_SIZE**2)}")
    print()
    pf.print_maze_walls()

    # --- Live robot loop: tune these values to your robot ---
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    CELL_DURATION_S = 1.9   # seconds to move ~1 cell forward (increase if still short)
    DRIVE_RPM = 45.0        # motor clamp is ±50 RPM in robot.py
    TURN_DURATION_S = 1.2   # seconds to turn ~90° (tune for exact 90)
    TURN_RPM = 40.0

    def move_forward_one_cell():
        bot.run_motors_for_seconds(CELL_DURATION_S, left_speed=DRIVE_RPM, right_speed=DRIVE_RPM)
        bot.stop_motors()
        time.sleep(0.1)

    def turn_left():
        rotate_90(bot, direction="left")

    def turn_right():
        rotate_90(bot, direction="right")

    # Replace with your desired route; each action triggers PF predict/correct/resample
    ACTION_SCRIPT = ["forward", "right", "forward", "left", "forward"]

    try:
        for action in ACTION_SCRIPT:
            if action == "forward":
                move_forward_one_cell()
            elif action == "left":
                turn_left()
            elif action == "right":
                turn_right()
            else:
                raise ValueError(f"Unknown action: {action}")

            obs = observe_walls_from_lidar(bot)
            print(f"Action: {action}, Observation: {obs}")
            mode_cell, localized = pf.step(action, obs)
            if localized:
                print(f"*** Localization achieved in cell {mode_cell}! ***")
                break
    finally:
        bot.stop_motors()
