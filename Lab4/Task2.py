import random
from collections import defaultdict

GRID_SIZE = 5          # 5x5 grid → 25 cells
NUM_PARTICLES = 250    # as in the task

# ------------------------
# World / Maze definition
# ------------------------

def rc_to_cell(r: int, c: int, n: int = GRID_SIZE) -> int:

    return r * n + c + 1

def cell_to_rc(cell: int, n: int = GRID_SIZE) -> tuple[int, int]:

    cell0 = cell - 1
    return divmod(cell0, n)

def build_outer_walls(n: int = GRID_SIZE) -> dict[int, tuple[int,int,int,int]]:

    cell_walls = {}
    for r in range(n):
        for c in range(n):
            N = 1 if r == 0     else 0
            S = 1 if r == n - 1 else 0
            W = 1 if c == 0     else 0
            E = 1 if c == n - 1 else 0
            cell_id = rc_to_cell(r, c, n)
            cell_walls[cell_id] = [N, E, S, W]  # stored as mutable list
    return cell_walls

def add_horizontal_wall(cell_walls: dict, cell_above: int, cell_below: int):

    cell_walls[cell_above][2] = 1  # S
    cell_walls[cell_below][0] = 1  # N

def add_vertical_wall(cell_walls: dict, cell_left: int, cell_right: int):

    cell_walls[cell_left][1] = 1  # E
    cell_walls[cell_right][3] = 1 # W

def build_maze2_map() -> dict[int, tuple[int,int,int,int]]:
    walls = build_outer_walls(GRID_SIZE)
    for cid in walls:
        walls[cid] = tuple(walls[cid])
    return walls

ORIENTS = ['N', 'E', 'S', 'W']
ORIENT_INDEX = {'N': 0, 'E': 1, 'S': 2, 'W': 3}

LEFT_TURN  = {'N':'W', 'W':'S', 'S':'E', 'E':'N'}
RIGHT_TURN = {'N':'E', 'E':'S', 'S':'W', 'W':'N'}

P_Z1_GIVEN_S1 = 0.8  # P(z=1 | s=1)  observed wall when wall truly present
P_Z1_GIVEN_S0 = 0.3  # P(z=1 | s=0)  false positive

def side_likelihood(true_side: int, z: int) -> float:

    if z not in (0, 1):
        raise ValueError("Observation z must be 0 or 1")
    if true_side not in (0, 1):
        raise ValueError("true_side must be 0 or 1")

    if z == 1:
        return P_Z1_GIVEN_S1 if true_side == 1 else P_Z1_GIVEN_S0
    else:  # z==0
        return (1.0 - P_Z1_GIVEN_S1) if true_side == 1 else (1.0 - P_Z1_GIVEN_S0)

class ParticleFilterGrid:
    def __init__(self, cell_walls: dict[int, tuple[int,int,int,int]],
                 grid_size: int = GRID_SIZE,
                 n_particles: int = NUM_PARTICLES):
        self.cell_walls = cell_walls
        self.grid_size = grid_size
        self.n_particles = n_particles

        self.particles = self._init_particles()
        self.weights   = [1.0 / n_particles] * n_particles


    def _init_particles(self):

        cells = list(self.cell_walls.keys())
        n_cells = len(cells)
        particles = []
        for i in range(self.n_particles):
            cell = cells[i % n_cells]  # roughly equal per cell
            orient = random.choice(ORIENTS)
            particles.append({'cell': cell, 'orient': orient})
        return particles


    def _forward_cell(self, cell: int, orient: str) -> int:
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
        a = action.lower()
        for p in self.particles:
            cell, orient = p['cell'], p['orient']

            if a in ('l', 'left'):
                p['orient'] = LEFT_TURN[orient]

            elif a in ('r', 'right'):
                p['orient'] = RIGHT_TURN[orient]

            elif a in ('f', 'forward'):
                side_idx = ORIENT_INDEX[orient]     # which side is "front"
                N, E, S, W = self.cell_walls[cell]
                front_wall = (N, E, S, W)[side_idx]
                if front_wall == 0:
                    new_cell = self._forward_cell(cell, orient)
                    p['cell'] = new_cell
                # else: wall in front → stay in same cell

            else:
                raise ValueError(f"Unknown action: {action}")


    def observation_likelihood(self, cell: int, obs: dict[str,int]) -> float:
        N, E, S, W = self.cell_walls[cell]
        true_sides = {'N': N, 'E': E, 'S': S, 'W': W}

        prob = 1.0
        for side, z in obs.items():
            prob *= side_likelihood(true_sides[side], z)
        return prob

    def sensor_update(self, obs: dict[str,int]):
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


    def particle_histogram(self) -> dict[int,int]:
        counts = defaultdict(int)
        for p in self.particles:
            counts[p['cell']] += 1
        return counts

    def estimate(self):
        counts = self.particle_histogram()
        if not counts:
            return {}, None, 0, False

        mode_cell, mode_count = max(counts.items(), key=lambda kv: kv[1])
        localized = (mode_count >= 0.8 * self.n_particles)
        return counts, mode_cell, mode_count, localized

    def print_distribution(self, counts: dict[int,int]):
        print("Particle counts per cell (row-wise, 1..25):")
        for r in range(self.grid_size):
            row_vals = []
            for c in range(self.grid_size):
                cell = rc_to_cell(r, c, self.grid_size)
                row_vals.append(f"{counts.get(cell, 0):3d}")
            print(" ".join(row_vals))
        print()


    def step(self, action: str, obs: dict[str,int]):
        self.motion_update(action)

        self.sensor_update(obs)

        self.systematic_resample()

        counts, mode_cell, mode_count, localized = self.estimate()
        self.print_distribution(counts)
        print(f"Mode cell: {mode_cell} "
              f"with {mode_count}/{self.n_particles} particles "
              f"({mode_count / self.n_particles:.1%})")
        print(f"Localized (>=80% in one cell)? {localized}")
        print("-" * 50)

        return mode_cell, localized

if __name__ == "__main__":

    maze_map = build_maze2_map()
    pf = ParticleFilterGrid(maze_map)

    example_steps = [
        ('forward', {'N': 1, 'E': 0, 'S': 0, 'W': 1}),
        ('forward', {'N': 1, 'E': 1, 'S': 0, 'W': 0}),
        ('right',   {'N': 0, 'E': 1, 'S': 1, 'W': 0}),
        ('forward', {'N': 0, 'E': 1, 'S': 1, 'W': 1}),
    ]

    for action, obs in example_steps:
        mode_cell, localized = pf.step(action, obs)
        if localized:
            print(f"*** Localization achieved in cell {mode_cell}! ***")
            break
