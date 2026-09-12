"""
maze.py - 2D Grid Maze Representation and Manipulation
======================================================
This module defines the Maze class representing a 2D grid of nodes with:
- Start node
- Goal node
- Walls / obstacles
- Walkable cells

Supports interactive operations:
- Setting Start and Goal coordinates
- Adding / removing / toggling walls
- Clearing walls
- Loading a pre-configured starter example maze
- Generating randomized mazes with controlled wall density
- Returning 4-directional orthogonal neighbors (Up, Down, Left, Right)
"""

import random
from typing import List, Optional, Set, Tuple


class Maze:
    """
    Represents a 2D grid maze for pathfinding.

    Cell values:
        0 = Walkable / Unvisited cell
        1 = Wall / Obstacle
    """

    def __init__(self, rows: int = 20, cols: int = 30):
        """
        Initialize a new 2D grid maze.

        Args:
            rows: Number of rows in the grid (default: 20).
            cols: Number of columns in the grid (default: 30).
        """
        self.rows = max(5, rows)
        self.cols = max(5, cols)

        # 2D matrix: 0 = empty/walkable, 1 = wall
        self.grid: List[List[int]] = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        # Default positions
        self.start: Tuple[int, int] = (3, 3)
        self.goal: Tuple[int, int] = (self.rows - 4, self.cols - 4)

        # Load the initial default example maze with interesting obstacles
        self.load_default_maze()

    def in_bounds(self, row: int, col: int) -> bool:
        """Check whether (row, col) is within grid bounds."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_wall(self, row: int, col: int) -> bool:
        """Check if cell is a wall."""
        if not self.in_bounds(row, col):
            return True
        return self.grid[row][col] == 1

    def is_walkable(self, row: int, col: int) -> bool:
        """Check if cell is within bounds and not a wall."""
        return self.in_bounds(row, col) and self.grid[row][col] == 0

    def set_start(self, row: int, col: int) -> bool:
        """
        Set the Start node coordinate.
        Start cannot be set at the Goal position.
        If a wall was present at (row, col), it is cleared.
        """
        if not self.in_bounds(row, col):
            return False
        if (row, col) == self.goal:
            return False

        self.grid[row][col] = 0
        self.start = (row, col)
        return True

    def set_goal(self, row: int, col: int) -> bool:
        """
        Set the Goal node coordinate.
        Goal cannot be set at the Start position.
        If a wall was present at (row, col), it is cleared.
        """
        if not self.in_bounds(row, col):
            return False
        if (row, col) == self.start:
            return False

        self.grid[row][col] = 0
        self.goal = (row, col)
        return True

    def add_wall(self, row: int, col: int) -> bool:
        """
        Add a wall at (row, col).
        Cannot add a wall on top of Start or Goal.
        """
        if not self.in_bounds(row, col):
            return False
        if (row, col) == self.start or (row, col) == self.goal:
            return False

        self.grid[row][col] = 1
        return True

    def remove_wall(self, row: int, col: int) -> bool:
        """
        Remove a wall at (row, col), making it walkable.
        """
        if not self.in_bounds(row, col):
            return False

        self.grid[row][col] = 0
        return True

    def toggle_wall(self, row: int, col: int) -> bool:
        """
        Toggle wall state at (row, col).
        Does nothing if clicked on Start or Goal.
        """
        if not self.in_bounds(row, col):
            return False
        if (row, col) == self.start or (row, col) == self.goal:
            return False

        self.grid[row][col] = 0 if self.grid[row][col] == 1 else 1
        return True

    def clear_walls(self):
        """
        Remove all walls from the maze while keeping Start and Goal intact.
        """
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[r][c] = 0

    def reset_grid(self):
        """
        Alias to clear all walls.
        """
        self.clear_walls()

    def get_neighbors(self, row: int, col: int) -> List[Tuple[int, int]]:
        """
        Return walkable orthogonal neighbors in 4 directions:
        - Up    (-1, 0)
        - Down  (+1, 0)
        - Left  (0, -1)
        - Right (0, +1)
        """
        # Direction vectors: (delta_row, delta_col)
        directions = [
            (-1, 0),  # Up
            (1, 0),   # Down
            (0, -1),  # Left
            (0, 1),   # Right
        ]

        neighbors = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if self.is_walkable(nr, nc):
                neighbors.append((nr, nc))

        return neighbors

    def load_default_maze(self):
        """
        Configure a rich default example maze with Start, Goal, and strategically
        placed obstacle walls forming rooms, corridors, and detours.
        The user can immediately click 'Start A* Search' upon launch.
        """
        self.clear_walls()

        # Set distinct start and goal
        self.start = (3, 3)
        self.goal = (self.rows - 4, self.cols - 4)

        # Build interesting obstacles (walls and barriers)
        # Vertical barrier with a gap
        mid_col = self.cols // 3
        for r in range(1, self.rows - 3):
            if r != self.rows // 2:  # Leave a passage gap
                self.grid[r][mid_col] = 1

        # Second vertical barrier with a staggered gap
        second_col = (2 * self.cols) // 3
        for r in range(3, self.rows - 1):
            if r != 4:  # Leave a passage gap near top
                self.grid[r][second_col] = 1

        # Horizontal baffle in the middle
        mid_row = self.rows // 2
        for c in range(mid_col + 3, second_col - 2):
            self.grid[mid_row][c] = 1

        # Additional small obstacle clusters
        for r in range(6, 11):
            if r < self.rows:
                self.grid[r][8] = 1

        for c in range(self.cols - 9, self.cols - 3):
            if c < self.cols and 6 < self.rows:
                self.grid[6][c] = 1

        # Guarantee Start and Goal are always clear and walkable
        self.grid[self.start[0]][self.start[1]] = 0
        self.grid[self.goal[0]][self.goal[1]] = 0

    def generate_random_maze(self, wall_density: float = 0.28):
        """
        Generate random walls across the grid with specified density.
        Guarantees:
        - Start and Goal cells are always clear
        - Immediate 4-directional neighborhood of Start and Goal has at least one walkable exit
        """
        self.clear_walls()
        protected: Set[Tuple[int, int]] = {self.start, self.goal}

        # Keep at least 2 orthogonal neighbors around start and goal free to prevent instant trapping
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            sr, sc = self.start[0] + dr, self.start[1] + dc
            if self.in_bounds(sr, sc):
                protected.add((sr, sc))

            gr, gc = self.goal[0] + dr, self.goal[1] + dc
            if self.in_bounds(gr, gc):
                protected.add((gr, gc))

        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) in protected:
                    continue
                if random.random() < wall_density:
                    self.grid[r][c] = 1

        # Always re-verify start and goal
        self.grid[self.start[0]][self.start[1]] = 0
        self.grid[self.goal[0]][self.goal[1]] = 0
