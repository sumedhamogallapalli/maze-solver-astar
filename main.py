"""
main.py - Main Entry Point for Maze Solver Desktop Application
==============================================================
Runs the Python desktop application using Tkinter.

Usage:
    # Run the GUI application:
    python main.py

    # Run automated self-validation tests (Headless CLI):
    python main.py --test
"""

import sys
import tkinter as tk
from gui import MazeSolverGUI
from maze import Maze
from astar import astar_search, manhattan_distance


def run_self_tests():
    """
    Run automated verification suite for A* Search algorithm,
    Manhattan distance calculations, path reconstruction, obstacle avoidance,
    and unreachable goal handling.
    """
    print("=" * 60)
    print("RUNNING AUTOMATED A* SEARCH VERIFICATION TESTS")
    print("=" * 60)

    # Test 1: Manhattan Distance
    print("\n[Test 1] Verifying Manhattan Distance calculation...")
    p1 = (0, 0)
    p2 = (3, 4)
    expected_h = abs(0 - 3) + abs(0 - 4)  # 7
    actual_h = manhattan_distance(p1, p2)
    assert actual_h == expected_h, f"Expected {expected_h}, got {actual_h}"
    print(f"  ✓ Manhattan distance between {p1} and {p2} = {actual_h} (Expected: 7)")

    # Test 2: Straight line path on empty maze
    print("\n[Test 2] Verifying shortest path on open grid...")
    maze = Maze(rows=10, cols=10)
    maze.clear_walls()
    maze.set_start(0, 0)
    maze.set_goal(0, 5)
    res = astar_search(maze, maze.start, maze.goal)
    assert res.path_found, "Failed to find path on empty grid"
    assert res.path_length == 5, f"Expected path length 5, got {res.path_length}"
    assert res.path[0] == (0, 0) and res.path[-1] == (0, 5)
    print(f"  ✓ Straight line path found! Steps: {res.path_length}, Nodes in path: {res.nodes_in_path}")

    # Test 3: Navigating around an obstacle wall
    print("\n[Test 3] Verifying obstacle wall avoidance...")
    maze.clear_walls()
    maze.set_start(2, 2)
    maze.set_goal(2, 6)
    # Place a vertical wall blocking the direct horizontal path at column 4
    for r in range(1, 5):
        maze.add_wall(r, 4)

    res = astar_search(maze, maze.start, maze.goal)
    assert res.path_found, "Failed to find path around obstacle"
    for r, c in res.path:
        assert not maze.is_wall(r, c), f"Path crossed wall at ({r}, {c})!"
    print(f"  ✓ Obstacle avoided! Path length: {res.path_length}. No walls crossed.")

    # Test 4: Unreachable Goal detection
    print("\n[Test 4] Verifying unreachable goal detection...")
    maze.clear_walls()
    maze.set_start(5, 5)
    maze.set_goal(1, 1)
    # Completely box in the goal with walls
    maze.add_wall(0, 1)
    maze.add_wall(2, 1)
    maze.add_wall(1, 0)
    maze.add_wall(1, 2)

    res = astar_search(maze, maze.start, maze.goal)
    assert not res.path_found, "Goal should be unreachable!"
    assert res.message == "No path exists between the Start and Goal."
    print(f"  ✓ Unreachable goal correctly detected: '{res.message}'")

    # Test 5: Identical Start and Goal
    print("\n[Test 5] Verifying identical Start and Goal...")
    maze.clear_walls()
    res = astar_search(maze, (4, 4), (4, 4))
    assert res.path_found
    assert res.path_length == 0
    assert len(res.path) == 1
    print("  ✓ Identical Start and Goal handled correctly (cost = 0).")

    # Test 6: Default starter maze
    print("\n[Test 6] Verifying default starter example maze...")
    default_maze = Maze(rows=20, cols=30)
    default_maze.load_default_maze()
    res = astar_search(default_maze, default_maze.start, default_maze.goal)
    assert res.path_found, "Default maze must have a valid path!"
    print(f"  ✓ Default maze solved! Path length: {res.path_length}, Nodes explored: {res.nodes_explored}, Time: {res.execution_time_ms:.2f} ms")

    print("\n" + "=" * 60)
    print("ALL 6 TESTS PASSED SUCCESSFULLY! (100% SUCCESS RATE)")
    print("=" * 60)


def main():
    """Application main entry point."""
    # Check if CLI testing flag was passed
    if "--test" in sys.argv or "--cli" in sys.argv:
        run_self_tests()
        sys.exit(0)

    # Launch Desktop Tkinter Application
    root = tk.Tk()
    app = MazeSolverGUI(root, rows=20, cols=30)
    root.mainloop()


if __name__ == "__main__":
    main()
