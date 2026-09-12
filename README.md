# Maze Solver using A* Search

A desktop-based Artificial Intelligence pathfinding application that visualizes the **A\* (A-Star) Search Algorithm** from scratch on an interactive 2D grid maze. Built with Python and `tkinter`.

---

## Overview

Pathfinding in complex environments is a fundamental challenge in artificial intelligence, robotics, video game development, and logistics. The **Maze Solver using A* Search** provides an interactive, visual environment to study and demonstrate how heuristic search algorithms efficiently navigate around obstacles to guarantee finding the shortest path between two points.

This project was built for an **Artificial Intelligence Internship Portfolio**, adhering strictly to first-principles: the A* search algorithm is implemented entirely from scratch without external pathfinding libraries or AI APIs.

---

## Features

- **Custom A* Implementation From Scratch**: Built using pure Python without third-party pathfinding libraries.
- **Interactive 2D Grid**:
  - Click & drag to draw or erase obstacle walls.
  - Relocate **Start (S)** and **Goal (G)** nodes dynamically.
  - Right-click quick toggle for instant wall editing.
- **Real-Time Step-by-Step Animation**:
  - Watch the Open Set (frontier) and Closed Set (explored) expand in real time.
  - Configurable animation speed slider (1 ms to 80 ms).
  - Instant solve mode for immediate evaluation.
- **Comprehensive Statistics Panel**:
  - Path Found / No Path indicator.
  - Path Length (in steps).
  - Total Nodes Explored.
  - Total Nodes in the Final Path.
  - Execution Time (high-precision milliseconds).
- **Procedural Random Maze Generator**:
  - Generates randomized maze layouts while preserving Start and Goal accessibility.
- **Pre-Loaded Default Example**:
  - Immediately click "Start A* Search" upon launch to observe pathfinding around barriers.
- **Robust Error Handling**:
  - Validates Start and Goal positions.
  - Accurately detects and handles unreachable Goals.
  - Prevents race conditions by managing search state transitions.
- **Embedded Educational Guide**:
  - In-app "About A* Search" section detailing mathematical principles, heuristics, and algorithmic mechanics.

---

## How A* Search Works

A* is an informed search algorithm that combines the benefits of **Dijkstra's Algorithm** (favoring vertices that are close to the starting point) and **Greedy Best-First Search** (favoring vertices that are estimated to be close to the goal).

Every node $n$ in the search space is scored using the evaluation function:

$$f(n) = g(n) + h(n)$$

Where:
- **$g(n)$ [Past Path Cost]**: The exact cost incurred to reach node $n$ from the Start node. Each orthogonal grid movement has a uniform cost of $1$.
- **$h(n)$ [Future Heuristic Estimate]**: The estimated cost to travel from node $n$ to the Goal node.
- **$f(n)$ [Total Estimated Cost]**: The estimated total cost of the cheapest solution passing through node $n$.

By expanding nodes with the lowest $f(n)$ score first, A* balances exploration of short paths with progress toward the destination.

---

## Manhattan Heuristic

In a 4-directional grid where diagonal movement is disallowed, the shortest distance between two points $(x_1, y_1)$ and $(x_2, y_2)$ is given by the **Manhattan Distance** (also known as $L_1$ norm or city-block distance):

$$h(n) = |x_1 - x_2| + |y_1 - y_2|$$

### Why Manhattan Distance?
1. **Admissibility**: An admissible heuristic never overestimates the actual cost to reach the goal ($h(n) \le h^*(n)$). In an orthogonal grid, a straight line with obstacles can never take fewer steps than the Manhattan distance. Admissibility guarantees that A* will find the **optimal shortest path**.
2. **Consistency (Monotonicity)**: The estimated cost from node $A$ to the goal is no greater than the step cost from $A$ to neighbor $B$ plus the estimated cost from $B$ to the goal ($h(A) \le c(A, B) + h(B)$). On our grid, $|h(A) - h(B)| \le 1$, satisfying consistency and ensuring no node needs to be re-opened.

---

## Algorithm Steps

1. **Initialization**:
   - Initialize an `open_set` (min-priority queue) containing only the `start` node with $f(\text{start}) = h(\text{start})$.
   - Initialize a `closed_set` to store nodes already evaluated.
   - Set $g(\text{start}) = 0$ and $g(n) = \infty$ for all other nodes.
   - Initialize `came_from` dictionary to track parent pointers.
2. **Iteration**:
   - While `open_set` is not empty:
     - Remove the node `current` with the lowest $f(n)$ score from `open_set`.
     - If `current == goal`, reconstruct the shortest path by tracing parent pointers from `goal` to `start` and return success.
     - Move `current` into `closed_set`.
3. **Neighbor Exploration**:
   - For each 4-directional orthogonal neighbor (Up, Down, Left, Right):
     - If neighbor is in `closed_set` or is a wall, skip it.
     - Compute tentative cost: $\text{tentative\_g} = g(\text{current}) + 1$.
     - If $\text{tentative\_g} < g(\text{neighbor})$:
       - Update $\text{came\_from}[\text{neighbor}] = \text{current}$.
       - Update $g(\text{neighbor}) = \text{tentative\_g}$.
       - Update $f(\text{neighbor}) = \text{tentative\_g} + h(\text{neighbor})$.
       - Add `neighbor` to `open_set` if not already present.
4. **Termination**:
   - If `open_set` is empty and `goal` was not reached, conclude that no path exists.

---

## Technologies Used

- **Language**: Python 3 (3.8+)
- **GUI & Graphics**: Python Standard Library `tkinter` and `ttk`
- **Data Structures**: `heapq` (Binary min-heap priority queue), `dataclasses`, `typing`
- **Benchmarking & Timing**: `time.perf_counter` (High-resolution timer)

*No external dependencies or third-party web frameworks are required.*

---

## Project Structure

```text
Maze Solver using A* Search/
│
├── main.py              # Application entry point & automated CLI validation suite
├── astar.py             # A* search algorithm implemented from scratch with generator
├── maze.py              # 2D Grid representation, neighbor discovery, and maze generators
├── gui.py               # Desktop Tkinter GUI, animations, canvas renderer, and educational panel
├── requirements.txt     # Dependency specifications (Python standard library)
└── README.md            # Complete project documentation
```

---

## Installation

### 1. Prerequisites
Ensure you have Python 3.8 or higher installed on your computer.

Check your Python version:
```bash
python --version
# or
python3 --version
```

### 2. Tkinter Support (Linux only)
If you are running on Debian, Ubuntu, or Linux Mint:
```bash
sudo apt-get update
sudo apt-get install -y python3-tk
```
*(On Windows and macOS, Tkinter is bundled by default with official Python installers).*

### 3. Clone / Download the Project
```bash
git clone <your-repository-url>
cd "Maze Solver using A* Search"
```

---

## How to Run

### Launch Desktop GUI Application:
```bash
python main.py
```
*(Or `python3 main.py` depending on your operating system configuration).*

### Run Automated Headless Self-Tests:
```bash
python main.py --test
```
This runs an internal test suite that verifies Manhattan distance calculation, straight line traversal, obstacle avoidance, unreachable goal detection, and default maze solutions directly from the terminal without opening a GUI window.

---

## How to Use

1. **Run the Solver**: Click the blue **▶ Start A* Search** button to watch the algorithm explore the maze step-by-step.
2. **Speed Controls**: Adjust the **Speed** slider to slow down or speed up the animation, or check **Instant** for immediate path display.
3. **Draw Obstacles**: Select **Add Wall** mode, then click or drag your mouse across the grid to draw barriers.
4. **Erase Obstacles**: Select **Remove Wall** mode or right-click directly on any cell to remove it.
5. **Relocate Start and Goal**: Choose **Set Start** or **Set Goal**, then click any open cell on the grid.
6. **Generate Random Maze**: Click **🎲 Random Maze** to procedurally scatter random obstacles across the board.
7. **Reset**:
   - **🔄 Reset Search**: Clears the path and visited nodes while keeping your custom walls intact.
   - **🗑 Clear Maze**: Clears all walls and resets the board.
   - **⭐ Default Example**: Restores the starter demonstration maze.

---

## Example

### Default Starter Maze
When the program launches, it pre-populates a maze featuring a Start node at $(3, 3)$, a Goal node at $(16, 26)$, and segmented vertical barriers with staggered openings. Clicking **Start A* Search** visually demonstrates how A* explores through corridor openings and curves around dead ends to reach the goal along the mathematically shortest trajectory.

---

## Handling Unreachable Paths

When walls completely enclose either the Start node or the Goal node:
- The A* search algorithm systematically examines all reachable cells until the priority queue (`open_set`) is completely exhausted.
- The program identifies that no valid path can be formed.
- The UI status badge clearly updates in red to:
  ```text
  No path exists between the Start and Goal.
  ```
- The statistics panel reports:
  - **Path Result**: `No Path`
  - **Path Length**: `0`
  - **Nodes in Final Path**: `0`
  - **Nodes Explored**: Total count of reachable cells checked before termination.

---

## Future Improvements

- Support for diagonal 8-directional movement with Euclidean / Octile distance heuristics ($h(n) = \sqrt{\Delta x^2 + \Delta y^2}$).
- Weighted terrain / cost maps (e.g., mud, water, grass) where cell movement costs vary ($g(n) > 1$).
- Comparison mode: Visualizing A* side-by-side with Breadth-First Search (BFS) and Greedy Best-First Search to demonstrate node expansion efficiency.
- Recursive backtracking and Prim's algorithm for generating perfect mazes (mazes with exactly one unique path and no loops).

---

## Author

- **Name**: [Your Name Here]
- **GitHub Profile**: [https://github.com/your-username](https://github.com/your-username)
- **Internship Project**: Artificial Intelligence Internship Portfolio
