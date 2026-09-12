"""
astar.py - A* Search Algorithm Implementation from Scratch
===========================================================
This module implements the A* (A-Star) search algorithm for finding the
shortest path on a 2D grid from scratch using Python's standard library.

Movement:
    4-directional orthogonal movement:
    - Up    (-1, 0)
    - Down  (+1, 0)
    - Left  (0, -1)
    - Right (0, +1)

Cost functions:
    g(n) = Exact cost from the Start node to current node n.
           Each orthogonal step has a uniform cost of 1.
    h(n) = Heuristic estimated cost from node n to the Goal node.
           Calculated using the Manhattan Distance:
           h(n) = |x1 - x2| + |y1 - y2|
           (or |row1 - row2| + |col1 - col2|)
    f(n) = Total estimated cost of path through node n:
           f(n) = g(n) + h(n)

Data structures:
    - Open Set: Min-priority queue (heapq) containing nodes discovered but not yet evaluated.
    - Closed Set: Set of nodes that have already been evaluated.
    - Came From / Parent pointers: Used to reconstruct the final optimal path.
"""

from dataclasses import dataclass, field
import heapq
import time
from typing import Generator, List, Optional, Set, Tuple, Any, Dict


def manhattan_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
    """
    Calculate the Manhattan distance heuristic between two 2D coordinates.

    Formula:
        h(n) = |x1 - x2| + |y1 - y2|

    For a 4-directional grid where each step has cost 1, the Manhattan distance
    is both ADMISSIBLE (it never overestimates the true remaining cost) and
    CONSISTENT (monotonic), guaranteeing that A* will return the optimal shortest path.

    Args:
        p1: Tuple (row, col) of first point.
        p2: Tuple (row, col) of second point.

    Returns:
        Integer Manhattan distance between p1 and p2.
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


@dataclass(order=True)
class PriorityItem:
    """
    Wrapper for storing items in the priority queue with tie-breaking.
    Comparison order:
        1. f_score (lowest total estimated cost first)
        2. h_score (tie-breaker: lowest heuristic first, moves toward goal faster)
        3. tie_breaker (counter to avoid comparing unorderable data)
    """
    f_score: float
    h_score: float
    tie_breaker: int
    position: Tuple[int, int] = field(compare=False)


@dataclass
class SearchResult:
    """
    Container for search execution statistics and path details.
    """
    path_found: bool
    path: List[Tuple[int, int]]
    path_length: int
    nodes_explored: int
    nodes_in_path: int
    execution_time_ms: float
    visited_nodes: List[Tuple[int, int]]
    cost: float
    message: str


def reconstruct_path(
    came_from: Dict[Tuple[int, int], Tuple[int, int]],
    current: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """
    Reconstruct the final path by following parent pointers from Goal back to Start.

    Args:
        came_from: Dictionary mapping node -> its parent node.
        current: The Goal node coordinate.

    Returns:
        List of coordinates ordered from Start to Goal: [Start, ..., Goal].
    """
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path


def astar_search(
    maze,
    start: Tuple[int, int],
    goal: Tuple[int, int]
) -> SearchResult:
    """
    Perform synchronous A* search on the given maze.

    Args:
        maze: Maze object providing grid boundaries and is_walkable(r, c) method.
        start: (row, col) of Start node.
        goal: (row, col) of Goal node.

    Returns:
        SearchResult object containing path and search statistics.
    """
    start_time = time.perf_counter()

    # Edge Case: Start and Goal are identical
    if start == goal:
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            path_found=True,
            path=[start],
            path_length=0,
            nodes_explored=1,
            nodes_in_path=1,
            execution_time_ms=elapsed_ms,
            visited_nodes=[start],
            cost=0.0,
            message="Start and Goal are the same node."
        )

    # Edge Case: Start or Goal is not walkable
    if not maze.is_walkable(start[0], start[1]):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            path_found=False,
            path=[],
            path_length=0,
            nodes_explored=0,
            nodes_in_path=0,
            execution_time_ms=elapsed_ms,
            visited_nodes=[],
            cost=0.0,
            message="Start node is placed on a wall or outside the maze."
        )

    if not maze.is_walkable(goal[0], goal[1]):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        return SearchResult(
            path_found=False,
            path=[],
            path_length=0,
            nodes_explored=0,
            nodes_in_path=0,
            execution_time_ms=elapsed_ms,
            visited_nodes=[],
            cost=0.0,
            message="Goal node is placed on a wall or outside the maze."
        )

    # Priority Queue for Open Set
    open_heap: List[PriorityItem] = []
    # Set of positions currently in Open Set for O(1) membership lookup
    open_set_hash: Set[Tuple[int, int]] = {start}
    # Set of positions already evaluated (Closed Set)
    closed_set: Set[Tuple[int, int]] = set()

    # g_score[n]: Lowest known exact cost from Start to n
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}

    # Parent pointers for path reconstruction
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    # Counter to break priority queue ties consistently
    tie_counter = 0

    # Initial heuristic
    initial_h = manhattan_distance(start, goal)
    heapq.heappush(
        open_heap,
        PriorityItem(
            f_score=float(initial_h),
            h_score=float(initial_h),
            tie_breaker=tie_counter,
            position=start
        )
    )

    visited_order: List[Tuple[int, int]] = []
    nodes_explored = 0

    while open_heap:
        # Pop node with lowest f(n) (and lowest h(n) as tie-breaker)
        current_item = heapq.heappop(open_heap)
        current = current_item.position

        # If already removed from open_set_hash, skip stale heap entries
        if current not in open_set_hash:
            continue

        open_set_hash.remove(current)
        closed_set.add(current)
        visited_order.append(current)
        nodes_explored += 1

        # Check if Goal has been reached
        if current == goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            final_path = reconstruct_path(came_from, current)
            return SearchResult(
                path_found=True,
                path=final_path,
                path_length=len(final_path) - 1,
                nodes_explored=nodes_explored,
                nodes_in_path=len(final_path),
                execution_time_ms=elapsed_ms,
                visited_nodes=visited_order,
                cost=g_score[current],
                message="Path found successfully!"
            )

        # Explore 4-directional neighbors: Up, Down, Left, Right
        for neighbor in maze.get_neighbors(current[0], current[1]):
            if neighbor in closed_set:
                # Node already evaluated in closed set
                continue

            # Orthogonal step cost is 1
            tentative_g = g_score[current] + 1.0

            # If this path to neighbor is better than any previous one:
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h_cost = manhattan_distance(neighbor, goal)
                f_cost = tentative_g + h_cost

                if neighbor not in open_set_hash:
                    tie_counter += 1
                    open_set_hash.add(neighbor)
                    heapq.heappush(
                        open_heap,
                        PriorityItem(
                            f_score=f_cost,
                            h_score=float(h_cost),
                            tie_breaker=tie_counter,
                            position=neighbor
                        )
                    )

    # If the open set is exhausted and Goal was never reached:
    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    return SearchResult(
        path_found=False,
        path=[],
        path_length=0,
        nodes_explored=nodes_explored,
        nodes_in_path=0,
        execution_time_ms=elapsed_ms,
        visited_nodes=visited_order,
        cost=0.0,
        message="No path exists between the Start and Goal."
    )


def astar_search_generator(
    maze,
    start: Tuple[int, int],
    goal: Tuple[int, int]
) -> Generator[Tuple[str, Any], None, None]:
    """
    Generator implementation of A* Search for step-by-step GUI animation.

    Yields events:
        - ("step", {
              "current": (r, c),
              "open_set": set of positions,
              "closed_set": set of positions,
              "nodes_explored": int,
              "g": float,
              "h": float,
              "f": float
          })
        - ("complete", SearchResult)
        - ("error", SearchResult)

    Args:
        maze: Maze object providing grid boundaries and walkable neighbors.
        start: (row, col) of Start node.
        goal: (row, col) of Goal node.
    """
    start_time = time.perf_counter()

    # Edge Case: Start and Goal are identical
    if start == goal:
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        res = SearchResult(
            path_found=True,
            path=[start],
            path_length=0,
            nodes_explored=1,
            nodes_in_path=1,
            execution_time_ms=elapsed_ms,
            visited_nodes=[start],
            cost=0.0,
            message="Start and Goal are the same node."
        )
        yield ("complete", res)
        return

    # Edge Case: Start or Goal invalid
    if not maze.is_walkable(start[0], start[1]):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        res = SearchResult(
            path_found=False,
            path=[],
            path_length=0,
            nodes_explored=0,
            nodes_in_path=0,
            execution_time_ms=elapsed_ms,
            visited_nodes=[],
            cost=0.0,
            message="Start node is placed on a wall or outside the maze."
        )
        yield ("error", res)
        return

    if not maze.is_walkable(goal[0], goal[1]):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        res = SearchResult(
            path_found=False,
            path=[],
            path_length=0,
            nodes_explored=0,
            nodes_in_path=0,
            execution_time_ms=elapsed_ms,
            visited_nodes=[],
            cost=0.0,
            message="Goal node is placed on a wall or outside the maze."
        )
        yield ("error", res)
        return

    open_heap: List[PriorityItem] = []
    open_set_hash: Set[Tuple[int, int]] = {start}
    closed_set: Set[Tuple[int, int]] = set()

    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    tie_counter = 0
    initial_h = manhattan_distance(start, goal)
    heapq.heappush(
        open_heap,
        PriorityItem(
            f_score=float(initial_h),
            h_score=float(initial_h),
            tie_breaker=tie_counter,
            position=start
        )
    )

    visited_order: List[Tuple[int, int]] = []
    nodes_explored = 0

    while open_heap:
        current_item = heapq.heappop(open_heap)
        current = current_item.position

        if current not in open_set_hash:
            continue

        open_set_hash.remove(current)
        closed_set.add(current)
        visited_order.append(current)
        nodes_explored += 1

        curr_g = g_score[current]
        curr_h = manhattan_distance(current, goal)
        curr_f = curr_g + curr_h

        # Yield visual step event for GUI animation
        yield ("step", {
            "current": current,
            "open_set": set(open_set_hash),
            "closed_set": set(closed_set),
            "nodes_explored": nodes_explored,
            "g": curr_g,
            "h": curr_h,
            "f": curr_f
        })

        # Check if Goal has been reached
        if current == goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            final_path = reconstruct_path(came_from, current)
            res = SearchResult(
                path_found=True,
                path=final_path,
                path_length=len(final_path) - 1,
                nodes_explored=nodes_explored,
                nodes_in_path=len(final_path),
                execution_time_ms=elapsed_ms,
                visited_nodes=visited_order,
                cost=g_score[current],
                message="Path found successfully!"
            )
            yield ("complete", res)
            return

        # Explore 4-directional neighbors
        for neighbor in maze.get_neighbors(current[0], current[1]):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + 1.0

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h_cost = manhattan_distance(neighbor, goal)
                f_cost = tentative_g + h_cost

                if neighbor not in open_set_hash:
                    tie_counter += 1
                    open_set_hash.add(neighbor)
                    heapq.heappush(
                        open_heap,
                        PriorityItem(
                            f_score=f_cost,
                            h_score=float(h_cost),
                            tie_breaker=tie_counter,
                            position=neighbor
                        )
                    )

    # Exhausted open set without finding goal
    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    res = SearchResult(
        path_found=False,
        path=[],
        path_length=0,
        nodes_explored=nodes_explored,
        nodes_in_path=0,
        execution_time_ms=elapsed_ms,
        visited_nodes=visited_order,
        cost=0.0,
        message="No path exists between the Start and Goal."
    )
    yield ("complete", res)
