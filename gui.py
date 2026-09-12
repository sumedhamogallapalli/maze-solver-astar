"""
gui.py - Desktop Tkinter Graphical User Interface for Maze Solver
==================================================================
This module provides the desktop GUI for the Maze Solver using A* Search.
Built with Python's built-in tkinter and ttk libraries.

Features:
- Interactive 2D Canvas grid with click & drag wall drawing / erasing
- Start and Goal placement tools
- Color-coded cell visualization for:
    - Start node (Green)
    - Goal node (Red)
    - Walls (Dark Slate)
    - Open Set / Frontier (Amber / Orange)
    - Closed Set / Explored (Sky Blue)
    - Final Shortest Path (Purple / Gold)
    - Unvisited cells (White)
- Real-time step-by-step animation with adjustable speed slider
- Legend explaining all cell states
- Live statistics panel (Path Found/No Path, Path Length, Nodes Explored, Nodes in Path, Execution Time)
- Pre-loaded default example maze
- Random maze generator
- Comprehensive educational 'About A* Search' tab
- Safe error handling for unreachable paths, identical start/goal, and runtime resets
"""

import math
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Optional, Set, Tuple

from astar import astar_search, astar_search_generator, SearchResult
from maze import Maze


# UI Color Palette Constants
COLOR_BG = "#0F172A"             # Dark navy background
COLOR_PANEL_BG = "#1E293B"       # Slate card background
COLOR_TEXT_PRIMARY = "#F8FAFC"   # White / Light slate text
COLOR_TEXT_SECONDARY = "#94A3B8" # Muted gray text
COLOR_BORDER = "#334155"         # Subtle divider border

# Grid Visualization Colors
COLOR_UNVISITED = "#FFFFFF"      # Walkable empty cell
COLOR_WALL = "#1E293B"           # Obstacle / Wall
COLOR_GRID_LINE = "#E2E8F0"      # Grid cell outline
COLOR_START = "#10B981"          # Start node (Emerald green)
COLOR_GOAL = "#EF4444"           # Goal node (Rose red)
COLOR_OPEN = "#F59E0B"           # Open Set / Frontier (Amber)
COLOR_CLOSED = "#60A5FA"         # Closed Set / Explored (Sky blue)
COLOR_PATH = "#8B5CF6"           # Final Shortest Path (Royal purple)
COLOR_PATH_LINE = "#D97706"      # Center connecting path line


class MazeSolverGUI:
    """
    Main Tkinter Desktop Application for the Maze Solver.
    """

    def __init__(self, root: tk.Tk, rows: int = 20, cols: int = 30):
        self.root = root
        self.root.title("Maze Solver using A* Search — Desktop Application")
        self.root.geometry("1220x780")
        self.root.minsize(1050, 680)
        self.root.configure(bg=COLOR_BG)

        # Core State
        self.rows = rows
        self.cols = cols
        self.maze = Maze(rows=self.rows, cols=self.cols)

        # Interaction Mode: 'wall', 'erase', 'start', 'goal'
        self.mode = tk.StringVar(value="wall")

        # Animation State
        self.is_searching = False
        self.is_paused = False
        self.search_generator = None
        self.after_id = None
        self.animation_delay_ms = tk.IntVar(value=15)
        self.instant_mode = tk.BooleanVar(value=False)

        # Visited tracking for rendering
        self.open_set_coords: Set[Tuple[int, int]] = set()
        self.closed_set_coords: Set[Tuple[int, int]] = set()
        self.final_path: list[Tuple[int, int]] = []

        # Statistics variables
        self.stat_status = tk.StringVar(value="Ready to search")
        self.stat_path_found = tk.StringVar(value="--")
        self.stat_path_length = tk.StringVar(value="0")
        self.stat_nodes_explored = tk.StringVar(value="0")
        self.stat_nodes_in_path = tk.StringVar(value="0")
        self.stat_time = tk.StringVar(value="0.0 ms")

        # Setup custom modern ttk styles
        self._setup_styles()

        # Build UI layout
        self._build_layout()

        # Initial Canvas render
        self.root.update_idletasks()
        self._draw_grid()

    def _setup_styles(self):
        """Configure modern dark ttk styling for widgets."""
        style = ttk.Style()
        style.theme_use("clam")

        # General frame
        style.configure("Dark.TFrame", background=COLOR_PANEL_BG)
        style.configure("Root.TFrame", background=COLOR_BG)

        # Notebook tabs
        style.configure("TNotebook", background=COLOR_BG, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background=COLOR_PANEL_BG,
            foreground=COLOR_TEXT_SECONDARY,
            padding=[12, 6],
            font=("Segoe UI", 10, "bold")
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", "#2563EB")],
            foreground=[("selected", "#FFFFFF")]
        )

        # Radiobuttons for Mode selection
        style.configure(
            "Mode.TRadiobutton",
            background=COLOR_PANEL_BG,
            foreground=COLOR_TEXT_PRIMARY,
            font=("Segoe UI", 10)
        )
        style.map(
            "Mode.TRadiobutton",
            background=[("active", COLOR_PANEL_BG)],
            foreground=[("active", "#38BDF8")]
        )

    def _build_layout(self):
        """Construct the overall application window structure."""
        # Top Header Bar
        header_frame = tk.Frame(self.root, bg=COLOR_PANEL_BG, height=54, padx=16, pady=8)
        header_frame.pack(fill=tk.X, side=tk.TOP)

        title_label = tk.Label(
            header_frame,
            text="Maze Solver using A* Search",
            font=("Segoe UI", 16, "bold"),
            fg="#FFFFFF",
            bg=COLOR_PANEL_BG
        )
        title_label.pack(side=tk.LEFT)

        subtitle_label = tk.Label(
            header_frame,
            text="Interactive Pathfinding with Manhattan Heuristic",
            font=("Segoe UI", 10),
            fg=COLOR_TEXT_SECONDARY,
            bg=COLOR_PANEL_BG
        )
        subtitle_label.pack(side=tk.LEFT, padx=(12, 0), pady=(3, 0))

        # Main Workspace: Left is Grid + Controls, Right is Statistics & Info
        workspace = tk.Frame(self.root, bg=COLOR_BG, padx=12, pady=10)
        workspace.pack(fill=tk.BOTH, expand=True)

        left_container = tk.Frame(workspace, bg=COLOR_BG)
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        right_container = tk.Frame(workspace, bg=COLOR_BG, width=370)
        right_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        right_container.pack_propagate(False)

        # 1. Left: Control Toolbar
        self._build_toolbar(left_container)

        # 2. Left: Canvas Viewport
        self._build_canvas_viewport(left_container)

        # 3. Left: Legend Bar
        self._build_legend(left_container)

        # 4. Right: Statistics & Educational Sections (Notebook tabs)
        self._build_right_panel(right_container)

    def _build_toolbar(self, parent):
        """Construct interactive tools, modes, and search control buttons."""
        toolbar = tk.Frame(parent, bg=COLOR_PANEL_BG, padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
        toolbar.pack(fill=tk.X, pady=(0, 8))

        # --- Row 1: Mode Selectors ---
        mode_frame = tk.Frame(toolbar, bg=COLOR_PANEL_BG)
        mode_frame.pack(fill=tk.X, side=tk.TOP, pady=(0, 6))

        mode_title = tk.Label(
            mode_frame,
            text="Mode:",
            font=("Segoe UI", 9, "bold"),
            fg=COLOR_TEXT_SECONDARY,
            bg=COLOR_PANEL_BG
        )
        mode_title.pack(side=tk.LEFT, padx=(0, 8))

        modes = [
            ("Add Wall", "wall"),
            ("Remove Wall", "erase"),
            ("Set Start", "start"),
            ("Set Goal", "goal")
        ]

        for text, value in modes:
            rb = ttk.Radiobutton(
                mode_frame,
                text=text,
                value=value,
                variable=self.mode,
                style="Mode.TRadiobutton"
            )
            rb.pack(side=tk.LEFT, padx=6)

        # --- Row 2: Action Buttons & Speed Slider ---
        action_frame = tk.Frame(toolbar, bg=COLOR_PANEL_BG)
        action_frame.pack(fill=tk.X, side=tk.TOP)

        self.btn_search = tk.Button(
            action_frame,
            text="▶ Start A* Search",
            font=("Segoe UI", 10, "bold"),
            bg="#2563EB",
            fg="#FFFFFF",
            activebackground="#1D4ED8",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=12,
            pady=4,
            cursor="hand2",
            command=self.start_search
        )
        self.btn_search.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_pause = tk.Button(
            action_frame,
            text="⏸ Pause",
            font=("Segoe UI", 9),
            bg="#475569",
            fg="#FFFFFF",
            activebackground="#334155",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.toggle_pause,
            state=tk.DISABLED
        )
        self.btn_pause.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_reset_search = tk.Button(
            action_frame,
            text="🔄 Reset Search",
            font=("Segoe UI", 9),
            bg="#334155",
            fg=COLOR_TEXT_PRIMARY,
            activebackground="#475569",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.reset_search
        )
        self.btn_reset_search.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_clear_maze = tk.Button(
            action_frame,
            text="🗑 Clear Maze",
            font=("Segoe UI", 9),
            bg="#334155",
            fg=COLOR_TEXT_PRIMARY,
            activebackground="#475569",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.clear_maze
        )
        self.btn_clear_maze.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_random = tk.Button(
            action_frame,
            text="🎲 Random Maze",
            font=("Segoe UI", 9),
            bg="#334155",
            fg=COLOR_TEXT_PRIMARY,
            activebackground="#475569",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.generate_random_maze
        )
        self.btn_random.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_default = tk.Button(
            action_frame,
            text="⭐ Default Example",
            font=("Segoe UI", 9),
            bg="#334155",
            fg=COLOR_TEXT_PRIMARY,
            activebackground="#475569",
            activeforeground="#FFFFFF",
            relief=tk.FLAT,
            padx=8,
            pady=4,
            cursor="hand2",
            command=self.load_default_maze
        )
        self.btn_default.pack(side=tk.LEFT, padx=(0, 10))

        # Speed Slider & Instant Checkbox
        speed_label = tk.Label(
            action_frame,
            text="Speed:",
            font=("Segoe UI", 9),
            fg=COLOR_TEXT_SECONDARY,
            bg=COLOR_PANEL_BG
        )
        speed_label.pack(side=tk.LEFT, padx=(6, 2))

        self.slider_speed = tk.Scale(
            action_frame,
            from_=1,
            to=80,
            orient=tk.HORIZONTAL,
            variable=self.animation_delay_ms,
            showvalue=0,
            bg=COLOR_PANEL_BG,
            fg=COLOR_TEXT_PRIMARY,
            troughcolor="#0F172A",
            highlightthickness=0,
            length=90
        )
        self.slider_speed.pack(side=tk.LEFT, padx=(0, 6))

        chk_instant = tk.Checkbutton(
            action_frame,
            text="Instant",
            variable=self.instant_mode,
            bg=COLOR_PANEL_BG,
            fg=COLOR_TEXT_PRIMARY,
            selectcolor="#0F172A",
            activebackground=COLOR_PANEL_BG,
            activeforeground=COLOR_TEXT_PRIMARY,
            font=("Segoe UI", 9)
        )
        chk_instant.pack(side=tk.LEFT)

    def _build_canvas_viewport(self, parent):
        """Construct the interactive Tkinter Canvas for the maze grid."""
        canvas_card = tk.Frame(parent, bg="#000000", highlightthickness=1, highlightbackground=COLOR_BORDER)
        canvas_card.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            canvas_card,
            bg="#FFFFFF",
            highlightthickness=0,
            cursor="crosshair"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Mouse Event Listeners for click & drag
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<Button-3>", self._on_canvas_right_click)
        self.canvas.bind("<B3-Motion>", self._on_canvas_right_drag)
        self.canvas.bind("<Configure>", self._on_canvas_resize)

    def _build_legend(self, parent):
        """Construct the color legend explaining all node states."""
        legend_frame = tk.Frame(parent, bg=COLOR_PANEL_BG, padx=8, pady=6, highlightthickness=1, highlightbackground=COLOR_BORDER)
        legend_frame.pack(fill=tk.X, pady=(8, 0))

        items = [
            ("Start", COLOR_START, "#FFFFFF"),
            ("Goal", COLOR_GOAL, "#FFFFFF"),
            ("Wall", COLOR_WALL, "#FFFFFF"),
            ("Open / Frontier", COLOR_OPEN, "#000000"),
            ("Explored / Closed", COLOR_CLOSED, "#000000"),
            ("Shortest Path", COLOR_PATH, "#FFFFFF"),
            ("Unvisited", "#FFFFFF", "#000000")
        ]

        title = tk.Label(legend_frame, text="Legend:", font=("Segoe UI", 9, "bold"), fg=COLOR_TEXT_SECONDARY, bg=COLOR_PANEL_BG)
        title.pack(side=tk.LEFT, padx=(4, 10))

        for label_text, swatch_color, text_color in items:
            chip = tk.Frame(legend_frame, bg=COLOR_PANEL_BG)
            chip.pack(side=tk.LEFT, padx=6)

            swatch = tk.Label(chip, text="   ", bg=swatch_color, relief=tk.SOLID, bd=1)
            swatch.pack(side=tk.LEFT, padx=(0, 4))

            lbl = tk.Label(chip, text=label_text, font=("Segoe UI", 9), fg=COLOR_TEXT_PRIMARY, bg=COLOR_PANEL_BG)
            lbl.pack(side=tk.LEFT)

    def _build_right_panel(self, parent):
        """Construct the Right Panel with Statistics and the 'About A*' Educational tab."""
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Live Statistics & Metrics
        tab_stats = ttk.Frame(notebook, style="Dark.TFrame", padding=12)
        notebook.add(tab_stats, text="Search Statistics")
        self._build_stats_tab(tab_stats)

        # Tab 2: Educational 'About A* Search'
        tab_about = ttk.Frame(notebook, style="Dark.TFrame", padding=10)
        notebook.add(tab_about, text="About A* Search")
        self._build_about_tab(tab_about)

    def _build_stats_tab(self, parent):
        """Build metrics cards displaying live search statistics."""
        # Status Box
        status_card = tk.Frame(parent, bg="#0F172A", padx=10, pady=8, relief=tk.FLAT)
        status_card.pack(fill=tk.X, pady=(0, 10))

        tk.Label(status_card, text="Search Status:", font=("Segoe UI", 9, "bold"), fg=COLOR_TEXT_SECONDARY, bg="#0F172A").pack(anchor=tk.W)
        self.lbl_status = tk.Label(
            status_card,
            textvariable=self.stat_status,
            font=("Segoe UI", 11, "bold"),
            fg="#38BDF8",
            bg="#0F172A",
            wraplength=320,
            justify=tk.LEFT
        )
        self.lbl_status.pack(anchor=tk.W, pady=(2, 0))

        # Metrics grid
        metrics_container = tk.Frame(parent, bg=COLOR_PANEL_BG)
        metrics_container.pack(fill=tk.BOTH, expand=True)

        metrics = [
            ("Path Result", self.stat_path_found, "#F8FAFC"),
            ("Path Length (steps)", self.stat_path_length, "#34D399"),
            ("Nodes in Final Path", self.stat_nodes_in_path, "#A78BFA"),
            ("Nodes Explored (Closed)", self.stat_nodes_explored, "#60A5FA"),
            ("Execution Time", self.stat_time, "#FBBF24")
        ]

        for label_text, var, highlight_color in metrics:
            card = tk.Frame(metrics_container, bg="#0F172A", padx=10, pady=8, highlightthickness=1, highlightbackground=COLOR_BORDER)
            card.pack(fill=tk.X, pady=4)

            tk.Label(
                card,
                text=label_text,
                font=("Segoe UI", 9),
                fg=COLOR_TEXT_SECONDARY,
                bg="#0F172A"
            ).pack(anchor=tk.W)

            tk.Label(
                card,
                textvariable=var,
                font=("Segoe UI", 14, "bold"),
                fg=highlight_color,
                bg="#0F172A"
            ).pack(anchor=tk.W, pady=(2, 0))

        # Instructions helper note
        tip_frame = tk.Frame(parent, bg="#1E293B", padx=8, pady=8)
        tip_frame.pack(fill=tk.X, side=tk.BOTTOM)

        tip_text = (
            "💡 Interaction Tips:\n"
            "• Click & drag to draw or erase walls\n"
            "• Right-click directly toggles any wall\n"
            "• Select 'Set Start' or 'Set Goal' to relocate\n"
            "• Hit 'Start A* Search' to watch it solve!"
        )
        tk.Label(
            tip_frame,
            text=tip_text,
            font=("Segoe UI", 8),
            fg=COLOR_TEXT_SECONDARY,
            bg="#1E293B",
            justify=tk.LEFT
        ).pack(anchor=tk.W)

    def _build_about_tab(self, parent):
        """Build comprehensive educational documentation inside the GUI."""
        scroll_container = tk.Frame(parent, bg=COLOR_PANEL_BG)
        scroll_container.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(scroll_container)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        text_area = tk.Text(
            scroll_container,
            wrap=tk.WORD,
            yscrollcommand=scrollbar.set,
            bg="#0F172A",
            fg=COLOR_TEXT_PRIMARY,
            insertbackground="#FFFFFF",
            font=("Segoe UI", 9),
            padx=10,
            pady=10,
            relief=tk.FLAT
        )
        text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=text_area.yview)

        # Style tags for highlighted headers
        text_area.tag_configure("title", font=("Segoe UI", 12, "bold"), foreground="#38BDF8")
        text_area.tag_configure("h1", font=("Segoe UI", 10, "bold"), foreground="#F59E0B")
        text_area.tag_configure("code", font=("Courier New", 9, "bold"), foreground="#34D399")
        text_area.tag_configure("formula", font=("Courier New", 10, "bold"), foreground="#F43F5E")
        text_area.tag_configure("body", font=("Segoe UI", 9), foreground="#E2E8F0")

        about_content = [
            ("About A* Search Algorithm\n\n", "title"),
            ("1. What is A* Search?\n", "h1"),
            ("A* (pronounced 'A-star') is one of the most widely used graph traversal and pathfinding algorithms in Artificial Intelligence and computer science. It finds the shortest path between a starting node and a goal node by intelligently guiding its search using a heuristic evaluation function.\n\n", "body"),

            ("2. What is a Node?\n", "h1"),
            ("A node represents a single discrete position or state in the graph. In this 2D grid maze, each grid cell at coordinate (row, col) is treated as a node with up to 4 neighbors (Up, Down, Left, Right).\n\n", "body"),

            ("3. The Evaluation Function: f(n) = g(n) + h(n)\n", "h1"),
            ("Every node n in the open frontier is ranked by its total estimated path cost:\n\n", "body"),
            ("    f(n) = g(n) + h(n)\n\n", "formula"),
            ("• g(n) [Exact Past Cost]:\n  The exact cost incurred to travel from the Start node to node n along the current path. On this grid, each orthogonal step has cost 1.\n\n", "body"),
            ("• h(n) [Heuristic Future Estimate]:\n  The estimated cost to travel from node n to the Goal node. In this project, it is calculated using the Manhattan Distance.\n\n", "body"),
            ("• f(n) [Total Estimated Cost]:\n  The sum of known cost g(n) plus estimated remaining cost h(n).\n\n", "body"),

            ("4. Manhattan Distance Heuristic\n", "h1"),
            ("For a grid allowing only 4-directional movements (no diagonal jumps), the shortest theoretical distance between two points is the sum of horizontal and vertical coordinate differences:\n\n", "body"),
            ("    h(n) = |x1 - x2| + |y1 - y2|\n\n", "formula"),
            ("Why Manhattan distance?\nBecause moving orthogonally mimics navigating city blocks in Manhattan, New York.\n\n", "body"),

            ("5. Open Set vs. Closed Set\n", "h1"),
            ("• Open Set (Frontier):\n  A min-priority queue (heap) containing all nodes discovered so far that have not yet been evaluated. The node with the lowest f(n) is chosen next.\n\n", "body"),
            ("• Closed Set (Explored):\n  A set of nodes that have already been evaluated and expanded. This prevents infinite cycles and redundant calculations.\n\n", "body"),

            ("6. Why does A* guarantee the shortest path?\n", "h1"),
            ("A* is mathematically proven to be OPTIMAL (guaranteed to find the true shortest path) provided the heuristic is ADMISSIBLE.\n\n", "body"),
            ("An admissible heuristic NEVER overestimates the actual cost to reach the goal. Because the Manhattan distance assumes a straight obstacle-free path, the actual cost in a maze can only be greater than or equal to the Manhattan distance, never less!\n\n", "body"),
            ("Therefore, A* will never overlook a shorter path in favor of a longer one.\n", "body")
        ]

        for text, tag in about_content:
            text_area.insert(tk.END, text, tag)

        text_area.config(state=tk.DISABLED)

    # -------------------------------------------------------------------------
    # Canvas Grid Rendering
    # -------------------------------------------------------------------------

    def _get_cell_size(self) -> Tuple[float, float, float, float]:
        """Compute cell dimensions and offset to center the grid on Canvas."""
        canvas_width = max(100, self.canvas.winfo_width())
        canvas_height = max(100, self.canvas.winfo_height())

        cell_w = canvas_width / self.cols
        cell_h = canvas_height / self.rows
        cell_size = min(cell_w, cell_h)

        # Center grid inside canvas
        grid_width = cell_size * self.cols
        grid_height = cell_size * self.rows
        offset_x = (canvas_width - grid_width) / 2
        offset_y = (canvas_height - grid_height) / 2

        return cell_size, offset_x, offset_y, grid_width

    def _on_canvas_resize(self, event):
        """Redraw grid whenever Canvas window geometry changes."""
        self._draw_grid()

    def _draw_grid(self):
        """Full redraw of the grid canvas."""
        self.canvas.delete("all")
        cell_size, offset_x, offset_y, _ = self._get_cell_size()
        if cell_size < 3:
            return

        path_set = set(self.final_path)

        for r in range(self.rows):
            for c in range(self.cols):
                x1 = offset_x + c * cell_size
                y1 = offset_y + r * cell_size
                x2 = x1 + cell_size
                y2 = y1 + cell_size

                # Determine fill color
                coord = (r, c)
                if coord == self.maze.start:
                    color = COLOR_START
                elif coord == self.maze.goal:
                    color = COLOR_GOAL
                elif self.maze.is_wall(r, c):
                    color = COLOR_WALL
                elif coord in path_set:
                    color = COLOR_PATH
                elif coord in self.closed_set_coords:
                    color = COLOR_CLOSED
                elif coord in self.open_set_coords:
                    color = COLOR_OPEN
                else:
                    color = COLOR_UNVISITED

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline=COLOR_GRID_LINE,
                    width=1,
                    tags=(f"cell_{r}_{c}", "grid_cell")
                )

                # Add labels on Start and Goal
                if coord == self.maze.start:
                    self.canvas.create_text(
                        (x1 + x2) / 2, (y1 + y2) / 2,
                        text="S",
                        fill="#FFFFFF",
                        font=("Segoe UI", max(8, int(cell_size * 0.55)), "bold"),
                        tags="marker"
                    )
                elif coord == self.maze.goal:
                    self.canvas.create_text(
                        (x1 + x2) / 2, (y1 + y2) / 2,
                        text="G",
                        fill="#FFFFFF",
                        font=("Segoe UI", max(8, int(cell_size * 0.55)), "bold"),
                        tags="marker"
                    )

        # Draw connecting line through final path for visual clarity
        if len(self.final_path) > 1:
            line_coords = []
            for r, c in self.final_path:
                cx = offset_x + c * cell_size + cell_size / 2
                cy = offset_y + r * cell_size + cell_size / 2
                line_coords.extend([cx, cy])

            self.canvas.create_line(
                *line_coords,
                fill=COLOR_PATH_LINE,
                width=max(2, int(cell_size * 0.2)),
                capstyle=tk.ROUND,
                joinstyle=tk.ROUND,
                tags="path_line"
            )

    def _draw_cell(self, row: int, col: int, color: str, label: Optional[str] = None):
        """Update a single cell on the Canvas without redrawing everything."""
        cell_size, offset_x, offset_y, _ = self._get_cell_size()
        x1 = offset_x + col * cell_size
        y1 = offset_y + row * cell_size
        x2 = x1 + cell_size
        y2 = y1 + cell_size

        self.canvas.delete(f"cell_{row}_{col}")
        self.canvas.create_rectangle(
            x1, y1, x2, y2,
            fill=color,
            outline=COLOR_GRID_LINE,
            width=1,
            tags=(f"cell_{row}_{col}", "grid_cell")
        )

        if label:
            self.canvas.create_text(
                (x1 + x2) / 2, (y1 + y2) / 2,
                text=label,
                fill="#FFFFFF",
                font=("Segoe UI", max(8, int(cell_size * 0.55)), "bold"),
                tags=(f"label_{row}_{col}", "marker")
            )

    # -------------------------------------------------------------------------
    # Mouse Interaction Handlers
    # -------------------------------------------------------------------------

    def _coords_from_event(self, event) -> Optional[Tuple[int, int]]:
        """Convert pixel coordinates on Canvas to (row, col) in grid."""
        cell_size, offset_x, offset_y, _ = self._get_cell_size()
        if cell_size <= 0:
            return None

        col = int((event.x - offset_x) // cell_size)
        row = int((event.y - offset_y) // cell_size)

        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (row, col)
        return None

    def _on_canvas_click(self, event):
        """Handle left mouse click according to active mode."""
        if self.is_searching:
            return

        coords = self._coords_from_event(event)
        if not coords:
            return

        row, col = coords
        self._apply_mode_action(row, col)

    def _on_canvas_drag(self, event):
        """Handle continuous dragging for painting walls / erasing."""
        if self.is_searching:
            return

        coords = self._coords_from_event(event)
        if not coords:
            return

        row, col = coords
        # Dragging is only for wall drawing and erasing
        if self.mode.get() in ("wall", "erase"):
            self._apply_mode_action(row, col)

    def _on_canvas_right_click(self, event):
        """Right click quickly toggles wall without switching tools."""
        if self.is_searching:
            return

        coords = self._coords_from_event(event)
        if not coords:
            return

        row, col = coords
        if coords != self.maze.start and coords != self.maze.goal:
            self.maze.toggle_wall(row, col)
            self._draw_grid()

    def _on_canvas_right_drag(self, event):
        """Right click drag erases walls."""
        if self.is_searching:
            return

        coords = self._coords_from_event(event)
        if not coords:
            return

        row, col = coords
        if coords != self.maze.start and coords != self.maze.goal:
            if self.maze.is_wall(row, col):
                self.maze.remove_wall(row, col)
                self._draw_grid()

    def _apply_mode_action(self, row: int, col: int):
        """Apply the selected action to cell (row, col)."""
        current_mode = self.mode.get()

        if current_mode == "wall":
            if (row, col) != self.maze.start and (row, col) != self.maze.goal:
                self.maze.add_wall(row, col)
                self._draw_grid()

        elif current_mode == "erase":
            self.maze.remove_wall(row, col)
            self._draw_grid()

        elif current_mode == "start":
            if (row, col) == self.maze.goal:
                messagebox.showwarning("Invalid Position", "Start node cannot be placed at the Goal position.")
                return
            self.maze.set_start(row, col)
            self._draw_grid()

        elif current_mode == "goal":
            if (row, col) == self.maze.start:
                messagebox.showwarning("Invalid Position", "Goal node cannot be placed at the Start position.")
                return
            self.maze.set_goal(row, col)
            self._draw_grid()

    # -------------------------------------------------------------------------
    # Algorithm Animation and Control Flow
    # -------------------------------------------------------------------------

    def start_search(self):
        """Initiate A* Search execution."""
        # Validation checks
        if self.maze.start == self.maze.goal:
            messagebox.showerror("Error", "Start and Goal cannot be the same node.")
            return

        if not self.maze.is_walkable(self.maze.start[0], self.maze.start[1]):
            messagebox.showerror("Error", "Start node is placed on a wall.")
            return

        if not self.maze.is_walkable(self.maze.goal[0], self.maze.goal[1]):
            messagebox.showerror("Error", "Goal node is placed on a wall.")
            return

        # Cancel any previous active search
        self.stop_search_animation()

        # Reset visualization state while keeping walls
        self.open_set_coords.clear()
        self.closed_set_coords.clear()
        self.final_path.clear()
        self._draw_grid()

        # Update UI state
        self.is_searching = True
        self.is_paused = False
        self.btn_search.config(state=tk.DISABLED)
        self.btn_pause.config(state=tk.NORMAL, text="⏸ Pause")
        self.stat_status.set("Searching for shortest path...")
        self.lbl_status.config(fg="#F59E0B")

        # Instant mode shortcut
        if self.instant_mode.get():
            result = astar_search(self.maze, self.maze.start, self.maze.goal)
            self._handle_search_complete(result)
            return

        # Step-by-step animated generator
        self.search_generator = astar_search_generator(self.maze, self.maze.start, self.maze.goal)
        self._schedule_next_step()

    def _schedule_next_step(self):
        """Schedule next animation step using Tkinter's after loop."""
        if not self.is_searching or self.is_paused:
            return

        delay = max(1, self.animation_delay_ms.get())
        self.after_id = self.root.after(delay, self._step_animation)

    def _step_animation(self):
        """Execute one step from the A* generator."""
        if not self.is_searching or self.is_paused:
            return

        try:
            event_type, data = next(self.search_generator)

            if event_type == "step":
                # Update sets
                self.open_set_coords = data["open_set"]
                self.closed_set_coords = data["closed_set"]
                self.stat_nodes_explored.set(str(data["nodes_explored"]))

                # Draw newly expanded cell
                curr_r, curr_c = data["current"]
                if (curr_r, curr_c) != self.maze.start and (curr_r, curr_c) != self.maze.goal:
                    self._draw_cell(curr_r, curr_c, COLOR_CLOSED)

                # Draw newly added frontier cells
                for orow, ocol in self.open_set_coords:
                    if (orow, ocol) != self.maze.start and (orow, ocol) != self.maze.goal and (orow, ocol) not in self.closed_set_coords:
                        self._draw_cell(orow, ocol, COLOR_OPEN)

                self._schedule_next_step()

            elif event_type in ("complete", "error"):
                self._handle_search_complete(data)

        except StopIteration:
            self.stop_search_animation()

    def _handle_search_complete(self, result: SearchResult):
        """Process and display final search results."""
        self.is_searching = False
        self.btn_search.config(state=tk.NORMAL)
        self.btn_pause.config(state=tk.DISABLED)

        # Update statistics panel
        self.stat_nodes_explored.set(str(result.nodes_explored))
        self.stat_time.set(f"{result.execution_time_ms:.2f} ms")

        if result.path_found:
            self.final_path = result.path
            self.stat_path_found.set("Path Found")
            self.stat_path_length.set(str(result.path_length))
            self.stat_nodes_in_path.set(str(result.nodes_in_path))
            self.stat_status.set(f"Shortest path found! ({result.path_length} steps)")
            self.lbl_status.config(fg="#34D399")
        else:
            self.final_path = []
            self.stat_path_found.set("No Path")
            self.stat_path_length.set("0")
            self.stat_nodes_in_path.set("0")
            self.stat_status.set("No path exists between the Start and Goal.")
            self.lbl_status.config(fg="#EF4444")

        # Full redraw to show final highlighted path
        self._draw_grid()

    def toggle_pause(self):
        """Toggle pause/resume during search animation."""
        if not self.is_searching:
            return

        self.is_paused = not self.is_paused
        if self.is_paused:
            self.btn_pause.config(text="▶ Resume")
            self.stat_status.set("Search paused")
        else:
            self.btn_pause.config(text="⏸ Pause")
            self.stat_status.set("Searching...")
            self._schedule_next_step()

    def stop_search_animation(self):
        """Safely terminate any pending animation callbacks."""
        if self.after_id:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None

        self.is_searching = False
        self.is_paused = False
        self.search_generator = None
        self.btn_search.config(state=tk.NORMAL)
        self.btn_pause.config(state=tk.DISABLED, text="⏸ Pause")

    def reset_search(self):
        """Reset search visualization while keeping walls intact."""
        self.stop_search_animation()
        self.open_set_coords.clear()
        self.closed_set_coords.clear()
        self.final_path.clear()

        self.stat_status.set("Ready to search")
        self.lbl_status.config(fg="#38BDF8")
        self.stat_path_found.set("--")
        self.stat_path_length.set("0")
        self.stat_nodes_explored.set("0")
        self.stat_nodes_in_path.set("0")
        self.stat_time.set("0.0 ms")

        self._draw_grid()

    def clear_maze(self):
        """Clear all walls and reset search visualization."""
        self.stop_search_animation()
        self.maze.clear_walls()
        self.reset_search()

    def generate_random_maze(self):
        """Generate random walls while keeping Start and Goal clear."""
        self.stop_search_animation()
        self.maze.generate_random_maze(wall_density=0.28)
        self.reset_search()

    def load_default_maze(self):
        """Load the starter example maze."""
        self.stop_search_animation()
        self.maze.load_default_maze()
        self.reset_search()
