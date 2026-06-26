# trajectory_plotter.py
import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib
matplotlib.use('TkAgg')  # Ensure TkAgg backend is used
import numpy as np
import math

# Set matplotlib to support Chinese
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

class TrajectoryPlotter:
    def __init__(self, parent_frame, trajectory_tracker=None):
        """
        Initialize trajectory plotter
        
        Args:
            parent_frame: parent frame
            trajectory_tracker: trajectory tracker instance
        """
        self.parent = parent_frame
        self.trajectory_tracker = trajectory_tracker
        
        # Trajectory data management
        self.x_data = []
        self.y_data = []
        
        # Create Matplotlib figure
        self.figure = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.figure.add_subplot(111)
        
        # Set axes
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Car Trajectory')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.axis('equal')
        
        # Create initial trajectory line
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, antialiased=True, label='Trajectory')
        self.current_pose_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Current Position')
        self.orientation_line, = self.ax.plot([], [], 'g-', linewidth=2, antialiased=True, label='Heading')
        
        # Add ideal trajectory line
        self.ideal_trajectory_line, = self.ax.plot([], [], 'r--', linewidth=1.5, 
                                                  antialiased=True, alpha=0.7, 
                                                  label='Ideal Path')
        
        # Add legend
        self.ax.legend()
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.figure, self.parent)
        self.canvas.draw()
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)
        
        # Performance optimization: reduce redraws
        self.last_update_time = 0
        self.update_interval = 0.2  # Update at most once every 200ms
        
        # Auto/manual refresh control
        self.auto_refresh_enabled = False
        self.auto_refresh_interval_ms = 500  # Auto refresh interval, default 500ms
        self._after_id = None
        
        # Color management
        self.current_color = "blue"
        self.color_changed = False
        
        # Manual zoom control
        self.manual_range = 2.0
        self.use_auto_scale = True
        
        # Ideal trajectory data
        self.ideal_trajectory_x = []
        self.ideal_trajectory_y = []
        
        # Control button frame
        control_frame = ttk.Frame(self.parent)
        control_frame.pack(fill="x", padx=5, pady=5)
        
        # Control buttons
        self.btn_clear = ttk.Button(control_frame, text="Clear trajectory", command=self.clear_trajectory)
        self.btn_clear.pack(side="left", padx=5)
        
        self.btn_save = ttk.Button(control_frame, text="Save trajectory", command=self.save_trajectory)
        self.btn_save.pack(side="left", padx=5)
        
        self.btn_reset_view = ttk.Button(control_frame, text="Reset view", command=self.reset_view)
        self.btn_reset_view.pack(side="left", padx=5)
        
        self.btn_export = ttk.Button(control_frame, text="Export image", command=self.export_image)
        self.btn_export.pack(side="left", padx=5)
        
        # Manual refresh button
        self.btn_manual_refresh = ttk.Button(control_frame, text="Manual refresh", command=self.update_plot)
        self.btn_manual_refresh.pack(side="left", padx=5)
        
        # Auto refresh toggle
        self.auto_refresh_var = tk.BooleanVar(value=False)
        self.auto_refresh_check = ttk.Checkbutton(control_frame, text="Auto refresh", variable=self.auto_refresh_var,
                                                 command=self._on_auto_refresh_toggled)
        self.auto_refresh_check.pack(side="left", padx=5)
        
        # Status label
        self.status_label = ttk.Label(control_frame, text="Trajectory points: 0 | Length: 0.00 m")
        self.status_label.pack(side="right", padx=5)
        
        # Trajectory display settings
        settings_frame = ttk.Frame(self.parent)
        settings_frame.pack(fill="x", padx=5, pady=5)
        
        # Trajectory color selection
        ttk.Label(settings_frame, text="Trajectory color:").pack(side="left", padx=5)
        self.color_var = tk.StringVar(value="blue")
        colors = [("Blue", "blue"), ("Red", "red"), ("Green", "green"), 
                 ("Black", "black"), ("Purple", "purple")]
        for text, color in colors:
            ttk.Radiobutton(settings_frame, text=text, variable=self.color_var, 
                          value=color, command=self.on_color_changed).pack(side="left", padx=2)
        
        # Auto scale option
        self.auto_scale_var = tk.BooleanVar(value=True)
        self.auto_scale_check = ttk.Checkbutton(settings_frame, text="Auto scale", 
                                               variable=self.auto_scale_var,
                                               command=self.on_auto_scale_changed)
        self.auto_scale_check.pack(side="right", padx=5)
        
        # Manual zoom control
        ttk.Label(settings_frame, text="Display range(m):").pack(side="right", padx=5)
        self.range_entry = ttk.Entry(settings_frame, width=8)
        self.range_entry.insert(0, "2.0")
        self.range_entry.pack(side="right", padx=5)
        
        ttk.Button(settings_frame, text="Apply range", command=self.apply_manual_range).pack(side="right", padx=5)
        
        # Initialize view
        self.reset_view()
    
    def set_ideal_trajectory(self, path_points):
        """
        Set ideal trajectory points
        Args:
            path_points: ideal trajectory point list [Point(x, y), ...]
        """
        if not path_points:
            return
        
        # Extract coordinates
        self.ideal_trajectory_x = [p.x for p in path_points]
        self.ideal_trajectory_y = [p.y for p in path_points]
        
        # Update ideal trajectory line
        self.ideal_trajectory_line.set_data(self.ideal_trajectory_x, self.ideal_trajectory_y)
        
        # Redraw
        self.canvas.draw()
    
    def clear_ideal_trajectory(self):
        """Clear ideal trajectory"""
        self.ideal_trajectory_x = []
        self.ideal_trajectory_y = []
        self.ideal_trajectory_line.set_data([], [])
        self.canvas.draw()
    
    def on_color_changed(self):
        """Callback when color changes"""
        new_color = self.color_var.get()
        self.current_color = new_color
        self.color_changed = True
        self.update_trajectory_style()
    
    def on_auto_scale_changed(self):
        """Auto scale option changed"""
        self.use_auto_scale = self.auto_scale_var.get()
        if not self.use_auto_scale:
            self.apply_manual_range()
    
    def update_trajectory_style(self):
        """Update trajectory display style"""
        if self.color_changed:
            # Only change newly drawn trajectory
            self.trajectory_line.set_color(self.current_color)
            self.color_changed = False
            self.canvas.draw()
    
    def apply_manual_range(self):
        """Apply manually set range"""
        try:
            range_val = float(self.range_entry.get())
            if range_val > 0:
                self.manual_range = range_val
                half_range = range_val / 2
                self.ax.set_xlim(-half_range, half_range)
                self.ax.set_ylim(-half_range, half_range)
                self.canvas.draw()
        except ValueError:
            pass
    
    def clear_trajectory(self):
        """Clear trajectory (only clear lines, keep current position)"""
        # Clear plot data
        self.trajectory_line.set_data([], [])
        
        # Clear current point data - keep current position display
        if self.trajectory_tracker:
            current_pose = self.trajectory_tracker.get_current_pose()
            # Reset current point to ensure only one point
            self.current_pose_marker.set_data([current_pose.x], [current_pose.y])
        
        # Clear orientation line data
        self.orientation_line.set_data([], [])
        
        # Note: do not clear ideal trajectory for comparison
        
        # Do not reset view range, keep current zoom and position
        # Do not call self.reset_view()
        
        # Redraw
        self.canvas.draw()
    
    def update_plot(self):
        """Update trajectory plot (fixed line connection issue)"""
        import time
        current_time = time.time()
        
        # Limit update frequency
        if current_time - self.last_update_time < self.update_interval:
            return
        
        if not self.trajectory_tracker or not self.trajectory_tracker.trajectory:
            return
        
        # Get trajectory data
        trajectory = self.trajectory_tracker.get_display_trajectory()
        current_pose = self.trajectory_tracker.get_current_pose()
        
        if len(trajectory) < 1:
            return
        
        # Extract latest trajectory points
        new_x_vals = [p.x for p in trajectory]
        new_y_vals = [p.y for p in trajectory]
        
        # Update actual trajectory data
        self.x_data = new_x_vals
        self.y_data = new_y_vals
        
        # Update actual trajectory line
        self.trajectory_line.set_data(self.x_data, self.y_data)
        
        # Key fix: ensure current point contains only one point, not connected to origin or other points
        # First clear current point data
        self.current_pose_marker.set_data([], [])
        # Then reset current point position (single point)
        self.current_pose_marker.set_data([current_pose.x], [current_pose.y])
        
        # Update orientation indicator line
        theta_rad = np.radians(current_pose.theta)
        arrow_length = 0.1
        
        dx = arrow_length * np.cos(theta_rad)
        dy = arrow_length * np.sin(theta_rad)
        self.orientation_line.set_data(
            [current_pose.x, current_pose.x + dx],
            [current_pose.y, current_pose.y + dy]
        )
        
        # Auto scale (consider both actual and ideal trajectories)
        if self.use_auto_scale:
            # Get all points to display
            all_x = self.x_data.copy()
            all_y = self.y_data.copy()
            
            if self.ideal_trajectory_x and self.ideal_trajectory_y:
                all_x.extend(self.ideal_trajectory_x)
                all_y.extend(self.ideal_trajectory_y)
            
            if len(all_x) > 1 and len(all_y) > 1:
                x_min, x_max = min(all_x), max(all_x)
                y_min, y_max = min(all_y), max(all_y)
                
                x_range = x_max - x_min
                y_range = y_max - y_min
                margin = max(x_range, y_range) * 0.1
                margin = max(margin, 0.1)
                
                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)
        
        # Update status label
        total_points = len(self.trajectory_tracker.trajectory)
        trajectory_length = self.trajectory_tracker.get_trajectory_length()
        vx, vy, vtheta = self.trajectory_tracker.get_current_velocity()
        linear_speed = (vx**2 + vy**2)**0.5
        
        # Add ideal trajectory information
        ideal_points_count = len(self.ideal_trajectory_x)
        status_text = f"Points: {total_points} | Length: {trajectory_length:.2f}m | Speed: {linear_speed:.2f}m/s"
        if ideal_points_count > 0:
            status_text += f" | Ideal trajectory points: {ideal_points_count}"
        
        self.status_label.config(text=status_text)
        
        # Redraw
        self.canvas.draw()
        self.last_update_time = current_time
    
    def enable_auto_refresh(self, interval_ms=None):
        """Enable auto refresh (based on Tk after scheduling)"""
        if interval_ms is not None:
            self.auto_refresh_interval_ms = int(interval_ms)
        if self.auto_refresh_enabled:
            return
        self.auto_refresh_enabled = True
        # Trigger an update immediately, then start loop
        self._schedule_next()
    
    def disable_auto_refresh(self):
        """Disable auto refresh and cancel scheduled tasks"""
        self.auto_refresh_enabled = False
        try:
            if self._after_id and hasattr(self.parent, 'after_cancel'):
                self.parent.after_cancel(self._after_id)
        except Exception:
            pass
        self._after_id = None
    
    def _auto_refresh_loop(self):
        """Internal loop: execute one update and schedule next"""
        try:
            self.update_plot()
        except Exception:
            pass
        self._schedule_next()
    
    def _schedule_next(self):
        """Schedule next auto refresh"""
        if not self.auto_refresh_enabled:
            return
        try:
            # Use parent.after for scheduling (parent is Frame, has after method)
            self._after_id = self.parent.after(self.auto_refresh_interval_ms, self._auto_refresh_loop)
        except Exception:
            # Fallback to using canvas widget after
            try:
                self._after_id = self.canvas_widget.after(self.auto_refresh_interval_ms, self._auto_refresh_loop)
            except Exception:
                self._after_id = None
    
    def _on_auto_refresh_toggled(self):
        """Auto refresh checkbox callback"""
        if self.auto_refresh_var.get():
            # Enable auto refresh
            self.enable_auto_refresh()
        else:
            self.disable_auto_refresh()
    
    def save_trajectory(self):
        """Save trajectory data to file"""
        if self.trajectory_tracker:
            self.trajectory_tracker.save_trajectory_to_file()
    
    def reset_view(self):
        """Reset view"""
        self.ax.set_xlim(-self.manual_range/2, self.manual_range/2)
        self.ax.set_ylim(-self.manual_range/2, self.manual_range/2)
        self.canvas.draw()
    
    def export_image(self):
        """Export trajectory plot as image"""
        if not self.trajectory_tracker or not self.trajectory_tracker.trajectory:
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG image", "*.png"), ("JPEG image", "*.jpg"), ("PDF file", "*.pdf"), ("All files", "*.*")]
        )
        
        if filename:
            self.figure.savefig(filename, dpi=300, bbox_inches='tight')
    
    def cleanup(self):
        """Clean up resources, cancel auto refresh timer"""
        self.disable_auto_refresh()