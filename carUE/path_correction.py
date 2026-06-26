# path_correction.py
import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import time
import random

@dataclass
class Point:
    """2D point"""
    x: float  # meters
    y: float  # meters
    
    def distance_to(self, other) -> float:
        """Calculate distance to another point"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __sub__(self, other):
        """Vector subtraction"""
        return Point(self.x - other.x, self.y - other.y)
    
    def __add__(self, other):
        """Vector addition"""
        return Point(self.x + other.x, self.y + other.y)
    
    def __mul__(self, scalar):
        """Scalar multiplication"""
        return Point(self.x * scalar, self.y * scalar)
    
    def __truediv__(self, scalar):
        """Scalar division"""
        return Point(self.x / scalar, self.y / scalar)
    
    def __str__(self):
        return f"({self.x:.3f}, {self.y:.3f})"

@dataclass
class Pose:
    """Pose representation"""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    theta: float = 0.0  # degrees

@dataclass
class PathSegment:
    """Path segment"""
    start: Point
    end: Point
    length: float
    
    @staticmethod
    def from_points(start: Point, end: Point) -> 'PathSegment':
        """Create path segment from two points"""
        length = start.distance_to(end)
        return PathSegment(start, end, length)
    
    def __str__(self):
        return f"[{self.start} -> {self.end}] Length: {self.length:.3f}m"

class PathCorrectionController:
    """
    Path correction controller
    Automatically adjusts motion control to keep the car close to the ideal trajectory when deviation is detected
    """
    
    def __init__(self, trajectory_tracker=None):
        """
        Initialize path correction controller
        
        Args:
            trajectory_tracker: Trajectory tracker instance
        """
        self.trajectory_tracker = trajectory_tracker
        
        # Ideal path (target path)
        self.target_path: List[Point] = []
        self.target_path_segments: List[PathSegment] = []
        
        # Current target point index
        self.current_target_idx = 0
        
        # Correction parameters
        self.correction_enabled = False
        self.lookahead_distance = 0.5  # Lookahead distance (meters)
        self.max_lateral_error = 0.05  # Maximum lateral error (meters)
        self.max_angular_error = 5.0   # Maximum angular error (degrees)
        
        # PID controller parameters (lateral error control)
        self.kp_lateral = 1.5    # Proportional coefficient (reduced)
        self.ki_lateral = 0.02   # Integral coefficient (reduced)
        self.kd_lateral = 0.3    # Derivative coefficient (reduced)
        
        # PID controller parameters (angular error control)
        self.kp_angular = 0.5    # Proportional coefficient (reduced)
        self.ki_angular = 0.005  # Integral coefficient (reduced)
        self.kd_angular = 0.05   # Derivative coefficient (reduced)
        
        # New: correction strength parameters
        self.correction_strength = 0.7  # Correction strength [0.1, 1.0]
        self.standalone_base_speed = 100  # Base speed for standalone correction mode (mm/s)
        
        # Error history
        self.lateral_error_history = []
        self.angular_error_history = []
        self.max_history_size = 100
        
        # Integral terms
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        
        # Last errors
        self.last_lateral_error = 0.0
        self.last_angular_error = 0.0
        
        # Last update time
        self.last_update_time = time.time()
        
        # Path completion conditions
        self.target_arrival_threshold = 0.05  # Arrival threshold for target point (meters)
        self.path_completion_angle_threshold = 10.0  # Final angle error threshold (degrees)
        
        # Status flags
        self.path_completed = False
        self.path_started = False
        
        # New: correction delay parameters
        self.correction_delay = 0.0  # Correction delay (seconds), 0 means no delay
        self.correction_start_time = 0  # Planned correction start time
        self.correction_delayed = False  # Whether in delay waiting state
        
        # New: correction execution state
        self.correction_executing = False  # Whether correction is executing
        self.max_correction_time = 60.0  # Maximum correction time (seconds)
        self.correction_execution_start_time = 0  # Actual correction execution start time
        self.queue_finished = False  # Whether queue has finished execution
        
        # Target position
        self.target_position = None
        
        # Path information log
        self.path_log = []
        self.max_log_entries = 200
        
        # New: trajectory generation parameters
        self.point_density_mode = "auto"  # "auto", "fixed", "manual", "adaptive"
        self.points_per_command = 5  # Number of points generated per command (for fixed mode)
        self.max_points_total = 1000  # Maximum total points
        self.min_distance_between_points = 0.01  # Minimum distance between points (meters, for manual mode)
        
        # Stop reason record
        self.stop_reason = "Not started"
        self.stop_details = ""
        
        # Debug information
        self.debug_info = {
            'current_error': 0.0,
            'lateral_error': 0.0,
            'angular_error': 0.0,
            'target_point': None,
            'nearest_point': None,
            'lookahead_point': None,
            'correction_applied': False,
            'correction_delayed': False,
            'time_until_start': 0.0,
            'correction_executing': False,
            'queue_finished': False,
            'path_points': [],
            'path_length': 0.0,
            'current_progress': 0.0,
            'point_density_mode': self.point_density_mode,
            'stop_reason': self.stop_reason
        }
    
    def body_to_world(self, vx_body: float, vy_body: float, theta_deg: float) -> Tuple[float, float]:
        """
        Car frame velocity -> World frame velocity
        
        Args:
            vx_body: Forward velocity (mm/s)
            vy_body: Leftward velocity (mm/s)
            theta_deg: Heading angle (degrees), counter-clockwise is positive
            
        Returns:
            (vx_world, vy_world): World frame velocity (mm/s)
        """
        theta_rad = math.radians(theta_deg)
        
        vx_world = vx_body * math.cos(theta_rad) - vy_body * math.sin(theta_rad)
        vy_world = vx_body * math.sin(theta_rad) + vy_body * math.cos(theta_rad)
        
        return vx_world, vy_world
    
    def world_to_body(self, vx_world: float, vy_world: float, theta_deg: float) -> Tuple[float, float]:
        """
        World frame velocity -> Car frame velocity
        
        Args:
            vx_world: World X direction velocity (mm/s)
            vy_world: World Y direction velocity (mm/s)
            theta_deg: Heading angle (degrees), counter-clockwise is positive
            
        Returns:
            (vx_body, vy_body): Car frame velocity (mm/s)
        """
        theta_rad = math.radians(theta_deg)
        
        vx_body = vx_world * math.cos(theta_rad) + vy_world * math.sin(theta_rad)
        vy_body = -vx_world * math.sin(theta_rad) + vy_world * math.cos(theta_rad)
        
        return vx_body, vy_body
    
    def set_target_path(self, path_points: List[Point]):
        """
        Set target path
        
        Args:
            path_points: Target path point list (world frame)
        """
        if len(path_points) < 2:
            raise ValueError("Path requires at least 2 points")
        
        self.target_path = path_points.copy()
        self.target_path_segments = []
        
        # Create path segments
        for i in range(len(path_points) - 1):
            segment = PathSegment.from_points(path_points[i], path_points[i+1])
            self.target_path_segments.append(segment)
        
        # Calculate total path length
        total_length = sum(segment.length for segment in self.target_path_segments)
        
        # Reset state
        self.current_target_idx = 0
        self.path_completed = False
        self.path_started = False
        self.queue_finished = False
        self.correction_delayed = False
        self.correction_executing = False
        self.stop_reason = "Not started"
        self.stop_details = ""
        
        # Reset PID controller
        self.integral_lateral = 0.0
        self.integral_angular = 0.0
        self.last_lateral_error = 0.0
        self.last_angular_error = 0.0
        self.lateral_error_history = []
        self.angular_error_history = []
        
        # Set target position
        self.target_position = self.target_path[-1] if self.target_path else None
        
        # Log path information to log
        self.path_log = []
        self.log_path_info(f"=== Set target path ===")
        self.log_path_info(f"Path points: {len(path_points)}")
        self.log_path_info(f"Total path length: {total_length:.3f}m")
        self.log_path_info(f"Density mode: {self.point_density_mode}")
        self.log_path_info(f"Path point coordinates:")
        for i, point in enumerate(path_points):
            self.log_path_info(f"  Point {i}: {point}")
        self.log_path_info(f"Path segments:")
        for i, segment in enumerate(self.target_path_segments):
            self.log_path_info(f"  Segment {i}: {segment}")
        self.log_path_info(f"Target position: {self.target_position}")
        self.log_path_info("")
        
        # Update debug information
        self.debug_info['path_points'] = [(p.x, p.y) for p in path_points]
        self.debug_info['path_length'] = total_length
        self.debug_info['current_progress'] = 0.0
        self.debug_info['point_density_mode'] = self.point_density_mode
        self.debug_info['stop_reason'] = self.stop_reason
    
    def set_ideal_trajectory_from_queue(self, speed_queue, density_mode="auto", points_per_command=None):
        """
        Generate ideal trajectory from speed queue (using correct velocity transformation)
        
        Args:
            speed_queue: Speed command queue
            density_mode: Density mode ("auto", "fixed", "manual", "adaptive")
            points_per_command: Number of points generated per command (only for fixed mode)
        """
        if not speed_queue:
            return
        
        self.point_density_mode = density_mode
        
        if density_mode == "fixed" and points_per_command:
            self.points_per_command = max(1, int(points_per_command))
        
        # Generate trajectory based on density mode
        ideal_points = self._generate_path_simple(speed_queue, density_mode)
        
        # Set target path
        self.set_target_path(ideal_points)
        
        # Record detailed information
        total_duration = sum(cmd['duration'] for cmd in speed_queue)
        self.log_path_info(f"Density mode: {density_mode}")
        self.log_path_info(f"Generated points: {len(ideal_points)}")
        self.log_path_info(f"Total duration: {total_duration:.1f}s")
        
        # Calculate path statistics
        if len(ideal_points) >= 2:
            total_length = 0
            distances = []
            for i in range(len(ideal_points)-1):
                dist = ideal_points[i].distance_to(ideal_points[i+1])
                total_length += dist
                distances.append(dist)
            
            self.log_path_info(f"Total path length: {total_length:.3f}m")
            self.log_path_info(f"Average point spacing: {total_length/(len(ideal_points)-1):.3f}m")
            self.log_path_info(f"Minimum spacing: {min(distances):.3f}m")
            self.log_path_info(f"Maximum spacing: {max(distances):.3f}m")
        
        self.log_path_info("")
    
    def _generate_path_simple(self, speed_queue, density_mode=None):
        """Generate trajectory points based on density mode"""
        if density_mode is None:
            density_mode = self.point_density_mode
        
        points = []
        
        # Initial state
        x, y, theta = 0.0, 0.0, 0.0  # Position (m), Angle (degrees)
        points.append(Point(x, y))
        
        # Set time step and sampling interval based on density mode
        if density_mode == "auto":
            base_dt = 0.02  # 20ms time step
            sample_interval = 5  # Sample every 5 steps
            min_distance = 0.01  # 1cm minimum interval
        elif density_mode == "fixed":
            base_dt = 0.05  # 50ms time step
            sample_interval = max(1, self.points_per_command)  # Use user-set point count
            min_distance = 0.0  # No minimum distance limit
        elif density_mode == "manual":
            base_dt = 0.01  # 10ms time step, finer
            sample_interval = 1  # Consider every step
            min_distance = self.min_distance_between_points  # User-set minimum distance
        elif density_mode == "adaptive":
            base_dt = 0.02  # 20ms time step
            sample_interval = 2  # Sample every 2 steps
            min_distance = 0.02  # 2cm minimum interval, adaptive based on speed
        else:
            base_dt = 0.02
            sample_interval = 5
            min_distance = 0.01
        
        self.log_path_info(f"Density mode: {density_mode}, Time step: {base_dt*1000}ms, Min distance: {min_distance}m")
        
        for command_idx, command in enumerate(speed_queue):
            duration = command['duration']
            vx_body = command['x']  # mm/s
            vy_body = command['y']  # mm/s
            omega = command['angular'] / 1000.0  # rad/s
            
            # Adaptive mode: adjust sampling interval based on speed
            if density_mode == "adaptive":
                # Calculate total speed
                total_speed = math.sqrt(vx_body**2 + vy_body**2)
                if total_speed > 500:  # Reduce sampling at high speed
                    sample_interval = 5
                    min_distance = 0.03
                elif total_speed > 200:  # Medium speed
                    sample_interval = 3
                    min_distance = 0.02
                else:  # Low speed
                    sample_interval = 2
                    min_distance = 0.01
            
            # Calculate total steps
            num_steps = max(1, int(duration / base_dt))
            
            for step in range(num_steps):
                # Current angle (radians)
                theta_rad = math.radians(theta)
                
                # Car frame velocity -> World velocity
                vx_world = vx_body * math.cos(theta_rad) - vy_body * math.sin(theta_rad)
                vy_world = vx_body * math.sin(theta_rad) + vy_body * math.cos(theta_rad)
                
                # Euler integration update position
                x += (vx_world * base_dt) / 1000.0  # mm -> m
                y += (vy_world * base_dt) / 1000.0  # mm -> m
                theta += math.degrees(omega * base_dt)
                
                # Normalize angle
                theta = ((theta + 180) % 360) - 180
                
                # Record points based on density mode
                record_point = False
                
                if density_mode == "fixed":
                    # fixed mode: generate fixed number of points per command
                    step_per_point = max(1, num_steps // self.points_per_command)
                    record_point = (step % step_per_point == 0) or (step == num_steps - 1)
                else:
                    # Other modes: based on interval and distance
                    record_point = (step % sample_interval == 0) or (step == num_steps - 1)
                
                if record_point:
                    point = Point(x, y)
                    # Check minimum distance (strict for manual mode, relaxed for others)
                    if density_mode == "manual":
                        if point.distance_to(points[-1]) >= min_distance:
                            points.append(point)
                    elif density_mode == "adaptive":
                        # adaptive mode: adjust minimum distance based on speed
                        if point.distance_to(points[-1]) >= min_distance:
                            points.append(point)
                    else:
                        # auto mode: has minimum distance limit
                        if point.distance_to(points[-1]) >= min_distance:
                            points.append(point)
                        elif step == num_steps - 1:  # Always record command end point
                            points.append(point)
        
        return points
    
    def log_path_info(self, message: str):
        """Log path information to log"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[{timestamp}] {message}"
        self.path_log.append(log_entry)
        
        # Limit log size
        if len(self.path_log) > self.max_log_entries:
            self.path_log = self.path_log[-self.max_log_entries:]
    
    def get_path_log(self) -> List[str]:
        """Get path log"""
        return self.path_log.copy()
    
    def set_correction_delay(self, delay_seconds: float):
        """
        Set correction delay
        
        Args:
            delay_seconds: Delay time (seconds), 0 means no delay
        """
        self.correction_delay = max(0.0, delay_seconds)
        self.log_path_info(f"Set correction delay: {delay_seconds:.1f}s")
    
    def set_point_density_mode(self, mode: str, **kwargs):
        """
        Set trajectory point density mode
        
        Args:
            mode: Density mode ("auto", "fixed", "manual", "adaptive")
            **kwargs: Additional parameters
                - points_per_command: Points per command (for fixed mode)
                - min_distance: Minimum point spacing (for manual mode)
        """
        if mode not in ["auto", "fixed", "manual", "adaptive"]:
            raise ValueError(f"Invalid density mode: {mode}")
        
        self.point_density_mode = mode
        
        if mode == "fixed" and "points_per_command" in kwargs:
            self.points_per_command = max(1, int(kwargs["points_per_command"]))
        elif mode == "manual" and "min_distance" in kwargs:
            self.min_distance_between_points = max(0.001, float(kwargs["min_distance"]))
        
        self.log_path_info(f"Set density mode: {mode}")
        self.debug_info['point_density_mode'] = mode
    
    def get_path_statistics(self):
        """Get path statistics"""
        if not self.target_path:
            return {
                'point_count': 0,
                'total_length': 0.0,
                'avg_spacing': 0.0,
                'density_mode': self.point_density_mode,
                'has_y_variation': self._has_y_variation()
            }
        
        path_points = self.target_path
        num_points = len(path_points)
        
        # Calculate total path length and average spacing
        total_length = 0
        if num_points >= 2:
            for i in range(num_points - 1):
                total_length += path_points[i].distance_to(path_points[i+1])
            avg_spacing = total_length / (num_points - 1)
        else:
            total_length = 0
            avg_spacing = 0
        
        return {
            'point_count': num_points,
            'total_length': total_length,
            'avg_spacing': avg_spacing,
            'density_mode': self.point_density_mode,
            'has_y_variation': self._has_y_variation()
        }
    
    def _has_y_variation(self):
        """Check if path has Y direction variation"""
        if len(self.target_path) < 2:
            return False
        
        y_values = [p.y for p in self.target_path]
        y_range = max(y_values) - min(y_values)
        
        return y_range > 0.001
    
    def enable_correction_with_delay(self, delay_seconds: float = None):
        """
        Enable path correction (with delay)
        
        Args:
            delay_seconds: Delay time (seconds), if None use current setting
        """
        if not self.target_path:
            raise ValueError("Please set target path first")
        
        if delay_seconds is not None:
            self.set_correction_delay(delay_seconds)
        
        current_time = time.time()
        
        if self.correction_delay > 0:
            # Set delayed start
            self.correction_start_time = current_time + self.correction_delay
            self.correction_delayed = True
            self.correction_enabled = False  # Do not enable correction during delay
            self.correction_executing = False
            
            self.log_path_info(f"Correction delay start: will begin in {self.correction_delay:.1f} seconds")
            self.log_path_info(f"Current time: {time.strftime('%H:%M:%S', time.localtime(current_time))}")
            self.log_path_info(f"Planned start: {time.strftime('%H:%M:%S', time.localtime(self.correction_start_time))}")
        else:
            # Start immediately
            self.correction_start_time = current_time
            self.correction_delayed = False
            self.correction_enabled = True
            self.correction_executing = True
            self.correction_execution_start_time = current_time
            self.queue_finished = False
            
            self.log_path_info("Correction started immediately")
            self.log_path_info(f"Start time: {time.strftime('%H:%M:%S', time.localtime(current_time))}")
        
        self.path_started = True
        self.path_completed = False
        self.stop_reason = "Running"
        self.stop_details = ""
        self.last_update_time = current_time
        
        # Record correction parameters
        self.log_path_info(f"Correction parameters:")
        self.log_path_info(f"  Lookahead distance: {self.lookahead_distance:.2f}m")
        self.log_path_info(f"  Lateral error threshold: {self.max_lateral_error:.3f}m")
        self.log_path_info(f"  Angular error threshold: {self.max_angular_error:.1f}°")
        self.log_path_info(f"  Arrival threshold: {self.target_arrival_threshold:.3f}m")
        self.log_path_info(f"  Max correction time: {self.max_correction_time:.1f}s")
        self.log_path_info(f"  Density mode: {self.point_density_mode}")
        self.log_path_info(f"  Correction strength: {self.correction_strength:.1f}")
        self.log_path_info("")
        
        # Update debug information
        self.debug_info['correction_delayed'] = self.correction_delayed
        self.debug_info['time_until_start'] = self.correction_delay if self.correction_delayed else 0.0
        self.debug_info['correction_executing'] = self.correction_executing
        self.debug_info['queue_finished'] = self.queue_finished
        self.debug_info['stop_reason'] = self.stop_reason
    
    def enable_correction(self):
        """Enable path correction immediately (no delay)"""
        self.enable_correction_with_delay(0.0)
    
    def mark_queue_finished(self):
        """Mark queue execution finished, but correction continues"""
        self.queue_finished = True
        self.log_path_info("=== Queue execution finished ===")
        self.log_path_info("Queue commands sent complete, start standalone correction mode")
        self.log_path_info("Correction will continue until reaching endpoint or timeout")
        self.debug_info['queue_finished'] = True
    
    def check_and_start_correction(self):
        """
        Check and start correction (if delay has ended)
        
        Returns:
            bool: Whether correction has started
        """
        if not self.correction_delayed:
            return self.correction_enabled
        
        current_time = time.time()
        
        if current_time >= self.correction_start_time:
            # Delay ended, start correction
            self.correction_delayed = False
            self.correction_enabled = True
            self.correction_executing = True
            self.correction_execution_start_time = current_time
            self.stop_reason = "Running"
            
            # Update debug information
            self.debug_info['correction_delayed'] = False
            self.debug_info['time_until_start'] = 0.0
            self.debug_info['correction_executing'] = True
            self.debug_info['stop_reason'] = self.stop_reason
            
            self.log_path_info("=== Correction delay ended ===")
            self.log_path_info(f"Start executing correction, time: {time.strftime('%H:%M:%S', time.localtime(current_time))}")
            self.log_path_info(f"Planned execution time: {self.max_correction_time:.1f}s")
            self.log_path_info("")
            
            return True
        else:
            # Still in delay
            remaining_time = self.correction_start_time - current_time
            self.debug_info['time_until_start'] = max(0.0, remaining_time)
            
            # Record delay status every 5 seconds
            if int(remaining_time) % 5 == 0 and int(remaining_time) != int(self.debug_info.get('last_remaining_time', -1)):
                self.log_path_info(f"Correction delay, remaining: {remaining_time:.1f}s")
                self.debug_info['last_remaining_time'] = remaining_time
            
            return False
    
    def disable_correction(self):
        """Disable path correction"""
        self.correction_enabled = False
        self.correction_executing = False
        self.correction_delayed = False
        self.stop_reason = "Manually disabled"
        
        # Update debug information
        self.debug_info['correction_delayed'] = False
        self.debug_info['time_until_start'] = 0.0
        self.debug_info['correction_executing'] = False
        self.debug_info['stop_reason'] = self.stop_reason
        
        self.log_path_info("=== Correction disabled ===")
    
    def find_nearest_point_on_path(self, current_pos: Point) -> Tuple[Point, int, float]:
        """
        Find nearest point on path to current position
        
        Args:
            current_pos: Current position
            
        Returns:
            tuple: (nearest point, segment index of nearest point, ratio on segment [0-1])
        """
        if not self.target_path_segments:
            return current_pos, 0, 0.0
        
        min_distance = float('inf')
        nearest_point = None
        segment_idx = 0
        segment_ratio = 0.0
        
        # Iterate all path segments
        for i, segment in enumerate(self.target_path_segments):
            # Calculate point projection on line segment
            ap = Point(current_pos.x - segment.start.x, current_pos.y - segment.start.y)
            ab = Point(segment.end.x - segment.start.x, segment.end.y - segment.start.y)
            
            # Calculate projection length ratio
            ab_squared = ab.x**2 + ab.y**2
            if ab_squared == 0:
                continue
            
            ap_dot_ab = ap.x * ab.x + ap.y * ab.y
            ratio = max(0.0, min(1.0, ap_dot_ab / ab_squared))
            
            # Calculate projection point
            projection = Point(
                segment.start.x + ab.x * ratio,
                segment.start.y + ab.y * ratio
            )
            
            # Calculate distance
            distance = current_pos.distance_to(projection)
            
            if distance < min_distance:
                min_distance = distance
                nearest_point = projection
                segment_idx = i
                segment_ratio = ratio
        
        return nearest_point, segment_idx, segment_ratio
    
    def calculate_path_errors(self, current_pose) -> Tuple[float, float]:
        """
        Calculate path tracking error
        
        Args:
            current_pose: Current pose (contains x, y, theta)
            
        Returns:
            tuple: (lateral error, angular error)
        """
        if not self.target_path:
            return 0.0, 0.0
        
        current_pos = Point(current_pose.x, current_pose.y)
        
        # Find nearest point
        nearest_point, segment_idx, segment_ratio = self.find_nearest_point_on_path(current_pos)
        
        # Calculate lateral error (perpendicular distance)
        if nearest_point:
            lateral_error = current_pos.distance_to(nearest_point)
            
            # Determine error direction (left side is positive)
            segment = self.target_path_segments[segment_idx]
            segment_vector = Point(segment.end.x - segment.start.x, segment.end.y - segment.start.y)
            
            # Calculate perpendicular vector
            current_vector = Point(current_pos.x - nearest_point.x, current_pos.y - nearest_point.y)
            
            # Calculate cross product to determine direction
            cross_product = segment_vector.x * current_vector.y - segment_vector.y * current_vector.x
            if cross_product < 0:
                lateral_error = -lateral_error
        else:
            lateral_error = 0.0
        
        # Calculate angular error
        if segment_idx < len(self.target_path_segments):
            segment = self.target_path_segments[segment_idx]
            
            # Calculate path direction angle (radians)
            path_angle = math.atan2(segment.end.y - segment.start.y, segment.end.x - segment.start.x)
            path_angle_deg = math.degrees(path_angle)
            
            # Angular error (current angle - path angle)
            angular_error = current_pose.theta - path_angle_deg
            
            # Normalize to [-180, 180]
            while angular_error > 180:
                angular_error -= 360
            while angular_error < -180:
                angular_error += 360
        else:
            angular_error = 0.0
        
        # Update debug information
        self.debug_info['nearest_point'] = nearest_point
        self.debug_info['lateral_error'] = lateral_error
        self.debug_info['angular_error'] = angular_error
        
        return lateral_error, angular_error
    
    def find_lookahead_point(self, current_pos: Point, segment_idx: int, segment_ratio: float) -> Point:
        """
        Find lookahead point (for pure pursuit algorithm)
        
        Args:
            current_pos: Current position
            segment_idx: Current segment index
            segment_ratio: Position ratio on current segment
            
        Returns:
            Lookahead point
        """
        if not self.target_path_segments:
            return current_pos
        
        # Search forward along path until accumulated distance reaches lookahead distance
        accumulated_distance = 0.0
        current_segment_idx = segment_idx
        current_segment_ratio = segment_ratio
        
        while current_segment_idx < len(self.target_path_segments):
            segment = self.target_path_segments[current_segment_idx]
            
            # Calculate remaining length of current segment
            if current_segment_idx == segment_idx:
                # From current position to segment end
                remaining_ratio = 1.0 - current_segment_ratio
                remaining_length = segment.length * remaining_ratio
            else:
                remaining_length = segment.length
            
            # Check if lookahead distance is reached
            if accumulated_distance + remaining_length >= self.lookahead_distance:
                # Find lookahead point within current segment
                needed_distance = self.lookahead_distance - accumulated_distance
                
                if current_segment_idx == segment_idx:
                    final_ratio = current_segment_ratio + (needed_distance / segment.length)
                else:
                    final_ratio = needed_distance / segment.length
                
                final_ratio = min(1.0, final_ratio)
                
                # Calculate lookahead point
                if current_segment_idx == segment_idx:
                    start_point = Point(
                        segment.start.x * (1 - current_segment_ratio) + segment.end.x * current_segment_ratio,
                        segment.start.y * (1 - current_segment_ratio) + segment.end.y * current_segment_ratio
                    )
                else:
                    start_point = segment.start
                
                end_point = segment.end
                
                lookahead_point = Point(
                    start_point.x * (1 - final_ratio) + end_point.x * final_ratio,
                    start_point.y * (1 - final_ratio) + end_point.y * final_ratio
                )
                
                self.debug_info['lookahead_point'] = lookahead_point
                return lookahead_point
            
            # Move to next segment
            accumulated_distance += remaining_length
            current_segment_idx += 1
            current_segment_ratio = 0.0
        
        # If reached end of path, return last point
        last_point = self.target_path[-1]
        self.debug_info['lookahead_point'] = last_point
        return last_point
    
    def pid_control(self, error, last_error, integral, kp, ki, kd, dt):
        """
        PID controller
        
        Args:
            error: Current error
            last_error: Last error
            integral: Integral term
            kp: Proportional coefficient
            ki: Integral coefficient
            kd: Derivative coefficient
            dt: Time interval
            
        Returns:
            tuple: (control output, updated integral term)
        """
        # Integral term
        integral += error * dt
        integral = max(-1.0, min(1.0, integral))  # Prevent integral windup
        
        # Derivative term
        derivative = (error - last_error) / dt if dt > 0 else 0.0
        
        # PID output
        output = kp * error + ki * integral + kd * derivative
        
        return output, integral
    
    def should_continue_correction(self):
        """
        Determine whether to continue correction (improved version)
        
        Returns:
            tuple: (whether to continue correction, stop reason)
        """
        # Check if correction is enabled
        if not self.correction_enabled:
            self.log_path_info("Correction not enabled, stop")
            return False, "Correction not enabled"
        
        if not self.correction_executing:
            self.log_path_info("Correction not executing, stop")
            return False, "Correction not executing"
        
        # Check if path is completed
        if self.path_completed:
            self.log_path_info("Path completed (flagged), stop")
            return False, "Path completed (flagged)"
        
        # Check if timed out
        current_time = time.time()
        elapsed_time = current_time - self.correction_execution_start_time
        
        # Record current state
        if self.target_position and hasattr(self, 'debug_info') and 'current_progress' in self.debug_info:
            progress = self.debug_info['current_progress'] * 100
            self.log_path_info(f"Correction status check: elapsed {elapsed_time:.1f}s, progress {progress:.1f}%, remaining {self.max_correction_time - elapsed_time:.1f}s")
        
        if elapsed_time > self.max_correction_time:
            self.path_completed = True
            self.stop_reason = f"Correction timeout ({self.max_correction_time}s)"
            self.stop_details = f"Execution time: {elapsed_time:.1f}s, exceeded max limit {self.max_correction_time}s"
            self.log_path_info(f"Correction timeout: {elapsed_time:.1f}s > {self.max_correction_time}s")
            return False, self.stop_reason
        
        return True, ""
    
    def calculate_correction_commands(self, current_pose):
        """
        Calculate correction control commands (key fix - supports arbitrary path directions)
        
        Args:
            current_pose: Current pose (x, y, theta)
            
        Returns:
            dict: corrected motion commands {x_speed, y_speed, angular_speed} (units: mm/s, mrad/s)
        """
        # Check if correction should execute
        if not self.correction_enabled:
            return None
        
        # Check if in delay waiting state
        if self.correction_delayed:
            self.check_and_start_correction()
            return None
        
        # Check if should stop correction
        should_continue, stop_reason = self.should_continue_correction()
        if not should_continue:
            if stop_reason:
                self.log_path_info(f"Stop correction reason: {stop_reason}")
                self.print_stop_reason(stop_reason)
            return None
        
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt <= 0:
            dt = 0.01
        
        # Calculate current errors (lateral error and angular error)
        lateral_error, angular_error = self.calculate_path_errors(current_pose)
        
        # Update error history
        self.lateral_error_history.append(lateral_error)
        self.angular_error_history.append(angular_error)
        
        if len(self.lateral_error_history) > self.max_history_size:
            self.lateral_error_history.pop(0)
        if len(self.angular_error_history) > self.max_history_size:
            self.angular_error_history.pop(0)
        
        # Check if reached endpoint
        current_pos = Point(current_pose.x, current_pose.y)
        
        if self.target_position:
            distance_to_target = current_pos.distance_to(self.target_position)
            angle_to_target = self.calculate_angle_to_target(current_pose)
            
            # Determine if reached endpoint
            position_reached = distance_to_target < self.target_arrival_threshold
            angle_reached = abs(angle_to_target) < self.path_completion_angle_threshold
            
            if position_reached and angle_reached:
                self.path_completed = True
                self.stop_reason = "Reached target point"
                return {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0}
            
            # Update progress
            total_length = self.debug_info.get('path_length', 1.0)
            completed_length = total_length - distance_to_target
            progress = max(0.0, min(1.0, completed_length / total_length))
            self.debug_info['current_progress'] = progress
        
        # Get current path segment index (from find_nearest_point_on_path)
        _, segment_idx, segment_ratio = self.find_nearest_point_on_path(current_pos)
        self.current_target_idx = segment_idx
        
        # PID control calculate correction amount
        vy_correction_val, self.integral_lateral = self.pid_control(
            lateral_error, self.last_lateral_error, self.integral_lateral,
            self.kp_lateral, self.ki_lateral, self.kd_lateral, dt
        )
        
        angular_correction, self.integral_angular = self.pid_control(
            angular_error, self.last_angular_error, self.integral_angular,
            self.kp_angular, self.ki_angular, self.kd_angular, dt
        )
        
        self.last_lateral_error = lateral_error
        self.last_angular_error = angular_error
        
        # ===== Key fix: calculate velocity based on path direction =====
        if self.queue_finished and self.target_path_segments:
            # Standalone correction mode: move forward along path tangent + lateral correction perpendicular to path
            
            # Get current path segment direction
            if segment_idx < len(self.target_path_segments):
                segment = self.target_path_segments[segment_idx]
                path_dx = segment.end.x - segment.start.x
                path_dy = segment.end.y - segment.start.y
                path_length_seg = math.sqrt(path_dx**2 + path_dy**2)
                
                if path_length_seg > 0.001:
                    # Path unit direction vector (ux, uy) - tangent direction
                    ux = path_dx / path_length_seg
                    uy = path_dy / path_length_seg
                    
                    # Path normal vector (right side direction, perpendicular to tangent)
                    # Tangent (ux, uy) rotated -90 degrees to get right normal (uy, -ux)
                    right_nx = uy
                    right_ny = -ux
                    
                    # Forward speed magnitude (decelerate based on distance)
                    if distance_to_target < 0.2:
                        speed_factor = 0.3
                    elif distance_to_target < 0.5:
                        speed_factor = 0.5
                    else:
                        speed_factor = min(1.0, distance_to_target / 2.0) if distance_to_target < 2.0 else 1.0
                    
                    forward_speed = self.standalone_base_speed * speed_factor  # mm/s
                    
                    # Lateral correction speed (along path right normal direction)
                    # lateral_error > 0 means on left side of path, need to correct right (positive direction)
                    # When PID output is positive (error>0, on left side), need to move right
                    lateral_speed = vy_correction_val * 800  # Coefficient adjustable, right is positive
                    lateral_speed = max(-300, min(300, lateral_speed))  # Limit lateral speed
                    
                    # Combined velocity in world frame
                    # Forward component (along path tangent) + lateral correction component (along path right normal)
                    vx_world = forward_speed * ux + lateral_speed * right_nx
                    vy_world = forward_speed * uy + lateral_speed * right_ny
                else:
                    vx_world = 0
                    vy_world = 0
            else:
                vx_world = 0
                vy_world = 0
        else:
            # Queue mode: only lateral correction (world frame Y direction)
            # Note: assumes path roughly along X-axis, for precise control use path direction calculation above
            vx_world = 0
            vy_world = vy_correction_val * 800  # mm/s
            vy_world = max(-200, min(200, vy_world))
        
        # Angular velocity correction (always used, unit: mrad/s)
        angular_speed_correction = angular_correction * 8  # Convert to mrad/s
        angular_speed_correction = max(-300, min(300, angular_speed_correction))
        
        # Convert world frame velocity to car frame
        # Input: vx_world, vy_world (mm/s), current_pose.theta (degrees)
        # Output: vx_body (forward, mm/s), vy_body (leftward, mm/s)
        vx_body, vy_body = self.world_to_body(vx_world, vy_world, current_pose.theta)
        
        # Apply correction strength
        vx_body = vx_body * self.correction_strength
        vy_body = vy_body * self.correction_strength
        angular_speed_correction = angular_speed_correction * self.correction_strength
        
        corrected_commands = {
            'x_speed': int(vx_body),
            'y_speed': int(vy_body),
            'angular_speed': int(angular_speed_correction)
        }
        
        # Debug information
        self.debug_info['correction_applied'] = True
        self.debug_info['current_error'] = math.sqrt(lateral_error**2 + angular_error**2)
        self.debug_info['lateral_error'] = lateral_error
        self.debug_info['angular_error'] = angular_error
        
        # Log correction information (for debugging)
        elapsed_time = current_time - self.correction_execution_start_time
        progress = self.debug_info.get('current_progress', 0.0) * 100
        
        mode = "Standalone" if self.queue_finished else "Queue"
        self.log_path_info(f"{mode}: Position({current_pos.x:.3f}, {current_pos.y:.3f})")
        self.log_path_info(f"  Error: lateral {lateral_error:.3f}m, angle {angular_error:.1f}°")
        self.log_path_info(f"  Distance to target: {distance_to_target:.3f}m, Progress: {progress:.1f}%")
        self.log_path_info(f"  World velocity: X={vx_world:.1f}mm/s, Y={vy_world:.1f}mm/s")
        self.log_path_info(f"  Car velocity: X={vx_body:.1f}mm/s, Y={vy_body:.1f}mm/s, angular={angular_speed_correction:.1f}mrad/s")
        self.log_path_info(f"  Correction strength: {self.correction_strength:.1f}")
        
        return corrected_commands
    
    def calculate_angle_to_target(self, current_pose):
        """Calculate angle between current heading and target direction"""
        if not self.target_position:
            return 0.0
        
        current_pos = Point(current_pose.x, current_pose.y)
        
        # Calculate target direction angle
        dx = self.target_position.x - current_pos.x
        dy = self.target_position.y - current_pos.y
        
        if dx == 0 and dy == 0:
            return 0.0
        
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Calculate angle difference
        angle_diff = target_angle - current_pose.theta
        
        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        return angle_diff
    
    def is_path_completed(self):
        """Check if path is completed"""
        return self.path_completed
    
    def get_progress(self):
        """Get path completion progress"""
        return self.debug_info.get('current_progress', 0.0)
    
    def get_debug_info(self):
        """Get debug information"""
        # Update stop reason
        self.debug_info['stop_reason'] = self.stop_reason
        return self.debug_info.copy()
    
    def validate_path(self):
        """Validate generated path"""
        if not self.target_path:
            return "No path generated"
        
        stats = self.get_path_statistics()
        result = f"Path validation result:\n"
        result += f"Density mode: {stats['density_mode']}\n"
        result += f"Total points: {stats['point_count']}\n"
        result += f"Path length: {stats['total_length']:.3f}m\n"
        result += f"Average point spacing: {stats['avg_spacing']:.3f}m\n"
        result += f"Has Y variation: {'Yes' if stats['has_y_variation'] else 'No'}\n"
        
        # Check for duplicate points
        unique_points = set((round(p.x, 3), round(p.y, 3)) for p in self.target_path)
        if len(unique_points) < len(self.target_path):
            result += f"Warning: {len(self.target_path) - len(unique_points)} duplicate points\n"
        
        return result
    
    # New method: Get detailed reason for correction stop
    def get_correction_stop_reason(self):
        """
        Get detailed reason for correction stop
        
        Returns:
            str: Stop reason, if still running returns "Running"
        """
        if not self.correction_enabled:
            return "Correction not enabled"
        
        if self.correction_delayed:
            remaining = max(0, self.correction_start_time - time.time())
            return f"Correction delay, remaining {remaining:.1f}s"
        
        if not self.correction_executing:
            return "Correction not executing"
        
        if self.path_completed:
            return self.stop_reason if self.stop_reason != "Running" else "Path completed"
        
        return "Running"
    
    def get_stop_details(self):
        """Get detailed stop information"""
        return self.stop_details
    
    # New method: Print stop reason in terminal
    def print_stop_reason(self, reason):
        """Print stop reason in terminal"""
        print("=== Path correction stopped ===")
        print(f"Reason: {reason}")
        if self.stop_details:
            print("Details:")
            print(self.stop_details)
        print("")
    
    # New method: Print correction completion information in terminal
    def print_correction_completion(self, reason, position_error, angle_error, elapsed_time):
        """Print correction completion information in terminal"""
        print("=" * 50)
        print("          Path correction completed")
        print("=" * 50)
        print(f"Completion reason: {reason}")
        print(f"Final position error: {position_error:.3f}m")
        print(f"Final angle error: {angle_error:.1f}°")
        print(f"Total correction time: {elapsed_time:.1f}s")
        print(f"Target point coordinates: ({self.target_position.x:.3f}, {self.target_position.y:.3f})")
        print(f"Total path points: {len(self.target_path)}")
        print(f"Density mode: {self.point_density_mode}")
        print(f"Correction strength: {self.correction_strength:.1f}")
        print("=" * 50)
        print("")
    
    # New method: Print detailed correction status in terminal
    def print_correction_status(self):
        """Print detailed correction status in terminal"""
        if not self.target_path:
            print("No target path")
            return
        
        status_lines = [
            "=== Path correction status ===",
            f"Target path points: {len(self.target_path)}",
            f"Density mode: {self.point_density_mode}",
            f"Correction enabled: {self.correction_enabled}",
            f"Correction executing: {self.correction_executing}",
            f"Queue finished: {self.queue_finished}",
            f"Path completed: {self.path_completed}",
            f"Delay status: {self.correction_delayed}",
            f"Stop reason: {self.stop_reason}",
            f"Correction strength: {self.correction_strength:.1f}",
        ]
        
        if self.correction_delayed:
            remaining = max(0, self.correction_start_time - time.time())
            status_lines.append(f"Delay remaining: {remaining:.1f}s")
        
        if self.correction_executing and not self.path_completed:
            current_time = time.time()
            elapsed = current_time - self.correction_execution_start_time
            status_lines.append(f"Elapsed time: {elapsed:.1f}s")
            status_lines.append(f"Max time: {self.max_correction_time}s")
            
            if elapsed > self.max_correction_time * 0.8:
                status_lines.append("Warning: Approaching timeout!")
            
            # Show progress
            progress = self.get_progress() * 100
            status_lines.append(f"Completion progress: {progress:.1f}%")
        
        if self.target_position and self.correction_executing:
            status_lines.append(f"Target position: ({self.target_position.x:.3f}, {self.target_position.y:.3f})")
        
        # Print all status lines
        for line in status_lines:
            print(line)
        
        print("")
    
    # New method: Get detailed status report
    def get_detailed_status_report(self):
        """Get detailed status report"""
        report = {
            'basic_info': {
                'has_target_path': len(self.target_path) > 0,
                'path_points': len(self.target_path),
                'density_mode': self.point_density_mode,
                'correction_enabled': self.correction_enabled,
                'correction_executing': self.correction_executing,
                'queue_finished': self.queue_finished,
                'path_completed': self.path_completed,
                'correction_delayed': self.correction_delayed,
                'stop_reason': self.stop_reason,
                'correction_strength': self.correction_strength
            },
            'timing_info': {
                'max_correction_time': self.max_correction_time,
                'correction_delay': self.correction_delay
            }
        }
        
        if self.correction_executing and not self.path_completed:
            current_time = time.time()
            report['timing_info']['elapsed_time'] = current_time - self.correction_execution_start_time
            report['timing_info']['remaining_time'] = self.max_correction_time - (current_time - self.correction_execution_start_time)
            report['basic_info']['progress'] = self.get_progress()
        
        if self.correction_delayed:
            current_time = time.time()
            report['timing_info']['time_until_start'] = max(0, self.correction_start_time - current_time)
        
        if self.target_position:
            report['target_info'] = {
                'x': self.target_position.x,
                'y': self.target_position.y,
                'arrival_threshold': self.target_arrival_threshold,
                'angle_threshold': self.path_completion_angle_threshold
            }
        
        return report
    
    # New method: Start standalone correction loop
    def start_standalone_correction_loop(self, gui_instance, callback_interval_ms=100):
        """
        Start standalone correction loop (called after queue ends)
        
        Args:
            gui_instance: GUI instance, for sending commands
            callback_interval_ms: Callback interval (milliseconds)
        """
        if not self.correction_enabled or not self.correction_executing:
            self.log_path_info("Error: correction not enabled, cannot start standalone correction loop")
            return
        
        self.queue_finished = True
        self.debug_info['queue_finished'] = True
        
        self.log_path_info("=== Start standalone correction loop ===")
        self.log_path_info(f"Start time: {time.strftime('%H:%M:%S', time.localtime(time.time()))}")
        self.log_path_info(f"Callback interval: {callback_interval_ms}ms")
        self.log_path_info("")
        
        # Start correction loop
        self._standalone_correction_loop(gui_instance, callback_interval_ms)
    
    def _standalone_correction_loop(self, gui_instance, callback_interval_ms):
        """Standalone correction loop internal implementation (fixed version)"""
        # Check if should continue correction
        should_continue, stop_reason = self.should_continue_correction()
        
        if not should_continue:
            self.log_path_info(f"Standalone correction loop ended: {stop_reason}")
            if stop_reason:
                self.print_stop_reason(stop_reason)
            # Disable correction
            self.disable_correction()
            return
        
        try:
            # Get current pose
            current_pose = None
            if gui_instance and gui_instance.trajectory_tracker:
                current_pose = gui_instance.trajectory_tracker.get_current_pose()
            
            if current_pose:
                # Calculate correction commands
                corrected_commands = self.calculate_correction_commands(current_pose)
                
                if corrected_commands:
                    # Send correction commands
                    success = gui_instance.send_frame_silent(
                        gui_instance.car_controller.send_motion_command(
                            corrected_commands['x_speed'],
                            corrected_commands['y_speed'],
                            corrected_commands['angular_speed']
                        )
                    )
                    
                    if success:
                        self.log_path_info(f"Send standalone correction command: X={corrected_commands['x_speed']}, Y={corrected_commands['y_speed']}, angular={corrected_commands['angular_speed']}")
                
                # Update status display
                if hasattr(gui_instance, 'update_correction_status'):
                    gui_instance.update_correction_status()
        
        except Exception as e:
            self.log_path_info(f"Standalone correction loop error: {str(e)}")
            import traceback
            self.log_path_info(traceback.format_exc())
        
        # Schedule next execution - always schedule next execution regardless
        try:
            if gui_instance and hasattr(gui_instance.root, 'after'):
                gui_instance.root.after(callback_interval_ms, 
                                      lambda: self._standalone_correction_loop(gui_instance, callback_interval_ms))
        except Exception as e:
            self.log_path_info(f"Schedule next execution failed: {str(e)}")
            # Try to restart loop
            try:
                if hasattr(gui_instance, 'root') and gui_instance.root:
                    gui_instance.root.after(1000, lambda: self.start_standalone_correction_loop(gui_instance, callback_interval_ms))
            except:
                pass

    def get_detailed_points_info(self) -> Optional[Dict[str, Any]]:
        """Get detailed trajectory point information"""
        if not self.target_path:
            return None
        
        detailed_info = {
            'total_points': len(self.target_path),
            'density_mode': self.point_density_mode,
            'min_distance': self.min_distance_between_points,
            'generation_time': time.strftime("%Y-%m-%d %H:%M:%S"),
            'points': []
        }
        
        accumulated_distance = 0.0
        for i, point in enumerate(self.target_path):
            if i == 0:
                distance_to_prev = 0.0
            else:
                distance_to_prev = self.target_path[i-1].distance_to(point)
                accumulated_distance += distance_to_prev
            
            detailed_info['points'].append({
                'index': i+1,
                'x': point.x,
                'y': point.y,
                'distance_to_prev': distance_to_prev,
                'accumulated_distance': accumulated_distance,
                'is_key_point': i % 10 == 0 or i == len(self.target_path)-1  # Mark key point every 10 points
            })
        
        # Add statistics
        if len(self.target_path) >= 2:
            distances = []
            for i in range(1, len(self.target_path)):
                distances.append(self.target_path[i-1].distance_to(self.target_path[i]))
            
            detailed_info['statistics'] = {
                'total_length': accumulated_distance,
                'avg_distance': sum(distances) / len(distances) if distances else 0.0,
                'min_distance': min(distances) if distances else 0.0,
                'max_distance': max(distances) if distances else 0.0,
                'x_range': max(p.x for p in self.target_path) - min(p.x for p in self.target_path),
                'y_range': max(p.y for p in self.target_path) - min(p.y for p in self.target_path)
            }
        
        return detailed_info
    
    def export_points_to_csv(self, filename: str) -> bool:
        """Export trajectory points to CSV file"""
        if not self.target_path:
            return False
        
        try:
            import csv
            
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                # Write header
                writer.writerow(['Index', 'X(m)', 'Y(m)', 'Distance to prev(m)', 'Accumulated distance(m)', 'Is key point'])
                
                accumulated_distance = 0.0
                for i, point in enumerate(self.target_path):
                    if i == 0:
                        distance_to_prev = 0.0
                    else:
                        distance_to_prev = self.target_path[i-1].distance_to(point)
                        accumulated_distance += distance_to_prev
                    
                    is_key_point = i % 10 == 0 or i == len(self.target_path)-1
                    
                    writer.writerow([
                        i+1,
                        f"{point.x:.6f}",
                        f"{point.y:.6f}",
                        f"{distance_to_prev:.6f}",
                        f"{accumulated_distance:.6f}",
                        "Yes" if is_key_point else "No"
                    ])
            
            return True
            
        except Exception as e:
            self.log_path_info(f"Export CSV failed: {str(e)}")
            return False
    
    def get_points_summary(self) -> str:
        """Get trajectory point summary information"""
        if not self.target_path:
            return "No trajectory point data"
        
        total_points = len(self.target_path)
        total_length = 0.0
        
        if total_points >= 2:
            for i in range(1, total_points):
                total_length += self.target_path[i-1].distance_to(self.target_path[i])
        
        x_values = [p.x for p in self.target_path]
        y_values = [p.y for p in self.target_path]
        
        summary = f"Trajectory point summary:\n"
        summary += f"  Total points: {total_points}\n"
        summary += f"  Total length: {total_length:.3f} m\n"
        summary += f"  Density mode: {self.point_density_mode}\n"
        summary += f"  X range: {min(x_values):.3f} ~ {max(x_values):.3f} m\n"
        summary += f"  Y range: {min(y_values):.3f} ~ {max(y_values):.3f} m\n"
        summary += f"  Y variation: {max(y_values) - min(y_values):.3f} m\n"
        
        return summary
    
    def check_control_priority(self):
        """
        Check control priority and permissions
        
        Returns:
            dict: Control status information
        """
        status = {
            'correction_enabled': self.correction_enabled,
            'correction_executing': self.correction_executing,
            'correction_delayed': self.correction_delayed,
            'queue_finished': self.queue_finished,
            'path_completed': self.path_completed,
            'has_target_path': len(self.target_path) > 0,
            'current_target_idx': self.current_target_idx,
            'total_targets': len(self.target_path),
            'should_correct': self.correction_enabled and not self.path_completed,
            'can_send_commands': self.correction_executing and not self.path_completed,
            'remaining_time': self.max_correction_time - (time.time() - self.correction_execution_start_time) 
                          if self.correction_executing else 0,
            'correction_strength': self.correction_strength
        }
        
        # Record check results
        self.log_path_info("=== Control priority check ===")
        for key, value in status.items():
            self.log_path_info(f"  {key}: {value}")
        
        return status