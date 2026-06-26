# trajectory_tracker.py
import time
import math
import threading
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

@dataclass
class Pose:
    """Pose representation"""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    theta: float = 0.0  # degrees
    
    def to_dict(self):
        return {'x': self.x, 'y': self.y, 'theta': self.theta}

class TrajectoryTracker:
    """Trajectory tracker"""
    
    def __init__(self, scale_factor=0.001):
        """
        Initialize trajectory tracker
        
        Args:
            scale_factor: scale factor, converts millimeters to meters or other units
        """
        self._lock = threading.Lock()  # Thread lock, protects shared data
        
        self.pose = Pose()  # Current pose
        self.trajectory: List[Pose] = [Pose()]  # Trajectory point list
        self.velocities: List[Tuple[float, float, float]] = []  # Velocity history
        self.timestamps: List[float] = [time.time()]  # Timestamps
        self.scale_factor = scale_factor
        self.last_update_time = time.time()
        
        # Trajectory display range limit
        self.max_trajectory_points = 10000  # Maximum trajectory points
        self.max_display_points = 1000  # Display points
        
        # Filter parameters
        self.velocity_filter_window = 5
        self.vx_history = []
        self.vy_history = []
        self.vtheta_history = []
        
        # Velocity component cache
        self.vx_cache = None
        self.vy_cache = None
        self.vtheta_cache = None
        self.vx_cache_time = 0
        self.vy_cache_time = 0
        self.vtheta_cache_time = 0
        
        # Latest received velocity data
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vtheta = 0.0
        
        # Direction calibration parameters
        self.vx_sign = +1   # Vehicle X direction: forward is positive (heading direction)
        self.vy_sign = +1   # Vehicle Y direction: left is positive (left side from heading view)
        self.vtheta_sign = +1  # Angular velocity: counter-clockwise is positive
        
        # Debug flag
        self.debug_enabled = False
        
        # Angle unit conversion constants
        self.DEG_TO_RAD = math.pi / 180.0
        self.RAD_TO_DEG = 180.0 / math.pi
    
    def reset(self):
        """Reset trajectory"""
        with self._lock:
            self.pose = Pose()
            self.trajectory = [Pose()]
            self.velocities = []
            self.timestamps = [time.time()]
        self.last_update_time = time.time()
        self.vx_history = []
        self.vy_history = []
        self.vtheta_history = []
    
    def update_from_frame(self, frame_info):
        """Update trajectory from CAN frame info (vehicle coordinate system to world coordinate system conversion)"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt <= 0:
            return self.pose
        
        # Update latest velocity data (atomic operation, no lock needed)
        if 'vx' in frame_info:  # mm/s (vehicle coordinate system)
            self.last_vx = frame_info['vx']
            self.vx_cache = frame_info['vx']
            self.vx_cache_time = current_time
            
        if 'vy' in frame_info:  # mm/s (vehicle coordinate system)
            self.last_vy = frame_info['vy']
            self.vy_cache = frame_info['vy']
            self.vy_cache_time = current_time
            
        if 'vtheta' in frame_info:  # mrad/s (milliradians per second, vehicle coordinate system)
            self.last_vtheta = frame_info['vtheta']
            self.vtheta_cache = frame_info['vtheta']
            self.vtheta_cache_time = current_time
        
        # Check if complete velocity data is available
        time_threshold = 1.0
        has_vx = current_time - self.vx_cache_time < time_threshold if self.vx_cache is not None else False
        has_vy = current_time - self.vy_cache_time < time_threshold if self.vy_cache is not None else False
        has_vtheta = current_time - self.vtheta_cache_time < time_threshold if self.vtheta_cache is not None else False
        
        if has_vx and has_vy and has_vtheta and dt > 0:
            # Velocity filtering
            vx_body = self._filter_velocity(self.vx_cache, self.vx_history)  # mm/s (vehicle coordinate system)
            vy_body = self._filter_velocity(self.vy_cache, self.vy_history)  # mm/s (vehicle coordinate system)
            vtheta_body = self._filter_velocity(self.vtheta_cache, self.vtheta_history)  # mrad/s (milliradians per second)
            
            # Apply direction calibration
            vx_body_adjusted = vx_body * self.vx_sign  # mm/s
            vy_body_adjusted = vy_body * self.vy_sign  # mm/s
            vtheta_body_adjusted = vtheta_body * self.vtheta_sign  # mrad/s
            
            # Convert to standard units
            vx_body_m_s = vx_body_adjusted / 1000.0  # mm/s → m/s
            vy_body_m_s = vy_body_adjusted / 1000.0  # mm/s → m/s
            
            # Key fix: correctly convert angular velocity units
            # mrad/s → rad/s → deg/s
            vtheta_rad_s = vtheta_body_adjusted / 1000.0  # mrad/s → rad/s
            vtheta_deg_s = vtheta_rad_s * self.RAD_TO_DEG  # rad/s → deg/s
            
            # Current angle (radians)
            theta_rad = math.radians(self.pose.theta)
            
            # Convert vehicle coordinate system velocity to world coordinate system velocity
            vx_world = vx_body_m_s * math.cos(theta_rad) - vy_body_m_s * math.sin(theta_rad)
            vy_world = vx_body_m_s * math.sin(theta_rad) + vy_body_m_s * math.cos(theta_rad)
            
            # Update pose
            self.pose.x += vx_world * dt  # meters (world coordinate system)
            self.pose.y += vy_world * dt  # meters (world coordinate system)
            
            # Key fix: correctly update angle (using deg/s * dt)
            angle_change_deg = vtheta_deg_s * dt
            self.pose.theta += angle_change_deg
            
            # Normalize angle to [-180, 180] degrees
            self.pose.theta = ((self.pose.theta + 180) % 360) - 180
            
            # Add to trajectory (only add points with changes)
            position_changed = abs(vx_world * dt) > 0.001 or abs(vy_world * dt) > 0.001
            orientation_changed = abs(angle_change_deg) > 0.1
            
            # Use lock to protect modifications to shared lists
            with self._lock:
                # Save velocity for display
                self.velocities.append((vx_body_m_s, vy_body_m_s, vtheta_rad_s))
                
                if position_changed or orientation_changed:
                    self.trajectory.append(Pose(self.pose.x, self.pose.y, self.pose.theta))
                    self.timestamps.append(current_time)
                
                # Limit trajectory length
                if len(self.trajectory) > self.max_trajectory_points:
                    keep_count = self.max_trajectory_points // 2
                    self.trajectory = self.trajectory[-keep_count:]
                    self.timestamps = self.timestamps[-keep_count:]
                    if len(self.velocities) > keep_count:
                        self.velocities = self.velocities[-keep_count:]
        
        self.last_update_time = current_time
        return self.pose
    
    def _filter_velocity(self, velocity, history):
        """Simple moving average filter"""
        history.append(velocity)
        if len(history) > self.velocity_filter_window:
            history.pop(0)
        return sum(history) / len(history)
    
    def calibrate_direction(self, axis, sign):
        """
        Calibrate direction sign
        
        Args:
            axis: 'x' - X direction, 'y' - Y direction, 'theta' - rotation direction
            sign: +1 or -1
        """
        if axis == 'x':
            self.vx_sign = sign
        elif axis == 'y':
            self.vy_sign = sign
        elif axis == 'theta':
            self.vtheta_sign = sign
    
    def set_debug_enabled(self, enabled):
        """Enable/disable debug output"""
        self.debug_enabled = enabled
    
    def get_display_trajectory(self):
        """Get trajectory for display (downsampled)"""
        with self._lock:
            if len(self.trajectory) <= self.max_display_points:
                return self.trajectory.copy()
            
            # Equal interval sampling
            step = len(self.trajectory) // self.max_display_points
            return self.trajectory[::step].copy()
    
    def get_trajectory_arrays(self):
        """Get trajectory in numpy array format"""
        with self._lock:
            # Copy data under lock protection
            traj = self.trajectory.copy()
        
        # Perform downsampling and conversion outside lock
        if len(traj) > self.max_display_points:
            step = len(traj) // self.max_display_points
            traj = traj[::step]
        
        x = np.array([p.x for p in traj])
        y = np.array([p.y for p in traj])
        theta = np.array([p.theta for p in traj])
        return x, y, theta
    
    def get_current_pose(self):
        """Get current pose (world coordinate system)"""
        return self.pose
    
    def get_current_velocity(self):
        """Get current velocity (vx_m/s, vy_m/s, vtheta_rad/s) - world coordinate system"""
        # Get latest vehicle coordinate system velocity
        vx_body_m_s = self.last_vx / 1000.0 * self.vx_sign if self.last_vx is not None else 0.0
        vy_body_m_s = self.last_vy / 1000.0 * self.vy_sign if self.last_vy is not None else 0.0
        vtheta_mrad_s = self.last_vtheta * self.vtheta_sign if self.last_vtheta is not None else 0.0
        
        # Convert to standard units
        vtheta_rad_s = vtheta_mrad_s / 1000.0  # mrad/s → rad/s
        
        # Convert to world coordinate system velocity
        theta_rad = math.radians(self.pose.theta)
        vx_world = vx_body_m_s * math.cos(theta_rad) - vy_body_m_s * math.sin(theta_rad)
        vy_world = vx_body_m_s * math.sin(theta_rad) + vy_body_m_s * math.cos(theta_rad)
        
        return (vx_world, vy_world, vtheta_rad_s)
    
    def get_current_angular_velocity_deg(self):
        """Get current angular velocity (degrees/second)"""
        vtheta_mrad_s = self.last_vtheta * self.vtheta_sign if self.last_vtheta is not None else 0.0
        vtheta_rad_s = vtheta_mrad_s / 1000.0  # mrad/s → rad/s
        return vtheta_rad_s * self.RAD_TO_DEG  # deg/s
    
    def get_trajectory_length(self):
        """Get trajectory length"""
        with self._lock:
            if len(self.trajectory) < 2:
                return 0.0
            
            # Copy data under lock protection，avoid concurrent modification
            trajectory_copy = self.trajectory.copy()
        
        # Use numpy vectorized computation to improve performance (execute outside lock)
        x = np.array([p.x for p in trajectory_copy])
        y = np.array([p.y for p in trajectory_copy])
        
        # Calculate distance between adjacent points
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx*dx + dy*dy)
        
        return np.sum(distances)
    
    def save_trajectory_to_file(self, filename="trajectory.csv"):
        """Save trajectory to CSV file"""
        import csv
        
        # Copy data under lock protection
        with self._lock:
            trajectory_copy = self.trajectory.copy()
            timestamps_copy = self.timestamps.copy()
            velocities_copy = self.velocities.copy()
        
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'x_m', 'y_m', 'theta_deg', 
                           'vx_body_mm_s', 'vy_body_mm_s', 'vtheta_mrad_s',
                           'vx_world_m_s', 'vy_world_m_s', 'omega_deg_s'])
            
            for i, pose in enumerate(trajectory_copy):
                timestamp = timestamps_copy[i] if i < len(timestamps_copy) else 0
                
                # Get corresponding velocity
                if i < len(velocities_copy):
                    vx_body_m_s, vy_body_m_s, vtheta_rad_s = velocities_copy[i]
                    # Restore original units
                    vx_body_mm_s = vx_body_m_s * 1000.0
                    vy_body_mm_s = vy_body_m_s * 1000.0
                    vtheta_mrad_s = vtheta_rad_s * 1000.0
                    omega_deg_s = vtheta_rad_s * self.RAD_TO_DEG
                    
                    # Convert to world coordinate system velocity
                    theta_rad = math.radians(pose.theta)
                    vx_world = vx_body_m_s * math.cos(theta_rad) - vy_body_m_s * math.sin(theta_rad)
                    vy_world = vx_body_m_s * math.sin(theta_rad) + vy_body_m_s * math.cos(theta_rad)
                else:
                    vx_body_mm_s, vy_body_mm_s, vtheta_mrad_s = 0.0, 0.0, 0.0
                    vx_world, vy_world = 0.0, 0.0
                    omega_deg_s = 0.0
                
                writer.writerow([timestamp, pose.x, pose.y, pose.theta, 
                               vx_body_mm_s, vy_body_mm_s, vtheta_mrad_s,
                               vx_world, vy_world, omega_deg_s])
    
    def clear_trajectory(self):
        """Clear trajectory, reset to origin"""
        with self._lock:
            self.pose = Pose(0, 0, 0)
            self.trajectory = [self.pose]
            self.velocities = []
            self.timestamps = [time.time()]
        
        # Clear history
        self.vx_history = []
        self.vy_history = []
        self.vtheta_history = []
    
    def reset_to_origin(self):
        """Reset to origin, keep current heading"""
        with self._lock:
            self.pose.x = 0
            self.pose.y = 0
            self.trajectory = [Pose(0, 0, self.pose.theta)]
            self.timestamps = [time.time()]