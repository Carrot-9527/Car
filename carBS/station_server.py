# station_server_gui_v2.py - runtime queue sync version

import socket
import threading
import time
import json
import csv
import math
import random
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, Canvas
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from trajectory_tracker import TrajectoryTracker, Pose
from path_correction import PathCorrectionController, Point
from trajectory_plotter import TrajectoryPlotter as ExternalTrajectoryPlotter
from packet_loss_simulator import PacketLossSimulator


class StationServerGUI:
    """Station server GUI - runtime queue sync integrated"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Station Server - Remote AGV Control Center V7.0 (Queue Sync)")
        self.root.geometry("1600x900")
        
        # TCP server parameters
        self.host = '0.0.0.0'
        self.port = 5000
        self.server_socket = None
        self.client_conn = None
        self.client_addr = None
        self.tcp_connected = False
        self.running = False
        
        # Communication buffer
        self.recv_buffer = ""
        
        # Vehicle state
        self.ue_state = {
            'pose': None,
            'vx': 0, 'vy': 0, 'vtheta': 0,
            'enabled': False,
            'control_mode': 0,
            'voltage': 0.0,
            'device_status': 'normal',
            'remote_status': 'offline',
            'local_correction_enabled': False,  # Added: whether the client has local correction enabled
            'trajectory_synced': False          # Added: whether the trajectory is synced
        }
        self.last_ue_state_time = 0
        
        # Trajectory and correction
        self.trajectory_tracker = TrajectoryTracker(scale_factor=0.001)
        self.path_correction_controller = PathCorrectionController(self.trajectory_tracker)
        self.trajectory_plotter = None
        self.ideal_path = None
        
        # Control state
        self.correction_mode = False
        self.correction_strength = 0.7
        self.correction_delay = 0.0
        self.base_speed = 100
        
        # Packet loss simulator
        self.packet_loss_simulator = PacketLossSimulator(loss_probability=0.0)
        self.packet_loss_enabled = False
        self._pending_packet_loss_flag = False  # New: pending packet loss flag to be sent
        self.last_loss_stats_update = 0
        
        # Queue control
        self.speed_queue = []
        self.queue_executing = False
        self.current_command_index = 0
        self.current_command_start_time = 0
        self.current_send_count = 0
        
        self.create_widgets()
        
        # Start TCP server
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        
        # Start timers
        self.root.after(100, self.update_gui)
        self.root.after(1000, self.check_connection_status)
        self.root.after(500, self.update_loss_stats)
        
        if self.trajectory_plotter:
            self.trajectory_plotter.enable_auto_refresh(200)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_widgets(self):
        """Create GUI interface with left scroll pane"""
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # ========== Left panel (with scroll) ==========
        left_container = ttk.Frame(main_frame, width=480)
        left_container.pack(side="left", fill="y", padx=5, pady=5)
        left_container.pack_propagate(False)
        
        # Create Canvas and Scrollbar
        left_canvas = Canvas(left_container, width=460, highlightthickness=0)
        left_scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        
        # Configure Canvas
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        
        # Place Canvas and Scrollbar
        left_canvas.pack(side="left", fill="both", expand=True)
        left_scrollbar.pack(side="right", fill="y")
        
        # Create scrollable Frame
        left_frame = ttk.Frame(left_canvas, width=460)
        left_canvas_window = left_canvas.create_window((0, 0), window=left_frame, anchor="nw", width=460)
        
        # Configure scroll region
        def configure_canvas(event):
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        
        left_frame.bind("<Configure>", configure_canvas)
        
        # Mouse wheel support
        def on_mousewheel(event):
            left_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        left_canvas.bind_all("<MouseWheel>", on_mousewheel)
        
        # Right panel (display)
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side="right", fill="both", expand=True, padx=5, pady=5)
        
        # ========== Left control panel ==========
        
        # TCP status
        frame_tcp = ttk.LabelFrame(left_frame, text="TCP Server Status")
        frame_tcp.pack(fill="x", padx=5, pady=3)
        
        self.tcp_status_label = ttk.Label(frame_tcp, text="Waiting for connection...", foreground="orange", font=("Arial", 10, "bold"))
        self.tcp_status_label.pack(padx=5, pady=3)
        
        ttk.Label(frame_tcp, text=f"Listening: {self.host}:{self.port}").pack(padx=5, pady=1)
        
        self.client_info_label = ttk.Label(frame_tcp, text="Client: None")
        self.client_info_label.pack(padx=5, pady=1)
        
        # Client local correction state display
        self.client_correction_label = ttk.Label(frame_tcp, 
            text="UE correction: Unknown | Trajectory synced: No",
            foreground="gray", font=("Arial", 9))
        self.client_correction_label.pack(padx=5, pady=2)
        
        # ========== Packet loss simulator panel ==========
        frame_loss = ttk.LabelFrame(left_frame, text="Packet Loss Simulator (Probability)")
        frame_loss.pack(fill="x", padx=5, pady=5)
        
        # Enable / disable
        self.loss_enable_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame_loss, text="Enable packet loss simulation", variable=self.loss_enable_var,
                       command=self._toggle_packet_loss).pack(anchor="w", padx=5, pady=2)
        
        # Packet loss probability
        prob_frame = ttk.Frame(frame_loss)
        prob_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(prob_frame, text="Packet loss rate:").pack(side="left")
        self.loss_prob_scale = ttk.Scale(prob_frame, from_=0, to=100, orient="horizontal", length=150)
        self.loss_prob_scale.set(0)
        self.loss_prob_scale.pack(side="left", padx=5)
        self.loss_prob_label = ttk.Label(prob_frame, text="0.0%", width=6)
        self.loss_prob_label.pack(side="left")
        
        self.loss_prob_scale.bind("<Motion>", self._update_prob_label)
        self.loss_prob_scale.bind("<ButtonRelease-1>", self._apply_prob_change)
        
        # Packet loss direction
        direction_frame = ttk.Frame(frame_loss)
        direction_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(direction_frame, text="Packet loss direction:").pack(side="left")
        self.loss_direction_var = tk.IntVar(value=PacketLossSimulator.DIRECTION_TX_ONLY)
        ttk.Radiobutton(direction_frame, text="TX", variable=self.loss_direction_var, 
                       value=PacketLossSimulator.DIRECTION_TX_ONLY, 
                       command=self._update_loss_direction).pack(side="left", padx=2)
        ttk.Radiobutton(direction_frame, text="RX", variable=self.loss_direction_var, 
                       value=PacketLossSimulator.DIRECTION_RX_ONLY,
                       command=self._update_loss_direction).pack(side="left", padx=2)
        ttk.Radiobutton(direction_frame, text="Both", variable=self.loss_direction_var, 
                       value=PacketLossSimulator.DIRECTION_BOTH,
                       command=self._update_loss_direction).pack(side="left", padx=2)
        
        # Packet loss stats display
        self.loss_stats_label = ttk.Label(frame_loss, 
            text="Stats: Tx 0/0 (0.0%) | Rx 0/0 (0.0%)",
            background="#f0f0f0", relief="solid", font=("Consolas", 9))
        self.loss_stats_label.pack(fill="x", padx=5, pady=3)
        
        # Quick set buttons
        quick_frame = ttk.Frame(frame_loss)
        quick_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(quick_frame, text="Quick set:").pack(side="left")
        for prob in [0, 10, 30, 50, 70, 90, 100]:
            ttk.Button(quick_frame, text=f"{prob}%", width=4, 
                      command=lambda p=prob: self._quick_set_loss_prob(p)).pack(side="left", padx=1)
        
        ttk.Button(frame_loss, text="Reset stats", command=self._reset_loss_stats).pack(fill="x", padx=5, pady=3)
        
        # Device control
        frame_control = ttk.LabelFrame(left_frame, text="Device Control (Remote)")
        frame_control.pack(fill="x", padx=5, pady=3)
        
        btn_frame = ttk.Frame(frame_control)
        btn_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Button(btn_frame, text="Enable", command=self.enable_device, width=8).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Disable", command=self.disable_device, width=8).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="E-Stop", command=self.emergency_stop, width=8).pack(side="left", padx=2)
        
        # Control mode
        frame_mode = ttk.LabelFrame(left_frame, text="Control Mode")
        frame_mode.pack(fill="x", padx=5, pady=3)
        
        self.mode_var = tk.StringVar(value="Idle")
        modes = [("Idle", 0x00), ("Remote", 0x01), ("CAN Control", 0x02), ("Follow", 0x03)]
        
        mode_frame = ttk.Frame(frame_mode)
        mode_frame.pack(fill="x", padx=5, pady=3)
        
        for text, value in modes:
            rb = ttk.Radiobutton(mode_frame, text=text, variable=self.mode_var, 
                               value=text, command=lambda v=value: self.set_control_mode(v))
            rb.pack(side="left", padx=3)
        
        # Auxiliary functions
        frame_aux = ttk.LabelFrame(left_frame, text="Auxiliary")
        frame_aux.pack(fill="x", padx=5, pady=3)
        
        self.buzzer_var = tk.BooleanVar()
        self.brake_var = tk.BooleanVar()
        self.special_var = tk.BooleanVar()
        
        ttk.Checkbutton(frame_aux, text="Buzzer", variable=self.buzzer_var).pack(side="left", padx=5)
        ttk.Checkbutton(frame_aux, text="Brake", variable=self.brake_var).pack(side="left", padx=5)
        ttk.Checkbutton(frame_aux, text="Special", variable=self.special_var).pack(side="left", padx=5)
        
        # Motion control
        frame_motion = ttk.LabelFrame(left_frame, text="Manual motion control")
        frame_motion.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(frame_motion, text="X speed (mm/s):").grid(row=0, column=0, padx=5, pady=3, sticky="w")
        self.x_speed_scale = ttk.Scale(frame_motion, from_=-1000, to=1000, orient="horizontal", length=180)
        self.x_speed_scale.set(0)
        self.x_speed_scale.grid(row=0, column=1, padx=5, pady=3)
        self.x_speed_label = ttk.Label(frame_motion, text="0", width=6)
        self.x_speed_label.grid(row=0, column=2, padx=5, pady=3)
        self.x_speed_scale.bind("<ButtonRelease-1>", lambda e: self.update_speed_display())
        
        ttk.Label(frame_motion, text="X speed (mm/s):").grid(row=1, column=0, padx=5, pady=3, sticky="w")
        self.y_speed_scale = ttk.Scale(frame_motion, from_=-500, to=500, orient="horizontal", length=180)
        self.y_speed_scale.set(0)
        self.y_speed_scale.grid(row=1, column=1, padx=5, pady=3)
        self.y_speed_label = ttk.Label(frame_motion, text="0", width=6)
        self.y_speed_label.grid(row=1, column=2, padx=5, pady=3)
        self.y_speed_scale.bind("<ButtonRelease-1>", lambda e: self.update_speed_display())
        
        ttk.Label(frame_motion, text="Angular speed (mrad/s):").grid(row=2, column=0, padx=5, pady=3, sticky="w")
        self.angular_speed_scale = ttk.Scale(frame_motion, from_=-1000, to=1000, orient="horizontal", length=180)
        self.angular_speed_scale.set(0)
        self.angular_speed_scale.grid(row=2, column=1, padx=5, pady=3)
        self.angular_speed_label = ttk.Label(frame_motion, text="0", width=6)
        self.angular_speed_label.grid(row=2, column=2, padx=5, pady=3)
        self.angular_speed_scale.bind("<ButtonRelease-1>", lambda e: self.update_speed_display())
        
        ttk.Button(frame_motion, text="Send motion command", command=self.send_motion_command, 
                  width=20).grid(row=3, column=0, columnspan=3, pady=8)
        
        # Queue control
        frame_queue = ttk.LabelFrame(left_frame, text="Speed queue control")
        frame_queue.pack(fill="x", padx=5, pady=3)
        
        input_frame = ttk.Frame(frame_queue)
        input_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(input_frame, text="X:").grid(row=0, column=0, padx=2)
        self.queue_x_entry = ttk.Entry(input_frame, width=8)
        self.queue_x_entry.grid(row=0, column=1, padx=2)
        self.queue_x_entry.insert(0, "100")
        
        ttk.Label(input_frame, text="Y:").grid(row=0, column=2, padx=2)
        self.queue_y_entry = ttk.Entry(input_frame, width=8)
        self.queue_y_entry.grid(row=0, column=3, padx=2)
        self.queue_y_entry.insert(0, "0")
        
        ttk.Label(input_frame, text="Angular:").grid(row=1, column=0, padx=2)
        self.queue_angular_entry = ttk.Entry(input_frame, width=8)
        self.queue_angular_entry.grid(row=1, column=1, padx=2)
        self.queue_angular_entry.insert(0, "0")
        
        ttk.Label(input_frame, text="Duration (s):").grid(row=1, column=2, padx=2)
        self.queue_duration_entry = ttk.Entry(input_frame, width=8)
        self.queue_duration_entry.grid(row=1, column=3, padx=2)
        self.queue_duration_entry.insert(0, "2.0")
        
        queue_btn_frame = ttk.Frame(frame_queue)
        queue_btn_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Button(queue_btn_frame, text="Add", command=self.add_queue_command, width=8).pack(side="left", padx=2)
        ttk.Button(queue_btn_frame, text="Clear", command=self.clear_queue, width=8).pack(side="left", padx=2)
        ttk.Button(queue_btn_frame, text="Execute", command=self.execute_queue, width=8).pack(side="left", padx=2)
        ttk.Button(queue_btn_frame, text="Stop", command=self.stop_queue, width=8).pack(side="left", padx=2)
        
        # Added: sync queue to client button
        ttk.Button(queue_btn_frame, text="Sync", command=self.sync_queue_to_client, 
                  width=8, state="disabled").pack(side="left", padx=2)
        self.sync_button = queue_btn_frame.winfo_children()[-1]  # keep reference for enable/disable
        
        self.queue_status_label = ttk.Label(frame_queue, text="Queue: Idle | Commands: 0", 
                                           background="#f0f0f0", relief="solid")
        self.queue_status_label.pack(fill="x", padx=5, pady=3)
        
        self.queue_listbox = tk.Listbox(frame_queue, height=5, font=("Consolas", 9))
        self.queue_listbox.pack(fill="x", padx=5, pady=3)
        
        # Trajectory control
        frame_trajectory = ttk.LabelFrame(left_frame, text="Trajectory Control")
        frame_trajectory.pack(fill="x", padx=5, pady=3)
        
        traj_btn_frame = ttk.Frame(frame_trajectory)
        traj_btn_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Button(traj_btn_frame, text="Load CSV", command=self.load_trajectory, width=10).pack(side="left", padx=2)
        ttk.Button(traj_btn_frame, text="Save CSV", command=self.save_ideal_trajectory, width=10).pack(side="left", padx=2)
        ttk.Button(traj_btn_frame, text="Start correction", command=self.start_correction, width=10).pack(side="left", padx=2)
        ttk.Button(traj_btn_frame, text="Stop correction", command=self.stop_correction, width=10).pack(side="left", padx=2)
        ttk.Button(traj_btn_frame, text="Clear path", command=self.clear_trajectory, width=10).pack(side="left", padx=2)
        
        param_frame = ttk.Frame(frame_trajectory)
        param_frame.pack(fill="x", padx=5, pady=3)
        
        ttk.Label(param_frame, text="Strength:").grid(row=0, column=0, padx=2)
        self.correction_strength_scale = ttk.Scale(param_frame, from_=0.1, to=1.0, orient="horizontal", length=100)
        self.correction_strength_scale.set(0.7)
        self.correction_strength_scale.grid(row=0, column=1, padx=2)
        self.correction_strength_label = ttk.Label(param_frame, text="0.7", width=4)
        self.correction_strength_label.grid(row=0, column=2, padx=2)
        self.correction_strength_scale.bind("<Motion>", 
            lambda e: self.correction_strength_label.config(text=f"{self.correction_strength_scale.get():.1f}"))
        
        ttk.Label(param_frame, text="Speed:").grid(row=1, column=0, padx=2)
        self.base_speed_entry = ttk.Entry(param_frame, width=6)
        self.base_speed_entry.insert(0, "100")
        self.base_speed_entry.grid(row=1, column=1, padx=2)
        
        self.trajectory_stats_label = ttk.Label(frame_trajectory, 
            text="Trajectory: Not loaded | Points: 0 | Length: 0.00m",
            background="#f0f0f0", relief="solid")
        self.trajectory_stats_label.pack(fill="x", padx=5, pady=3)
        
        # ========== Right display panel ==========
        right_notebook = ttk.Notebook(right_frame)
        right_notebook.pack(fill="both", expand=True)
        
        trajectory_page = ttk.Frame(right_notebook)
        right_notebook.add(trajectory_page, text="Trajectory Visualization")
        
        self.trajectory_plotter = ExternalTrajectoryPlotter(trajectory_page, self.trajectory_tracker)
        
        status_page = ttk.Frame(right_notebook)
        right_notebook.add(status_page, text="Status Monitor")
        
        frame_status = ttk.LabelFrame(status_page, text="Device Status")
        frame_status.pack(fill="x", padx=5, pady=5)
        
        status_grid = ttk.Frame(frame_status)
        status_grid.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(status_grid, text="Connected:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.status_connected = ttk.Label(status_grid, text="Disconnected", foreground="red")
        self.status_connected.grid(row=0, column=1, sticky="w", padx=5, pady=2)
        
        ttk.Label(status_grid, text="Enabled:").grid(row=0, column=2, sticky="w", padx=5, pady=2)
        self.status_enabled = ttk.Label(status_grid, text="Disabled", foreground="red")
        self.status_enabled.grid(row=0, column=3, sticky="w", padx=5, pady=2)
        
        ttk.Label(status_grid, text="Mode:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.status_mode = ttk.Label(status_grid, text="Idle")
        self.status_mode.grid(row=1, column=1, sticky="w", padx=5, pady=2)
        
        ttk.Label(status_grid, text="Voltage:").grid(row=1, column=2, sticky="w", padx=5, pady=2)
        self.status_voltage = ttk.Label(status_grid, text="0.0V")
        self.status_voltage.grid(row=1, column=3, sticky="w", padx=5, pady=2)
        
        ttk.Label(status_grid, text="Position:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.status_pos = ttk.Label(status_grid, text="X: 0.000m | Y: 0.000m | θ: 0.0°")
        self.status_pos.grid(row=2, column=1, columnspan=3, sticky="w", padx=5, pady=2)
        
        ttk.Label(status_grid, text="Velocity:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        self.status_vel = ttk.Label(status_grid, text="Vx: 0 | Vy: 0 | ω: 0")
        self.status_vel.grid(row=3, column=1, columnspan=3, sticky="w", padx=5, pady=2)
        
        frame_corr = ttk.LabelFrame(status_page, text="Correction Status")
        frame_corr.pack(fill="x", padx=5, pady=5)
        
        self.corr_status_label = ttk.Label(frame_corr, 
            text="Correction: Not started | Progress: 0% | Error: 0.000m / 0.0°",
            background="#f0f0f0", relief="solid")
        self.corr_status_label.pack(fill="x", padx=5, pady=5)
        
        frame_log = ttk.LabelFrame(status_page, text="Comm Log")
        frame_log.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(frame_log, height=10, font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.status_bar = ttk.Label(self.root, text="Ready | Waiting for client connection...", relief="sunken", anchor="w")
        self.status_bar.pack(side="bottom", fill="x", padx=10, pady=5)
    
    # ========== New: queue sync methods ==========
    
    def sync_queue_to_client(self):
        """Sync current speed queue to UE client"""
        if not self.tcp_connected:
            messagebox.showwarning("Warning", "Client not connected")
            return
        
        if not self.speed_queue:
            messagebox.showwarning("Warning", "Queue is empty")
            return
        
        try:
            cmd = {
                'type': 'speed_queue_sync',
                'queue': self.speed_queue,
                'timestamp': time.time(),
                'server_trajectory_points': len(self.ideal_path) if self.ideal_path else 0
            }
            
            msg = json.dumps(cmd) + '\n'
            self.client_conn.sendall(msg.encode('utf-8'))
            
            self.log(f"[Queue Sync] Sent {len(self.speed_queue)} commands to UE client")
            
        except Exception as e:
            self.log(f"[Queue Sync] Failed: {e}")
            messagebox.showerror("Error", f"Sync failed: {e}")
    
    # ========== Packet loss simulation control methods ==========
    
    def _toggle_packet_loss(self):
        """Enable/disable packet loss simulation"""
        enabled = self.loss_enable_var.get()
        self.packet_loss_enabled = enabled
        
        if enabled:
            self.packet_loss_simulator.enable()
            prob = self.loss_prob_scale.get() / 100.0
            self.packet_loss_simulator.set_loss_probability(prob)
            self.log(f"[Packet loss simulation] Enabled, probability: {prob:.1%}")
        else:
            self.packet_loss_simulator.disable()
            self.log("[Packet loss simulation] Disabled")
    
    def _update_prob_label(self, event=None):
        """Update probability label display"""
        prob = self.loss_prob_scale.get()
        self.loss_prob_label.config(text=f"{prob:.1f}%")
    
    def _apply_prob_change(self, event=None):
        """Apply probability change"""
        prob = self.loss_prob_scale.get() / 100.0
        self.packet_loss_simulator.set_loss_probability(prob)
        if self.packet_loss_enabled:
            self.log(f"[Packet loss simulation] Probability adjusted to: {prob:.1%}")
    
    def _update_loss_direction(self):
        """Update packet loss direction"""
        direction = self.loss_direction_var.get()
        self.packet_loss_simulator.set_direction(direction)
        direction_text = {1: "TX", 2: "RX", 3: "Both"}.get(direction, "Unknown")
        if self.packet_loss_enabled:
            self.log(f"[Packet loss simulation] Direction switched to: {direction_text}")
    
    def _quick_set_loss_prob(self, prob):
        """Quick set packet loss rate"""
        self.loss_prob_scale.set(prob)
        self._update_prob_label()
        self._apply_prob_change()
        if not self.packet_loss_enabled:
            self.loss_enable_var.set(True)
            self._toggle_packet_loss()
        self.log(f"[Packet loss simulation] Quick set to: {prob}%")
    
    def _reset_loss_stats(self):
        """Reset packet loss statistics"""
        self.packet_loss_simulator.reset_stats()
        self.log("[Packet loss simulation] Statistics reset")
    
    def update_loss_stats(self):
        """Periodically update packet loss statistics display"""
        if hasattr(self, 'packet_loss_simulator'):
            stats = self.packet_loss_simulator.get_stats()
            
            tx_rate = stats['tx_loss_rate'] * 100
            rx_rate = stats['rx_loss_rate'] * 100
            
            tx_total = stats['total_tx_packets']
            tx_lost = stats['lost_tx_packets']
            rx_total = stats['total_rx_packets']
            rx_lost = stats['lost_rx_packets']
            
            status_text = f"Stats: TX {tx_lost}/{tx_total} ({tx_rate:.1f}%) | RX {rx_lost}/{rx_total} ({rx_rate:.1f}%)"
            
            set_prob = self.loss_prob_scale.get()
            if self.packet_loss_enabled and tx_total > 50:
                actual_tx = (tx_lost / max(tx_total, 1)) * 100
                if abs(actual_tx - set_prob) > 10:
                    status_text += f" ⚠Deviation {actual_tx-set_prob:+.1f}%"
            
            self.loss_stats_label.config(text=status_text)
        
        self.root.after(500, self.update_loss_stats)
    
    # ========== Queue control methods ==========
    
    def add_queue_command(self):
        """Add a command to the queue"""
        try:
            x = float(self.queue_x_entry.get())
            y = float(self.queue_y_entry.get())
            angular = float(self.queue_angular_entry.get())
            duration = float(self.queue_duration_entry.get())
            
            if duration <= 0:
                messagebox.showerror("Error", "Duration must be greater than 0")
                return
            
            x = max(-1000, min(1000, x))
            y = max(-500, min(500, y))
            angular = max(-1000, min(1000, angular))
            
            command = {
                'x_speed': x,
                'y_speed': y,
                'angular_speed': angular,
                'duration': duration
            }
            
            self.speed_queue.append(command)
            self.update_queue_display()
            self.log(f"Added queue command #{len(self.speed_queue)}: X={x}, Y={y}, duration={duration}s")
            
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
    
    def _generate_queue_trajectory(self):
        """Generate ideal trajectory"""
        if not self.speed_queue:
            return
        
        points = []
        
        if self.ue_state['pose']:
            current_x = self.ue_state['pose'].x
            current_y = self.ue_state['pose'].y
            current_theta = self.ue_state['pose'].theta
        else:
            current_x, current_y, current_theta = 0.0, 0.0, 0.0
        
        points.append(Point(current_x, current_y))
        
        dt = 0.05
        
        for cmd in self.speed_queue:
            duration = cmd['duration']
            vx = cmd['x_speed'] * 0.001
            vy = cmd['y_speed'] * 0.001
            omega = cmd['angular_speed'] * 0.001
            
            num_steps = int(duration / dt)
            remaining_time = duration - num_steps * dt
            
            for step in range(num_steps):
                theta_rad = math.radians(current_theta)
                
                if abs(omega) < 0.001:
                    current_x += vx * dt * math.cos(theta_rad) - vy * dt * math.sin(theta_rad)
                    current_y += vx * dt * math.sin(theta_rad) + vy * dt * math.cos(theta_rad)
                else:
                    delta_theta = math.degrees(omega * dt)
                    avg_theta_rad = math.radians(current_theta + delta_theta / 2)
                    current_x += vx * dt * math.cos(avg_theta_rad) - vy * dt * math.sin(avg_theta_rad)
                    current_y += vx * dt * math.sin(avg_theta_rad) + vy * dt * math.cos(avg_theta_rad)
                    current_theta += delta_theta
                    current_theta = ((current_theta + 180) % 360) - 180
                
                points.append(Point(current_x, current_y))
            
            if remaining_time > 0.001:
                theta_rad = math.radians(current_theta)
                if abs(omega) < 0.001:
                    current_x += vx * remaining_time * math.cos(theta_rad) - vy * remaining_time * math.sin(theta_rad)
                    current_y += vx * remaining_time * math.sin(theta_rad) + vy * remaining_time * math.cos(theta_rad)
                else:
                    delta_theta = math.degrees(omega * remaining_time)
                    avg_theta_rad = math.radians(current_theta + delta_theta / 2)
                    current_x += vx * remaining_time * math.cos(avg_theta_rad) - vy * remaining_time * math.sin(avg_theta_rad)
                    current_y += vx * remaining_time * math.sin(avg_theta_rad) + vy * remaining_time * math.cos(avg_theta_rad)
                    current_theta += delta_theta
                    current_theta = ((current_theta + 180) % 360) - 180
                points.append(Point(current_x, current_y))
        
        self.ideal_path = points
        try:
            self.path_correction_controller.set_target_path(points)
        except ValueError as e:
            self.log(f"Failed to set target path: {e}")
            return
        
        self.trajectory_plotter.set_ideal_trajectory(points)
        
        length = sum(points[i-1].distance_to(points[i]) for i in range(1, len(points)))
        self.trajectory_stats_label.config(
            text=f"Trajectory: Queue generated | Points: {len(points)} | Length: {length:.2f}m"
        )
        self.log(f"Generated queue ideal trajectory: {len(points)} points, {length:.2f}m")
    
    def clear_queue(self):
        """Clear the queue"""
        self.speed_queue.clear()
        self.queue_executing = False
        self.current_command_index = 0
        self.update_queue_display()
        self.log("Cleared queue")
    
    def execute_queue(self):
        """Execute the queue (modified: sync to client first)"""
        if not self.speed_queue:
            messagebox.showwarning("Warning", "Queue is empty")
            return
        
        if self.queue_executing:
            messagebox.showwarning("Warning", "Queue is already executing")
            return
        
        if not self.tcp_connected:
            messagebox.showerror("Error", "Client not connected")
            return
        
        # Key modification: sync queue to client first
        self.sync_queue_to_client()
        
        self._generate_queue_trajectory()
        
        self.queue_executing = True
        self.current_command_index = 0
        self.current_command_start_time = time.time()
        self.current_send_count = 0
        
        self.log(f"Starting queue execution, {len(self.speed_queue)} commands")
        self._execute_queue_step()
    
    def _execute_queue_step(self):
        """Execute a queue step (with correction and packet loss integrated)"""
        if not self.queue_executing:
            return
        
        if self.current_command_index >= len(self.speed_queue):
            self.finish_queue()
            return
        
        command = self.speed_queue[self.current_command_index]
        elapsed = time.time() - self.current_command_start_time
        
        if elapsed >= command['duration']:
            self.current_command_index += 1
            self.current_command_start_time = time.time()
            self.current_send_count = 0
            
            if self.current_command_index < len(self.speed_queue):
                self.log(f"Executing command {self.current_command_index + 1}/{len(self.speed_queue)}")
            
            self.update_queue_display()
            self.root.after(50, self._execute_queue_step)
            return
        
        # Calculate send speeds (with correction)
        x_speed = int(command['x_speed'])
        y_speed = int(command['y_speed'])
        angular_speed = int(command['angular_speed'])
        
        if self.correction_mode and self.ue_state['pose'] and self.path_correction_controller.correction_enabled:
            try:
                original_queue_finished = self.path_correction_controller.queue_finished
                self.path_correction_controller.queue_finished = False
                
                correction_cmds = self.path_correction_controller.calculate_correction_commands(self.ue_state['pose'])
                
                self.path_correction_controller.queue_finished = original_queue_finished
                
                if correction_cmds:
                    x_speed = int(command['x_speed'] + correction_cmds['x_speed'])
                    y_speed = int(command['y_speed'] + correction_cmds['y_speed'])
                    angular_speed = int(command['angular_speed'] + correction_cmds['angular_speed'])
                    
                    x_speed = max(-1000, min(1000, x_speed))
                    y_speed = max(-500, min(500, y_speed))
                    angular_speed = max(-1000, min(1000, angular_speed))
                    
                    if self.current_send_count % 20 == 0:
                        self.log(f"Queue correction: base=({command['x_speed']},{command['y_speed']}), "
                                f"adjust=({correction_cmds['x_speed']:.0f},{correction_cmds['y_speed']:.0f})")
            except Exception as e:
                self.log(f"Correction calculation error: {e}")
        
        # Send command (through packet loss simulator)
        if self.tcp_connected and self.client_conn:
            try:
                cmd = {
                    'type': 'motion',
                    'timestamp': time.time(),
                    'x_speed': x_speed,
                    'y_speed': y_speed,
                    'angular_speed': angular_speed
                }
                
                # Packet loss simulation (TX direction)
                if self.packet_loss_enabled:
                    is_lost, _, reason = self.packet_loss_simulator.simulate_tx_loss(cmd)
                    if is_lost:
                        if self.current_send_count % 10 == 0:
                            self.log(f"[Packet loss] TX packet lost: {reason}")
                        
                        # Important: mark packet loss and set the pending flag
                        self._pending_packet_loss_flag = True
                        self.current_send_count += 1
                        self.root.after(50, self._execute_queue_step)
                        return
                
                # Important: if there is a pending packet loss flag, add it to the command
                if self._pending_packet_loss_flag:
                    cmd['packet_loss_detected'] = True
                    self._pending_packet_loss_flag = False
                    self.log("[Packet loss] Notify UE client to enter local correction mode")
                
                msg = json.dumps(cmd) + '\n'
                self.client_conn.sendall(msg.encode('utf-8'))
                self.current_send_count += 1
                
            except Exception as e:
                self.log(f"Queue send error: {e}")
                self.queue_executing = False
                return
        
        self.root.after(50, self._execute_queue_step)
    
    def stop_queue(self):
        """Stop queue execution"""
        if self.queue_executing:
            self.queue_executing = False
            self._send_direct_command('motion', {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0})
            self.log("Stop queue execution")
            self.update_queue_display()
    
    def finish_queue(self):
        """Queue execution complete"""
        self.queue_executing = False
        
        if self.correction_mode and self.path_correction_controller.correction_enabled:
            self.path_correction_controller.queue_finished = True
            self.log("Queue execution complete, switching to standalone correction mode")
            
            if not self.path_correction_controller.correction_executing:
                self.path_correction_controller.correction_executing = True
                self.path_correction_controller.correction_execution_start_time = time.time()
        else:
            self._send_direct_command('motion', {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0})
        
        self.update_queue_display()
        messagebox.showinfo("Complete", "Queue execution finished, entering standalone correction mode" 
                           if (self.correction_mode and self.path_correction_controller.correction_enabled) 
                           else "Queue execution finished")
    
    def update_queue_display(self):
        """Update queue display"""
        if self.queue_executing:
            status = f"Executing [{self.current_command_index + 1}/{len(self.speed_queue)}]"
        else:
            status = "Idle"
        
        self.queue_status_label.config(text=f"Queue: {status} | Commands: {len(self.speed_queue)}")
        
        self.queue_listbox.delete(0, tk.END)
        for i, cmd in enumerate(self.speed_queue, 1):
            prefix = "▶ " if (self.queue_executing and i-1 == self.current_command_index) else "  "
            text = f"{prefix}#{i}: X={cmd['x_speed']:.0f}, Y={cmd['y_speed']:.0f}, ω={cmd['angular_speed']:.0f}, t={cmd['duration']:.1f}s"
            self.queue_listbox.insert(tk.END, text)
            
            if self.queue_executing and i-1 == self.current_command_index:
                self.queue_listbox.itemconfig(tk.END, {'bg': 'yellow'})
    
    # ========== TCP server methods (packet loss integrated) ==========
    
    def _server_loop(self):
        """TCP server main loop"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            
            self.log("TCP server started, waiting for connection...")
            
            while True:
                try:
                    self.server_socket.settimeout(1.0)
                    conn, addr = self.server_socket.accept()
                    
                    if self.client_conn:
                        try:
                            self.client_conn.close()
                        except:
                            pass
                    
                    self.client_conn = conn
                    self.client_addr = addr
                    self.tcp_connected = True
                    
                    self.log(f"Client connected: {addr}")
                    
                    # Enable sync button
                    if hasattr(self, 'sync_button'):
                        self.sync_button.config(state="normal")
                    
                    threading.Thread(target=self._receive_loop, daemon=True).start()
                    threading.Thread(target=self._send_loop, daemon=True).start()
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"Accept connection error: {e}")
                    
        except Exception as e:
            self.log(f"Server error: {e}")
    
    def _receive_loop(self):
        """Receive loop (packet loss integrated)"""
        try:
            while self.tcp_connected and self.client_conn:
                try:
                    self.client_conn.settimeout(1.0)
                    data = self.client_conn.recv(4096)
                    
                    if not data:
                        self.log("Client disconnected")
                        self.tcp_connected = False
                        # Disable sync button
                        if hasattr(self, 'sync_button'):
                            self.root.after(0, lambda: self.sync_button.config(state="disabled"))
                        break
                    
                    # Packet loss simulation (RX direction)
                    if self.packet_loss_enabled:
                        is_lost, processed_data, reason = self.packet_loss_simulator.simulate_rx_loss(data)
                        if is_lost:
                            continue
                        data = processed_data if processed_data is not None else data
                    
                    data_str = data.decode('utf-8', errors='ignore')
                    self.recv_buffer += data_str
                    
                    while '\n' in self.recv_buffer:
                        line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                        if line.strip():
                            self._process_ue_message(line.strip())
                            
                except socket.timeout:
                    continue
                except Exception as e:
                    self.log(f"Receive error: {e}")
                    break
        finally:
            self.tcp_connected = False
            if hasattr(self, 'sync_button'):
                self.root.after(0, lambda: self.sync_button.config(state="disabled"))
    
    def _send_loop(self):
        """Send loop (packet loss integrated)"""
        try:
            while self.tcp_connected and self.client_conn:
                try:
                    cmd = self._generate_control_command()
                    if cmd:
                        # Packet loss simulation (TX direction)
                        if self.packet_loss_enabled:
                            is_lost, processed_cmd, reason = self.packet_loss_simulator.simulate_tx_loss(cmd)
                            if is_lost:
                                time.sleep(0.05)
                                continue
                            cmd = processed_cmd if processed_cmd is not None else cmd
                        
                        msg = json.dumps(cmd) + '\n'
                        self.client_conn.sendall(msg.encode('utf-8'))
                    time.sleep(0.05)
                except Exception as e:
                    self.log(f"Send error: {e}")
                    break
        finally:
            self.tcp_connected = False
    
    def _process_ue_message(self, message):
        """Process UE message"""
        try:
            data = json.loads(message)
            if data.get('type') == 'ue_state':
                self._update_ue_state(data)
        except Exception as e:
            pass
    
    def _update_ue_state(self, state):
        """Update UE state"""
        self.last_ue_state_time = time.time()
        
        if 'pose' in state:
            pose_data = state['pose']
            current_pose = Pose(x=pose_data['x'], y=pose_data['y'], theta=pose_data.get('theta', 0))
            self.ue_state['pose'] = current_pose
            
            self.trajectory_tracker.pose = current_pose
            self.trajectory_tracker.trajectory.append(current_pose)
            self.trajectory_tracker.timestamps.append(time.time())
            
            if len(self.trajectory_tracker.trajectory) > self.trajectory_tracker.max_trajectory_points:
                keep_count = self.trajectory_tracker.max_trajectory_points // 2
                self.trajectory_tracker.trajectory = self.trajectory_tracker.trajectory[-keep_count:]
                self.trajectory_tracker.timestamps = self.trajectory_tracker.timestamps[-keep_count:]
        
        self.ue_state['vx'] = state.get('vx', 0)
        self.ue_state['vy'] = state.get('vy', 0)
        self.ue_state['vtheta'] = state.get('vtheta', 0)
        self.ue_state['enabled'] = state.get('enabled', False)
        self.ue_state['control_mode'] = state.get('control_mode', 0)
        self.ue_state['voltage'] = state.get('voltage', 0.0)
        
        # New: update client correction state
        self.ue_state['local_correction_enabled'] = state.get('local_correction_enabled', False)
        self.ue_state['trajectory_synced'] = state.get('trajectory_synced', False)
        
        # Update UI display
        self.root.after(0, self._update_client_status_display)
    
    def _update_client_status_display(self):
        """Update client status display"""
        corr_status = "Enabled" if self.ue_state['local_correction_enabled'] else "Disabled"
        sync_status = "Yes" if self.ue_state['trajectory_synced'] else "No"
        self.client_correction_label.config(
            text=f"UE correction: {corr_status} | Trajectory synced: {sync_status}",
            foreground="green" if self.ue_state['trajectory_synced'] else "orange"
        )
    
    def _generate_control_command(self):
        """Generate control command (standalone correction only)"""
        if self.queue_executing:
            return None
        
        if self.correction_mode and self.ue_state['pose'] and self.path_correction_controller.correction_enabled:
            if not self.path_correction_controller.queue_finished:
                if len(self.speed_queue) > 0 and not self.queue_executing:
                    self.path_correction_controller.queue_finished = True
            
            cmd = {
                'type': 'control_command',
                'timestamp': time.time(),
                'packet_loss_detected': False
            }
            
            current_pose = self.ue_state['pose']
            correction_cmds = self.path_correction_controller.calculate_correction_commands(current_pose)
            
            if correction_cmds:
                cmd['x_speed'] = int(correction_cmds['x_speed'])
                cmd['y_speed'] = int(correction_cmds['y_speed'])
                cmd['angular_speed'] = int(correction_cmds['angular_speed'])
                
                if hasattr(self, '_last_corr_log_time'):
                    if time.time() - self._last_corr_log_time > 1.0:
                        self.log(f"Standalone correction: X={cmd['x_speed']}, Y={cmd['y_speed']}, "
                                f"progress={self.path_correction_controller.get_progress()*100:.0f}%")
                        self._last_corr_log_time = time.time()
                else:
                    self._last_corr_log_time = time.time()
            else:
                cmd['x_speed'] = 0
                cmd['y_speed'] = 0
                cmd['angular_speed'] = 0
                if self.path_correction_controller.is_path_completed():
                    if self.correction_mode:
                        self.log("Standalone correction complete, target reached")
                        self.correction_mode = False
                
            cmd['control_mode'] = 'correction'
            cmd['buzzer'] = self.buzzer_var.get()
            cmd['brake'] = self.brake_var.get()
            cmd['special'] = self.special_var.get()
            return cmd
        
        cmd = {
            'type': 'control_command',
            'timestamp': time.time(),
            'packet_loss_detected': False,
            'x_speed': 0,
            'y_speed': 0,
            'angular_speed': 0,
            'control_mode': self.mode_var.get(),
            'buzzer': self.buzzer_var.get(),
            'brake': self.brake_var.get(),
            'special': self.special_var.get()
        }
        
        return cmd
    
    # ========== Control methods ==========
    
    def enable_device(self):
        self._send_direct_command('enable_device', {'enabled': True})
        self.log("Device enabled")
    
    def disable_device(self):
        self._send_direct_command('enable_device', {'enabled': False})
        self.log("Device disabled")
    
    def set_control_mode(self, mode):
        self._send_direct_command('set_mode', {'mode': mode})
        self.log(f"Set mode: {mode}")
    
    def send_motion_command(self):
        """Send motion command"""
        x = int(self.x_speed_scale.get())
        y = int(self.y_speed_scale.get())
        angular = int(self.angular_speed_scale.get())
        
        self._send_direct_command('motion', {
            'x_speed': x,
            'y_speed': y,
            'angular_speed': angular
        })
        self.log(f"Manual control: X={x}, Y={y}, θ={angular}")
    
    def emergency_stop(self):
        """Emergency stop"""
        self.stop_queue()
        self.x_speed_scale.set(0)
        self.y_speed_scale.set(0)
        self.angular_speed_scale.set(0)
        self.update_speed_display()
        
        self._send_direct_command('motion', {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0})
        self._send_direct_command('set_mode', {'mode': 0})
        self.log("Emergency stop!")
    
    def _send_direct_command(self, cmd_type, params):
        """Send direct command (with packet loss notification)"""
        if not self.tcp_connected or not self.client_conn:
            return
        
        cmd = {'type': cmd_type, 'timestamp': time.time(), **params}
        
        # Packet loss simulation (TX direction)
        if self.packet_loss_enabled:
            is_lost, processed_cmd, reason = self.packet_loss_simulator.simulate_tx_loss(cmd)
            if is_lost:
                self.log(f"[Packet loss] Command lost: {reason}")
                self._pending_packet_loss_flag = True  # Mark packet loss, notify client next time
                return
            cmd = processed_cmd if processed_cmd is not None else cmd
        
        # Key fix: if packet loss not reported, add flag
        if self._pending_packet_loss_flag:
            cmd['packet_loss_detected'] = True
            self._pending_packet_loss_flag = False
            self.log("[Packet loss] Notify UE client to enter local correction mode")
        
        try:
            msg = json.dumps(cmd) + '\n'
            self.client_conn.sendall(msg.encode('utf-8'))
        except Exception as e:
            self.log(f"Send failed: {e}")
    
    def update_speed_display(self):
        """Update speed display"""
        self.x_speed_label.config(text=str(int(self.x_speed_scale.get())))
        self.y_speed_label.config(text=str(int(self.y_speed_scale.get())))
        self.angular_speed_label.config(text=str(int(self.angular_speed_scale.get())))
    
    # ========== Trajectory methods ==========
    
    def load_trajectory(self):
        """Load trajectory"""
        from tkinter import filedialog
        filename = filedialog.askopenfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        
        if not filename:
            return
        
        try:
            points = []
            with open(filename, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                header = next(reader)
                
                x_idx = y_idx = None
                for i, col in enumerate(header):
                    if 'x' in col.lower() and ('coordinate' in col.lower() or 'pos' in col.lower()):
                        x_idx = i
                    if 'y' in col.lower() and ('coordinate' in col.lower() or 'pos' in col.lower()):
                        y_idx = i
                
                for row in reader:
                    try:
                        x = float(row[x_idx])
                        y = float(row[y_idx])
                        points.append(Point(x, y))
                    except:
                        continue
            
            if points:
                self.ideal_path = points
                try:
                    self.path_correction_controller.set_target_path(points)
                except ValueError as e:
                    self.log(f"Failed to set target path: {e}")
                    return
                
                self.trajectory_plotter.set_ideal_trajectory(points)
                
                length = sum(points[i-1].distance_to(points[i]) for i in range(1, len(points)))
                self.trajectory_stats_label.config(text=f"Trajectory: Loaded | Points: {len(points)} | Length: {length:.2f}m")
                self.log(f"Loaded trajectory: {len(points)} points, {length:.2f}m")
                
        except Exception as e:
            messagebox.showerror("Error", f"Load failed: {e}")
    
    def save_ideal_trajectory(self):
        """Save ideal trajectory to CSV file"""
        from tkinter import filedialog
        
        if not self.ideal_path:
            messagebox.showwarning("Warning", "No ideal trajectory to save. Load a trajectory or generate one from the queue first.")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Save ideal trajectory"
        )
        
        if not filename:
            return
        
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # Write header
                writer.writerow(['x_coordinate(m)', 'y_coordinate(m)'])
                # Write trajectory point data
                for point in self.ideal_path:
                    writer.writerow([f"{point.x:.6f}", f"{point.y:.6f}"])
            
            length = sum(self.ideal_path[i-1].distance_to(self.ideal_path[i]) for i in range(1, len(self.ideal_path)))
            self.log(f"Saved ideal trajectory: {len(self.ideal_path)} points, {length:.2f}m -> {filename}")
            messagebox.showinfo("Success", f"Trajectory saved:\n{filename}\n\nPoints: {len(self.ideal_path)}\nLength: {length:.2f}m")
            
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")
    
    def start_correction(self):
        """Start correction"""
        if not self.ideal_path:
            messagebox.showwarning("Warning", "Please load a trajectory or generate one from the queue first")
            return
        
        self.correction_mode = True
        
        try:
            self.base_speed = int(self.base_speed_entry.get())
        except:
            self.base_speed = 100
            
        self.path_correction_controller.correction_strength = self.correction_strength_scale.get()
        self.path_correction_controller.standalone_base_speed = self.base_speed
        
        if self.ideal_path:
            target = self.ideal_path[-1]
            self.path_correction_controller.target_position = target
        
        self.path_correction_controller.queue_finished = False
        self.path_correction_controller.correction_enabled = True
        self.path_correction_controller.correction_executing = True
        self.path_correction_controller.correction_execution_start_time = time.time()
        self.path_correction_controller.path_completed = False
        
        self.log(f"Start correction, base speed: {self.base_speed}mm/s, strength: {self.correction_strength_scale.get():.1f}")
    
    def stop_correction(self):
        self.correction_mode = False
        self.path_correction_controller.disable_correction()
        self.log("Stop correction")
    
    def clear_trajectory(self):
        self.trajectory_tracker.trajectory.clear()
        self.trajectory_plotter.clear_trajectory()
        self.log("Cleared actual trajectory")
    
    # ========== Update methods ==========
    
    def update_gui(self):
        """Update GUI"""
        try:
            if self.tcp_connected:
                self.tcp_status_label.config(text="Connected", foreground="green")
                self.status_connected.config(text="Connected", foreground="green")
                if self.client_addr:
                    self.client_info_label.config(text=f"Client: {self.client_addr[0]}:{self.client_addr[1]}")
            else:
                self.tcp_status_label.config(text="Waiting for connection...", foreground="orange")
                self.status_connected.config(text="Disconnected", foreground="red")
                self.client_info_label.config(text="Client: None")
                self.client_correction_label.config(text="UE correction: Unknown | Trajectory synced: No", foreground="gray")
            
            if self.ue_state['enabled']:
                self.status_enabled.config(text="Enabled", foreground="green")
            else:
                self.status_enabled.config(text="Disabled", foreground="red")
            
            modes = {0: 'Idle', 1: 'Remote', 2: 'CAN Control', 3: 'Follow'}
            self.status_mode.config(text=modes.get(self.ue_state['control_mode'], 'Unknown'))
            self.status_voltage.config(text=f"{self.ue_state['voltage']:.1f}V")
            
            if self.ue_state['pose']:
                pose = self.ue_state['pose']
                self.status_pos.config(text=f"X: {pose.x:.3f}m | Y: {pose.y:.3f}m | θ: {math.degrees(pose.theta):.1f}°")
            
            self.status_vel.config(text=f"Vx: {self.ue_state['vx']} | Vy: {self.ue_state['vy']} | ω: {self.ue_state['vtheta']}")
            
            if self.correction_mode:
                progress = self.path_correction_controller.get_progress() * 100
                mode_text = "Standalone correction" if self.path_correction_controller.queue_finished else "Queue correction"
                self.corr_status_label.config(text=f"Correction: {mode_text} | Progress: {progress:.0f}%")
            
            if self.tcp_connected:
                self.status_bar.config(text=f"Running | Client: {self.client_addr[0] if self.client_addr else 'Unknown'}")
            else:
                self.status_bar.config(text="Waiting for client connection...")
                
        except Exception:
            pass
        
        self.root.after(100, self.update_gui)
    
    def check_connection_status(self):
        if self.tcp_connected and time.time() - self.last_ue_state_time > 3:
            self.status_bar.config(text="Warning: no data received for 3 seconds")
        self.root.after(1000, self.check_connection_status)
    
    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        
        content = self.log_text.get("1.0", "end-1c")
        lines = content.split("\n")
        if len(lines) > 200:
            self.log_text.delete("1.0", f"{len(lines)-200}.0")
    
    def on_closing(self):
        self.running = False
        if self.trajectory_plotter:
            self.trajectory_plotter.cleanup()
        if self.client_conn:
            try:
                self.client_conn.close()
            except:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = StationServerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()