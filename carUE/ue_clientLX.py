# ue_clientLX_FULL.py - Full version (includes CAN initialization, enable, and mode selection)
# No initialHB2.py needed; run directly!

import socket
import threading
import time
import json
import csv
import argparse
import sys
import math
import numpy as np
from ctypes import *
from car_control import CarController
from trajectory_tracker import TrajectoryTracker
from path_correction import PathCorrectionController, Point
from config import Config

# ===== CAN related constants =====
VCI_USBCAN2 = 4
STATUS_OK = 1


class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint),
                ("AccMask", c_uint),
                ("Reserved", c_uint),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)
                ]


class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_uint),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte * 8),
                ("Reserved", c_ubyte * 3)
                ]


class CANHandler:
    """CAN hardware handler - full initialization, enable, and mode selection"""

    def __init__(self):
        self.canDLL = cdll.LoadLibrary('/home/ubuntu22/controlcan/libcontrolcan.so')
        self.is_connected = False
        self.receiving = False

    def init_can(self):
        """🔧 Initialize CAN hardware (copied from initialHB2)"""
        print('\n🔧 Starting CAN initialization...')

        ret = self.canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('  ✓ VCI_OpenDevice succeeded')
        else:
            print('  ✗ VCI_OpenDevice failed')
            return False

        vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0,
                                         0, 0x00, 0x1C, 0)
        ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret == STATUS_OK:
            print('  ✓ VCI_InitCAN(channel 0) succeeded')
        else:
            print('  ✗ VCI_InitCAN(channel 0) failed')
            return False

        ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('  ✓ VCI_StartCAN(channel 0) succeeded')
        else:
            print('  ✗ VCI_StartCAN(channel 0) failed')
            return False

        ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_initconfig))
        if ret == STATUS_OK:
            print('  ✓ VCI_InitCAN(channel 1) succeeded')
        else:
            print('  ✗ VCI_InitCAN(channel 1) failed')
            return False

        ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)
        if ret == STATUS_OK:
            print('  ✓ VCI_StartCAN(channel 1) succeeded')
        else:
            print('  ✗ VCI_StartCAN(channel 1) failed')
            return False

        print('✅ CAN initialization complete\n')
        return True

    def enable_device_and_set_mode(self):
        """🔧 Enable the device and set CAN control mode (copied from initialHB2)"""
        print('🔧 Starting device enable and mode setup...')

        # Step 1: enable device (function code 0x03)
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}03"
        data = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}01"
        frame = self._build_can_frame(can_id, data)
        self._send_frame(frame, 0)
        print('  ✓ Device enabled')
        time.sleep(0.1)

        # Step 2: switch to CAN control mode (function code 0x11)
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}11"
        data = "02000000"  # mode=0x02(CAN control), buzzer=0x00, brake=0x00, special=0x00
        frame = self._build_can_frame(can_id, data)
        self._send_frame(frame, 0)
        print('  ✓ Switched to CAN control mode')
        print('✅ Device enable and mode setup complete\n')

    def _build_can_frame(self, can_id_hex, data_hex):
        """🔧 Build CAN transmit frame (copied from initialHB2)"""
        try:
            data_str = data_hex.replace(" ", "").upper()
            if len(data_str) % 2 != 0:
                data_str = data_str + "0"
            data_len = len(data_str) // 2

            if data_len > 0x0F:
                raise ValueError(f"Data length {data_len} out of range (0-15)")

            byte1 = 0x80 | (data_len & 0x0F)

            channel = Config.CAN_CHANNEL
            channel_high = (channel >> 1) & 0x03
            byte2 = (channel_high << 3)

            if Config.CAN_IS_EXTENDED:
                byte2 |= (1 << 2)

            can_id = int(can_id_hex, 16)
            id_bytes = can_id.to_bytes(4, byteorder='big')
            data_bytes = bytes.fromhex(data_str)

            frame_bytes = bytes([Config.FRAME_HEADER, byte1, byte2]) + id_bytes + data_bytes + bytes(
                [Config.FRAME_TAIL])

            return frame_bytes.hex().upper()
        except Exception as e:
            print(f"CAN frame build error: {e}")
            return None

    def _send_frame(self, frame_hex, channel=0):
        """🔧 Send CAN frame (copied from initialHB2)"""
        try:
            frame_bytes = bytes.fromhex(frame_hex)

            tx_obj = VCI_CAN_OBJ()

            byte1 = frame_bytes[1]
            dlc = byte1 & 0x0F

            can_id_bytes = frame_bytes[3:7]
            can_id = int.from_bytes(can_id_bytes, byteorder='big')

            tx_obj.ID = can_id
            tx_obj.DataLen = dlc
            tx_obj.ExternFlag = 1
            tx_obj.RemoteFlag = 0
            tx_obj.SendType = 0

            data = frame_bytes[7:7 + dlc]
            for i in range(dlc):
                tx_obj.Data[i] = data[i]

            ret = self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, channel, byref(tx_obj), 1)
            return ret == STATUS_OK
        except Exception as e:
            print(f'发送错误: {e}')
            return False

    def connect(self):
        """Connect and initialize CAN (init + enable + mode)"""
        try:
            # ✅ Step 1: init CAN
            if not self.init_can():
                return False

            # ✅ Step 2: enable device and set CAN control mode
            self.enable_device_and_set_mode()

            self.is_connected = True
            print('✅ CAN fully initialized and ready\n')
            return True
        except Exception as e:
            print(f'✗ CAN connection failed: {e}')
            return False

    def send_hex_string(self, frame_hex):
        """Send CAN frame"""
        if not self.is_connected:
            return False

        try:
            return self._send_frame(frame_hex, 0)
        except Exception as e:
            print(f'Send error: {e}')
            return False

    def start_receive(self, callback):
        """Start receive thread"""
        self.receiving = True
        receive_thread = threading.Thread(target=self._receive_loop, args=(callback,), daemon=False)
        receive_thread.start()

    def _receive_loop(self, callback):
        """Receive loop"""
        rx_vci_can_obj = (VCI_CAN_OBJ * 2500)()

        while self.receiving:
            ret = self.canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj), 2500, 100)

            if ret > 0:
                for i in range(ret):
                    obj = rx_vci_can_obj[i]
                    data = bytes([obj.Data[j] for j in range(obj.DataLen)])

                    frame = self._parse_can_frame(obj.ID, obj.DataLen, data)
                    if frame:
                        callback(frame)
            else:
                time.sleep(0.01)

    def _parse_can_frame(self, can_id, dlc, data):
        """Parse CAN frame format"""
        try:
            if dlc < 1:
                return None

            # Function code is in the lowest byte of CAN ID (consistent with sender frame logic)
            function_code = can_id & 0xFF

            frame = {
                'can_id': can_id,
                'dlc': dlc,
                'function_code': function_code,
                'data': list(data)
            }

            return frame
        except:
            return None

    def stop_receive(self):
        """Stop receiving"""
        self.receiving = False

    def disconnect(self):
        """Close CAN"""
        self.receiving = False
        self.is_connected = False
        try:
            self.canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
        except:
            pass
        print('✓ CAN 已关闭')


# ========== UEClient class ==========
class UEClient:
    def __init__(self, station_host='10.45.0.1', station_port=5000,
                 csv_trajectory_file=None,
                 enable_local_correction=None,
                 correction_delay=0.0,
                 trajectory_density=0.4,
                 base_speed=200,
                 max_correction_time=120.0):

        # TCP parameters
        self.station_host = station_host
        self.station_port = station_port
        self.socket = None
        self.tcp_connected = False
        self.recv_buffer = ""

        # CAN parameters
        self.can_handler = CANHandler()
        self.car_controller = CarController()

        # Local correction configuration
        self.local_correction_enabled = enable_local_correction
        self.correction_delay = correction_delay
        self.trajectory_density = trajectory_density
        self.base_speed = base_speed
        self.max_correction_time = max_correction_time

        # Key distinction
        self.has_correction_delay = (correction_delay > 0)

        # Linux file path
        if csv_trajectory_file is None:
            self.csv_trajectory_file = "/home/ubuntu22/carue/ideal.csv"
        else:
            self.csv_trajectory_file = csv_trajectory_file

        # Status
        self.current_pose = None
        self.latest_frame = None
        self.enabled = False
        self.control_mode = 0

        self._last_base_station_cmd = {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0}
        self.packet_loss_detected = False
        self.standalone_correction_active = False
        self.correction_thread = None
        self.correction_locked = True
        self._correction_started_flag = False
        self._full_target_path = None

        self.trajectory_synced = False
        self.received_speed_queue = []
        self.server_trajectory_points = 0

        self.running = False
        self.last_control_cmd = None

        self.trajectory_tracker = TrajectoryTracker(scale_factor=0.001)
        self.path_correction_controller = PathCorrectionController(self.trajectory_tracker)

        self._setup_correction_controller()

    def _setup_correction_controller(self):
        """Configure correction controller parameters"""
        self.path_correction_controller.standalone_base_speed = self.base_speed
        self.path_correction_controller.max_correction_time = self.max_correction_time
        self.path_correction_controller.lookahead_distance = 0.8
        self.path_correction_controller.target_arrival_threshold = 0.08

    def interactive_setup(self):
        """Interactive setup for local correction parameters"""
        print("\n" + "=" * 70)
        print("🔧 Local correction setup (forced completion mode)")
        if self.has_correction_delay:
            print("   ⛔ Delay mode: execute last command -> overshoot -> correct toward a point 0.3m ahead (less accurate)")
        else:
            print("   ✅ No-delay mode: switch immediately -> precise tracking (better)")
        print("=" * 70)

        if self.local_correction_enabled is None:
            while True:
                choice = input("\nEnable local correction? (y/n): ").strip().lower()
                if choice in ['y', 'yes', '是', '1']:
                    self.local_correction_enabled = True
                    break
                elif choice in ['n', 'no', '否', '0']:
                    self.local_correction_enabled = False
                    break
                else:
                    print("❌ Please enter y or n")
        else:
            print(f"Command-line option used: local correction={'enabled' if self.local_correction_enabled else 'disabled'}")

        if not self.local_correction_enabled:
            return

        # Configure base speed
        print(f"\n⚡ Local correction base speed (current: {self.base_speed}mm/s)")
        speed_input = input("   Enter speed (mm/s, press Enter for 200): ").strip()
        if speed_input:
            try:
                self.base_speed = int(speed_input)
                self.path_correction_controller.standalone_base_speed = self.base_speed
            except:
                pass

        # Configure delay
        if self.has_correction_delay:
            print(f"\n📍 Delay execution settings (current: {self.correction_delay:.1f}s)")
            print("   Behavior: continue last command -> overshoot -> skip nearby points -> correct toward a point 0.3m ahead")
            delay_input = input("   Enter delay time (s): ").strip()
            if delay_input:
                try:
                    self.correction_delay = float(delay_input)
                    self.has_correction_delay = (self.correction_delay > 0)
                except:
                    pass

        # Configure density
        print("\n📉 Trajectory point downsampling settings")
        print(f"   Current default: {self.trajectory_density * 100:.0f}%")
        while True:
            density_input = input("   Enter keep ratio (0.1-1.0, press Enter for default): ").strip()
            if density_input == "":
                break
            try:
                self.trajectory_density = float(density_input)
                if 0 < self.trajectory_density <= 1.0:
                    break
            except:
                print("   ❌ Please enter a valid number")

        # Configure max correction time
        print(f"\n⏱️  Max correction time (current: {self.max_correction_time}s)")
        time_input = input("   Enter max time (s, press Enter for 120): ").strip()
        if time_input:
            try:
                self.max_correction_time = float(time_input)
                self.path_correction_controller.max_correction_time = self.max_correction_time
            except:
                pass

        # Summary
        print("\n" + "=" * 70)
        print("📋 Configuration summary:")
        mode_str = "⛔ Delay mode (correct toward 0.3m ahead)" if self.has_correction_delay else "✅ No-delay mode (precise)"
        print(f"   Operation mode: {mode_str}")
        print(f"   Forced completion: ✅ Yes")
        print(f"   Base speed: {self.base_speed} mm/s")
        if self.has_correction_delay:
            print(f"   Delay execution: {self.correction_delay:.1f}s (default 100mm/s)")
            print(f"   Correction method: correct toward a point 0.3m ahead on the X-axis path (avoid spinning)")
        print("=" * 70 + "\n")

    def downsample_trajectory(self, points, density=None):
        """Downsample trajectory points"""
        if density is None:
            density = self.trajectory_density

        if density >= 1.0 or len(points) <= 2:
            return points

        target_count = max(2, int(len(points) * density))
        indices = np.linspace(0, len(points) - 1, target_count, dtype=int)
        downsampled = [points[i] for i in indices]

        if downsampled[-1] != points[-1]:
            downsampled[-1] = points[-1]

        return downsampled

    def generate_trajectory_from_queue(self, speed_queue):
        """Generate local trajectory"""
        if not speed_queue:
            print("⚠️  Speed queue is empty, cannot generate trajectory")
            return False

        print(f"\n🔄 Generating local trajectory:")
        print(f"   Received command count: {len(speed_queue)}")
        print(f"   Local trajectory density: {self.trajectory_density * 100:.0f}%")

        try:
            converted_queue = []
            for cmd in speed_queue:
                converted_cmd = {
                    'x': float(cmd.get('x_speed', cmd.get('x', 0))),
                    'y': float(cmd.get('y_speed', cmd.get('y', 0))),
                    'angular': float(cmd.get('angular_speed', cmd.get('angular', 0))),
                    'duration': float(cmd.get('duration', 1.0))
                }
                converted_queue.append(converted_cmd)

            points_per_cmd = max(1, int(5 * self.trajectory_density))

            self.path_correction_controller.set_ideal_trajectory_from_queue(
                converted_queue,
                density_mode="fixed",
                points_per_command=points_per_cmd
            )

            original_points = self.path_correction_controller.target_path
            print(f"   Original point count: {len(original_points)}")

            if self.trajectory_density < 0.5 and len(original_points) > 10:
                downsampled = self.downsample_trajectory(original_points)
                self.path_correction_controller.set_target_path(downsampled)
                print(f"   After second downsampling: {len(downsampled)} points")

            points = self.path_correction_controller.target_path
            if points:
                length = sum(p1.distance_to(p2) for p1, p2 in zip(points[:-1], points[1:]))
                print(f"✅ Local trajectory generated successfully")
                print(f"   Start point: {points[0]}")
                print(f"   End point: {points[-1]}")
                print(f"   Total length: {length:.2f}m")

            # Save full target path (for delay mode)
            self._full_target_path = self.path_correction_controller.target_path.copy()

            self.trajectory_synced = True
            self.received_speed_queue = speed_queue
            return True

        except Exception as e:
            print(f"❌ Local trajectory generation failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _load_trajectory_from_csv_backup(self):
        """Load trajectory backup from CSV"""
        print(f"🔄 Attempting to load backup from CSV: {self.csv_trajectory_file}")
        try:
            speed_queue = []
            with open(self.csv_trajectory_file, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                header = next(reader)

                x_idx = y_idx = angular_idx = duration_idx = None

                for i, col in enumerate(header):
                    col_lower = col.lower().strip()
                    if x_idx is None and ('x' in col_lower or 'vx' in col_lower):
                        if any(k in col_lower for k in ['speed', 'vel', 'velocity']) or col_lower in ['x', 'vx']:
                            x_idx = i
                    if y_idx is None and ('y' in col_lower or 'vy' in col_lower):
                        if any(k in col_lower for k in ['speed', 'vel', 'velocity']) or col_lower in ['y', 'vy']:
                            y_idx = i
                    if angular_idx is None and any(k in col_lower for k in ['angular', 'omega', 'ω', 'angle']):
                        angular_idx = i
                    if duration_idx is None and any(k in col_lower for k in ['duration', 'time', '时长', 'dt']):
                        duration_idx = i

                if x_idx is None:
                    for i, col in enumerate(header):
                        col_lower = col.lower()
                        if 'x' in col_lower and any(k in col_lower for k in ['坐标', 'pos', 'position']):
                            print("   ⚠️  Detected coordinate-format CSV, will try conversion...")
                            return self._load_trajectory_from_points_backup()

                if x_idx is None:
                    print(f"   ❌ Could not identify CSV columns")
                    return False

                for row in reader:
                    try:
                        max_idx = max([i for i in [x_idx, y_idx, angular_idx, duration_idx] if i is not None])
                        if len(row) <= max_idx:
                            continue

                        cmd = {
                            'x_speed': float(row[x_idx]) if x_idx is not None else 0,
                            'y_speed': float(row[y_idx]) if y_idx is not None else 0,
                            'angular_speed': float(row[angular_idx]) if angular_idx is not None else 0,
                            'duration': float(row[duration_idx]) if duration_idx is not None else 1.0
                        }
                        speed_queue.append(cmd)
                    except:
                        continue

            if speed_queue:
                print(f"   Read {len(speed_queue)} commands from CSV")
                return self.generate_trajectory_from_queue(speed_queue)
            else:
                print("   ❌ CSV is empty")
                return False

        except FileNotFoundError:
            print(f"   ❌ File not found: {self.csv_trajectory_file}")
            return False
        except Exception as e:
            print(f"   ❌ Failed to load CSV: {e}")
            return False

    def _load_trajectory_from_points_backup(self):
        """Load from coordinate points CSV"""
        try:
            points = []
            with open(self.csv_trajectory_file, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                header = next(reader)

                x_idx = y_idx = None
                for i, col in enumerate(header):
                    col_lower = col.lower()
                    if 'x' in col_lower and any(k in col_lower for k in ['坐标', 'pos', 'position']):
                        x_idx = i
                    if 'y' in col_lower and any(k in col_lower for k in ['坐标', 'pos', 'position']):
                        y_idx = i

                for row in reader:
                    try:
                        x = float(row[x_idx])
                        y = float(row[y_idx])
                        points.append(Point(x, y))
                    except:
                        continue

            if len(points) < 2:
                return False

            speed_queue = []
            dt = 0.1
            for i in range(len(points) - 1):
                dx = points[i + 1].x - points[i].x
                dy = points[i + 1].y - points[i].y
                vx = (dx / dt) * 1000
                vy = (dy / dt) * 1000
                speed_queue.append({
                    'x_speed': vx,
                    'y_speed': vy,
                    'angular_speed': 0,
                    'duration': dt
                })

            if speed_queue:
                return self.generate_trajectory_from_queue(speed_queue)
            return False

        except Exception as e:
            return False

    def print_status(self):
        """Print current status"""
        print("\n" + "=" * 60)
        print("🔵 UE Client status")
        print(f"   Local correction: {'✅ enabled' if self.local_correction_enabled else '❌ disabled'}")
        if self.local_correction_enabled:
            mode_str = "⛔ delay (correct toward 0.3m ahead)" if self.has_correction_delay else "✅ no-delay (precise)"
            print(f"   Operation mode: {mode_str}")
            print(f"   Forced completion: ✅ yes")
            print(f"   Base speed: {self.base_speed} mm/s")
            status = "local correction active" if self.standalone_correction_active else "idle"
            print(f"   Current state: {status}")
            if self.has_correction_delay:
                print(f"   Delay execution: {self.correction_delay:.1f}s (default 100mm/s)")
        print("=" * 60 + "\n")

    def connect_to_station(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.station_host, self.station_port))
            self.socket.settimeout(1.0)
            self.tcp_connected = True
            print(f"✅ Connected to station {self.station_host}:{self.station_port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to station: {e}")
            return False

    def connect_can(self):
        """Connect CAN device (includes init, enable, mode setup)"""
        try:
            if self.can_handler.connect():  # ✅ auto init + enable + set_mode
                print(f"✅ CAN fully initialized, enabled, and mode set")
                self.can_handler.start_receive(self._on_can_frame_received)
                return True
            else:
                return False
        except Exception as e:
                print(f"CAN connection failed: {e}")

    def _on_can_frame_received(self, frame):
        """Process received CAN frame"""
        try:
            function_code = frame.get('function_code')

            if function_code == 0xB2:
                self.latest_frame = frame
                # Parse velocity/angle from chassis motion info for trajectory tracker integration
                data = frame.get('data', [])
                parsed_motion = self._parse_motion_data(data)
                if parsed_motion:
                    self.trajectory_tracker.update_from_frame(parsed_motion)
                else:
                    self.trajectory_tracker.update_from_frame(frame)
                self.current_pose = self.trajectory_tracker.get_current_pose()
            elif function_code == 0xB0:
                data = frame.get('data', [])
                if len(data) >= 1:
                    self.enabled = (data[0] == 0x01)
            elif function_code == 0xB1:
                data = frame.get('data', [])
                if len(data) >= 2:
                    self.control_mode = data[1]

        except Exception as e:
            pass

    def _parse_motion_data(self, data):
        """Parse vx/vy/vtheta/theta from 0xB2 chassis motion info frame"""
        if len(data) not in (6, 8):
            return None
        try:
            def parse_int16_le(low, high):
                val = (high << 8) | low
                return val - 0x10000 if val >= 0x8000 else val

            vx = parse_int16_le(data[0], data[1])          # mm/s
            vy = parse_int16_le(data[2], data[3])          # mm/s
            vtheta_raw = parse_int16_le(data[4], data[5])  # 0.1°/s
            vtheta_mrad = vtheta_raw * 1.74533             # mrad/s

            result = {
                'vx': vx,
                'vy': vy,
                'vtheta': vtheta_mrad,
            }
            if len(data) == 8:
                result['theta'] = parse_int16_le(data[6], data[7])  # degrees
            return result
        except Exception:
            return None

    def start(self):
        self.interactive_setup()

        if not self.connect_to_station():
            return False

        if not self.connect_can():
            print("⚠️  CAN connection failed")
            return False

        if self.local_correction_enabled:
            print("⏳ Waiting for server speed queue sync...")

        self.running = True
        self.print_status()

        threads = [
            threading.Thread(target=self._tcp_receive_loop, daemon=True, name="TCPRecv"),
            threading.Thread(target=self._tcp_send_loop, daemon=True, name="TCPSend"),
            threading.Thread(target=self._keyboard_monitor, daemon=True, name="Keyboard"),
        ]

        for t in threads:
            t.start()

        print("✅ Client running...")
        if self.has_correction_delay:
            print("⛔ Delay mode: execute last command -> overshoot -> correct toward a point 0.3m ahead (avoid spinning)")
        else:
            print("✅ No-delay mode: switch immediately -> precise tracking (better)")
        print("💡 Tip: press 'h' for commands, 'x' to stop\n")

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Stop signal received")
        finally:
            self.stop()

    def _keyboard_monitor(self):
        """Keyboard monitor - Linux adapted version"""
        if sys.platform.startswith('linux'):
            import select
            while self.running:
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        char = sys.stdin.read(1)
                        if char:
                            self._handle_key(char)
                except:
                    time.sleep(0.5)
        else:
            while self.running:
                try:
                    import msvcrt
                    if msvcrt.kbhit():
                        char = msvcrt.getch().decode('utf-8', errors='ignore')
                        self._handle_key(char)
                    time.sleep(0.1)
                except:
                    time.sleep(0.5)

    def _handle_key(self, char):
        """Handle keyboard input"""
        if char == 'h':
            print("\n📋 Available commands:")
            print("   h - show help")
            print("   s - show status")
            print("   p - show trajectory info")
            print("   c - force start local correction")
            print("   x - stop local correction")
            print("   q - quit program\n")

        elif char == 's':
            self.print_status()

        elif char == 'p' and self.local_correction_enabled:
            self._print_trajectory_info()

        elif char == 'c' and self.local_correction_enabled:
            print("🔧 Manual force start local correction")
            self._start_local_correction()

        elif char == 'x':
            if self.standalone_correction_active:
                print("🔧 Manual stop local correction")
                self._stop_local_correction()

        elif char == 'q':
            self.running = False

    def _print_trajectory_info(self):
        """Print trajectory info"""
        if not self.path_correction_controller.target_path:
            print("❌ No trajectory loaded")
            return

        path = self.path_correction_controller.target_path
        print("\n📍 Trajectory info:")
        print(f"   Total points: {len(path)}")
        print(f"   Start point: {path[0]}")
        print(f"   End point: {path[-1]}")

    def _tcp_receive_loop(self):
        while self.running and self.tcp_connected:
            try:
                data = self.socket.recv(4096)
                if not data:
                    print("[TCP receive] Station disconnected")
                    self.tcp_connected = False
                    break

                data_str = data.decode('utf-8', errors='ignore')
                self.recv_buffer += data_str

                while '\n' in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line.strip():
                        self._process_command(line.strip())

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP receive] Error: {e}")
                break

        self.tcp_connected = False

    def _process_command(self, message):
        try:
            cmd = json.loads(message)
            cmd_type = cmd.get('type')

            if cmd_type == 'speed_queue_sync':
                print("\n📥 Received server speed queue sync")
                queue = cmd.get('queue', [])
                self.server_trajectory_points = cmd.get('server_trajectory_points', 0)

                if self.local_correction_enabled:
                    success = self.generate_trajectory_from_queue(queue)
                    if success:
                        print("✅ Local trajectory is ready")
                    else:
                        self._load_trajectory_from_csv_backup()
                return

            # ===== New: handle control_command (independent correction command) =====
            if cmd_type == 'control_command':
                # Extract motion parameters
                motion_params = {
                    'x_speed': cmd.get('x_speed', 0),
                    'y_speed': cmd.get('y_speed', 0),
                    'angular_speed': cmd.get('angular_speed', 0)
                }

                # If correction speed is nonzero, send to CAN
                if any(motion_params.values()):
                    print(f"[Independent correction] Received correction command: {motion_params}")
                    self._send_can_command('motion', motion_params)

            packet_loss = cmd.get('packet_loss_detected', False)

            if not packet_loss and not self.standalone_correction_active:
                if cmd_type == 'motion':
                    self._last_base_station_cmd = {
                        'x_speed': cmd.get('x_speed', 0),
                        'y_speed': cmd.get('y_speed', 0),
                        'angular_speed': cmd.get('angular_speed', 0)
                    }

            if packet_loss:
                if not self.packet_loss_detected:
                    print(f"🔴 Station reported packet loss!")
                    if self.local_correction_enabled:
                        if not self.trajectory_synced:
                            self._load_trajectory_from_csv_backup()

                        if self.has_correction_delay:
                            print(f"   ⛔ Delay mode: continue last command for {self.correction_delay}s -> overshoot")
                            print(
                                f"   📌 Last command: X={self._last_base_station_cmd['x_speed']}, Y={self._last_base_station_cmd['y_speed']}")
                        else:
                            print(f"   ✅ No-delay mode: switch to local correction immediately")

                        if not self._correction_started_flag:
                            self._start_local_correction()

                    self.packet_loss_detected = True

            else:
                if self.packet_loss_detected:
                    print("✅ Station packet loss cleared")
                    self.packet_loss_detected = False

                    if self.standalone_correction_active:
                        print("   🔒 Forced mode: ignore station recovery and continue local correction until complete")

            if self.standalone_correction_active:
                if cmd_type == 'motion' and cmd.get('x_speed') == 0 and cmd.get('y_speed') == 0:
                    print("[Local correction] Emergency stop command received, stopping")
                    self._stop_local_correction()
                elif cmd_type not in ['enable_device', 'set_mode']:
                    pass  # ignore station instructions
            else:
                if cmd_type == 'motion':
                    self._send_can_command('motion', cmd)
                elif cmd_type in ['enable_device', 'set_mode', 'set_aux']:
                    self._send_can_command(cmd_type, cmd)

        except Exception as e:
            print(f"[Command processing] Error: {e}")

    def _start_local_correction(self):
        """Start local correction"""
        if self._correction_started_flag:
            return

        self._correction_started_flag = True
        self.standalone_correction_active = True
        self.correction_locked = False

        self.path_correction_controller.enable_correction()
        self.path_correction_controller.mark_queue_finished()

        if self.has_correction_delay:
            self.correction_thread = threading.Thread(
                target=self._enter_buffer_mode,
                args=(self.correction_delay,),
                daemon=True
            )
        else:
            self.correction_thread = threading.Thread(
                target=self._correction_loop,
                daemon=True
            )

        self.correction_thread.start()

    def _stop_local_correction(self):
        """Stop local correction"""
        self.standalone_correction_active = False
        self._correction_started_flag = False
        self.path_correction_controller.disable_correction()
        self._send_can_command('motion', {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0})
        print("[Local correction] Stopped")

    def _enter_buffer_mode(self, delay_seconds):
        """Delayed execution mode"""
        start_time = time.time()

        last_cmd = self._last_base_station_cmd
        vx = int(last_cmd['x_speed'])
        vy = int(last_cmd['y_speed'])
        angular = int(last_cmd['angular_speed'])

        if abs(vx) < 50 and abs(vy) < 50:
            vx = 100
            vy = 0

        print(f"[Delayed execution] Starting {delay_seconds}s delay...")
        print(f"[Delayed execution] Executing command: X={vx}, Y={vy}")

        while time.time() - start_time < delay_seconds:
            if not self.running:
                return False

            self._send_can_command('motion', {
                'x_speed': vx,
                'y_speed': vy,
                'angular_speed': angular
            })

            time.sleep(0.1)

        print(f"[Delayed execution] ✅ Delay complete")

        self.path_correction_controller.enable_correction()
        self.path_correction_controller.mark_queue_finished()

        self._correction_loop()
        return True

    def _correction_loop(self):
        """Local correction loop"""
        print(f"[Local correction] Running loop ({self.base_speed}mm/s) ...")

        while self.running and self.standalone_correction_active:
            try:
                should_continue, stop_reason = self.path_correction_controller.should_continue_correction()
                if not should_continue:
                    if stop_reason:
                        print(f"[Local correction] {stop_reason}")
                    self._stop_local_correction()
                    break

                if not self.current_pose:
                    time.sleep(0.1)
                    continue

                corrected_cmd = self.path_correction_controller.calculate_correction_commands(
                    self.current_pose
                )

                if corrected_cmd:
                    vx = int(corrected_cmd.get('x_speed', 0))
                    vy = int(corrected_cmd.get('y_speed', 0))
                    angular = int(corrected_cmd.get('angular_speed', 0))

                    self._send_can_command('motion', {
                        'x_speed': vx,
                        'y_speed': vy,
                        'angular_speed': angular
                    })

                time.sleep(0.1)

            except Exception as e:
                print(f"[Local correction] Error: {e}")
                break

        self.standalone_correction_active = False

    def _send_can_command(self, cmd_type, params):
        """Send command to CAN"""
        if not self.can_handler.is_connected:
            return

        try:
            if cmd_type == 'enable_device':
                enabled = params.get('enabled', False)
                frame = self.car_controller.enable_device() if enabled else self.car_controller.disable_device()

            elif cmd_type == 'set_mode':
                mode = params.get('mode', 0)
                frame = self.car_controller.set_control_mode(mode, False, False, False)

            elif cmd_type == 'motion':
                x = int(params.get('x_speed', 0))
                y = int(params.get('y_speed', 0))
                angular = int(params.get('angular_speed', 0))
                frame = self.car_controller.send_motion_command(x, y, angular)
            else:
                return

            if frame:
                self.can_handler.send_hex_string(frame)

        except Exception as e:
            print(f"[CAN send] Error: {e}")

    def _tcp_send_loop(self):
        """Send status to station"""
        while self.running and self.tcp_connected:
            try:
                state = {
                    'type': 'ue_state',
                    'timestamp': time.time(),
                    'enabled': self.enabled,
                    'control_mode': self.control_mode,
                    'local_correction_enabled': self.local_correction_enabled,
                    'trajectory_synced': self.trajectory_synced,
                }

                if self.current_pose:
                    state['pose'] = {
                        'x': self.current_pose.x,
                        'y': self.current_pose.y,
                        'theta': self.current_pose.theta
                    }

                msg = json.dumps(state) + '\n'
                self.socket.sendall(msg.encode('utf-8'))
                time.sleep(0.05)

            except Exception as e:
                print(f"[TCP send] Error: {e}")
                break

        self.tcp_connected = False

    def stop(self):
        """Stop client"""
        print("🛑 Shutting down...")
        self.running = False
        self._stop_local_correction()

        if self.can_handler.is_connected:
            self._send_can_command('motion', {'x_speed': 0, 'y_speed': 0, 'angular_speed': 0})
            time.sleep(0.1)
            self.can_handler.stop_receive()
            self.can_handler.disconnect()

        if self.socket:
            try:
                self.socket.close()
            except:
                pass

        print("✅ Closed")


def main():
    parser = argparse.ArgumentParser(description='UE Client - CAN full version (Linux, no initialHB2 required)')
    parser.add_argument('--host', default='10.45.0.1', help='Station IP')
    parser.add_argument('--port', type=int, default=5000, help='Station port')
    parser.add_argument('--csv', default="/home/ubuntu22/carue/ideal.csv", help='Backup trajectory file')

    parser.add_argument('--enable-correction', action='store_true', help='Enable local correction')
    parser.add_argument('--no-correction', action='store_true', help='Disable local correction')
    parser.add_argument('--delay', type=float, default=0, help='Delay seconds')
    parser.add_argument('--density', type=float, default=0.4, help='Trajectory point density')
    parser.add_argument('--speed', type=int, default=200, help='Local correction speed')
    parser.add_argument('--max-time', type=float, default=120.0, help='Max correction time')

    args = parser.parse_args()

    enable_corr = None
    if args.enable_correction and args.no_correction:
        print("❌ Error: cannot use --enable-correction and --no-correction together")
        sys.exit(1)
    elif args.enable_correction:
        enable_corr = True
    elif args.no_correction:
        enable_corr = False

    client = UEClient(
        station_host=args.host,
        station_port=args.port,
        csv_trajectory_file=args.csv,
        enable_local_correction=enable_corr,
        correction_delay=args.delay,
        trajectory_density=args.density if args.density is not None else 0.4,
        base_speed=args.speed if args.speed is not None else 200,
        max_correction_time=args.max_time if args.max_time is not None else 120.0
    )

    try:
        client.start()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
