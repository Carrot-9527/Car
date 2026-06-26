# car_control.py
from config import Config
from can_protocol import CANProtocolParser

class CarController:
    def __init__(self):
        self.parser = CANProtocolParser()
        self.enabled = False
        self.control_mode = 0x00
        self.debug_mode = Config.ENABLE_DEBUG_OUTPUT  # Debug mode
        
    def enable_device(self):
        """Enable device"""
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}03"
        data = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}01"
        frame = self.parser.build_can_frame(can_id, data, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        return frame
    
    def disable_device(self):
        """Disable device"""
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}03"
        data = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}00"
        frame = self.parser.build_can_frame(can_id, data, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        return frame
    
    def set_control_mode(self, mode, buzzer=False, brake=False, special=False):
        """Set control mode"""
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}11"
        buzzer_val = 0x01 if buzzer else 0x00
        brake_val = 0x01 if brake else 0x00
        special_val = 0x01 if special else 0x00
        data = f"{mode:02X}{buzzer_val:02X}{brake_val:02X}{special_val:02X}"
        frame = self.parser.build_can_frame(can_id, data, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        return frame
    
    def send_motion_command(self, x_speed, y_speed, angular_speed):
        """Send motion control command"""
        # CAN ID format: Device type + Model + ID + Function code (0x12 = Motion control)
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}12"
        
        # Check input type
        if not isinstance(x_speed, (int, float)):
            try:
                x_speed = float(x_speed)
            except:
                x_speed = 0
        
        if not isinstance(y_speed, (int, float)):
            try:
                y_speed = float(y_speed)
            except:
                y_speed = 0
        
        if not isinstance(angular_speed, (int, float)):
            try:
                angular_speed = float(angular_speed)
            except:
                angular_speed = 0
        
        # Convert to 16-bit signed integer (little-endian)
        def to_bytes_16bit_le(value):
            """Convert value to 16-bit signed little-endian bytes"""
            try:
                value_float = float(value)
                value_int = int(round(value_float))
            except Exception as e:
                value_int = 0
            
            # Limit range to 16-bit signed integer (-32768 to 32767)
            value_int = max(-32768, min(32767, value_int))
            
            # Convert to bytes
            result = value_int.to_bytes(2, byteorder='little', signed=True)
            return result
        
        # Process linear velocity (use mm/s directly)
        x_bytes = to_bytes_16bit_le(x_speed)
        
        # Process Y velocity
        y_bytes = to_bytes_16bit_le(y_speed)
        
        # Process angular velocity: mrad/s → need to check protocol requirements
        # Correction: changed to 0.572958, actual protocol unit may be 0.1°/s
        angular_converted = int(round(float(angular_speed) * 0.572958))  # mrad/s → 0.1°/s
        
        # Process angular velocity
        angular_bytes = to_bytes_16bit_le(angular_converted)
        
        # 8-byte data: X velocity(2) + Y velocity(2) + angular velocity(2) + angle(2)
        data_bytes = x_bytes + y_bytes + angular_bytes + b'\x00\x00'
        
        # Convert to hex string
        data_hex = data_bytes.hex().upper()
        
        # Build CAN frame
        frame = self.parser.build_can_frame(can_id, data_hex, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        
        return frame
    
    def query_version(self):
        """Query device version"""
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}02"
        data = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}"
        frame = self.parser.build_can_frame(can_id, data, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        return frame
    
    def device_reboot(self):
        """Device reboot"""
        can_id = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}01"
        data = f"{Config.DEVICE_CLASS:02X}{Config.DEVICE_MODEL:02X}{Config.DEVICE_ID:02X}"
        frame = self.parser.build_can_frame(can_id, data, Config.CAN_CHANNEL, Config.CAN_IS_EXTENDED)
        return frame
    
    def parse_received_frame(self, frame_data):
        """Parse received frame data"""
        self.parser.add_data(frame_data)
        complete_frames = self.parser.get_complete_frames()
        
        parsed_frames = []
        for frame in complete_frames:
            frame_info = self.parser.parse_frame(frame)
            if frame_info:
                frame_info = self.parser.parse_car_frame(frame_info)
                parsed_frames.append(frame_info)
        
        return parsed_frames