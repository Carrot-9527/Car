# can_protocol.py
from config import Config

class CANProtocolParser:
    # Class-level cache for aggregated logging
    last_frame_data = {}
    frame_change_count = {}
    
    def __init__(self):
        self.frame_buffer = bytearray()
        self.complete_frames = []
        
    def add_data(self, data):
        """Add received data and split into complete frames"""
        self.frame_buffer.extend(data)
        self._split_frames()
        
    def _split_frames(self):
        """Split data into complete CAN frames (based on correct Byte1 format)"""
        buf = self.frame_buffer
        start = 0
        end = len(buf)
        
        while start < end:
            # Find frame header
            i = buf.find(bytes([Config.FRAME_HEADER]), start, end)
            if i == -1:
                # No frame header found, clear buffer
                del buf[:end]
                break
            
            # Check if enough bytes to read Byte1
            if i + 1 >= end:
                break  # Insufficient data
            
            # Read Byte1 and parse DLC
            byte1 = buf[i + 1]
            
            # Validate Byte1 format: must be 0x8X format
            if (byte1 & 0xF0) != 0x80:
                # Byte1 format error, skip this frame header
                start = i + 1
                continue
            
            # Extract DLC (low 4 bits)
            dlc = byte1 & 0x0F
            
            # Calculate expected frame length
            expected_len = 1 + 1 + 1 + 4 + dlc + 1  # Header+BYTE1+BYTE2+ID+Data+Tail
            
            # Check if there is enough data
            if i + expected_len > end:
                # Insufficient data, wait for more data
                break
            
            # Check frame tail
            tail_pos = i + expected_len - 1
            if buf[tail_pos] == Config.FRAME_TAIL:
                # Found complete frame
                frame = bytes(buf[i:tail_pos + 1])
                self.complete_frames.append(frame)
                
                # Move start position
                start = tail_pos + 1
            else:
                # Frame tail mismatch, possibly wrong frame header or mixed data
                
                # Try to find next frame header
                next_header = buf.find(bytes([Config.FRAME_HEADER]), i + 1, end)
                if next_header != -1:
                    start = next_header
                else:
                    break
        
        # Clean up processed data
        if start > 0:
            del buf[:start]
            pass
        
        return len(self.complete_frames) > 0
    
    def get_complete_frames(self):
        """Get all complete frames"""
        frames = self.complete_frames.copy()
        self.complete_frames.clear()
        return frames
    
    @staticmethod
    def parse_frame(frame):
        """Parse a single CAN frame (based on correct Byte1 format)"""
        if len(frame) < 9:
            return None
        
        try:
            # Frame header confirmed
            byte1 = frame[1]
            byte2 = frame[2]
            
            # Validate Byte1 format
            if (byte1 & 0xF0) != 0x80:
                return None
            
            # Parse DLC (low 4 bits of Byte1)
            dlc = byte1 & 0x0F
            
            # Parse channel number (needs adjustment based on actual Byte1 format)
            # Assume high 4 bits of Byte1 are channel info: low 3 bits of X in 0x8X are DLC, highest bit is channel low bit?
            channel_low = (byte1 >> 4) & 0x01  # Needs confirmation based on actual protocol
            channel_high = (byte2 >> 3) & 0x03
            channel = (channel_high << 1) | channel_low
            
            # Parse frame format
            frame_format = (byte2 >> 2) & 0x01  # 0=Standard frame, 1=Extended frame
            
            # Parse frame type
            frame_type = (byte2 >> 1) & 0x01  # 0=Data frame, 1=Remote frame
            
            # Parse accelerated flag
            accelerated = byte2 & 0x01  # 0=Not accelerated, 1=Accelerated
            
            # Extract CAN ID and data
            can_id_bytes = frame[3:7]
            can_id = int.from_bytes(can_id_bytes, byteorder='big')
            data = frame[7:7+dlc] if dlc > 0 else b''
            
            parsed_frame = {
                'raw': frame.hex().upper(),
                'byte1_hex': f"0x{byte1:02X}",
                'dlc': dlc,
                'channel': channel,
                'frame_format': 'Extended frame' if frame_format else 'Standard frame',
                'frame_type': 'Remote frame' if frame_type else 'Data frame',
                'accelerated': accelerated,
                'can_id': can_id,
                'can_id_bytes': can_id_bytes,
                'data': data,
                'data_hex': data.hex().upper() if dlc > 0 else 'No data'
            }
            
            # Aggregate logs: only log changes for same frame ID
            frame_key = f"{can_id:08X}"
            if frame_key not in CANProtocolParser.last_frame_data:
                CANProtocolParser.last_frame_data[frame_key] = parsed_frame.copy()
                CANProtocolParser.frame_change_count[frame_key] = 1
            else:
                # Check if there are changes
                last_data = CANProtocolParser.last_frame_data[frame_key]
                changes = []
                for key in ['channel', 'dlc', 'frame_format', 'frame_type', 'accelerated', 'data_hex']:
                    if parsed_frame[key] != last_data[key]:
                        changes.append(f"{key}: {last_data[key]} -> {parsed_frame[key]}")
                
                if changes:
                    CANProtocolParser.frame_change_count[frame_key] += 1
                    CANProtocolParser.last_frame_data[frame_key] = parsed_frame.copy()
                else:
                    # No change, only print count occasionally
                    CANProtocolParser.frame_change_count[frame_key] += 1
            
            return parsed_frame
        except Exception as e:
            return None
    
    @staticmethod
    def parse_car_frame(frame_info):
        """Parse custom car frame (based on correct DLC and Byte1 format)"""
        if len(frame_info['can_id_bytes']) < 4:
            return frame_info
        
        # Parse CAN ID
        can_id_bytes = frame_info['can_id_bytes']
        device_class = can_id_bytes[0]
        device_model = can_id_bytes[1]
        device_id = can_id_bytes[2]
        function_code = can_id_bytes[3]
        
        # Basic info
        frame_info.update({
            'device_class': device_class,
            'device_model': device_model,
            'device_id': device_id,
            'function_code': function_code
        })
        
        # Parse data based on function code
        data = frame_info.get('data', b'')
        data_len = len(data)
        dlc = frame_info.get('dlc', 0)
        
        # Verify if DLC matches data length
        if data_len != dlc:
            pass  # Remove warning print
        
        if function_code == 0xB0:  # Device heartbeat packet
            frame_info['frame_type_desc'] = 'Device heartbeat packet'
            if data_len >= 1:
                frame_info['enabled'] = data[0]  # 0=Disabled, 1=Enabled
        
        elif function_code == 0xB1:  # Chassis status info
            frame_info['frame_type_desc'] = 'Chassis status info'
            if data_len == 8:
                # Temporarily only record raw data, specific parsing method needed
                frame_info['status_raw'] = data.hex().upper()
        
        elif function_code == 0xB2:  # Chassis motion info
            frame_info['frame_type_desc'] = 'Chassis motion info'
            if data_len == 6:
                # DLC=6: Only contains velocity info, no angle
                def parse_int16_le(low, high):
                    """Parse 16-bit signed little-endian integer"""
                    val = (high << 8) | low
                    return val - 0x10000 if val >= 0x8000 else val
                
                vx = parse_int16_le(data[0], data[1])      # X-direction velocity (mm/s)
                vy = parse_int16_le(data[2], data[3])      # Y-direction velocity (mm/s)
                vtheta_raw = parse_int16_le(data[4], data[5])  # Angular velocity raw value
                
                # Correction: angular velocity unit conversion, matching sender's 0.572958
                # vtheta_raw unit is 0.1°/s, needs conversion to mrad/s
                # 0.1°/s → °/s → rad/s → mrad/s
                # Conversion factor: 0.1 × (π/180) × 1000 = 1.74533
                vtheta_mrad = vtheta_raw * 1.74533  # Convert to mrad/s
                
                frame_info.update({
                    'vx': vx,          # mm/s
                    'vy': vy,          # mm/s
                    'vtheta_raw': vtheta_raw,  # 0.1°/s (raw value)
                    'vtheta': vtheta_mrad,     # mrad/s (converted)
                    'vx_m_s': vx / 1000.0,     # Convert to m/s
                    'vy_m_s': vy / 1000.0      # Convert to m/s
                })
                
            elif data_len == 8:
                # DLC=8: Contains velocity and angle info
                def parse_int16_le(low, high):
                    val = (high << 8) | low
                    return val - 0x10000 if val >= 0x8000 else val
                
                vx = parse_int16_le(data[0], data[1])      # X-direction velocity (mm/s)
                vy = parse_int16_le(data[2], data[3])      # Y-direction velocity (mm/s)
                vtheta_raw = parse_int16_le(data[4], data[5])  # Angular velocity raw value
                theta = parse_int16_le(data[6], data[7])   # Angle (degrees)
                
                # Correction: angular velocity unit conversion
                vtheta_mrad = vtheta_raw * 1.74533  # Convert to mrad/s
                
                frame_info.update({
                    'vx': vx,
                    'vy': vy,
                    'vtheta_raw': vtheta_raw,
                    'vtheta': vtheta_mrad,
                    'theta': theta,
                    'vx_m_s': vx / 1000.0,
                    'vy_m_s': vy / 1000.0
                })
        
        elif 0xA1 <= function_code <= 0xAF:  # Command feedback
            feedback_type = function_code - 0xA0
            feedback_names = {
                1: 'Prepare to restart', 2: 'Firmware version feedback', 3: 'General settings success',
                4: 'Special reset success', 5: 'Motion reset success', 6: 'ID setting success'
            }
            frame_info['frame_type_desc'] = feedback_names.get(feedback_type, f'Command feedback {feedback_type}')
        
        else:
            frame_info['frame_type_desc'] = 'Unknown command'
        
        return frame_info
    
    @staticmethod
    def build_can_frame(can_id_hex, data_hex, channel=Config.CAN_CHANNEL, 
                       is_extended=Config.CAN_IS_EXTENDED, is_data_frame=True, 
                       is_accelerated=False):
        """Build CAN transmit frame"""
        try:
            # Calculate data length
            data_str = data_hex.replace(" ", "").upper()  # Convert to uppercase
            
            # If data length is odd, append a 0
            if len(data_str) % 2 != 0:
                data_str = data_str + "0"
                
            data_len = len(data_str) // 2

            if data_len > 0x0F:
                raise ValueError(f"Data length {data_len} out of range (0-15)")

            # BYTE1: 0x8X format, where X is DLC
            byte1 = 0x80 | (data_len & 0x0F)
            
            # BYTE2: Send type + channel high 2 bits + frame format + frame type + accelerated flag
            channel_high = (channel >> 1) & 0x03
            byte2 = (channel_high << 3)
            
            # Set frame format
            if is_extended:
                byte2 |= (1 << 2)
            
            # Set frame type
            if not is_data_frame:
                byte2 |= (1 << 1)
            
            # Set accelerated flag
            if is_accelerated:
                byte2 |= (1 << 0)
            
            # Build complete frame
            can_id = int(can_id_hex, 16)
            
            # Ensure CAN ID is 4 bytes
            id_bytes = can_id.to_bytes(4, byteorder='big')
            
            # Data bytes
            data_bytes = bytes.fromhex(data_str)
            
            # Build complete frame bytes
            frame_bytes = bytes([Config.FRAME_HEADER, byte1, byte2]) + id_bytes + data_bytes + bytes([Config.FRAME_TAIL])
            
            # Convert to hex string and return
            return frame_bytes.hex().upper()
            
        except Exception as e:
            print(f"Build CAN frame error: {e}")
            return None