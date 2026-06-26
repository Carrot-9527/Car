# config.py
class Config:
    # Car device parameters
    DEVICE_CLASS = 0x01  # Category: Mobile chassis
    DEVICE_MODEL = 0x09  # Model: 09 chassis
    DEVICE_ID = 0x01     # ID: 01
    
    # Serial port default parameters
    DEFAULT_BAUDRATE = 460800
    DEFAULT_TIMEOUT = 1
    
    # CAN parameters
    CAN_CHANNEL = 1      # Use CAN1 channel
    CAN_IS_EXTENDED = True
    
    # Heartbeat detection
    HEARTBEAT_TIMEOUT_MS = 2000
    
    # Frame markers
    FRAME_HEADER = 0x5A
    FRAME_TAIL = 0xA5
    
    # Logging output control
    ENABLE_CONSOLE_LOGGING = False  # Console logging switch
    ENABLE_DEBUG_OUTPUT = False     # Debug output switch