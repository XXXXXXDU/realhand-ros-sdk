#!/usr/bin/env python3
import can
import time, sys, os
import threading
import numpy as np
from enum import Enum
from utils.open_can import OpenCan
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
"""
Thumb 41:  [thumb_abduction, thumb_roll, thumb_root, reserved, reserved, thumb_tip]
Index 42:  [index_abduction, reserved, index_root, reserved, reserved, index_tip]
Middle 43: [middle_abduction, reserved, middle_root, reserved, reserved, middle_tip]
Ring 44:   [ring_abduction, reserved, ring_root, reserved, reserved, ring_tip]
Little 45: [little_abduction, reserved, little_root, reserved, reserved, little_tip]
"""
CMD_MAP = [
    "thumb_root",
    "index_root",
    "middle_root",
    "ring_root",
    "little_root",
    "thumb_abduction",
    "index_abduction",
    "middle_abduction",
    "ring_abduction",
    "little_abduction",
    "thumb_roll",
    "reserved",
    "reserved",
    "reserved",
    "reserved",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip"
]

class FrameProperty(Enum):
    # Finger motion control - parallel control commands (control the same joint of all fingers)
    ROLL_POS = 0x01  # Roll joint position
    YAW_POS = 0x02  # Yaw joint position
    ROOT1_POS = 0x03  # Root joint 1 position
    ROOT2_POS = 0x04  # Root joint 2 position
    ROOT3_POS = 0x05  # Root joint 3 position
    TIP_POS = 0x06  # Fingertip joint position
    
    # Joint speed commands
    ROLL_SPEED = 0x09  # Roll joint speed
    YAW_SPEED = 0x0A  # Yaw joint speed
    ROOT1_SPEED = 0x0B  # Root joint 1 speed
    ROOT2_SPEED = 0x0C  # Root joint 2 speed
    ROOT3_SPEED = 0x0D  # Root joint 3 speed
    TIP_SPEED = 0x0E  # Fingertip joint speed
    
    # Joint torque commands
    ROLL_TORQUE = 0x11  # Roll joint torque
    YAW_TORQUE = 0x12  # Yaw joint torque
    ROOT1_TORQUE = 0x13  # Root joint 1 torque
    ROOT2_TORQUE = 0x14  # Root joint 2 torque
    ROOT3_TORQUE = 0x15  # Root joint 3 torque
    TIP_TORQUE = 0x16  # Fingertip joint torque
    
    # Joint fault codes
    ROLL_FAULT = 0x19  # Roll joint fault code
    YAW_FAULT = 0x1A  # Yaw joint fault code
    ROOT1_FAULT = 0x1B  # Root joint 1 fault code
    ROOT2_FAULT = 0x1C  # Root joint 2 fault code
    ROOT3_FAULT = 0x1D  # Root joint 3 fault code
    TIP_FAULT = 0x1E  # Fingertip joint fault code
    
    # Joint temperature
    ROLL_TEMPERATURE = 0x21  # Roll joint over-temperature protection threshold
    YAW_TEMPERATURE = 0x22  # Yaw joint over-temperature protection threshold
    ROOT1_TEMPERATURE = 0x23  # Root joint 1 over-temperature protection threshold
    ROOT2_TEMPERATURE = 0x24  # Root joint 2 over-temperature protection threshold
    ROOT3_TEMPERATURE = 0x25  # Root joint 3 over-temperature protection threshold
    TIP_TEMPERATURE = 0x26  # Fingertip joint over-temperature protection threshold
    
    # Finger motion control - serial control commands (control all joints of the same finger)
    THUMB_POS = 0x41  # Thumb joint position
    INDEX_POS = 0x42  # Index finger joint position
    MIDDLE_POS = 0x43  # Middle finger joint position
    RING_POS = 0x44  # Ring finger joint position
    LITTLE_POS = 0x45  # Little finger joint position
    
    # Finger speed
    THUMB_SPEED = 0x49  # Thumb speed
    INDEX_SPEED = 0x4A  # Index finger speed
    MIDDLE_SPEED = 0x4B  # Middle finger speed
    RING_SPEED = 0x4C  # Ring finger speed
    LITTLE_SPEED = 0x4D  # Little finger speed
    
    # Finger torque
    THUMB_TORQUE = 0x51  # Thumb torque
    INDEX_TORQUE = 0x52  # Index finger torque
    MIDDLE_TORQUE = 0x53  # Middle finger torque
    RING_TORQUE = 0x54  # Ring finger torque
    LITTLE_TORQUE = 0x55  # Little finger torque
    
    # Finger fault codes
    THUMB_FAULT = 0x59  # Thumb fault code
    INDEX_FAULT = 0x5A  # Index finger fault code
    MIDDLE_FAULT = 0x5B  # Middle finger fault code
    RING_FAULT = 0x5C  # Ring finger fault code
    LITTLE_FAULT = 0x5D  # Little finger fault code
    
    # Finger temperature
    THUMB_TEMPERATURE = 0x61  # Thumb over-temperature protection threshold
    INDEX_TEMPERATURE = 0x62  # Index finger over-temperature protection threshold
    MIDDLE_TEMPERATURE = 0x63  # Middle finger over-temperature protection threshold
    RING_TEMPERATURE = 0x64  # Ring finger over-temperature protection threshold
    LITTLE_TEMPERATURE = 0x65  # Little finger over-temperature protection threshold
    
    # Finger motion control - combined command area
    FINGER_SPEED = 0x81  # Set finger speed
    FINGER_TORQUE = 0x82  # Set finger output torque
    FINGER_FAULT = 0x83  # Clear finger faults and fault codes
    FINGER_TEMPERATURE = 0x84  # Finger joint temperatures
    
    # Fingertip sensor data
    HAND_NORMAL_FORCE = 0x90  # Normal force of five fingers
    HAND_TANGENTIAL_FORCE = 0x91  # Tangential force of five fingers
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # Tangential direction of five fingers
    HAND_APPROACH_INC = 0x93  # Approach sensing of five fingers
    
    # All finger data
    THUMB_ALL_DATA = 0x98  # Thumb all data
    INDEX_ALL_DATA = 0x99  # Index finger all data
    MIDDLE_ALL_DATA = 0x9A  # Middle finger all data
    RING_ALL_DATA = 0x9B  # Ring finger all data
    LITTLE_ALL_DATA = 0x9C  # Little finger all data
    
    # Tactile sensors
    TOUCH_SENSOR_TYPE = 0xB0  # Tactile sensor type
    THUMB_TOUCH = 0xB1  # Thumb tactile sensing
    INDEX_TOUCH = 0xB2  # Index finger tactile sensing
    MIDDLE_TOUCH = 0xB3  # Middle finger tactile sensing
    RING_TOUCH = 0xB4  # Ring finger tactile sensing
    LITTLE_TOUCH = 0xB5  # Little finger tactile sensing
    PALM_TOUCH = 0xB6  # Palm tactile sensing
    
    # Query commands
    HAND_UID_GET = 0xC0  # Unique identifier query
    HAND_HARDWARE_VERSION_GET = 0xC1  # Hardware version query
    HAND_SOFTWARE_VERSION_GET = 0xC2  # Software version query
    HAND_COMM_ID_GET = 0xC3  # Device ID query
    HAND_STRUCT_VERSION_GET = 0xC4  # Structure version query
    
    # Factory commands
    HOST_CMD_HAND_ERASE_POS_CALI = 0xCD  # Erase position calibration values
    HAND_COMM_ID_SET = 0xD1  # Communication ID setting
    HAND_UID_SET = 0xF0  # Unique identifier setting

class LinkerHandG20Can:
    def __init__(self, can_channel='can0', baudrate=1000000, can_id=0x28, yaml=""):
        self.can_id = can_id
        self.can_channel = can_channel
        self.baudrate = baudrate
        self.open_can = OpenCan(load_yaml=yaml)

        self.running = True
        
        # Last command cache
        self.last_thumb_pos, self.last_index_pos, self.last_ring_pos, self.last_middle_pos, self.last_little_pos = None, None, None, None, None
        self.last_root1, self.last_yaw, self.last_roll, self.last_root2, self.last_tip = None, None, None, None, None
        
        # Parallel control data storage
        self.x01, self.x02, self.x03, self.x04, self.x05, self.x06 = [], [], [], [], [], []
        self.x09, self.x0A, self.x0B, self.x0C, self.x0D, self.x0E = [], [], [], [], [], []
        self.x11, self.x12, self.x13, self.x14, self.x15, self.x16 = [], [], [], [], [], []
        self.x19, self.x1A, self.x1B, self.x1C, self.x1D, self.x1E = [], [], [], [], [], []
        self.x21, self.x22, self.x23, self.x24, self.x25, self.x26 = [], [], [], [], [], []
        
        # Serial control data storage
        self.x41, self.x42, self.x43, self.x44, self.x45 = [], [], [], [], []
        self.x49, self.x4A, self.x4B, self.x4C, self.x4D = [0] * 6, [0] * 6, [0] * 6, [0] * 6, [0] * 6
        self.x51, self.x52, self.x53, self.x54, self.x55 = [], [], [], [], []
        self.x59, self.x5A, self.x5B, self.x5C, self.x5D = [], [], [], [], []
        self.x61, self.x62, self.x63, self.x64, self.x65 = [], [], [], [], []
        
        # Combined command area data storage
        self.x81, self.x82, self.x83, self.x84 = [], [], [], []
        
        # Sensor data storage
        self.x90, self.x91, self.x92, self.x93 = [], [], [], []
        self.x98, self.x99, self.x9A, self.x9B, self.x9C = [], [], [], [], []
        self.xB0, self.xB1, self.xB2, self.xB3, self.xB4, self.xB5, self.xB6 = [], [], [], [], [], [], []
        self.normal_force, self.tangential_force, self.tangential_force_dir, self.approach_inc = [[-1] * 5 for _ in range(4)]
        
        # Query command data storage
        self.xC0, self.xC1, self.xC2, self.xC3, self.xC4 = [], [], [], [], []
        
        # Tactile sensor matrix data
        self.thumb_matrix = np.full((12, 6), -1)
        self.index_matrix = np.full((12, 6), -1)
        self.middle_matrix = np.full((12, 6), -1)
        self.ring_matrix = np.full((12, 6), -1)
        self.little_matrix = np.full((12, 6), -1)
        self.matrix_map = {
            0: 0, 16: 1, 32: 2, 48: 3, 64: 4, 80: 5,
            96: 6, 112: 7, 128: 8, 144: 9, 160: 10, 176: 11,
        }
        
        # Initialize CAN bus
        try:
            if sys.platform == "linux":
                self.bus = can.interface.Bus(
                    channel=can_channel, interface="socketcan", bitrate=baudrate, 
                    can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
                )
            elif sys.platform == "win32":
                self.bus = can.interface.Bus(
                    channel=can_channel, interface='pcan', bitrate=baudrate, 
                    can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
                )
            else:
                raise EnvironmentError("Unsupported platform for CAN interface")
        except:
            print("Please insert CAN device")

        # Start receive thread
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list, sleep_time=0.003):
        """
        Send command to CAN bus
        :param frame_property: Data frame property
        :param data_list: Data payload
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Failed to send message: {e}")
            self.open_can.open_can(self.can_channel)
            time.sleep(1)
            self.is_can = self.open_can.is_can_up_sysfs(interface=self.can_channel)
            time.sleep(1)
            if self.is_can:
                self.bus = can.interface.Bus(channel=self.can_channel, interface="socketcan", bitrate=self.baudrate)
            else:
                print("Reconnecting CAN devices ....")
        time.sleep(sleep_time)

    def receive_response(self):
        """
        Receive and process CAN bus response messages
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")

    def process_response(self, msg):
        """
        Process CAN response messages
        """
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            
            # Parallel control command responses
            if frame_type == 0x01: self.x01 = list(response_data)
            elif frame_type == 0x02: self.x02 = list(response_data)
            elif frame_type == 0x03: self.x03 = list(response_data)
            elif frame_type == 0x04: self.x04 = list(response_data)
            elif frame_type == 0x05: self.x05 = list(response_data)
            elif frame_type == 0x06: self.x06 = list(response_data)
            elif frame_type == 0x09: self.x09 = list(response_data)
            elif frame_type == 0x0A: self.x0A = list(response_data)
            elif frame_type == 0x0B: self.x0B = list(response_data)
            elif frame_type == 0x0C: self.x0C = list(response_data)
            elif frame_type == 0x0D: self.x0D = list(response_data)
            elif frame_type == 0x0E: self.x0E = list(response_data)
            elif frame_type == 0x11: self.x11 = list(response_data)
            elif frame_type == 0x12: self.x12 = list(response_data)
            elif frame_type == 0x13: self.x13 = list(response_data)
            elif frame_type == 0x14: self.x14 = list(response_data)
            elif frame_type == 0x15: self.x15 = list(response_data)
            elif frame_type == 0x16: self.x16 = list(response_data)
            elif frame_type == 0x19: self.x19 = list(response_data)
            elif frame_type == 0x1A: self.x1A = list(response_data)
            elif frame_type == 0x1B: self.x1B = list(response_data)
            elif frame_type == 0x1C: self.x1C = list(response_data)
            elif frame_type == 0x1D: self.x1D = list(response_data)
            elif frame_type == 0x1E: self.x1E = list(response_data)
            elif frame_type == 0x21: self.x21 = list(response_data)
            elif frame_type == 0x22: self.x22 = list(response_data)
            elif frame_type == 0x23: self.x23 = list(response_data)
            elif frame_type == 0x24: self.x24 = list(response_data)
            elif frame_type == 0x25: self.x25 = list(response_data)
            elif frame_type == 0x26: self.x26 = list(response_data)
            
            # Serial control command responses
            elif frame_type == 0x41: self.x41 = list(response_data)
            elif frame_type == 0x42: self.x42 = list(response_data)
            elif frame_type == 0x43: self.x43 = list(response_data)
            elif frame_type == 0x44: self.x44 = list(response_data)
            elif frame_type == 0x45: self.x45 = list(response_data)
            elif frame_type == 0x49: self.x49 = list(response_data)
            elif frame_type == 0x4A: self.x4A = list(response_data)
            elif frame_type == 0x4B: self.x4B = list(response_data)
            elif frame_type == 0x4C: self.x4C = list(response_data)
            elif frame_type == 0x4D: self.x4D = list(response_data)
            elif frame_type == 0x51: self.x51 = list(response_data)
            elif frame_type == 0x52: self.x52 = list(response_data)
            elif frame_type == 0x53: self.x53 = list(response_data)
            elif frame_type == 0x54: self.x54 = list(response_data)
            elif frame_type == 0x55: self.x55 = list(response_data)
            elif frame_type == 0x59: self.x59 = list(response_data)
            elif frame_type == 0x5A: self.x5A = list(response_data)
            elif frame_type == 0x5B: self.x5B = list(response_data)
            elif frame_type == 0x5C: self.x5C = list(response_data)
            elif frame_type == 0x5D: self.x5D = list(response_data)
            elif frame_type == 0x61: self.x61 = list(response_data)
            elif frame_type == 0x62: self.x62 = list(response_data)
            elif frame_type == 0x63: self.x63 = list(response_data)
            elif frame_type == 0x64: self.x64 = list(response_data)
            elif frame_type == 0x65: self.x65 = list(response_data)
            
            # Combined command area responses
            elif frame_type == 0x81: self.x81 = list(response_data)
            elif frame_type == 0x82: self.x82 = list(response_data)
            elif frame_type == 0x83: self.x83 = list(response_data)
            elif frame_type == 0x84: self.x84 = list(response_data)
            
            # Sensor data responses
            elif frame_type == 0x90: self.x90 = list(response_data)
            elif frame_type == 0x91: self.x91 = list(response_data)
            elif frame_type == 0x92: self.x92 = list(response_data)
            elif frame_type == 0x93: self.x93 = list(response_data)
            elif frame_type == 0x98: self.x98 = list(response_data)
            elif frame_type == 0x99: self.x99 = list(response_data)
            elif frame_type == 0x9A: self.x9A = list(response_data)
            elif frame_type == 0x9B: self.x9B = list(response_data)
            elif frame_type == 0x9C: self.x9C = list(response_data)
            
            # Tactile sensor responses
            elif frame_type == 0xB0: self.xB0 = list(response_data)
            elif frame_type == 0xB1:
                d = list(response_data)
                if len(d) == 2:
                    self.xB1 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.thumb_matrix[index] = d[1:]
                        
            elif frame_type == 0xB2: 
                d = list(response_data)
                if len(d) == 2:
                    self.xB2 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.index_matrix[index] = d[1:]
            elif frame_type == 0xB3: 
                d = list(response_data)
                if len(d) == 2:
                    self.xB3 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.middle_matrix[index] = d[1:]
            elif frame_type == 0xB4: 
                d = list(response_data)
                if len(d) == 2:
                    self.xB4 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.ring_matrix[index] = d[1:]
            elif frame_type == 0xB5: 
                d = list(response_data)
                if len(d) == 2:
                    self.xB5 = d
                elif len(d) == 7:
                    index = self.matrix_map.get(d[0])
                    if index is not None:
                        self.little_matrix[index] = d[1:]
            elif frame_type == 0xB6: self.xB6 = list(response_data)
            
            # Query command responses
            elif frame_type == 0xC0: self.xC0 = list(response_data)
            elif frame_type == 0xC1: self.xC1 = list(response_data)
            elif frame_type == 0xC2: self.xC2 = list(response_data)
            elif frame_type == 0xC3: self.xC3 = list(response_data)
            elif frame_type == 0xC4: self.xC4 = list(response_data)

    # Parallel control methods
    def set_roll_positions(self, joint_ranges):
        """Set roll joint positions of all fingers"""
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    
    def set_yaw_positions(self, joint_ranges):
        """Set yaw joint positions of all fingers"""
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    
    def set_root1_positions(self, joint_ranges):
        """Set root joint 1 positions of all fingers"""
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    
    def set_root2_positions(self, joint_ranges):
        """Set root joint 2 positions of all fingers"""
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    
    def set_root3_positions(self, joint_ranges):
        """Set root joint 3 positions of all fingers"""
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    
    def set_tip_positions(self, joint_ranges=[80]*5):
        """Set fingertip joint positions of all fingers"""
        self.send_command(FrameProperty.TIP_POS, joint_ranges)

    # Serial control methods
    def set_thumb_positions(self, joint_ranges):
        """Set all thumb joint positions"""
        self.send_command(FrameProperty.THUMB_POS, joint_ranges)
    
    def set_index_positions(self, joint_ranges):
        """Set all index finger joint positions"""
        self.send_command(FrameProperty.INDEX_POS, joint_ranges)
    
    def set_middle_positions(self, joint_ranges):
        """Set all middle finger joint positions"""
        self.send_command(FrameProperty.MIDDLE_POS, joint_ranges)
    
    def set_ring_positions(self, joint_ranges):
        """Set all ring finger joint positions"""
        self.send_command(FrameProperty.RING_POS, joint_ranges)
    
    def set_little_positions(self, joint_ranges):
        """Set all little finger joint positions"""
        self.send_command(FrameProperty.LITTLE_POS, joint_ranges)

    # Torque setters
    def set_thumb_torque(self, torque_values):
        """Set thumb torque"""
        self.send_command(FrameProperty.THUMB_TORQUE, torque_values)
    
    def set_index_torque(self, torque_values):
        """Set index finger torque"""
        self.send_command(FrameProperty.INDEX_TORQUE, torque_values)
    
    def set_middle_torque(self, torque_values):
        """Set middle finger torque"""
        self.send_command(FrameProperty.MIDDLE_TORQUE, torque_values)
    
    def set_ring_torque(self, torque_values):
        """Set ring finger torque"""
        self.send_command(FrameProperty.RING_TORQUE, torque_values)
    
    def set_little_torque(self, torque_values):
        """Set little finger torque"""
        self.send_command(FrameProperty.LITTLE_TORQUE, torque_values)

    # Speed setters
    def set_thumb_speed(self, speed_values):
        """Set thumb speed"""
        self.send_command(FrameProperty.THUMB_SPEED, speed_values)
    
    def set_index_speed(self, speed_values):
        """Set index finger speed"""
        self.send_command(FrameProperty.INDEX_SPEED, speed_values)
    
    def set_middle_speed(self, speed_values):
        """Set middle finger speed"""
        self.send_command(FrameProperty.MIDDLE_SPEED, speed_values)
    
    def set_ring_speed(self, speed_values):
        """Set ring finger speed"""
        self.send_command(FrameProperty.RING_SPEED, speed_values)
    
    def set_little_speed(self, speed_values):
        """Set little finger speed"""
        self.send_command(FrameProperty.LITTLE_SPEED, speed_values)

    # Position query
    def get_thumb_positions(self):
        """Get current thumb joint positions"""
        self.send_command(FrameProperty.THUMB_POS, [])
        return self.x41
    
    def get_index_positions(self):
        """Get current index finger joint positions"""
        self.send_command(FrameProperty.INDEX_POS, [])
        return self.x42
    
    def get_middle_positions(self):
        """Get current middle finger joint positions"""
        self.send_command(FrameProperty.MIDDLE_POS, [])
        return self.x43
    
    def get_ring_positions(self):
        """Get current ring finger joint positions"""
        self.send_command(FrameProperty.RING_POS, [])
        return self.x44
    
    def get_little_positions(self):
        """Get current little finger joint positions"""
        self.send_command(FrameProperty.LITTLE_POS, [])
        return self.x45

    # Speed query
    def get_thumb_speed(self):
        """Get thumb speed"""
        self.send_command(FrameProperty.THUMB_SPEED, [])
    
    def get_index_speed(self):
        """Get index finger speed"""
        self.send_command(FrameProperty.INDEX_SPEED, [])
    
    def get_middle_speed(self):
        """Get middle finger speed"""
        self.send_command(FrameProperty.MIDDLE_SPEED, [])
    
    def get_ring_speed(self):
        """Get ring finger speed"""
        self.send_command(FrameProperty.RING_SPEED, [])
    
    def get_little_speed(self):
        """Get little finger speed"""
        self.send_command(FrameProperty.LITTLE_SPEED, [])

    # Torque query
    def get_thumb_torque(self):
        """Get thumb torque"""
        self.send_command(FrameProperty.THUMB_TORQUE, [])
    
    def get_index_torque(self):
        """Get index finger torque"""
        self.send_command(FrameProperty.INDEX_TORQUE, [])
    
    def get_middle_torque(self):
        """Get middle finger torque"""
        self.send_command(FrameProperty.MIDDLE_TORQUE, [])
    
    def get_ring_torque(self):
        """Get ring finger torque"""
        self.send_command(FrameProperty.RING_TORQUE, [])
    
    def get_little_torque(self):
        """Get little finger torque"""
        self.send_command(FrameProperty.LITTLE_TORQUE, [])

    # Fault query
    def get_thumb_fault(self):
        """Get thumb joint fault codes"""
        self.send_command(FrameProperty.THUMB_FAULT, [])
        return self.x59
    
    def get_index_fault(self):
        """Get index finger joint fault codes"""
        self.send_command(FrameProperty.INDEX_FAULT, [])
        return self.x5A
    
    def get_middle_fault(self):
        """Get middle finger joint fault codes"""
        self.send_command(FrameProperty.MIDDLE_FAULT, [])
        return self.x5B
    
    def get_ring_fault(self):
        """Get ring finger joint fault codes"""
        self.send_command(FrameProperty.RING_FAULT, [])
        return self.x5C
    
    def get_little_fault(self):
        """Get little finger joint fault codes"""
        self.send_command(FrameProperty.LITTLE_FAULT, [])
        return self.x5D

    # Temperature query
    def get_thumb_temperature(self):
        """Get current thumb joint temperatures"""
        self.send_command(FrameProperty.THUMB_TEMPERATURE, [])
        return self.x61
    
    def get_index_temperature(self):
        """Get current index finger joint temperatures"""
        self.send_command(FrameProperty.INDEX_TEMPERATURE, [])
        return self.x62
    
    def get_middle_temperature(self):
        """Get current middle finger joint temperatures"""
        self.send_command(FrameProperty.MIDDLE_TEMPERATURE, [])
        return self.x63
    
    def get_ring_temperature(self):
        """Get current ring finger joint temperatures"""
        self.send_command(FrameProperty.RING_TEMPERATURE, [])
        return self.x64
    
    def get_little_temperature(self):
        """Get current little finger joint temperatures"""
        self.send_command(FrameProperty.LITTLE_TEMPERATURE, [])
        return self.x65

    # Combined command area
    def set_finger_speed(self, speed_values):
        """Set finger speed"""
        self.send_command(FrameProperty.FINGER_SPEED, speed_values)
    
    def set_finger_torque(self, torque_values):
        """Set finger output torque"""
        self.send_command(FrameProperty.FINGER_TORQUE, torque_values)
    
    def clear_finger_faults(self, finger_mask=[1, 1, 1, 1, 1]):
        """Clear finger faults and fault codes"""
        self.send_command(FrameProperty.FINGER_FAULT, finger_mask)
        return self.x83
    
    def get_finger_temperature(self):
        """Get temperatures of all finger joints"""
        self.send_command(FrameProperty.FINGER_TEMPERATURE, [])
        return self.x84

    # Sensor data
    def get_normal_force(self):
        """Get normal force of five fingers"""
        self.send_command(FrameProperty.HAND_NORMAL_FORCE, [])
        return self.x90
    
    def get_tangential_force(self):
        """Get tangential force of five fingers"""
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE, [])
        return self.x91
    
    def get_tangential_force_dir(self):
        """Get tangential direction of five fingers"""
        self.send_command(FrameProperty.HAND_TANGENTIAL_FORCE_DIR, [])
        return self.x92
    
    def get_approach_sensing(self):
        """Get approach sensing of five fingers"""
        self.send_command(FrameProperty.HAND_APPROACH_INC, [])
        return self.x93

    # Tactile sensor
    def get_touch_sensor_type(self):
        """Get tactile sensor type"""
        self.send_command(FrameProperty.TOUCH_SENSOR_TYPE, [])
        return self.xB0
    
    def get_thumb_touch(self):
        """Get thumb tactile sensor data"""
        self.send_command(FrameProperty.THUMB_TOUCH, [0xC6], sleep_time=0.007)
        #return self.thumb_matrix
    
    def get_index_touch(self):
        """Get index finger tactile sensor data"""
        self.send_command(FrameProperty.INDEX_TOUCH, [0xC6], sleep_time=0.007)
        #return self.xB2
    
    def get_middle_touch(self):
        """Get middle finger tactile sensor data"""
        self.send_command(FrameProperty.MIDDLE_TOUCH, [0xC6], sleep_time=0.007)
        #return self.xB3
    
    def get_ring_touch(self):
        """Get ring finger tactile sensor data"""
        self.send_command(FrameProperty.RING_TOUCH, [0xC6], sleep_time=0.007)
        #return self.xB4
    
    def get_little_touch(self):
        """Get little finger tactile sensor data"""
        self.send_command(FrameProperty.LITTLE_TOUCH, [0xC6], sleep_time=0.007)
        #return self.xB5
    
    def get_palm_touch(self):
        """Get palm tactile sensor data"""
        self.send_command(FrameProperty.PALM_TOUCH, [], sleep_time=0.015)
        #return self.xB6

    # Query commands
    def get_uid(self):
        """Get device unique identifier"""
        self.send_command(FrameProperty.HAND_UID_GET, [])
        return self.xC0
    
    def get_hardware_version(self):
        """Get hardware version"""
        self.send_command(FrameProperty.HAND_HARDWARE_VERSION_GET, [])
        return self.xC1
    
    def get_software_version(self):
        """Get software version"""
        self.send_command(FrameProperty.HAND_SOFTWARE_VERSION_GET, [])
        return self.xC2
    
    def get_comm_id(self):
        """Get device communication ID"""
        self.send_command(FrameProperty.HAND_COMM_ID_GET, [])
        return self.xC3
    
    def get_struct_version(self):
        """Get structure version number"""
        self.send_command(FrameProperty.HAND_STRUCT_VERSION_GET, [])
        return self.xC4

    # Factory commands
    def erase_position_calibration(self):
        """Erase position calibration values"""
        self.send_command(FrameProperty.HOST_CMD_HAND_ERASE_POS_CALI, [])
    
    def set_comm_id(self, new_id):
        """Set communication ID"""
        self.send_command(FrameProperty.HAND_COMM_ID_SET, [new_id])
    
    def set_uid(self, uid_data):
        """Set unique identifier (factory use)"""
        self.send_command(FrameProperty.HAND_UID_SET, uid_data)

    # Helper
    def slice_list(self, input_list, slice_size):
        """Slice list into pieces of specified size"""
        return [input_list[i:i + slice_size] for i in range(0, len(input_list), slice_size)]

    # -----------------------------------------------------
    # API region
    #------------------------------------------------------
    def set_joint_positions(self, joint_ranges):
        """API: set all finger joint positions"""
        j = self.cmd_range_to_joint_range(cmd_list=joint_ranges)
        self.set_thumb_positions(j[0])
        self.set_index_positions(j[1])
        self.set_middle_positions(j[2])
        self.set_ring_positions(j[3])
        self.set_little_positions(j[4])

    def set_speed(self, speed=[250] * 5):
        """API: set finger speed"""
        self.set_thumb_speed(speed_values=[speed[0]] * 6)
        self.set_index_speed(speed_values=[speed[1]] * 6)
        self.set_middle_speed(speed_values=[speed[2]] * 6)
        self.set_ring_speed(speed_values=[speed[3]] * 6)
        self.set_little_speed(speed_values=[speed[4]] * 6)

    def set_torque(self, torque=[250] * 5):
        """API: set maximum finger torque"""
        self.set_thumb_torque(torque_values=[torque[0]] * 6)
        self.set_index_torque(torque_values=[torque[1]] * 6)
        self.set_middle_torque(torque_values=[torque[2]] * 6)
        self.set_ring_torque(torque_values=[torque[3]] * 6)
        self.set_little_torque(torque_values=[torque[4]] * 6)

    def get_version(self):
        """API: get embedded version information"""
        return self.get_software_version()
    
    def get_current_status(self):
        """API: get current finger state"""
        self.get_thumb_positions()
        self.get_index_positions()
        self.get_middle_positions()
        self.get_ring_positions()
        self.get_little_positions()
        time.sleep(0.002)
        s = [self.x41, self.x42, self.x43, self.x44, self.x45]
        cmd_state = self.joint_state_to_cmd_state(list=s)
        return cmd_state

    def get_current_pub_status(self):
        """API: get current finger state"""
        self.get_current_status()

    def get_speed(self):
        """API: get finger speed"""
        self.get_thumb_speed()
        self.get_index_speed()
        self.get_middle_speed()
        self.get_ring_speed()
        self.get_little_speed()
        time.sleep(0.002)

        joint_speed = [self.x49, self.x4A, self.x4B, self.x4C, self.x4D]
        state_speed = self.joint_state_to_cmd_state(list=joint_speed)
        return state_speed

    def get_touch_type(self):
        """API: get tactile sensor type"""
        self.send_command(0xb0,[],sleep_time=0.03)
        self.send_command(0xb1,[],sleep_time=0.03)
        t = []
        for i in range(3):
            t = self.xB1
            time.sleep(0.01)
        if len(t) == 2:
            return 2
        else:
            self.send_command(0x20,[],sleep_time=0.03)
            time.sleep(0.01)
            if self.normal_force[0] == -1:
                return -1
            else:
                return 1
    
    def get_matrix_touch(self):
        """API: get all finger matrix tactile sensor data"""
        self.get_thumb_touch()
        self.get_index_touch()
        self.get_middle_touch()
        self.get_ring_touch()
        self.get_little_touch()
        return self.thumb_matrix , self.index_matrix , self.middle_matrix , self.ring_matrix , self.little_matrix

    def get_matrix_touch_v2(self):
        """API: get all finger matrix tactile sensor data"""
        return self.get_matrix_touch()

    def get_thumb_matrix_touch(self,sleep_time=0):
        """API: get [thumb] matrix tactile sensor data"""
        self.get_thumb_touch()
        return self.thumb_matrix
    
    def get_index_matrix_touch(self,sleep_time=0):
        """API: get [index finger] matrix tactile sensor data"""
        self.get_index_touch()
        return self.index_matrix
    
    def get_middle_matrix_touch(self,sleep_time=0):
        """API: get [middle finger] matrix tactile sensor data"""
        self.get_middle_touch()
        return self.middle_matrix
    
    def get_ring_matrix_touch(self,sleep_time=0):
        """API: get [ring finger] matrix tactile sensor data"""
        self.get_ring_touch()
        return self.ring_matrix
    
    def get_little_matrix_touch(self,sleep_time=0):
        """API: get [little finger] matrix tactile sensor data"""
        self.get_little_touch()
        return self.little_matrix

    def get_torque(self):
        """API: get maximum finger torque"""
        self.get_thumb_torque()
        self.get_index_torque()
        self.get_middle_torque()
        self.get_middle_torque()
        self.get_little_torque()
        time.sleep(0.003)
        t = [self.x51, self.x52, self.x53, self.x54, self.x55]
        cmd_torque = self.joint_state_to_cmd_state(list=t)
        return cmd_torque

    def get_current(self):
        """API: get finger current"""
        return [-1] * 20

    def get_temperature(self):
        """API: get finger temperature"""
        joint_temperature = [self.get_thumb_temperature(), self.get_index_temperature(), self.get_middle_temperature(), self.get_ring_temperature(), self.get_little_temperature()]
        cmd_temperature = self.joint_state_to_cmd_state(list=joint_temperature)
        return cmd_temperature

    def get_fault(self):
        """API: get finger fault codes"""
        joint_fault = [self.get_thumb_fault(), self.get_index_fault(), self.get_middle_fault(), self.get_ring_fault(), self.get_little_fault()]
        cmd_fault = self.joint_state_to_cmd_state(list=joint_fault)
        return cmd_fault

    def clear_faults(self):
        """API: clear finger fault codes"""
        pass

    def cmd_range_to_joint_range(self,cmd_list):
        """Convert command list to per-finger joint list according to mapping"""
        finger_mapping = {
            'thumb': [5, 10, 0, 11, 12, 15],
            'index': [6, 11, 1, 13, 14, 16],
            'middle': [7, 12, 2, 13, 14, 17],
            'ring': [8, 13, 3, 14, 15, 18],
            'little': [9, 14, 4, 15, 16, 19]
        }
        
        result = []
        
        for finger, indices in finger_mapping.items():
            finger_data = [cmd_list[i] for i in indices]
            result.append(finger_data)
        
        return result

    def joint_state_to_cmd_state(self,list):
        """
        Convert per-finger state list to command sequence state list
        """
        original = [""] * 20
        try:        
            for i, finger_data in enumerate(list):
                # Basic position mapping
                original[i] = finger_data[2]        # root
                original[i + 5] = finger_data[0]    # abduction
                original[i + 15] = finger_data[5]   # tip
                
                # Special positions
                if i == 0:  # thumb
                    original[10] = finger_data[1]   # roll
                    original[11] = finger_data[3]   # reserved
                    original[12] = finger_data[4]   # reserved
                else:  # other fingers
                    original[10 + i] = finger_data[1]  # reserved
                    if i <= 2:  # index, middle
                        original[12 + i] = finger_data[3]  # reserved
                        original[13 + i] = finger_data[4]  # reserved
                    else:  # ring, little
                        original[11 + i] = finger_data[3]  # reserved
                        original[12 + i] = finger_data[4]  # reserved
            
            return original
        except:
            return [-1] * 20

    def _list_d_value(self, list1, list2):
        """Check whether two lists differ significantly"""
        if list1 is None:
            return True
        for a, b in zip(list1, list2):
            if abs(b - a) > 2:
                return True
        return False

    def close_can_interface(self):
        """Close CAN interface"""
        if self.bus:
            self.bus.shutdown()
            self.running = False
    
    def get_serial_number(self):
        return [0] * 6

    def get_finger_order(self):
        return [
            "Thumb Base", "Index Finger Base", "Middle Finger Base", "Ring Finger Base", "Pinky Finger Base",
            "Thumb Abduction", "Index Finger Abduction", "Middle Finger Abduction", "Ring Finger Abduction",
            "Pinky Finger Abduction", "Thumb Horizontal Abduction", "Reserved", "Reserved", "Reserved", "Reserved",
            "Thumb Tip", "Index Finger Tip", "Middle Finger Tip", "Ring Finger Tip", "Pinky Finger Tip"
        ]
