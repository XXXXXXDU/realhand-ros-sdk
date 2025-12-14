#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
from utils.color_msg import ColorMsg

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # Invalid CAN frame property | No return
    # Parallel instruction area
    ROLL_POS = 0x01  # Roll joint position | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    YAW_POS = 0x02  # Yaw joint position | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    ROOT1_POS = 0x03  # Root 1 joint position | The finger root joint closest to the palm
    ROOT2_POS = 0x04  # Root 2 joint position | The finger root joint closest to the palm
    ROOT3_POS = 0x05  # Root 3 joint position | The finger root joint closest to the palm
    TIP_POS = 0x06  # Tip joint position | The finger root joint closest to the palm

    ROLL_SPEED = 0x09  # Roll joint speed | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    YAW_SPEED = 0x0A  # Yaw joint speed | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    ROOT1_SPEED = 0x0B  # Root 1 joint speed | The finger root joint closest to the palm
    ROOT2_SPEED = 0x0C  # Root 2 joint speed | The finger root joint closest to the palm
    ROOT3_SPEED = 0x0D  # Root 3 joint speed | The finger root joint closest to the palm
    TIP_SPEED = 0x0E  # Tip joint speed | The finger root joint closest to the palm

    ROLL_TORQUE = 0x11  # Roll joint torque | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    YAW_TORQUE = 0x12  # Yaw joint torque | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    ROOT1_TORQUE = 0x13  # Root 1 joint torque | The finger root joint closest to the palm
    ROOT2_TORQUE = 0x14  # Root 2 joint torque | The finger root joint closest to the palm
    ROOT3_TORQUE = 0x15  # Root 3 joint torque | The finger root joint closest to the palm
    TIP_TORQUE = 0x16  # Tip joint torque | The finger root joint closest to the palm

    ROLL_FAULT = 0x19  # Roll joint fault code | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    YAW_FAULT = 0x1A  # Yaw joint fault code | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    ROOT1_FAULT = 0x1B  # Root 1 joint fault code | The finger root joint closest to the palm
    ROOT2_FAULT = 0x1C  # Root 2 joint fault code | The finger root joint closest to the palm
    ROOT3_FAULT = 0x1D  # Root 3 joint fault code | The finger root joint closest to the palm
    TIP_FAULT = 0x1E  # Tip joint fault code | The finger root joint closest to the palm

    ROLL_TEMPERATURE = 0x21  # Roll joint temperature | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    YAW_TEMPERATURE = 0x22  # Yaw joint temperature | Coordinate system is at the base of each finger, rotation angle defined by straight finger state
    ROOT1_TEMPERATURE = 0x23  # Root 1 joint temperature | The finger root joint closest to the palm
    ROOT2_TEMPERATURE = 0x24  # Root 2 joint temperature | The finger root joint closest to the palm
    ROOT3_TEMPERATURE = 0x25  # Root 3 joint temperature | The finger root joint closest to the palm
    TIP_TEMPERATURE = 0x26  # Tip joint temperature | The finger root joint closest to the palm
    # End of parallel instruction area

    # Serial instruction area
    THUMB_POS = 0x41  # Thumb joint position | Returns this type of data
    INDEX_POS = 0x42  # Index finger joint position | Returns this type of data
    MIDDLE_POS = 0x43  # Middle finger joint position | Returns this type of data
    RING_POS = 0x44  # Ring finger joint position | Returns this type of data
    LITTLE_POS = 0x45  # Little finger joint position | Returns this type of data

    THUMB_SPEED = 0x49  # Thumb speed | Returns this type of data
    INDEX_SPEED = 0x4A  # Index finger speed | Returns this type of data
    MIDDLE_SPEED = 0x4B  # Middle finger speed | Returns this type of data
    RING_SPEED = 0x4C  # Ring finger speed | Returns this type of data
    LITTLE_SPEED = 0x4D  # Little finger speed | Returns this type of data

    THUMB_TORQUE = 0x51  # Thumb torque | Returns this type of data
    INDEX_TORQUE = 0x52  # Index finger torque | Returns this type of data
    MIDDLE_TORQUE = 0x53  # Middle finger torque | Returns this type of data
    RING_TORQUE = 0x54  # Ring finger torque | Returns this type of data
    LITTLE_TORQUE = 0x55  # Little finger torque | Returns this type of data

    THUMB_FAULT = 0x59  # Thumb fault code | Returns this type of data
    INDEX_FAULT = 0x5A  # Index finger fault code | Returns this type of data
    MIDDLE_FAULT = 0x5B  # Middle finger fault code | Returns this type of data
    RING_FAULT = 0x5C  # Ring finger fault code | Returns this type of data
    LITTLE_FAULT = 0x5D  # Little finger fault code | Returns this type of data

    THUMB_TEMPERATURE = 0x61  # Thumb temperature | Returns this type of data
    INDEX_TEMPERATURE = 0x62  # Index finger temperature | Returns this type of data
    MIDDLE_TEMPERATURE = 0x63  # Middle finger temperature | Returns this type of data
    RING_TEMPERATURE = 0x64  # Ring finger temperature | Returns this type of data
    LITTLE_TEMPERATURE = 0x65  # Little finger temperature | Returns this type of data
    # End of serial instruction area

    # Combined instruction area, merging non-essential single-control data for the same finger
    FINGER_SPEED = 0x81  # Finger speed | Returns this type of data
    FINGER_TORQUE = 0x82  # Torque | Returns this type of data
    FINGER_FAULT = 0x83  # Finger fault code | Returns this type of data

    # Fingertip sensor data group
    HAND_NORMAL_FORCE = 0x90  # Five-finger normal pressure
    HAND_TANGENTIAL_FORCE = 0x91  # Five-finger tangential pressure
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # Five-finger tangential direction
    HAND_APPROACH_INC = 0x93  # Five-finger proximity sensing

    THUMB_ALL_DATA = 0x98  # All data for the thumb
    INDEX_ALL_DATA = 0x99  # All data for the index finger
    MIDDLE_ALL_DATA = 0x9A  # All data for the middle finger
    RING_ALL_DATA = 0x9B  # All data for the ring finger
    LITTLE_ALL_DATA = 0x9C  # All data for the little finger
    # Action instruction · ACTION
    ACTION_PLAY = 0xA0  # Action

    # Configuration command · CONFIG
    HAND_UID = 0xC0  # Device unique identifier
    HAND_HARDWARE_VERSION = 0xC1  # Hardware version
    HAND_SOFTWARE_VERSION = 0xC2  # Software version
    HAND_COMM_ID = 0xC3  # Device ID
    HAND_FACTORY_RESET = 0xCE  # Factory reset
    HAND_SAVE_PARAMETER = 0xCF  # Save parameters

    WHOLE_FRAME = 0xF0  # Full frame transmission | Returns one-byte frame property + entire struct, exclusive for 485 and network transmission

class LinkerHandL25Can:
    def __init__(self, config, can_channel='can0', baudrate=1000000, can_id=0x28):
        self.config = config
        self.can_id = can_id
        self.running = True
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        # Speed
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d = [],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        # Initialize CAN bus based on the operating system
        if sys.platform == "linux":
            self.bus = can.interface.Bus(
                channel=can_channel, interface="socketcan", bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        elif sys.platform == "win32":
            self.bus = can.interface.Bus(
                channel='PCAN_USBBUS1', interface='pcan', bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        else:
            raise EnvironmentError("Unsupported platform for CAN interface")

        # Initialize publisher and related parameters based on can_id
        if can_id == 0x28:  # Left hand
            self.hand_exists = config['LINKER_HAND']['LEFT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['LEFT_HAND']['NAME']
        elif can_id == 0x27:  # Right hand

            self.hand_exists = config['LINKER_HAND']['RIGHT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['RIGHT_HAND']['NAME']


        # Start the receiving thread
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list):
        """
        Send a command to the CAN bus
        :param frame_property: Data frame property
        :param data_list: Data payload
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            #print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")
        time.sleep(0.002)

    def receive_response(self):
        """
        Receive and process response messages from the CAN bus
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)  # Blocking receive with a 1-second timeout
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
    

    def set_joint_positions(self, joint_ranges):
        if len(joint_ranges) == 25:
            l24_pose = self.joint_map(joint_ranges)
            # Use a list comprehension to slice the list into sub-arrays of 6 elements each
            chunks = [l24_pose[i:i+6] for i in range(0, 30, 6)]
            self.send_command(FrameProperty.THUMB_POS, chunks[0])
            time.sleep(0.001)
            self.send_command(FrameProperty.INDEX_POS, chunks[1])
            time.sleep(0.001)
            self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            time.sleep(0.001)
            self.send_command(FrameProperty.RING_POS, chunks[3])
            time.sleep(0.001)
            self.send_command(FrameProperty.LITTLE_POS, chunks[4])
            time.sleep(0.001)
        #self.set_tip_positions(joint_ranges[:5])
        #print(l24_pose)
    
    # Set roll positions for all fingers
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # Set yaw positions for all fingers
    def set_yaw_positions(self, joint_ranges):
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # Set root1 positions for all fingers
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # Set root2 positions for all fingers
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # Set root3 positions for all fingers
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # Set tip positions for all fingers
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # Get thumb joint positions
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # Get index finger joint positions
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # Get middle finger joint positions
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # Get ring finger joint positions
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # Get little finger joint positions
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # Disable mode 01
    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)
    # Enable mode 00
    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)

    
    def set_speed(self, speed):
        self.speed = [speed]*6
        ColorMsg(msg=f"L25 speed set to: {self.speed}", color="yellow")
        self.send_command(FrameProperty.THUMB_SPEED, self.speed)
        self.send_command(FrameProperty.INDEX_SPEED, self.speed)
        self.send_command(FrameProperty.MIDDLE_SPEED, self.speed)
        self.send_command(FrameProperty.RING_SPEED, self.speed)
        self.send_command(FrameProperty.LITTLE_SPEED, self.speed)
        
    def set_finger_torque(self, torque):
        self.send_command(0x42, torque)

    def request_device_info(self):
        self.send_command(0xC0, [0])
        self.send_command(0xC1, [0])
        self.send_command(0xC2, [0])

    def save_parameters(self):
        self.send_command(0xCF, [])
    def process_response(self, msg):
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == 0x01:
                self.x01 = list(response_data)
            elif frame_type == 0x02:
                self.x02 = list(response_data)
            elif frame_type == 0x03:
                self.x03 = list(response_data)
            elif frame_type == 0x04:
                self.x04 = list(response_data)
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
                print("_-"*20)
                print(self.x06)
            elif frame_type == 0xC0:
                print(f"Device ID info: {response_data}")
                if self.can_id == 0x28:
                    self.right_hand_info = response_data
                elif self.can_id == 0x27:
                    self.left_hand_info = response_data
            elif frame_type == 0x08:
                self.x08 = list(response_data)
            elif frame_type == 0x09:
                self.x09 = list(response_data)
            elif frame_type == 0x0A:
                self.x0A = list(response_data)
            elif frame_type == 0x0B:
                self.x0B = list(response_data)
            elif frame_type == 0x0C:
                self.x0C = list(response_data)
            elif frame_type == 0x0D:
                self.x0D = list(response_data)
            elif frame_type == 0x22:
                #ColorMsg(msg=f"Five-finger tangential pressure direction: {list(response_data)}")
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                #ColorMsg(msg=f"Five-finger proximity: {list(response_data)}")
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x41: # Thumb joint position return value
                self.x41 = list(response_data)
            elif frame_type == 0x42: # Index finger joint position return value
                #print("Index finger all joint states",list(response_data))
                self.x42 = list(response_data)
            elif frame_type == 0x43: # Middle finger joint position return value
                self.x43 = list(response_data)
            elif frame_type == 0x44: # Ring finger joint position return value
                self.x44 = list(response_data)
            elif frame_type == 0x45: # Little finger joint position return value
                self.x45 = list(response_data)
            elif frame_type == 0x49: # Thumb speed return value
                self.x49 = list(response_data)
            elif frame_type == 0x4a: # Index finger speed return value
                self.x4a = list(response_data)
            elif frame_type == 0x4b: # Middle finger speed return value
                self.x4b = list(response_data)
            elif frame_type == 0x4c: # Ring finger speed return value
                self.x4c = list(response_data)
            elif frame_type == 0x4d: # Little finger speed return value
                self.x4d = list(response_data)

    # topic mapping to L24
    def joint_map(self, pose):
        # L24 CAN data defaults to receiving 30 data points
        l24_pose = [0.0] * 30  # Initialize l24_pose with 30 zeros

        # Mapping table, simplified with a dictionary
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: None, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        # Iterate through the mapping dictionary to map values
        for l24_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l24_pose[l24_idx] = pose[pose_idx]

        return l24_pose

    # Convert L24 state values to CMD format state values
    def state_to_cmd(self, l24_state):
        # L24 CAN defaults to receiving 30 data points, initialize pose with 25 zeros
        pose = [0.0] * 25  # The original command data for controlling L24 was 25 points

        # Mapping relationship, the dictionary stores the mapping between l24_state index and pose index
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        # Iterate through the mapping dictionary to update pose values
        for l24_idx, pose_idx in mapping.items():
            pose[pose_idx] = l24_state[l24_idx]
        return pose

    # Get all joint data
    def get_current_status(self, j=''):
        time.sleep(0.01)
        self.send_command(FrameProperty.THUMB_POS, j)
        self.send_command(FrameProperty.INDEX_POS,j)
        self.send_command(FrameProperty.MIDDLE_POS,j)
        self.send_command(FrameProperty.RING_POS,j)
        self.send_command(FrameProperty.LITTLE_POS, j)
        #return self.x41, self.x42, self.x43, self.x44, self.x45
        time.sleep(0.01)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l24_state = self.state_to_cmd(l24_state=state)
            return l24_state
    
    def get_speed(self,j=''):
        time.sleep(0.1)
        self.send_command(FrameProperty.THUMB_SPEED, j) # Thumb speed
        self.send_command(FrameProperty.INDEX_SPEED, j) # Index finger speed
        self.send_command(FrameProperty.MIDDLE_SPEED, j) # Middle finger speed
        self.send_command(FrameProperty.RING_SPEED, j) # Ring finger speed
        self.send_command(FrameProperty.LITTLE_SPEED, j) # Little finger speed
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l24_speed = self.state_to_cmd(l24_state=speed)
            return l24_speed
    
    def get_finger_torque(self):
        return self.finger_torque
    # def get_current(self):
    #     return self.x06
    # def get_fault(self):
    #     return self.x07
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # Close the CAN bus

    '''
    This method is only for demonstrating data relationship mapping. It's better to use the method above for actual use.
    '''
    def joint_map_2(self, pose):
        l24_pose = [0.0]*30 # L24 CAN defaults to receiving 30 data points. The pose command for L24 sends 25 data points by default, so mapping is needed here.
        '''
        Mapping is required
        # L24 CAN data format
        #["Thumb Yaw 0-10", "Thumb Side 1-5", "Thumb Root 2-0", "Thumb Middle 3-15", "Reserved 4-", "Thumb Tip 5-20", "Reserved 6-", "Index Side 7-6", "Index Root 8-1", "Index Middle 9-16", "Reserved 10-", "Index Tip 11-21", "Reserved 12-", "Reserved 13-", "Middle Root 14-2", "Middle Middle 15-17", "Reserved 16-", "Middle Tip 17-22", "Reserved 18-", "Ring Side 19-8", "Ring Root 20-3", "Ring Middle 21-18", "Reserved 22-", "Ring Tip 23-23", "Reserved 24-", "Little Side 25-9", "Little Root 26-4", "Little Middle 27-19", "Reserved 28-", "Little Tip 29-24"]
        # CMD received data format
        #["Thumb Root 0", "Index Root 1", "Middle Root 2", "Ring Root 3", "Little Root 4", "Thumb Side 5", "Index Side 6", "Middle Side", "Ring Side 8", "Little Side 9", "Thumb Yaw 10", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Middle 15", "Index Middle 16", "Middle Middle 17", "Ring Middle 18", "Little Middle 19", "Thumb Tip 20", "Index Tip 21", "Middle Tip 22", "Ring Tip 23", "Little Tip 24"]
        '''
        l24_pose[0] = pose[10]
        l24_pose[1] = pose[5]
        l24_pose[2] = pose[0]
        l24_pose[3] = pose[15]
        l24_pose[4] = 0.0
        l24_pose[5] = pose[20]
        l24_pose[6] = 0.0
        l24_pose[7] = pose[6]
        l24_pose[8] = pose[1]
        l24_pose[9] = pose[16]
        l24_pose[10] = 0.0
        l24_pose[11] = pose[21]
        l24_pose[12] = 0.0
        l24_pose[13] = 0.0
        l24_pose[14] = pose[2]
        l24_pose[15] = pose[17]
        l24_pose[16] = 0.0
        l24_pose[17] = pose[22]
        l24_pose[18] = 0.0
        l24_pose[19] = pose[8]
        l24_pose[20] = pose[3]
        l24_pose[21] = pose[18]
        l24_pose[22] = 0.0
        l24_pose[23] = pose[23]
        l24_pose[24] = 0.0
        l24_pose[25] = pose[9]
        l24_pose[26] = pose[4]
        l24_pose[27] = pose[19]
        l24_pose[28] = 0.0
        l24_pose[29] = pose[24]
        return l24_pose
