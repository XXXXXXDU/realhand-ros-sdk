#!/usr/bin/env python3
import rospy,rospkg
import signal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import can
import json
import yaml
import time
import threading
import sys
import os
import subprocess
from std_msgs.msg import Header, Float32MultiArray
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.real_hand_l25_can import RealHandL25Can
from utils.color_msg import ColorMsg
from utils.open_can import OpenCan
global package_path
# Create rospkg.RosPack object
rospack = rospkg.RosPack()
# Get the path of the specified package
package_name = "real_hand_sdk_ros"
package_path = rospack.get_path(package_name)

'''
Note: The L25 dual right-hand remote control mode is for internal testing, try not to use it.
This is the L25 dexterous hand 'be manipulated' module. Do not use this module on the same machine as real_hand_sdk_ros.
The ROS master starts real_hand_sdk_ros and `python set_remote_control.py --hand_type=right` in the examples/L25 directory.
The ROS slave starts this ROS module.
rosrun be_manipulated be_manipulated.py
'''
class BeManipulated:
    def __init__(self):
        self.left_hand = None
        self.right_hand = None
        self.motor_mode = rospy.get_param("~motor_mode", "enable") # Motor disable | enable mode parameter
        self.thumb_pos,self.index_pos,self.middle_pos,self.ring_pos,self.little_pos = [0.0]*5,[0.0]*5,[0.0]*5,[0.0]*5,[0.0]*5
        self.load_yaml()
        time.sleep(0.1)
        ColorMsg(msg=f"SDK version:{self.sdk_version}", color="green")
        self.open_can0()
        time.sleep(0.01)
        self.is_can_up_sysfs()
        self.manipulated_right()
    def manipulated_right(self):
        self.right_hand=RealHandL25Can(config=self.config, can_channel="can0",baudrate=1000000,can_id=0x27)
        self.right_hand.set_enable_mode()
        # Set finger speed 0~255
        self.right_hand.set_speed(speed=255)
        self.right_hand_cmd_sub = rospy.Subscriber("/cb_right_hand_control_cmd", JointState,self.right_position_send,queue_size=10)
    
        
    def left_position_send(self,msg):
        pos = msg.position
        self.left_hand.set_joint_positions(joint_ranges=list(pos))
    def right_position_send(self,msg):
        pos = msg.position
        self.right_hand.set_joint_positions(joint_ranges=list(pos))
    def pub_hand_status(self):
        while True:
            self.get_hand_status()

    def get_hand_status(self):
        if self.left_hand != None:
            left_hand_state = self.left_hand.get_current_status()
            if left_hand_state != None and len(left_hand_state) == 25:
                msg = self.create_joint_state_msg(position=left_hand_state)
                self.left_hand_status_pub.publish(msg)
        if self.right_hand != None:
            right_hand_state = self.right_hand.get_current_status()
            if right_hand_state != None and len(right_hand_state) == 25:
                msg = self.create_joint_state_msg(position=right_hand_state)
                self.right_hand_status_pub.publish(msg)
    
    def create_joint_state_msg(self, position, names=[]):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = list(map(float, position))
        msg.velocity = [0.0] * len(position)
        msg.effort = [0.0] * len(position)
        return msg


    
    def open_can0(self):
        try:
            # Check if the can0 interface already exists and is in the 'up' state
            result = subprocess.run(
                ["ip", "link", "show", "can0"],
                check=True,
                text=True,
                capture_output=True
            )
            if "state UP" in result.stdout:
                rospy.loginfo("CAN interface is already in UP state")
                return
            # If not in UP state, configure the interface
            subprocess.run(
                ["sudo", "-S", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"],
                input=f"{self.password}\n",
                check=True,
                text=True,
                capture_output=True
            )
            rospy.loginfo("CAN interface set up successfully")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to set up CAN interface: {e.stderr}")
        except Exception as e:
            rospy.logerr(f"An error occurred: {str(e)}")

    def is_can_up_sysfs(self, interface="can0"):
    # Check if the interface directory exists
        if not os.path.exists(f"/sys/class/net/{interface}"):
            return False
        # Read interface status
        try:
            with open(f"/sys/class/net/{interface}/operstate", "r") as f:
                state = f.read().strip()
            if state == "up":
                self.can_status = True
            return self.can_status
        except Exception as e:
            print(f"Error reading CAN interface state: {e}")
            return False
    def shutdown(self):
        pass
    
def signal_handler(sig, frame):
    sys.exit(0)  # Exit the program normally
if __name__ == '__main__':
    rospy.init_node('be_manipulated', anonymous=True)
    rospy.Rate(60)
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill 命令
    
    try:
        # Check if the can port is open, if not, wait and retry. Usually, the usb-to-can device is not plugged in.
        while True:
            can = OpenCan()
            can.open_can0()
            time.sleep(0.001)
            o = can.is_can_up_sysfs()
            if o == False:
                ColorMsg(msg=f"Failed to open can0 port, retrying in 3 seconds", color="red")
                time.sleep(3)
            else:
                break
        real_hand = BeManipulated()
        rospy.spin()
    except rospy.ROSInterruptException:
        real_hand.shutdown()
        rospy.loginfo("Node shutdown complete.")