#!/usr/bin/env python3
import rospy,rospkg
import time,os,sys,json
from std_msgs.msg import String,Header, Float32MultiArray
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg


'''
/cb_left_hand_info # Left hand topic
/cb_right_hand_info # Right hand topic

'''

class GetLinkerHandFault():
    def __init__(self,loop=False):
        self.loop = loop
        if self.loop == True:
            self.loop_acquisition()
            rospy.spin()
        else:
            self.single_acquisition()
    
    def loop_acquisition(self):
        rospy.Subscriber("/cb_left_hand_info",String,self.left_hand_cb,queue_size=1)
        rospy.Subscriber("/cb_right_hand_info", String, self.right_hand_cb, queue_size=1)
    def left_hand_cb(self,msg):
        data = json.loads(msg.data)
        # Current error codes of the five fingers [thumb, index, middle, ring, little] -> [0, 16, 0, 16, 0]
        left_fault = data["right_hand"]["fault"]
        tmp = []
        for index,item in enumerate(left_fault):
            a = self.get_active_bits(item)
            if len(a) > 0:
                tmp.append(a[0])
            else:
                tmp.append(0)
        ColorMsg(msg=f"Current left-hand five-finger error codes: {tmp}", color="green")
    
    def right_hand_cb(self, msg):
        data = json.loads(msg.data)
       # Current error codes of the five fingers [thumb, index, middle, ring, little] -> [0, 16, 0, 16, 0]
        right_fault = data["right_hand"]["fault"]
        tmp = []
        for index,item in enumerate(right_fault):
            a = self.get_active_bits(item)
            if len(a) > 0:
                tmp.append(a[0])
            else:
                tmp.append(0)
        ColorMsg(msg=f"Current right-hand five-finger error codes: {tmp}", color="green")
       


    def single_acquisition(self):
        left_hand = None
        right_hand = None
        try:
            left_hand = rospy.wait_for_message("/cb_left_hand_info",String,timeout=0.1)
        except:
            ColorMsg(msg="No data from left hand", color="yellow")
        try:
            right_hand = rospy.wait_for_message("/cb_right_hand_info",String,timeout=0.1)
        except:
            ColorMsg(msg="No data from right hand", color="yellow")
        if left_hand != None:
            data = json.loads(left_hand.data)
            # Current error codes of the five fingers [thumb, index, middle, ring, little] -> [0, 16, 0, 16, 0]
            left_fault = data["right_hand"]["fault"]
            ColorMsg(msg=f"Current left-hand five-finger error codes: {left_fault}", color="green")
        if right_hand != None:
            data = json.loads(right_hand.data)
            # Current error codes of the five fingers [thumb, index, middle, ring, little] -> [0, 16, 0, 16, 0]
            right_fault = data["right_hand"]["fault"]
            ColorMsg(msg=f"Current right-hand five-finger error codes: {right_fault}", color="green")
    
    def decimal_to_bits(self,decimal_value, bit_length=8):
        """
        Convert a decimal number to a fixed-length list of binary bits.
        """
        return [int(x) for x in f"{decimal_value:0{bit_length}b}"][::-1]

    def get_active_bits(self,decimal_value, bit_length=8):
        """
        Get all active bit positions (indices of bits with value 1) in the binary
        representation corresponding to the given decimal number.

        :param decimal_value: int, decimal input value.
        :param bit_length: int, binary length, default is 8.
        :return: list, indices of active bits.
        """
        # Call decimal_to_bits to convert decimal to a list of binary bits
        bit_list = self.decimal_to_bits(decimal_value, bit_length)
        # Traverse the list and find all positions where the value is 1
        return [i for i, bit in enumerate(bit_list) if bit == 1]

if __name__ == '__main__':
    rospy.init_node('get_linker_hand_fault', anonymous=True)
    loop = rospy.get_param('~loop', default=True)  # Default to getting the global parameter
    gh = GetLinkerHandFault(loop=loop)
