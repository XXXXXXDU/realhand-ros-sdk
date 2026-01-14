import rospy,rospkg
import time,os,sys
from std_msgs.msg import String,Header, Float32MultiArray
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg


'''
/cb_left_hand_force # Left hand force sensor
/cb_right_hand_force # Right hand force sensor
'''

class GetRealHandPressure():
    def __init__(self,loop=False):
        self.loop = loop
        if self.loop == True:
            self.loop_acquisition()
            rospy.spin()
        else:
            self.single_acquisition()
    
    def loop_acquisition(self):
        rospy.Subscriber("/cb_left_hand_force",Float32MultiArray,self.left_hand_cb,queue_size=1)
        rospy.Subscriber("/cb_right_hand_force", Float32MultiArray, self.right_hand_cb, queue_size=1)
    def left_hand_cb(self,msg):
        data = self.list_slice(data=msg.data)
        # Five-finger normal force. The larger the value, the greater the normal force.
        # [Thumb, Index, Middle, Ring, Little]->[0.0, 26.0, 34.0, 255.0, 0.0]
        hand_normal_force = data[0]
        ColorMsg(msg=f"Left hand five-finger normal force: {list(hand_normal_force)}", color="green")
        # Five-finger tangential force. The larger the value, the greater the tangential force.
        # [Thumb, Index, Middle, Ring, Little]->[0.0, 12.0, 3.0, 4.0, 0.0]
        hand_tangential_force = data[1]
        ColorMsg(msg=f"Left hand five-finger tangential force: {list(hand_tangential_force)}", color="green")
        # Five-finger tangential force direction. Values 0-127 correspond to actual tangential force angles 0-359. Note that when the finger is not under force and the direction cannot be determined, the value remains 255.
        hand_tangential_force_dir = data[2]
        ColorMsg(msg=f"Left hand five-finger tangential force direction: {list(hand_tangential_force_dir)}", color="green")
        # Five-finger proximity sensing
        hand_approach_inc = data[3]
        ColorMsg(msg=f"Left hand five-finger proximity sensing: {list(hand_approach_inc)}", color="green")
    
    def right_hand_cb(self, msg):
        data = self.list_slice(data=msg.data)
        # Five-finger normal force. The larger the value, the greater the normal force.
        # [Thumb, Index, Middle, Ring, Little]->[0.0, 26.0, 34.0, 255.0, 0.0]
        hand_normal_force = data[0]
        ColorMsg(msg=f"Right hand five-finger normal force: {list(hand_normal_force)}", color="green")
        # Five-finger tangential force. The larger the value, the greater the tangential force.
        # [Thumb, Index, Middle, Ring, Little]->[0.0, 12.0, 3.0, 4.0, 0.0]
        hand_tangential_force = data[1]
        ColorMsg(msg=f"Right hand five-finger tangential force: {list(hand_tangential_force)}", color="green")
        # Five-finger tangential force direction. Values 0-127 correspond to actual tangential force angles 0-359. Note that when the finger is not under force and the direction cannot be determined, the value remains 255.
        hand_tangential_force_dir = data[2]
        ColorMsg(msg=f"Right hand five-finger tangential force direction: {list(hand_tangential_force_dir)}", color="green")
        # Five-finger proximity sensing
        hand_approach_inc = data[3]
        ColorMsg(msg=f"Right hand five-finger proximity sensing: {list(hand_approach_inc)}", color="green")


    def single_acquisition(self):
        left_hand = None
        right_hand = None
        try:
            left_hand = rospy.wait_for_message("/cb_left_hand_force",Float32MultiArray,timeout=0.1)
        except:
            ColorMsg(msg="No data for left hand", color="yellow")
        try:
            right_hand = rospy.wait_for_message("/cb_right_hand_force",Float32MultiArray,timeout=0.1)
        except:
            ColorMsg(msg="No data for right hand", color="yellow")
        if left_hand != None:
            data = self.list_slice(data=left_hand.data)
            # Five-finger normal force. The larger the value, the greater the normal force.
            # [Thumb, Index, Middle, Ring, Little]->[0.0, 26.0, 34.0, 255.0, 0.0]
            left_hand_normal_force = data[0]
            ColorMsg(msg=f"Left hand five-finger normal force: {list(left_hand_normal_force)}", color="green")
            # Five-finger tangential force. The larger the value, the greater the tangential force.
            # [Thumb, Index, Middle, Ring, Little]->[0.0, 12.0, 3.0, 4.0, 0.0]
            left_hand_tangential_force = data[1]
            ColorMsg(msg=f"Left hand five-finger tangential force: {list(left_hand_tangential_force)}", color="green")
            # Five-finger tangential force direction. Values 0-127 correspond to actual tangential force angles 0-359. Note that when the finger is not under force and the direction cannot be determined, the value remains 255.
            left_hand_tangential_force_dir = data[2]
            ColorMsg(msg=f"Left hand five-finger tangential force direction: {list(left_hand_tangential_force_dir)}", color="green")
            # Five-finger proximity sensing
            left_hand_approach_inc = data[3]
            ColorMsg(msg=f"Left hand five-finger proximity sensing: {list(left_hand_approach_inc)}", color="green")
        if right_hand != None:
            data = self.list_slice(data=right_hand.data)
            # Five-finger normal force. The larger the value, the greater the normal force.
            # [Thumb, Index, Middle, Ring, Little]->[0.0, 26.0, 34.0, 255.0, 0.0]
            right_hand_normal_force = data[0]
            ColorMsg(msg=f"Right hand five-finger normal force: {list(right_hand_normal_force)}", color="green")
            # Five-finger tangential force. The larger the value, the greater the tangential force.
            # [Thumb, Index, Middle, Ring, Little]->[0.0, 12.0, 3.0, 4.0, 0.0]
            right_hand_tangential_force = data[1]
            ColorMsg(msg=f"Right hand five-finger tangential force: {list(right_hand_tangential_force)}", color="green")
            # Five-finger tangential force direction. Values 0-127 correspond to actual tangential force angles 0-359. Note that when the finger is not under force and the direction cannot be determined, the value remains 255.
            right_hand_tangential_force_dir = data[2]
            ColorMsg(msg=f"Right hand five-finger tangential force direction: {list(right_hand_tangential_force_dir)}", color="green")
            # Five-finger proximity sensing
            right_hand_approach_inc = data[3]
            ColorMsg(msg=f"Right hand five-finger proximity sensing: {list(right_hand_approach_inc)}", color="green")
    
    def list_slice(self,data,n=5):
        n = 5  # Length of each sublist
        result = [data[i:i + n] for i in range(0, len(data), n)]
        return result

if __name__ == '__main__':
    rospy.init_node('get_real_hand_force', anonymous=True)
    loop = rospy.get_param('~loop', default=True)  # Get global parameter by default
    gh = GetRealHandPressure(loop=loop)
    