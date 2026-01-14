import rospy,rospkg
import time,os,sys
from std_msgs.msg import String,Header
from sensor_msgs.msg import JointState

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.color_msg import ColorMsg

'''
/cb_left_hand_state
/cb_left_hand_state_arc
/cb_right_hand_state
/cb_right_hand_state_arc
'''
class L20GetRealHandState():
    def __init__(self,loop=False):
        self.loop = loop # Used to determine whether to loop to get status data
        if self.loop == True:
            self.left_hand_state_sub = rospy.Subscriber("/cb_left_hand_state",JointState,self.left_hand_state_cb,queue_size=1)
            self.left_hand_state_arc_sub = rospy.Subscriber("/cb_left_hand_state_src",JointState,self.left_hand_state_arc_cb,queue_size=1)
            self.right_hand_state_sub = rospy.Subscriber("/cb_right_hand_state",JointState,self.right_hand_state_cb,queue_size=1)
            self.right_hand_state_arc_sub = rospy.Subscriber("/cb_right_hand_state_arc",JointState,self.right_hand_state_arc_cb,queue_size=1)
            rospy.spin()
        else:
            self.single_smg() # Get status data once
    # Left hand range status value
    def left_hand_state_cb(self,msg):
        position = msg.position
        if len(list(position)) > 10:
            hand_type = "L20"
        else:
            hand_type = "L10"
        velocity = msg.velocity
        effort = msg.effort
        name = msg.name
        print("="*30)
        ColorMsg(msg=f"{hand_type} left hand current state position: {position}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state velocity: {velocity}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state effort: {effort}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state name: {name}", color="yellow")
        print("="*30)
    # Left hand radian status value
    def left_hand_state_arc_cb(self,msg):
        position = msg.position
        if len(list(position)) > 10:
            hand_type = "L20"
        else:
            hand_type = "L10"
        velocity = msg.velocity
        effort = msg.effort
        name = msg.name
        print("-"*30)
        ColorMsg(msg=f"{hand_type} left hand current state radian position: {position}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state radian velocity: {velocity}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state radian effort: {effort}", color="yellow")
        ColorMsg(msg=f"{hand_type} left hand current state radian name: {name}", color="yellow")
        print("-"*30)
    # Right hand range status value
    def right_hand_state_cb(self,msg):
        position = msg.position
        if len(list(position)) > 10:
            hand_type = "L20"
        else:
            hand_type = "L10"
        velocity = msg.velocity
        effort = msg.effort
        name = msg.name
        print("="*30)
        ColorMsg(msg=f"{hand_type} right hand current state position: {position}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state velocity: {velocity}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state effort: {effort}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state name: {name}", color="green")
        print("="*30)
    # Right hand radian status value
    def right_hand_state_arc_cb(self,msg):
        position = msg.position
        if len(list(position)) > 10:
            hand_type = "L20"
        else:
            hand_type = "L10"
        velocity = msg.velocity
        effort = msg.effort
        name = msg.name
        print("-"*30)
        ColorMsg(msg=f"{hand_type} right hand current state radian position: {position}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state radian velocity: {velocity}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state radian effort: {effort}", color="green")
        ColorMsg(msg=f"{hand_type} right hand current state radian name: {name}", color="green")
        print("-"*30)

    def single_smg(self):
        left_hand_range_state = None
        left_hand_arc_state = None
        right_hand_range_state = None
        right_hand_arc_state = None
        try:
            left_hand_range_state = rospy.wait_for_message("/cb_left_hand_state", JointState, timeout=0.3)
        except rospy.ROSException as e:
            #rospy.logerr("cb_left_hand_state topic timed out")
            pass
        try:
            left_hand_arc_state = rospy.wait_for_message("/cb_left_hand_state_arc", JointState, timeout=0.3)
        except rospy.ROSException as e:
            #rospy.logerr("cb_left_hand_state_arc topic timed out")
            pass
        try:
            right_hand_range_state = rospy.wait_for_message("/cb_right_hand_state", JointState, timeout=0.3)
        except rospy.ROSException as e:
            #rospy.logerr("cb_right_hand_state topic timed out")
            pass
        try:
            right_hand_arc_state = rospy.wait_for_message("/cb_right_hand_state_arc", JointState, timeout=0.3)
        except rospy.ROSException as e:
            #rospy.logerr("cb_right_hand_state_arc topic timed out")
            pass
        if left_hand_range_state != None:
            position = left_hand_range_state.position
            if len(list(position)) > 10:
                hand_type = "L20"
            else:
                hand_type = "L10"
            velocity = left_hand_range_state.velocity
            effort = left_hand_range_state.effort
            name = left_hand_range_state.name
            print("="*30)
            ColorMsg(msg=f"{hand_type} left hand current state position: {position}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state velocity: {velocity}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state effort: {effort}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state name: {name}", color="yellow")
            print("="*30)
        if left_hand_arc_state !=None:
            position = left_hand_arc_state.position
            if len(list(position)) > 10:
                hand_type = "L20"
            else:
                hand_type = "L10"
            velocity = left_hand_arc_state.velocity
            effort = left_hand_arc_state.effort
            name = left_hand_arc_state.name
            print("-"*30)
            ColorMsg(msg=f"{hand_type} left hand current state radian position: {position}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state radian velocity: {velocity}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state radian effort: {effort}", color="yellow")
            ColorMsg(msg=f"{hand_type} left hand current state radian name: {name}", color="yellow")
            print("-"*30)
        if right_hand_range_state != None:
            position = right_hand_range_state.position
            if len(list(position)) > 10:
                hand_type = "L20"
            else:
                hand_type = "L10"
            velocity = right_hand_range_state.velocity
            effort = right_hand_range_state.effort
            name = right_hand_range_state.name
            print("="*30)
            ColorMsg(msg=f"{hand_type} right hand current state position: {position}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state velocity: {velocity}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state effort: {effort}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state name: {name}", color="green")
            print("="*30)
        if right_hand_arc_state != None:
            position = right_hand_arc_state.position
            if len(list(position)) > 10:
                hand_type = "L20"
            else:
                hand_type = "L10"
            velocity = right_hand_arc_state.velocity
            effort = right_hand_arc_state.effort
            name = right_hand_arc_state.name
            print("-"*30)
            ColorMsg(msg=f"{hand_type} right hand current state radian position: {position}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state radian velocity: {velocity}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state radian effort: {effort}", color="green")
            ColorMsg(msg=f"{hand_type} right hand current state radian name: {name}", color="green")
            print("-"*30)


if __name__ == '__main__':
    rospy.init_node('L20_get_real_hand_state', anonymous=True)
    loop = rospy.get_param('~loop', default="False")  # Get global parameter by default
    lh = L20GetRealHandState(loop=loop)
    