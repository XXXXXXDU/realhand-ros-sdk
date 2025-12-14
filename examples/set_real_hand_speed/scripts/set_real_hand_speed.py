#!/usr/bin/env python3
import rospy,json
from std_msgs.msg import String

if __name__ == '__main__':
    '''
    L7:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[255,255,255,255,255,255,255]"

    other:
    rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=right _speed:="[100,100,100,100,100]"
    '''
    rospy.init_node('get_linker_hand_speed', anonymous=True)
    hand_type = rospy.get_param("~hand_type",default="left") # Set which hand's speed
    speed = rospy.get_param('~speed', default=[255,255,255,255,255])  # Get global parameters by default. O6 has 6 values, L7 has 7 values, others have 5 values.

    pub = rospy.Publisher("/cb_hand_setting_cmd",String,queue_size=10)
    msg = String()  # Create msg object
    count = 0  # counter
    # Set loop frequency
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # Since a single topic publish in ROS1 might be lost, loop 3 times to avoid loss
        dic = {
            "setting_cmd":"set_speed",
            "params":{
                "hand_type": hand_type,
                "speed":speed
            }
        }
        # Concatenate string
        msg.data = json.dumps(dic)

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("Data written: %s",msg.data)
        count += 1
        if count > 2:
            break
    
