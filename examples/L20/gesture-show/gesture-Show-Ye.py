#!/usr/bin/env python

 

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
import std_msgs.msg

import time

import threading

import signal
import sys

show_count=0
show_count_obj=0
show_step=0
joint_state = JointState()
def send_messages():

    rospy.init_node('dong_test_sender', anonymous=True)

    pub = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)

    rate = rospy.Rate(30)  # Set frequency to 30Hz
    joint_state.header = std_msgs.msg.Header()
    joint_state.header.seq=0
    joint_state.header.stamp = rospy.Time.now() 
    joint_state.header.frame_id = ''
    joint_state.name=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',

                        'joint7', 'joint8', 'joint9', 'joint10', 'joint11', 'joint12',

                        'joint13', 'joint14', 'joint15', 'joint16', 'joint17', 'joint18',

                        'joint19', 'joint20']
    joint_state.velocity = [0] * len(joint_state.position)  # Same length as the position array, all filled with 0
    joint_state.effort = [0] * len(joint_state.position)  
    pub.publish(joint_state)
    while not rospy.is_shutdown():  # Loop for 1 second
        position =show_left()
        if(position is not None):
            joint_state.position = position
        rospy.loginfo(f"Publishing joint states {joint_state.__str__}")
        pub.publish(joint_state)
        rate.sleep()

def show_left():
    global show_count
    global show_count_obj
    global show_step
    show_count= show_count+1
    if(show_count>=show_count_obj):
        show_count=0
        if(show_step==0):
            show_step=show_step+1
            show_count_obj = 50
            return[250, 250, 250, 250, 250, 250, 128, 128, 128, 128, 250, 0, 0, 0, 0, 250, 250, 250, 250, 250]
        elif(show_step==1): #// Close little and ring fingers
            show_step=show_step+1
            show_count_obj = 10
            return[250,250,250, 0,  0, 250,128,128,128,128,250, 0, 0, 0, 0,250,250,250,  0, 0]
        elif(show_step==2): #// Place thumb on top of little and ring fingers
            show_step=show_step+1
            show_count_obj = 30
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==3): #// Tilt index and middle fingers to one side
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180,200,200,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==4): #// Other side
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180, 50, 50,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==5): #// Return both to center
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==6): #// Index and middle fingers make a Y shape
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180, 58,200,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==7): #// Retract Y shape
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==8): #// Index and middle fingers make a Y shape
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180, 58,200,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==9): #// Retract Y shape
            show_step=show_step+1
            show_count_obj = 10
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==10): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            return[100,100,100, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,100,100,  0, 0]
        elif(show_step==11): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==12): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            return[100,100,100, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,100,100,  0, 0]
        elif(show_step==13): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,250, 0,  0, 180,128,128,128,128,200, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==14): #// Curl thumb
            show_step=show_step+1
            show_count_obj = 40
            return[250,250,250, 0,  0, 150,128,128,128,128,250, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==15): #// Tuck thumb into palm
            show_step=show_step+1
            show_count_obj = 10
            return[250,250,250, 0,  0,  5 ,128,128,128,128,250, 0, 0, 0, 0,  0,250,250,  0, 0]
        elif(show_step==16): #// Close 4 fingers
            show_step=show_step+1
            show_count_obj = 30
            return[250,100,100,100,100, 5 ,128,128,128,128,250, 0, 0, 0, 0,  0,10, 10,  10, 10]
        elif(show_step==17): #// Open 4 fingers and thumb sequentially
            show_step=show_step+1
            show_count_obj = 15
            return[250,100,100,100,250, 5 ,128,128,128,128,250, 0, 0, 0, 0,  0,10, 10,  10, 250]
        elif(show_step==18): #// 1
            show_step=show_step+1
            show_count_obj = 15
            return[250,100,100,250,250, 5 ,128,128,128,128,250, 0, 0, 0, 0,  0,10, 10,  250, 250]
        elif(show_step==19): #// 2
            show_step=show_step+1
            show_count_obj = 15
            return[250,100,250,250,250, 5 ,128,128,128,128,250, 0, 0, 0, 0,  0,10, 250,  250, 250]
        elif(show_step==20): #// 3
            show_step=show_step+1
            show_count_obj = 15
            return[250,250,250,250,250, 5 ,128,128,128,128,250, 0, 0, 0, 0,  0,250, 250,  250, 250]
        elif(show_step==21): #// 4
            show_step=show_step+1
            show_count_obj = 10
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==22): #// Bring thumb closer
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250,250,250,128,128,128,128, 10, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==23): #// Rotate thumb towards palm
            show_step=show_step+1
            show_count_obj = 40
            return[  0,250,250,250,250,250,128,128,128,128, 10, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==24): #// Return to initial position in two steps
            show_step=show_step+1
            show_count_obj = 30
            return[  0,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==25): #// 1
            show_step=show_step+1
            show_count_obj = 50
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==26): #// 2
            show_step=show_step+1
            show_count_obj = 10
            return[250,250,250,250,250,250,200,200,200,200,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==27): #// 3
            show_step=show_step+1
            show_count_obj = 15
            return[250,250,250,250,250,250, 80, 80, 80, 80,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==28): #// 4
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250, 250,  250, 250]
        elif(show_step==29): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            return [250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0, 250,  250, 250]
        elif(show_step==30): #// Curl 4 fingers
            show_step=show_step+1
            show_count_obj = 15
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,  250, 250]
        elif(show_step==31): #// 4
            show_step=show_step+1
            show_count_obj = 15
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0, 250]
        elif(show_step==32): #// 4
            show_step=show_step+1
            show_count_obj = 15
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==33): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            return[250,  0,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==34): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            return[250,  0,  0,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==35): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            return[250,  0,  0,  0,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==36): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            return[250,  0,  0,  0,  0,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==37): #// Curl thumb
            show_step=show_step+1
            show_count_obj = 40
            return [  0,  0,  0,  0,  0,250,128,128,128,128,250, 0, 0, 0, 0, 200,  0,  0,   0,  0]
        elif(show_step==38): #// Open index and little fingers
            show_step=show_step+1
            show_count_obj = 40
            return[250,  0,  0,  0,  0,250,128,128,128,128,250, 0, 0, 0, 0, 250,  0,  0,   0,  0]
        elif(show_step==39): #// Open index and little fingers
            show_step=show_step+1
            show_count_obj = 30
            return[250,250,  0,  0,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250,  0,   0,250]
        elif(show_step==40): #// Place thumb for '666' gesture
            show_step=show_step+1
            show_count_obj = 40
            return[100,250,  0,  0,250,200,128,128,128,128,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==41): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,  0,  0,250,200, 80,128,128,200,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==42): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,  0,  0,250,200,200,128,128, 80,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==43): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,  0,  0,250,200, 80,128,128,200,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==44): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,  0,  0,250,200,200,128,128, 80,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==45): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            return[100,250,  0,  0,250,200,128,128,128,128,100, 0, 0, 0, 0,100,250,  0,   0,250]
        elif(show_step==46): #//  Unfold
            show_step=show_step+1
            show_count_obj = 50
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==47): #// Pinch with thumb and index finger
            show_step=show_step+1
            show_count_obj = 50
            return[ 55, 0,250,250,250,170,128,128,128,128,70, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==48): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250, 20,250,250,220,128,128,128,128,100, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==49): #// Pinch with thumb and middle finger
            show_step=show_step+1
            show_count_obj = 35
            return[ 55,250, 0,250,250,140,128,128,128,128,60, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==50): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250, 20,250,170,128,128,128,128,100, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==51): #// Pinch with thumb and ring finger
            show_step=show_step+1
            show_count_obj = 35
            return[ 55,250,250, 0 ,250,110,128,128,128,128,50, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==52): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250, 20,130,128,128,128,128,100, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==53): #// Pinch with thumb and little finger
            show_step=show_step+1
            show_count_obj = 40
            return[ 55,250,250,250,  0, 60,128,128,128,128,50, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==54): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250, 20,130,128,128,128,128,100, 0, 0, 0, 0,250,250,250, 250,250]
        elif(show_step==55): #// Pinch tip with thumb and little finger
            show_step=show_step+1
            show_count_obj = 40
            return[160,250,250,250,160, 60,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 250, 80]
        elif(show_step==56): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250,250,130,128,128,128,128,100, 0, 0, 0, 0,250,250,250,  50, 250]
        elif(show_step==57): #// Pinch tip with thumb and ring finger
            show_step=show_step+1
            show_count_obj = 35
            return[160,250,250,150,250,100,128,128,128,128,50, 0, 0, 0, 0,100,250,250, 80, 250]
        elif(show_step==58): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250,250,180,128,128,128,128,100, 0, 0, 0, 0, 250,250, 50,  250, 250]
        elif(show_step==59): #// Pinch tip with thumb and middle finger
            show_step=show_step+1
            show_count_obj = 35
            return[160,250,150,250,250,135,128,128,128,128,70, 0, 0, 0, 0,100,250,85, 250, 250]
        elif(show_step==60): #// 1
            show_step=show_step+1
            show_count_obj = 20
            return[250,250,250,250,250,220,128,128,128,128,100, 0, 0, 0, 0, 250, 50,250,  250, 250]
        elif(show_step==61): #// Pinch tip with thumb and index finger
            show_step=show_step+1
            show_count_obj = 35
            return[165,150,250,250,250,170,128,128,128,128,70, 0, 0, 0, 0,100,80,250, 250, 250]
        elif(show_step==62): #// 1
            show_step=show_step+1
            show_count_obj = 60
            return[250,250,250,250,250,250,128,128,128,128,250, 0, 0, 0, 0, 250,250,250,  250, 250]

def signal_handler(sig, frame):

    print('You pressed Ctrl+C!')

    sys.exit(0)  # 0 means normal exit
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':

    try:
        print("Testing")
        send_messages()
    except KeyboardInterrupt:
         print("Caught KeyboardInterrupt, exiting gracefully.")
    except rospy.ROSInterruptException:
        print("ROSInterruptException")
    finally:
         print("Cleaning up...")