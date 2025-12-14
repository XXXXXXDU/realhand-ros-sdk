#!/usr/bin/env python
#This demo is for the L10 dexterous hand with a spherical thumb root joint. It supports the left hand by default. To support the right hand, change cb_left_hand_control_cmd to cb_right_hand_control_cmd
 

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
hand = {"joint1":255,   #Thumb root flexion
        "joint2":128,   #Thumb side swing
        "joint3":255,   #Index finger root flexion
        "joint4":255,   #Middle finger root flexion
        "joint5":255,   #Ring finger root flexion
        "joint6":255,   #Little finger root flexion
        "joint7":128,   #Index finger side swing
        "joint8":128,   #Middle finger side swing
        "joint9":128,   #Ring finger side swing
        "joint10":255,  #Thumb rotation
        }

def send_messages():

    rospy.init_node('dong_test_sender', anonymous=True) 

    pub = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=10)

    rate = rospy.Rate(30)  # Set frequency to 30Hz
    joint_state.header = std_msgs.msg.Header()
    joint_state.header.seq=0
    joint_state.header.stamp = rospy.Time.now() # Or use rospy.Time(secs=0, nsecs=0) to get a specific time
    joint_state.header.frame_id = ''
    joint_state.name=list(hand.keys())
    joint_state.velocity = [0] * len(joint_state.position)  # Same length as the position array, all filled with 0
    joint_state.effort = [0] * len(joint_state.position)  # Set effort to zero for each joint
    pub.publish(joint_state)
    while not rospy.is_shutdown():  # Loop for 1 second
        position =show_left()
        if(position is not None):
            joint_state.position = position
        pub.publish(joint_state)
        rate.sleep()

def show_left():
    global show_count #Current step waiting time
    global show_count_obj  #Current step action should wait for duration
    global show_step  #Execution step number
    global hand
    show_count= show_count+1
    if(show_count>=show_count_obj):
        show_count=0
        if(show_step==0):
            show_step=show_step+1
            show_count_obj = 100
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==1): #// Close little and ring fingers
            show_step=show_step+1
            show_count_obj = 10
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint5'] = 0
            hand['joint6'] = 0
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==2): #// Place thumb on top of little and ring fingers
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint10'] = 80
            return list(hand.values())
        elif(show_step==3): #// Tilt index and middle fingers to one side
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 200
            return list(hand.values())
        elif(show_step==4): #// Other side
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==5): #//  Return both to center
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==6): #// Index and middle fingers make a Y shape
            show_step=show_step+1
            show_count_obj = 2  
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==7): #// Retract Y shape
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==8): #// Index and middle fingers make a Y shape
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 50
            return list(hand.values())
        elif(show_step==9): #// Retract Y shape
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 128
            return list(hand.values())
        elif(show_step==10): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==11): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==12): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 100
            hand['joint4'] = 100
            return list(hand.values())
        elif(show_step==13): #// Bend and straighten middle and index fingers alternately twice
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==14): #// Curl thumb
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 40
            hand['joint2'] = 240
            hand['joint10'] = 80
            return list(hand.values())
        elif(show_step==15): #// Tuck thumb into palm
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==16): #// Close 4 fingers
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 10
            hand['joint4'] = 10
            hand['joint5'] = 10
            hand['joint6'] = 10
            return list(hand.values())
        elif(show_step==17): #// Open 4 fingers and thumb sequentially
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==18): #// 1
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==19): #// 2
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 250
            return list(hand.values())
        elif(show_step==20): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 250
            return list(hand.values())
        elif(show_step==21): #// 4
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 110
            hand['joint10'] = 240
            return list(hand.values())
        elif(show_step==22): #// Bring thumb closer
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 10
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==23): #// Rotate thumb towards palm
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            hand['joint2'] = 10
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==24): #// Return to initial position in two steps
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 0
            hand['joint2'] = 240
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==25): #// 1
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 110
            return list(hand.values())
        elif(show_step==26): #// 2
            show_step=show_step+1
            show_count_obj = 10
            hand['joint7'] = 200
            hand['joint8'] = 200
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==27): #// 3
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint8'] = 80
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==28): #// 4
            show_step=show_step+1
            show_count_obj = 20
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            return list(hand.values())
        elif(show_step==29): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==30): #// Curl 4 fingers
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==31): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==32): #// 4
            show_step=show_step+1
            return list(hand.values())
        elif(show_step==33): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            hand['joint3'] = 0
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==34): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            hand['joint4'] = 0
            return list(hand.values())
        elif(show_step==35): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            hand['joint5'] = 0
            return list(hand.values())
        elif(show_step==36): #// Sequentially curl 4 small fingers
            show_step=show_step+1
            show_count_obj = 15
            hand['joint6'] = 0
            return list(hand.values())
        elif(show_step==37): #// Curl thumb
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 0
            return list(hand.values())
        elif(show_step==38): #// Open index and little fingers
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 250
            hand['joint2'] = 230
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==39): #// Open index and little fingers
            show_step=show_step+1
            show_count_obj = 30
            hand['joint3'] = 250
            hand['joint6'] = 250
            return list(hand.values())
        elif(show_step==40): #// Place thumb for '666' gesture
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 10
            hand['joint2'] = 40
            hand['joint10'] = 60
            return list(hand.values())
        elif(show_step==41): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==42): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 200
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==43): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 80
            hand['joint9'] = 200
            return list(hand.values())
        elif(show_step==44): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 200
            hand['joint9'] = 80
            return list(hand.values())
        elif(show_step==45): #// Move fingers left and right
            show_step=show_step+1
            show_count_obj = 15
            hand['joint7'] = 128
            hand['joint9'] = 128
            return list(hand.values())
        elif(show_step==46): #//  Unfold
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())
        elif(show_step==47): #// Pinch with thumb and index finger
            show_step=show_step+1
            show_count_obj = 50
            hand['joint1'] = 130
            hand['joint2'] = 130
            hand['joint3'] = 130
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint10'] = 90
            return list(hand.values())
        elif(show_step==48): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 120
            return list(hand.values())
        elif(show_step==49): #// Pinch with thumb and middle finger
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 120
            hand['joint4'] = 130
            hand['joint10'] = 60
            return list(hand.values())
        elif(show_step==50): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 145
            return list(hand.values())
        elif(show_step==51): #// Pinch with thumb and ring finger
            show_step=show_step+1
            show_count_obj = 35
            hand['joint1'] = 113
            hand['joint2'] = 103
            hand['joint5'] = 128
            hand['joint10'] = 42
            return list(hand.values())
        elif(show_step==52): #// 1
            show_step=show_step+1
            show_count_obj = 30
            hand['joint1'] = 250
            hand['joint5'] = 250
            return list(hand.values())
        elif(show_step==53): #// Pinch with thumb and little finger
            show_step=show_step+1
            show_count_obj = 40
            hand['joint1'] = 118
            hand['joint2'] = 103
            hand['joint6'] = 120
            hand['joint10'] = 22
            return list(hand.values())
        elif(show_step==54): #// 1
            show_step=show_step+1
            show_count_obj = 20
            hand['joint1'] = 250
            hand['joint2'] = 250
            hand['joint3'] = 250
            hand['joint4'] = 250
            hand['joint5'] = 250
            hand['joint6'] = 250
            hand['joint7'] = 128
            hand['joint8'] = 128
            hand['joint9'] = 128
            hand['joint10'] = 250
            return list(hand.values())


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