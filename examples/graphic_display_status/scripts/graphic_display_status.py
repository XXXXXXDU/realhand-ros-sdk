#!/usr/bin/env python3
import rospy, rospkg
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
from PyQt5.QtWidgets import QMainWindow, QSplitter, QApplication, QMessageBox, QPushButton
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import Header, Float32MultiArray

rospack = rospkg.RosPack()
ros_linker_hand_sdk_path = rospack.get_path('linker_hand_sdk_ros')
sys.path.append(ros_linker_hand_sdk_path + '/scripts')
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.color_msg import ColorMsg
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from views.temperature_plot import TemperaturePlot
from views.wave_form_plot import WaveformPlot


class GraphicDisplayStatus:
    def __init__(self):
        rospy.init_node('graphic_display_status', anonymous=True)
        self.rate = rospy.Rate(30)
        self.left_hand = True
        self.left_touch_type = 0
        self.right_hand = True
        self.right_touch_type = 0
        try:
            self.left_hand_msg = rospy.wait_for_message("/cb_left_hand_info", String, timeout=1)
        except rospy.exceptions.ROSException:
            rospy.logwarn("Failed to receive /cb_left_hand_info topic message")
            self.left_hand = False

        try:
            self.right_hand_msg = self.r = rospy.wait_for_message("/cb_right_hand_info", String, timeout=1)
        except rospy.exceptions.ROSException:
            rospy.logwarn("Failed to receive /cb_right_hand_info topic message")
            self.right_hand = False
        self._init_left_hand_info()
        self._init_right_hand_info()
        # rospy.spin()  # If enabled, it will block the GUI thread

    def _init_left_hand_info(self):
        """Initialize left hand"""
        if self.left_hand == True:
            self.left_hand_info = json.loads(self.left_hand_msg.data)
            self.left_hand_joint = self.left_hand_info["hand_joint"]
            self.left_hand_force = self.left_hand_info["is_touch"]
            self.left_info_sub = rospy.Subscriber("/cb_left_hand_info", String, self.get_left_info_data, queue_size=1)
            if self.left_hand_force == True:
                time.sleep(0.1)
                self._init_left_hand_temperature_plot(hand_type="left")
                self._init_left_hand_normal_force_plot(hand_type="left")  # Normal force waveform plot
                if self.left_touch_type != 2:
                    self._init_left_hand_approach_inc_plot(hand_type="left")  # Proximity sensing waveform plot

    def _init_right_hand_info(self):
        """Initialize right hand"""
        if self.right_hand == True:
            self.right_hand_info = json.loads(self.right_hand_msg.data)
            self.right_hand_joint = self.right_hand_info["hand_joint"]
            self.right_hand_force = self.right_hand_info["is_touch"]
            self.right_info_sub = rospy.Subscriber("/cb_right_hand_info", String, self.get_right_info_data, queue_size=1)
            if self.right_hand_force == True:
                time.sleep(0.1)
                self._init_right_hand_temperature_plot(hand_type="right")
                self._init_right_hand_normal_force_plot(hand_type="right")  # Normal force waveform plot
                if self.right_touch_type != 2:
                    self._init_right_hand_approach_inc_plot(hand_type="right")  # Proximity sensing waveform plot

    def _init_left_hand_temperature_plot(self, hand_type="left"):
        if self.left_hand_joint == "L7":
            num_lines = 7
        elif self.left_hand_joint == "L21" or self.left_hand_joint == "L25":
            num_lines = 25
        else:
            num_lines = 10
        # Initialize temperature waveform plot
        self.left_temperature_plot = TemperaturePlot(
            num_lines=num_lines,
            labels=None,
            title=f"{hand_type} hand motor temperature waveform"
        )
        self.left_temperature_plot.setGeometry(700, 1000, 800, 700)
        self.left_temperature_plot.show()
        self.left_timer_left_temperature = QTimer()
        self.left_timer_left_temperature.timeout.connect(lambda: self.update_temperature_plot(hand_type=hand_type))
        self.left_timer_left_temperature.start(50)

    # Right hand temperature
    def _init_right_hand_temperature_plot(self, hand_type="right"):
        if self.right_hand_joint == "L7":
            num_lines = 7
        elif self.right_hand_joint == "L21" or self.right_hand_joint == "L25":
            num_lines = 25
        else:
            num_lines = 10
        # Initialize temperature waveform plot
        self.right_temperature_plot = TemperaturePlot(
            num_lines=num_lines,
            labels=None,
            title=f"{hand_type} hand motor temperature waveform"
        )
        self.right_temperature_plot.setGeometry(700, 1000, 800, 700)
        self.right_temperature_plot.show()
        self.right_timer_left_temperature = QTimer()
        self.right_timer_left_temperature.timeout.connect(lambda: self.update_temperature_plot(hand_type=hand_type))
        self.right_timer_left_temperature.start(50)

    # Update temperature waveform plot
    def update_temperature_plot(self, hand_type="left"):
        if hand_type == "left":
            try:
                self.left_temperature_plot.update_data(self.left_motor_temperature)
            except:
                ColorMsg(msg=f"Left hand {self.left_hand_joint} has no temperature data", color="yellow")
        elif hand_type == "right":
            try:
                self.right_temperature_plot.update_data(self.right_motor_temperature)
            except:
                ColorMsg(msg=f"Right hand {self.right_hand_joint} has no temperature data", color="yellow")

    # Update temperature data
    def get_left_info_data(self, msg):
        data = json.loads(msg.data)
        self.left_motor_temperature = list(data["motor_temperature"])
        self.left_touch_type = data["touch_type"]
        if self.left_touch_type == 2:
            self.last_left_normal_force = list(data["touch"])
        else:
            touch = list(data["touch"])
            self.last_left_normal_force = touch[:5]
            self.last_left_approach_inc = touch[15:20]

    def get_right_info_data(self, msg):
        data = json.loads(msg.data)
        self.right_motor_temperature = list(data["motor_temperature"])
        self.right_touch_type = data["touch_type"]
        if self.right_touch_type == 2:
            self.last_right_normal_force = list(data["touch"])
        else:
            touch = list(data["touch"])
            self.last_right_normal_force = touch[:5]
            self.last_right_approach_inc = touch[15:20]

    # Get pressure sensing data
    # def get_left_force_data(self,force_data):
    #     data = force_data.data
    #     self.last_left_normal_force = data[:5]
    #     self.last_left_approach_inc = data[15:20]
    # def get_right_force_data(self,force_data):
    #     data = force_data.data
    #     self.last_right_normal_force = data[:5]
    #     self.last_right_approach_inc = data[15:20]

    # Initialize pressure-sensing (normal force) plots
    def _init_left_hand_normal_force_plot(self, hand_type="left"):
        # Initialize waveform plot
        self.left_normal_force_plot = WaveformPlot(
            num_lines=5,
            labels=["thumb", "index finger", "middle finger", "ring finger", "little finger"],
            title=f"{hand_type} normal force waveform"
        )
        # Set waveform plot position
        self.left_normal_force_plot.setGeometry(700, 100, 800, 400)
        self.left_normal_force_plot.show()
        self.left_timer = QTimer()
        # self.timer.timeout.connect(self.update_normal_force_plot)
        self.left_timer.timeout.connect(lambda: self.update_normal_force_plot(hand_type=hand_type))
        self.left_timer.start(50)

    def _init_right_hand_normal_force_plot(self, hand_type="right"):
        # Initialize waveform plot
        self.right_normal_force_plot = WaveformPlot(
            num_lines=5,
            labels=["thumb", "index finger", "middle finger", "ring finger", "little finger"],
            title=f"{hand_type} normal force waveform"
        )
        # Set waveform plot position
        self.right_normal_force_plot.setGeometry(700, 100, 800, 400)
        self.right_normal_force_plot.show()
        self.right_timer = QTimer()
        # self.timer.timeout.connect(self.update_normal_force_plot)
        self.right_timer.timeout.connect(lambda: self.update_normal_force_plot(hand_type=hand_type))
        self.right_timer.start(50)

    # Initialize proximity sensing waveform plot
    def _init_left_hand_approach_inc_plot(self, hand_type="left"):
        # Initialize proximity sensing waveform plot
        self.left_approach_inc_plot = WaveformPlot(
            num_lines=5,
            labels=["thumb", "index finger", "middle finger", "ring finger", "little finger"],
            title=f"{hand_type} proximity sensing waveform"
        )
        # Set waveform plot position
        self.left_approach_inc_plot.setGeometry(700, 600, 800, 400)
        self.left_approach_inc_plot.show()
        self.left_timer2 = QTimer()
        # self.timer2.timeout.connect(self.update_approach_inc_plot)
        self.left_timer2.timeout.connect(lambda: self.update_approach_inc_plot(hand_type=hand_type))
        self.left_timer2.start(50)

    def _init_right_hand_approach_inc_plot(self, hand_type="right"):
        # Initialize proximity sensing waveform plot
        self.right_approach_inc_plot = WaveformPlot(
            num_lines=5,
            labels=["thumb", "index finger", "middle finger", "ring finger", "little finger"],
            title=f"{hand_type} proximity sensing waveform"
        )
        # Set waveform plot position
        self.right_approach_inc_plot.setGeometry(700, 600, 800, 400)
        self.right_approach_inc_plot.show()
        self.right_timer2 = QTimer()
        # self.timer2.timeout.connect(self.update_approach_inc_plot)
        self.right_timer2.timeout.connect(lambda: self.update_approach_inc_plot(hand_type=hand_type))
        self.right_timer2.start(50)

    # Update normal force waveform plot
    def update_normal_force_plot(self, hand_type="left"):
        if hand_type == "left":
            data = self.last_left_normal_force
            self.left_normal_force_plot.update_data(data)
        elif hand_type == "right":
            data = self.last_right_normal_force
            self.right_normal_force_plot.update_data(data)

    # Update proximity sensing waveform plot
    def update_approach_inc_plot(self, hand_type="left"):
        # converted_list = []
        # # Iterate over original list and convert
        # for value in self.last_approach_inc:
        #     # Since the value decreases from 255 to 0, 255 - value gives a range from 0 to 255
        #     new_value = 255 - value
        #     converted_list.append(new_value)
        if hand_type == "left":
            data = self.last_left_approach_inc
            self.left_approach_inc_plot.update_data(data)
        elif hand_type == "right":
            data = self.last_right_approach_inc
            self.right_approach_inc_plot.update_data(data)

    def closeEvent(self, event):
        # Close waveform windows
        if self.left_hand_force == True:
            self.left_normal_force_plot.close()      # Normal force waveform plot
            self.left_temperature_plot.close()
            self.left_approach_inc_plot.close()      # Proximity sensing waveform plot
        if self.right_hand_force == True:
            self.right_approach_inc_plot.close()     # Proximity sensing waveform plot
            self.right_temperature_plot.close()
            self.right_normal_force_plot.close()     # Normal force waveform plot
        sys.exit(0)


def signal_handler(sig, frame):
    sys.exit(0)  # Exit the program normally


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill command
    app = QApplication(sys.argv)
    window = GraphicDisplayStatus()
    # window.show()

    sys.exit(app.exec_())
