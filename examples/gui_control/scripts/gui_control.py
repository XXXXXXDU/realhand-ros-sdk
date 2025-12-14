#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time, json, threading
from dataclasses import dataclass
from typing import List, Dict

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.mapping import *
from config.constants import _HAND_CONFIGS

LOOP_TIME = 1000  # Loop action interval time in milliseconds


class ROS1NodeManager(QObject):
    status_updated = pyqtSignal(str, str)

    def __init__(self, node_name: str = "hand_control_node"):
        super().__init__()
        self.node_name = node_name
        self.publisher = None
        self.speed_pub = None
        self.torque_pub = None
        self.joint_state = JointState()

        # Initialize rospy directly
        self.init_ros()

    def init_ros(self):
        try:
            rospy.init_node(self.node_name, anonymous=True)

            # Read from parameter server
            self.hand_type   = rospy.get_param('~hand_type', 'left')
            self.hand_joint  = rospy.get_param('~hand_joint', 'L10')
            self.hz          = rospy.get_param('~topic_hz', 30)
            self.is_arc      = rospy.get_param('~is_arc', False)

            topic_base = f'/cb_{self.hand_type}_hand_control_cmd'
            self.pub = rospy.Publisher(topic_base, JointState, queue_size=10)
            if self.is_arc:
                self.pub_arc = rospy.Publisher(topic_base + '_arc', JointState, queue_size=10)

            self.speed_pub  = rospy.Publisher('/cb_hand_setting_cmd', String, queue_size=10)
            self.torque_pub = rospy.Publisher('/cb_hand_setting_cmd', String, queue_size=10)

            # NOTE: this string was originally Chinese; translated only, not used for logic checks
            self.status_updated.emit("info", f"ROS1 node initialized successfully: {self.hand_type} {self.hand_joint}")
        except Exception as e:
            self.status_updated.emit("error", f"ROS initialization failed: {str(e)}")
            raise

    # ---------- Publish functions using rospy.Time / rospy.Publisher ----------
    def publish_joint_state(self, positions: List[int]):
        if not self.pub:
            self.status_updated.emit("error", "ROS publisher not initialized")
            return
        try:
            self.joint_state.header.stamp = rospy.Time.now()
            self.joint_state.position = [float(p) for p in positions]
            hand_config = _HAND_CONFIGS[self.hand_joint]
            if len(hand_config.joint_names) == len(positions):
                self.joint_state.name = hand_config.joint_names_en or hand_config.joint_names
            self.pub.publish(self.joint_state)

            if self.is_arc:
                pose = self.calc_arc(positions)   # Wrap original range-to-radian conversion
                self.joint_state.position = [float(p) for p in pose]
                self.pub_arc.publish(self.joint_state)

            self.status_updated.emit("info", "Joint state published")
        except Exception as e:
            self.status_updated.emit("error", f"Publish failed: {str(e)}")

    def publish_speed(self, val: int):
        joint_len = self._joint_len()
        msg = String()
        msg.data = json.dumps({
            "setting_cmd": "set_speed",
            "params": {"hand_type": self.hand_type, "speed": [val] * joint_len}
        })
        self.speed_pub.publish(msg)

    def publish_torque(self, val: int):
        joint_len = self._joint_len()
        msg = String()
        msg.data = json.dumps({
            "setting_cmd": "set_max_torque_limits",
            "params": {"hand_type": self.hand_type, "torque": [val] * joint_len}
        })
        self.torque_pub.publish(msg)

    # Utility
    def _joint_len(self):
        j = self.hand_joint.upper()
        if j in ("O6", "L6"):
            return 6
        return {"L7": 7, "L10": 10}.get(j, 5)

    def calc_arc(self, positions):
        if self.hand_joint == "O6":
            if self.hand_type == "left":
                pose = range_to_arc_left(positions, self.hand_joint)
            elif self.hand_type == "right":
                pose = range_to_arc_right(positions, self.hand_joint)
        elif self.hand_joint == "L7" or self.hand_joint == "L21" or self.hand_joint == "L25":
            if self.hand_type == "left":
                pose = range_to_arc_left(positions, self.hand_joint)
            elif self.hand_type == "right":
                pose = range_to_arc_right(positions, self.hand_joint)
        elif self.hand_joint == "L10":
            if self.hand_type == "left":
                pose = range_to_arc_left_10(positions)
            elif self.hand_type == "right":
                pose = range_to_arc_right_10(positions)
        elif self.hand_joint == "L20":
            if self.hand_type == "left":
                pose = range_to_arc_left_l20(positions)
            elif self.hand_type == "right":
                pose = range_to_arc_right_l20(positions)
        else:
            print(f"Current {self.hand_joint} {self.hand_type} does not support radian conversion", flush=True)
        # NOTE: original function always returns 'positions' instead of 'pose'; kept as-is
        return positions


class HandControlGUI(QWidget):
    """Dexterous hand control interface"""
    status_updated = pyqtSignal(str, str)  # status type, message content

    def __init__(self, ros_manager: ROS1NodeManager):
        super().__init__()

        # Loop control variables
        self.cycle_timer = None  # Loop timer
        self.current_action_index = -1  # Current action index
        self.preset_buttons = []  # Store references to preset action buttons

        # Set ROS manager
        self.ros_manager = ros_manager

        # Get hand configuration
        self.hand_joint = self.ros_manager.hand_joint
        self.hand_type = self.ros_manager.hand_type
        self.hand_config = _HAND_CONFIGS[self.hand_joint]

        # Initialize UI
        self.init_ui()

        # Timer to publish joint state
        self.publish_timer = QTimer(self)
        self.publish_timer.setInterval(int(1000 / self.ros_manager.hz))
        self.publish_timer.timeout.connect(self.publish_joint_state)
        self.publish_timer.start()

    def init_ui(self):
        """Initialize user interface"""
        # Window properties
        self.setWindowTitle(f'Dexterous Hand Control GUI - {self.hand_type} {self.hand_joint}')
        self.setMinimumSize(1200, 900)

        # Styles (only style, no logic)
        self.setStyleSheet("""
            QWidget {
                font-family: 'Microsoft YaHei', 'SimHei', sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #CCCCCC;
                border-radius: 6px;
                margin-top: 6px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #165DFF;
                font-weight: bold;
            }
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 8px;
                border-radius: 4px;
                background: #CCCCCC;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #165DFF, stop:1 #0E42D2);
                border: 1px solid #5C8AFF;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 5px 10px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #F0F0F0;
            }
            QPushButton:pressed {
                background-color: #D0D0D0;
            }
            QPushButton[category="preset"] {
                background-color: #E6F7FF;
                color: #1890FF;
                border-color: #91D5FF;
            }
            QPushButton[category="preset"]:hover {
                background-color: #B3E0FF;
            }
            QPushButton[category="action"] {
                background-color: #FFF7E6;
                color: #FA8C16;
                border-color: #FFD591;
            }
            QPushButton[category="action"]:hover {
                background-color: #FFE6B3;
            }
            QPushButton[category="danger"] {
                background-color: #FFF1F0;
                color: #F5222D;
                border-color: #FFCCC7;
            }
            QPushButton[category="danger"]:hover {
                background-color: #FFE8E6;
            }
            QLabel#StatusLabel {
                padding: 5px;
                border-radius: 4px;
            }
            QLabel#StatusInfo {
                background-color: #F0F7FF;
                color: #0066CC;
            }
            QLabel#StatusError {
                background-color: #FFF0F0;
                color: #CC0000;
            }
            QTextEdit#ValueDisplay {
                background-color: #F8F8F8;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 10px;
                font-family: Consolas, monospace;
                font-size: 12px;
            }
        """)

        # Main vertical layout
        main_layout = QVBoxLayout(self)

        # Horizontal splitter (three panels)
        splitter = QSplitter(Qt.Horizontal)

        # Left panel: joint control
        self.joint_control_panel = self.create_joint_control_panel()
        splitter.addWidget(self.joint_control_panel)

        # Middle panel: preset actions
        self.preset_actions_panel = self.create_preset_actions_panel()
        splitter.addWidget(self.preset_actions_panel)

        # Right panel: status monitor
        self.status_monitor_panel = self.create_status_monitor_panel()
        splitter.addWidget(self.status_monitor_panel)

        # Splitter sizes
        splitter.setSizes([500, 300, 400])

        # Add splitter to main layout
        main_layout.addWidget(splitter, stretch=1)

        # Value display panel at the bottom
        self.value_display_panel = self.create_value_display_panel()
        main_layout.addWidget(self.value_display_panel, stretch=0)

        # Initial value display update
        self.update_value_display()

    def create_joint_control_panel(self):
        """Create joint control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Title
        title_label = QLabel(f"Joint Control - {self.hand_joint}")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # Scroll area for sliders
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        scroll_content = QWidget()
        self.sliders_layout = QGridLayout(scroll_content)
        self.sliders_layout.setSpacing(10)

        # Create sliders
        self.create_joint_sliders()

        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        return panel

    def create_joint_sliders(self):
        """Create joint sliders"""
        # Clear existing sliders
        for i in reversed(range(self.sliders_layout.count())):
            item = self.sliders_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()

        # Create new sliders
        self.sliders = []
        self.slider_labels = []

        for i, (name, value) in enumerate(zip(
            self.hand_config.joint_names, self.hand_config.init_pos
        )):
            # Label
            label = QLabel(f"{name}: {value}")
            label.setMinimumWidth(120)

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 255)
            slider.setValue(value)
            slider.valueChanged.connect(
                lambda val, idx=i: self.on_slider_value_changed(idx, val)
            )

            # Add to layout
            row, col = divmod(i, 1)
            self.sliders_layout.addWidget(label, row, 0)
            self.sliders_layout.addWidget(slider, row, 1)

            self.sliders.append(slider)
            self.slider_labels.append(label)

    def create_preset_actions_panel(self):
        """Create preset actions panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # System preset actions
        sys_preset_group = QGroupBox("System Presets")
        sys_preset_layout = QGridLayout(sys_preset_group)
        sys_preset_layout.setSpacing(8)

        # Add system preset buttons
        self.create_system_preset_buttons(sys_preset_layout)
        layout.addWidget(sys_preset_group)

        # Action buttons
        actions_layout = QHBoxLayout()

        # Cycle button
        self.cycle_button = QPushButton("Loop Preset Actions")
        self.cycle_button.setProperty("category", "action")
        self.cycle_button.clicked.connect(self.on_cycle_clicked)
        actions_layout.addWidget(self.cycle_button)

        self.home_button = QPushButton("Go to Home Position")
        self.home_button.setProperty("category", "action")
        self.home_button.clicked.connect(self.on_home_clicked)
        actions_layout.addWidget(self.home_button)

        self.stop_button = QPushButton("Stop All Actions")
        self.stop_button.setProperty("category", "danger")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        actions_layout.addWidget(self.stop_button)

        layout.addLayout(actions_layout)

        return panel

    def create_system_preset_buttons(self, parent_layout):
        """Create system preset action buttons"""
        self.preset_buttons = []  # Clear button list
        if self.hand_config.preset_actions:
            buttons = []
            for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
                button = QPushButton(name)
                button.setProperty("category", "preset")
                button.clicked.connect(
                    lambda checked, pos=positions: self.on_preset_action_clicked(pos)
                )
                buttons.append(button)
                self.preset_buttons.append(button)  # Save button reference

            # Add to grid layout
            cols = 2
            for i, button in enumerate(buttons):
                row, col = divmod(i, cols)
                parent_layout.addWidget(button, row, col)

    def create_status_monitor_panel(self):
        """Create status monitor panel (speed/torque each on one row with real-time values)"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Title
        title_label = QLabel("Status Monitor")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # Quick settings group
        quick_set_gb = QGroupBox("Quick Settings")
        qv_layout = QVBoxLayout(quick_set_gb)

        # Speed row
        speed_hbox = QHBoxLayout()
        speed_hbox.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 255)
        self.speed_slider.setValue(255)
        self.speed_slider.setMinimumWidth(150)
        speed_hbox.addWidget(self.speed_slider)
        self.speed_val_lbl = QLabel("255")  # Real-time value
        self.speed_val_lbl.setMinimumWidth(30)
        speed_hbox.addWidget(self.speed_val_lbl)
        self.speed_btn = QPushButton("Set Speed")
        self.speed_btn.clicked.connect(
            lambda: (
                self.ros_manager.publish_speed(self.speed_slider.value()),
                self.status_updated.emit(
                    "info", f"Speed set to {self.speed_slider.value()}")
            ))
        speed_hbox.addWidget(self.speed_btn)
        speed_hbox.addStretch()
        qv_layout.addLayout(speed_hbox)

        # Torque row
        torque_hbox = QHBoxLayout()
        torque_hbox.addWidget(QLabel("Torque:"))
        self.torque_slider = QSlider(Qt.Horizontal)
        self.torque_slider.setRange(0, 255)
        self.torque_slider.setValue(255)
        self.torque_slider.setMinimumWidth(150)
        torque_hbox.addWidget(self.torque_slider)
        self.torque_val_lbl = QLabel("255")
        self.torque_val_lbl.setMinimumWidth(30)
        torque_hbox.addWidget(self.torque_val_lbl)
        self.torque_btn = QPushButton("Set Torque")
        self.torque_btn.clicked.connect(
            lambda: (
                self.ros_manager.publish_torque(self.torque_slider.value()),
                self.status_updated.emit(
                    "info", f"Torque set to {self.torque_slider.value()}")
            ))
        torque_hbox.addWidget(self.torque_btn)
        torque_hbox.addStretch()
        qv_layout.addLayout(torque_hbox)

        layout.addWidget(quick_set_gb)

        # Tab widget
        tab_widget = QTabWidget()

        # System info tab
        sys_info_widget = QWidget()
        sys_info_layout = QVBoxLayout(sys_info_widget)

        conn_group = QGroupBox("Connection Status")
        conn_layout = QVBoxLayout(conn_group)
        self.connection_status = QLabel("ROS2 node connected")
        self.connection_status.setObjectName("StatusLabel")
        self.connection_status.setObjectName("StatusInfo")
        conn_layout.addWidget(self.connection_status)

        hand_info_group = QGroupBox("Hand Info")
        hand_info_layout = QVBoxLayout(hand_info_group)
        info_text = f"""Hand type: {self.hand_type}
Joint model: {self.hand_joint}
Number of joints: {len(self.hand_config.joint_names)}
Publish frequency: {self.ros_manager.hz} Hz"""
        self.hand_info_label = QLabel(info_text)
        self.hand_info_label.setWordWrap(True)
        hand_info_layout.addWidget(self.hand_info_label)

        sys_info_layout.addWidget(conn_group)
        sys_info_layout.addWidget(hand_info_group)
        sys_info_layout.addStretch()
        tab_widget.addTab(sys_info_widget, "System Info")

        # Status log tab
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        self.status_log = QLabel("Waiting for system startup...")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.setWordWrap(True)
        self.status_log.setMinimumHeight(300)
        log_layout.addWidget(self.status_log)
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.clicked.connect(self.clear_status_log)
        log_layout.addWidget(clear_log_btn)
        tab_widget.addTab(log_widget, "Status Log")

        layout.addWidget(tab_widget)

        # Real-time slider value updates
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_val_lbl.setText(str(v)))
        self.torque_slider.valueChanged.connect(
            lambda v: self.torque_val_lbl.setText(str(v)))
        return panel

    def create_value_display_panel(self):
        """Create the slider value display panel"""
        panel = QGroupBox("Joint Value List")
        layout = QVBoxLayout(panel)

        layout.setContentsMargins(10, 20, 10, 20)

        self.value_display = QTextEdit()
        self.value_display.setObjectName("ValueDisplay")
        self.value_display.setReadOnly(True)  # Read-only but copyable
        self.value_display.setMinimumHeight(60)
        self.value_display.setMaximumHeight(80)
        self.value_display.setText("[]")

        layout.addWidget(self.value_display)

        return panel

    def on_slider_value_changed(self, index: int, value: int):
        """Handle slider value change"""
        if 0 <= index < len(self.slider_labels):
            joint_name = self.hand_config.joint_names[index]
            self.slider_labels[index].setText(f"{joint_name}: {value}")

        # Update value display
        self.update_value_display()

    def update_value_display(self):
        """Update the value display panel content"""
        values = [slider.value() for slider in self.sliders]
        self.value_display.setText(f"{values}")

    def on_preset_action_clicked(self, positions: List[int]):
        """Handle preset action button click"""
        if len(positions) != len(self.sliders):
            QMessageBox.warning(
                self, "Action Mismatch",
                f"Number of joints in preset action ({len(positions)}) does not match current number of joints ({len(self.sliders)})"
            )
            return

        # Update sliders
        for i, (slider, pos) in enumerate(zip(self.sliders, positions)):
            slider.setValue(pos)
            self.on_slider_value_changed(i, pos)

        # Publish joint state
        self.publish_joint_state()

    def on_home_clicked(self):
        """Handle 'Go to Home Position' button click"""
        for slider, pos in zip(self.sliders, self.hand_config.init_pos):
            slider.setValue(pos)

        self.publish_joint_state()
        self.status_updated.emit("info", "Returned to home position")

        # Update value display
        self.update_value_display()

    def on_stop_clicked(self):
        """Handle 'Stop All Actions' button click"""
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Loop Preset Actions")
            self.reset_preset_buttons_color()

        self.status_updated.emit("warning", "All actions stopped")

    def on_cycle_clicked(self):
        """Handle 'Loop Preset Actions' button click"""
        if not self.hand_config.preset_actions:
            QMessageBox.warning(self, "No Preset Actions", "Current hand model has no preset actions to loop")
            return

        if self.cycle_timer and self.cycle_timer.isActive():
            # Stop loop
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Loop Preset Actions")
            self.reset_preset_buttons_color()
            self.status_updated.emit("info", "Stopped looping preset actions")
        else:
            # Start loop
            self.current_action_index = -1
            self.cycle_timer = QTimer(self)
            self.cycle_timer.timeout.connect(self.run_next_action)
            self.cycle_timer.start(LOOP_TIME)
            self.cycle_button.setText("Stop Looping")
            self.status_updated.emit("info", "Started looping preset actions")
            self.run_next_action()

    def run_next_action(self):
        """Run the next preset action"""
        if not self.hand_config.preset_actions:
            return

        # Reset button colors
        self.reset_preset_buttons_color()

        # Compute next index
        self.current_action_index = (self.current_action_index + 1) % len(self.hand_config.preset_actions)

        # Get next action
        action_names = list(self.hand_config.preset_actions.keys())
        action_name = action_names[self.current_action_index]
        action_positions = self.hand_config.preset_actions[action_name]

        # Execute action
        self.on_preset_action_clicked(action_positions)

        # Highlight current button
        if 0 <= self.current_action_index < len(self.preset_buttons):
            button = self.preset_buttons[self.current_action_index]
            button.setStyleSheet("background-color: green; color: white; border-color: #91D5FF;")

        self.status_updated.emit("info", f"Running preset action: {action_name}")

    def reset_preset_buttons_color(self):
        """Reset all preset buttons' colors"""
        for button in self.preset_buttons:
            button.setStyleSheet("")
            button.setProperty("category", "preset")
            button.style().unpolish(button)
            button.style().polish(button)

    def on_joint_type_changed(self, joint_type: str):
        """Handle joint type change"""
        self.hand_joint = joint_type
        self.hand_config = _HAND_CONFIGS[self.hand_joint]

        info_text = f"""Hand type: {self.hand_type}
Joint model: {self.hand_joint}
Number of joints: {len(self.hand_config.joint_names)}
Publish frequency: {self.ros_manager.hz} Hz"""
        self.hand_info_label.setText(info_text)

        # Recreate sliders and preset buttons
        self.create_joint_sliders()
        # NOTE: original code assumes sys_preset_layout is a class variable; this may raise if used
        self.create_system_preset_buttons(self.sys_preset_layout)

        self.update_value_display()
        self.status_updated.emit("info", f"Switched to hand model: {joint_type}")

    def publish_joint_state(self):
        """Publish current joint state"""
        positions = [slider.value() for slider in self.sliders]
        self.ros_manager.publish_joint_state(positions)

    def update_status(self, status_type: str, message: str):
        """Update status display"""
        # This condition uses original Chinese substring for compatibility
        if status_type == "info" and "ROS2节点初始化成功" in message:
            self.connection_status.setText("ROS2 node connected")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")

        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        current_log = self.status_log.text()

        if len(current_log) > 10000:
            current_log = current_log[-10000:]

        self.status_log.setText(log_entry + current_log)

        self.status_log.setObjectName("StatusLabel")
        if status_type == "error":
            self.status_log.setObjectName("StatusError")
        else:
            self.status_log.setObjectName("StatusInfo")

    def clear_status_log(self):
        """Clear status log"""
        self.status_log.setText("Log cleared")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")

    def closeEvent(self, event):
        """Handle window close event"""
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)

    ros_manager = ROS1NodeManager()
    window = HandControlGUI(ros_manager)
    ros_manager.status_updated.connect(window.update_status)

    window.show()

    # Use QTimer to drive rospy event loop
    timer = QTimer()
    timer.timeout.connect(lambda: rospy.rostime.wallsleep(0.001))
    timer.start(10)

    exit_code = app.exec_()
    rospy.signal_shutdown("GUI closed")
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
