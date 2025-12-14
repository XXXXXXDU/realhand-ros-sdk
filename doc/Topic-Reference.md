---

# Real Hand ROS SDK Topic Documentation

## Topic Overview

This document provides a detailed overview of the ROS Topic for the Real Hand, including functions for controlling the hand's movements, retrieving sensor data, and setting operational parameters.

## Topic List
```bash
/cb_hand_setting_cmd # Topic for setting Real Hand commands
/cb_left_hand_control_cmd # Topic for controlling left hand motion by range 0~255
/cb_left_hand_control_cmd_arc # Topic for controlling left hand motion by arc -3.14~3.14 (radians)
/cb_left_hand_force # Topic for left hand pressure data display
/cb_left_hand_matrix_touch # Topic for left hand matrix pressure data display list(6x12), String format with timestamp
/cb_left_hand_matrix_touch_pc # Topic for left hand matrix pressure data display list(6x12), PointCloud2 format
/cb_left_hand_info  # Topic for left hand configuration information display
/cb_left_hand_state # Topic for left hand state display (range)
/cb_left_hand_state_arc # Topic for left hand state display (radians)
/cb_right_hand_control_cmd # Topic for controlling right hand motion by range 0~255
/cb_right_hand_control_cmd_arc # Topic for controlling right hand motion by arc -3.14~3.14 (radians)
/cb_right_hand_force # Topic for right hand pressure data display
/cb_right_hand_matrix_touch # Topic for right hand matrix pressure data display list(6x12)
/cb_right_hand_info # Topic for right hand configuration information display
/cb_right_hand_state # Topic for right hand state display (range)
/cb_right_hand_state_arc # Topic for right hand state display (radians)
```

### Set Topic /cb_hand_setting_cmd

### Set Maximum Torque
```bash
rostopic pub /cb_hand_setting_cmd std_msgs/String '{data: "{\"setting_cmd\":\"set_max_torque_limits\",\"params\":{\"hand_type\":\"right\",\"torque\":180}}"}'
```
**Description**:  
Set the maximum torque of the manipulator. Data format: std_msgs/msg/String  
**Parameters**:
- `hand_type`: left or right 
- `torque`: int or list(int), length 5, value range 0~255

---

### Set Speed
```bash
rostopic pub /cb_hand_setting_cmd std_msgs/String '{data: "{\"setting_cmd\":\"set_speed\",\"params\":{\"hand_type\":\"right\",\"speed\":200}}"}'
```
**Description**:  
Set the maximum speed of the manipulator. Data format: std_msgs/msg/String  
**Parameters**:
- `hand_type`: left or right 
- `speed`: int or list(int), length 5, value range 0~255

---

### Set Current
```bash
rostopic pub /cb_hand_setting_cmd std_msgs/String '{data: "{\"setting_cmd\":\"set_electric_current\",\"params\":{\"hand_type\":\"left\",\"electric_current\":250}}"}'
```
**Description**:  
Set the maximum current of the manipulator. Data format: std_msgs/msg/String  
**Parameters**:
- `hand_type`: left or right 
- `electric_current`: int or list(int), length 5, value range 0~255

---

### Clear Faults
```bash
rostopic pub /cb_hand_setting_cmd std_msgs/String '{data: "{\"setting_cmd\":\"clear_faults\",\"params\":{\"hand_type\":\"left\"}}"}'
```
**Description**:  
Clear faults. Data format: std_msgs/msg/String. Currently only supported for L20.  
**Parameters**:
- `hand_type`: left or right 

---

### Control RealHand Topic /cb_left_hand_control_cmd or /cb_right_hand_control_cmd

### RealHand finger motion to specified position

# L10
```bash
# Left hand
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [80,80,80,80,80,80,80,80,80,80], velocity: [], effort: []}"
# Right hand
rostopic pub /cb_right_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [80,80,80,80,80,80,80,80,80,80], velocity: [], effort: []}"
```
# L20
```bash
# Left hand
rostopic pub /cb_left_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10], velocity: [], effort: []}"
# Right hand
rostopic pub /cb_right_hand_control_cmd sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10], velocity: [], effort: []}"
```
**Description**:  
Command finger motion to specified positions. Data format: sensor_msgs/JointState  
**Parameters**:
- `position`: finger motion values list(float)  
  L7 length: 7, L10 length: 10, L20 length: 20, L25 length: 25, each element range 0~255 

---

### Get Hand State Topic /cb_left_hand_state or /cb_right_hand_state
```bash

header: 
  seq: 211345
  stamp: 
    secs: 1744703535
    nsecs: 722361087
  frame_id: ''
name: 
  - joint71
  - joint72
  - joint73
  - joint77
  - joint75
  - joint76
  - joint77
  - joint78
  - joint79
  - joint80
  - joint81
  - joint82
  - joint83
  - joint84
  - joint88
  - joint86
  - joint87
  - joint88
  - joint89
  - joint90
position: [255.0, 132.0, 255.0, 255.0, 255.0, 255.0, 131.0, 127.0, 129.0, 127.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
**Description**:  
Current finger joint state. Data format: sensor_msgs/JointState  
**Parameters**:
- `position`: current finger joint state list(float)  
  L7 length: 7, L10 length: 10, L20 length: 20, L25 length: 25, each element range 0~255 

---

### Get Pressure Data Topic /cb_left_hand_force or /cb_right_hand_force
```bash
rostopic echo /cb_left_hand_touch
data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 255.0, 255.0, 255.0, 255.0, 255.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
**Description**:  
Get finger pressure sensing data. Data format: std_msgs/Float32MultiArray  
**Parameters**:
- `data`:
```bash
Index 0: thumb normal pressure value 0~255
Index 1: index finger normal pressure value 0~255
Index 2: middle finger normal pressure value 0~255
Index 3: ring finger normal pressure value 0~255
Index 4: little finger normal pressure value 0~255

Index 5: thumb tangential pressure value 0~255
Index 6: index finger tangential pressure value 0~255
Index 7: middle finger tangential pressure value 0~255
Index 8: ring finger tangential pressure value 0~255
Index 9: little finger tangential pressure value 0~255

Index 10: thumb tangential pressure direction value 0~255 # 255 means no pressure direction
Index 11: index finger tangential pressure direction value 0~255
Index 12: middle finger tangential pressure direction value 0~255
Index 13: ring finger tangential pressure direction value 0~255
Index 14: little finger tangential pressure direction value 0~255

Index 15: thumb proximity sensing value 0~255
Index 16: index finger proximity sensing value 0~255
Index 17: middle finger proximity sensing value 0~255
Index 18: ring finger proximity sensing value 0~255
Index 19: little finger proximity sensing value 0~255
```
---

### Get Matrix Pressure Data Topic /cb_left_hand_matrix_touch or /cb_right_hand_matrix_touch (Note: only second-generation pressure sensors)
```bash
rostopic echo /cb_left_hand_matrix_touch
data: "{"thumb_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0]], "index_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,  0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "middle_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "ring_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "little_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]}"
```
**Description**:  
Get finger matrix pressure data. Data format: std_msgs/String JSON  
**Parameters**:
- `data`:
```bash
thumb_matrix: thumb matrix pressure values 0~255
index_matrix: index finger matrix pressure values 0~255
middle_matrix: middle finger matrix pressure values 0~255
ring_matrix: ring finger matrix pressure values 0~255
little_matrix: little finger matrix pressure values 0~255
```

### Get Matrix Pressure Data Topic /cb_left_hand_matrix_touch_pc or /cb_right_hand_matrix_touch_pc (Note: only second-generation pressure sensors)
```bash
rostopic echo /cb_left_hand_matrix_touch_pc
---
header:
  seq: 6857
  stamp:
    secs: 1765263761
    nsecs: 911985874
  frame_id: "map"
height: 1
width: 360
fields:
  -
    name: "val"
    offset: 0
    datatype: 7
    count: 1
is_bigendian: False
point_step: 4
row_step: 1440
data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 64, 0, 0, 64, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 48, 65, 0, 0, 248, 65, 0, 0, 36, 66, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 200, 65, 0, 0, 28, 66, 0, 0, 128, 66, 0, 0, 116, 66, 0, 0, 128, 65, 0, 0, 0, 0, 0, 0, 224, 65, 0, 0, 52, 66, 0, 0, 100, 66, 0, 0, 136, 66, 0, 0, 232, 65, 0, 0, 0, 0, 0, 0, 80, 66, 0, 0, 116, 66, 0, 0, 130, 66, 0, 0, 128, 66, 0, 0, 56, 66, 0, 0, 0, 0, 0, 0, 44, 66, 0, 0, 56, 66, 0, 0, 64, 66, 0, 0, 40, 66, 0, 0, 184, 65, 0, 0, 0, 0, 0, 0, 128, 65, 0, 0, 32, 66, 0, 0, 224, 65, 0, 0, 168, 65, 0, 0, 216, 65, 0, 0, 0, 0, 0, 0, 64, 64, 0, 0, 160, 65, 0, 0, 240, 65, 0, 0, 240, 65, 0, 0, 200, 65, 0, 0, 0, 0, 0, 0, 64, 64, 0, 0, 208, 65, 0, 0, 28, 66, 0, 0, 160, 65, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 192, 64, 0, 0, 192, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 160, 64, 0, 0, 192, 64, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 160, 64, 0, 0, 192, 65, 0, 0, 44, 66, 0, 0, 28, 66, 0, 0, 64, 64, 0, 0, 0, 0, 0, 0, 248, 65, 0, 0, 92, 66, 0, 0, 136, 66, 0, 0, 104, 66, 0, 0, 136, 65, 0, 0, 0, 0, 0, 0, 8, 66, 0, 0, 128, 66, 0, 0, 138, 66, 0, 0, 120, 66, 0, 0, 176, 65, 0, 0, 0, 0, 0, 0, 48, 66, 0, 0, 132, 66, 0, 0, 148, 66, 0, 0, 100, 66, 0, 0, 160, 65, 0, 0, 0, 0, 0, 0, 76, 66, 0, 0, 140, 66, 0, 0, 146, 66, 0, 0, 128, 66, 0, 0, 232, 65, 0, 0, 0, 0, 0, 0, 32, 66, 0, 0, 136, 66, 0, 0, 150, 66, 0, 0, 120, 66, 0, 0, 144, 65, 0, 0, 0, 0, 0, 0, 224, 65, 0, 0, 92, 66, 0, 0, 112, 66, 0, 0, 40, 66, 0, 0, 160, 64, 0, 0, 0, 0, 0, 0, 160, 64, 0, 0, 80, 65, 0, 0, 184, 65, 0, 0, 184, 65, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
is_dense: False
---
```
**Description**:  
Get finger matrix pressure data. Data format: PointCloud2  

- Example processing on receiver side
```python
def pc2_to_6x12x5(msg):
    arr = np.frombuffer(msg.data, np.float16)  # 360
    return arr.reshape(5, 6, 12)  # Original data is 5 sets of 6*12 matrices
```

---

### Get RealHand Configuration Info Topic /cb_left_hand_info or /cb_right_hand_info
```bash
rostopic echo /cb_right_hand_info
data: "{\"version\": [7, 0, 0, 0], \"hand_joint\": \"L21\", \"speed\": [1, 0, 0, 0, 0, 0,\
  \ 0, 0, 0, 0, 6, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0], \"current\"\
  : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \"fault\": [[0,\
  \ 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,\
  \ 0, 0, 0, 0, 0]], \"motor_temperature\": [71, 52, 62, 46, 0, 65, 0, 50, 40, 0,\
  \ 0, 39, 0, 52, 41, 0, 0, 38, 0, 53, 41, 0, 0, 39, 0, 50, 40, 0, 0, 38], \"torque\"\
  : [16, 8, 3, 0, 0, 9, 0, 2, 0, 0, 0, 9, 0, 2, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 8,\
  \ 0, 0, 0, 8], \"is_touch\": true, \"touch_type\": 2, \"touch\": [0, 0, 0, 0, 0,\
  \ 0], \"finger_order\": [\"thumb_root\", \"index_finger_root\", \"middle_finger_root\"\
  , \"ring_finger_root\", \"little_finger_root\", \"thumb_abduction\", \"index_finger_abduction\"\
  , \"middle_finger_abduction\", \"ring_finger_abduction\", \"little_finger_abduction\"\
  , \"thumb_roll\", \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_middle_joint\"\
  , \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_tip\", \"index_finger_tip\"\
  , \"middle_finger_tip\", \"ring_finger_tip\", \"little_finger_tip\"]}"
```
**Description**:  
Get RealHand configuration information. Data format: std_msgs/String containing JSON  
**Parameters**:
- `version`: hand version info.  
  - version[0]: hand type (e.g. L10)  
  - version[1]: version  
  - version[2]: batch number  
  - version[3]: 76 for left hand, 82 for right hand, others are internal codes
- `hand_joint`: L10 or L20 or L25, etc.
- `speed`: finger speed
- `current`: current finger voltage (if supported)
- `torque`: finger torque (if supported)
- `is_touch`: whether there is a pressure sensor
- `touch_type`: sensor type (if supported)
- `touch`: sensor data (if supported)
- `max_press_rco`: maximum current
- `fault`: motor fault. 0 = normal, 1 = over-/under-voltage, 2 = magnetic encoder error, 4 = motor over-temperature, 8 = over-current, 32 = overload
- `motor_temperature`: current motor temperature
- `finger_order`: current motor order of the dexterous hand fingers

---



## range_to_arc Radian–Range Conversion Table

Obtain and send radian values for L10 and L20.

topic: /cb_left_hand_state_arc and /cb_right_hand_state_arc get RealHand state with position in radians.

topic: /cb_left_hand_control_cmd_arc and /cb_right_hand_control_cmd_arc publish position in radians to control RealHand finger motion.

## Radian and Range Correspondence Table



#---------------------------------------------------------------------------------------------------

L7 dexterous hand joint order = ["thumb flexion", "thumb abduction","index flexion", "middle flexion", "ring flexion","little finger flexion","thumb rotation"]
# L7 L OK
l7_l_min = [0, 0, 0, 0, 0, 0, -0.52]
l7_l_max = [0.44, 1.43, 1.62, 1.62, 1.62, 1.62, 1.01]
l7_l_derict = [-1, -1, -1, -1, -1, -1, -1]
# L7 R OK (URDF will be changed later!!!)
l7_r_min = [0, -1.43, 0, 0, 0, 0, 0]
l7_r_max = [0.75, 0, 1.62, 1.62, 1.62, 1.62, 1.54]
l7_r_derict = [-1, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L10 dexterous hand joint order = ["thumb base", "thumb abduction","index base", "middle base", "ring base","little finger base","index abduction","ring abduction","little finger abduction","thumb rotation"]
# L10 L OK
l10_l_min = [0, 0, 0, 0, 0, 0, 0, -0.26, -0.26, -0.52]
l10_l_max = [1.45, 1.43, 1.62, 1.62, 1.62, 1.62, 0.26, 0, 0, 1.01]
l10_l_derict = [-1, -1, -1, -1, -1, -1, 0, -1, -1, -1]
# L10 R OK
l10_r_min = [0, 0, 0, 0, 0, 0, -0.26, 0, 0, -0.52]
l10_r_max = [0.75, 1.43, 1.62, 1.62, 1.62, 1.62, 0, 0.13, 0.26, 1.01]
l10_r_derict = [-1, -1, -1, -1, -1, -1, -1, 0, 0, -1]
#---------------------------------------------------------------------------------------------------

L20 dexterous hand joint order = ["thumb base", "index base", "middle base", "ring base","little finger base","thumb abduction","index abduction","middle abduction","ring abduction","little finger abduction","thumb horizontal abduction","reserved","reserved","reserved","reserved","thumb tip","index tip","middle tip","ring tip","little finger tip"]
# L20 L OK
l20_l_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0.122, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_l_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
# L20 R OK
l20_r_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_r_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L21 dexterous hand joint order = ["thumb base","index base","middle base","ring base","little finger base","thumb abduction","index abduction","middle abduction","ring abduction","little finger abduction","thumb roll","reserved","reserved","reserved","reserved","thumb middle joint","reserved","reserved","reserved","reserved","thumb tip","index tip","middle tip","ring tip","little finger tip"]
# L21 L OK
l21_l_min = [0, 0, 0, 0, 0, 0, 0, -0.18, -0.18, 0, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_l_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
# L21 R OK
l21_r_min = [0, 0, 0, 0, 0, 0, -0.18, -0.18, -0.18, -0.18, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_r_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

















### Set Joint Positions
```python
def finger_move(self,pose=[])
```
**Description**:  
Set target joint positions to control finger motion.  
**Parameters**:  
- `pose`: a list of floats containing target position data; L7 length 7 elements, L10 length 10 elements, L20 length 20 elements, L25 length 25 elements.

---

### Set Motor Current Values
```python
def set_current(self, current=[])
```
**Description**:  
Set the motor current values.  
**Parameters**:  
- `current`: a list of ints containing target current values, length 5 elements. Currently only supported for the L20 version.

---

### Get Speed
```python
def get_speed(self)
return [180, 200, 200, 200, 200]
```
**Description**:  
Get the currently configured speed values. Note: joint positions must be set before speed can be obtained.

**Returns**:  
- Returns a list containing the current speed settings for the fingers.

---

### Get Current Joint State
```python
def get_state(self)
return [81, 79, 79, 79, 79, 79, 83, 76, 80, 78]
```
**Description**:  
Get the current joint state as a list of floats. Note: joint positions must be set before state information can be obtained. L7 length 7 elements, L10 length 10 elements, L20 length 20 elements, L25 length 25 elements.

**Returns**:  
- Returns a list of floats containing the current joint state data.

---

### Get Normal Force, Tangential Force, Tangential Direction, Proximity Sensing
```python
def get_force(self)
return [[255.0, 0.0, 0.0, 77.0, 192.0], [82.0, 0.0, 0.0, 230.0, 223.0], [107.0, 255.0, 255.0, 31.0, 110.0], [255.0, 0.0, 20.0, 255.0, 255.0]]
```
**Description**:  
Get combined hand sensor data, including normal force, tangential force, tangential direction, and proximity sensing.  
**Returns**:  
- Returns a 2D list; each sublist contains a different category of pressure data [[normal_force], [tangential_force], [tangential_direction], [proximity]]. Within each category, each element corresponds to thumb, index, middle, ring, and little finger.

---

### Get Version
```python
def get_version(self)
return [10, 6, 22, 82, 20, 17, 0]
```
**Description**:  
Get the current software or hardware version information.  
**Returns**:  
- Returns a list whose elements, in order, represent: degrees of freedom, version number, index, 76 for left hand / 82 for right hand, and internal serial numbers.

---
--------------------------------------------------------------
### Get Torque
```python
def get_torque(self)
return [200, 200, 200, 200, 200]
```
**Description**:  
Get the current finger torque list information. Each value represents the current motor torque for each finger. Supported for L20 and L25.

**Returns**:  
- Returns a list of floats.

---

### Get Motor Temperature
```python
def get_temperature(self)
return [41, 71, 45, 40, 50, 47, 58, 50, 63, 70]
```
**Description**:  
Get the current motor temperature of each joint.

**Returns**:  
- Returns a list containing the current motor temperatures of each joint.

---

### Get Motor Fault Codes
```python
def get_fault(self)
return [0, 4, 0, 0, 0, 0, 0, 0, 0, 0]
```
**Description**:  
Get the current motor fault codes for each joint. 0 means normal; 1 means current overload; 2 means temperature too high; 3 means encoder error; 4 means over-/under-voltage.

**Returns**:  
- Returns a list of floats containing the motor fault codes for each joint.

---

### Clear Motor Fault Codes
```python
def clear_faults(self)
```
**Description**:  
Attempt to clear motor faults. No return value. Only supported for L20.  
**Returns**:  

---

## Example Usage

The following is a complete example showing how to use the above APIs:

```python

from RealHand.real_hand_api import RealHandApi
def main():
    # Initialize API hand_type: left or right   hand_joint: L7 or L10 or L20 or L25
    real_hand = RealHandApi(hand_type="left", hand_joint="L10")
    # Set finger speed
    real_hand.set_speed(speed=[120,200,200,200,200])
    # Set finger torque
    real_hand.set_torque(torque=[200,200,200,200,200])
    # Get current hand state
    hand_state = real_hand.get_state()
    # Print state values
    print(hand_state)

```

---

## Notes
- Before using the API, please ensure the hand device is properly connected and initialized.
- For the specific ranges and meanings of parameter values (such as speed, force, etc.), please refer to the device technical manual.

---

## Contact
- If you have any questions or need further support, please contact [support@realhand.com](mailto:support@realhand.com).

---
