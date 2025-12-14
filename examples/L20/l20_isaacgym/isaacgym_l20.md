# Simulating the L20 Environment with IsaacGym

> Note: The following tutorial assumes you have already installed IsaacGym and successfully run its examples. If you have not installed it or failed to run the examples, please refer to the Nvidia official website (https://developer.nvidia.com/isaac-gym) for a detailed installation guide.

---

## Step 1: Configure the URDF File Path

Fill in the URDF file path for the L20 in `AssetDesc` to ensure the simulation environment can load the model correctly.

---

## Step 2: Start ROS Core

Open a new terminal and run the following command to start the ROS core:

    roscore

---

## Step 3: Start the L20 Simulation Example

Run `l20_example.py` to start the simulation environment:

    python l20_example.py

- Expected result: The IsaacGym visualization window starts and displays the L20 dexterous hand model.

---

## Step 4: Verify the Control Node

Open a new terminal and check whether the `/arm_control` topic exists:

    rostopic list

- If it exists: Proceed to the next step.
- If it does not exist: Check whether `roscore` is running, or whether `l20_example.py` started successfully.

---

## Step 5: Manually Control the Dexterous Hand

### 5.1 Send an Initial Control Command

Open a new terminal and send an initial joint state command:

    rostopic pub /arm_control sensor_msgs/JointState "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, name: [], position: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], velocity: [], effort: []}"

### 5.2 Observe and Adjust the Motion

- Observe: Watch the hand’s motion in the IsaacGym visualization window.
- Adjust: Modify the values in the `position` array and send new commands to control the hand’s pose.

---

## Step 6: Control via ROS Code

In your ROS code, publish control commands to `/arm_control` using the same format as the manual control. Example:

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node('l20_controller')
    pub = rospy.Publisher('/arm_control', JointState, queue_size=10)

    # Define joint state
    joint_state = JointState()
    joint_state.name = ['joint1', 'joint2', ..., 'joint20']  # Replace with actual joint names
    joint_state.position = [0.1, 0.2, ..., 0.0]              # Replace with target joint angles

    # Publish control command
    pub.publish(joint_state)

---

Notes:

- URDF version: This tutorial only supports the `l20_8` version of the URDF file. Other versions may not be controllable.
- Dependencies: Make sure ROS and the required dependent libraries (such as `sensor_msgs`) are installed.

If you encounter any issues, please check the logs from IsaacGym and ROS to troubleshoot errors.
