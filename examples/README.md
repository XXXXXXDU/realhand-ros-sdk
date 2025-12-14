# LinkerHandROS SDK Example Programs
<!-- TOC --> 
[examples (Samples)](#)
## L10/L20
- [0001-get_linker_hand_state (Get current LinkerHand dexterous hand state)](L20_get_linker_hand_state/)
- [0002-gui_control (GUI control)](gui_control/)
- [0003-get_linker_hand_force (Get LinkerHand dexterous hand force sensor data)](get_linker_hand_force/)
- [0004-get_linker_hand_speed (Get current LinkerHand dexterous hand speed)](get_linker_hand_speed/)
- [0005-get_linker_hand_current (Get current LinkerHand dexterous hand current)](get_linker_hand_current/)
- [0006-set_linker_hand_speed (Set LinkerHand dexterous hand speed)](set_linker_hand_speed/)
- [0007-set_linker_hand_current (Set current for LinkerHand dexterous hand)](set_linker_hand_current/)
- [0008-set_linker_hand_torque (Set LinkerHand dexterous hand torque)](set_linker_hand_torque/)
- [0009-finger_guessing (Interactive example, rock–paper–scissors game)](finger_guessing/) Note: requires an RGB camera
---
## Python L20
- [0101-lipcontroller (Use tactile sensor with dexterous hand for pinching/grasping)](L20/gesture-show/lipcontroller.py)
- [0102-gesture-Show-OK (Use Python to control the hand to make an OK gesture)](L20/gesture-show/gesture-Show-OK.py)
- [0103-gesture-Show-Surround-Index-Finger (Use Python to control the hand to do a rotating index finger gesture)](L20/gesture-show/gesture-Show-Surround-Index-Finger.py)
- [0104-gesture-Show-Wave (Use Python to control the hand to do a waving motion)](L20/gesture-show/gesture-Show-Wave.py)
- [0105-gesture-Show-Ye (Use Python to control the hand to perform a complex demonstration sequence)](L20/gesture-show/gesture-Show-Ye.py)
- [0106-gesture-Loop (Use Python to control the hand to repeatedly perform a grasping motion)](L20/gesture-show/gesture-Loop.py)
## Python L25
- [0107-action_group_l25 (Use Python to control L25 “finger dance”)](L25/gesture/action_group_l25.py)
## Python L7
- [0108-action-group-show-ti (Use Python to control L7 “finger dance”)](L7/gesture/action-group-show-ti.py)
---
## L25
- [0201-set_disability (Set L25 dexterous hand to disabled mode)](L25/set_disability.py)  

      $ python set_disability.py --hand_type=left or right

- [0202-set_enable (Set L25 dexterous hand to enabled mode)](L25/set_enable.py)  

      $ python set_enable.py --hand_type=left or right

- [0203-set_remote_control (Set L25 dexterous hand to teleoperation mode)](L25/set_remote_control.py)  

      $ python set_remote_control.py --hand_type=left or right

---
## Imitation Learning
- [1001-human-dex (Use LinkerHand dexterous hand for imitation learning training and autonomous grasping)](https://github.com/linkerbotai/human-dex)
- [1002-linker_unidexgrasp (Unidexgrasp dexterous hand grasping algorithm based on LinkerHand)](https://github.com/linkerbotai/linker_unidexgrasp)



## LinkerHand Dexterous Hand Configuration File Description
The LinkerHand dexterous hand needs to be configured using a parameter file. Modify the corresponding configuration parameters according to your actual needs.

(1) Modify the configuration file to match the actual LinkerHand dexterous hand hardware:

    $ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros/config
    $ sudo vim setting.yaml

![SETTING](../doc/setting.png) 

Because the GUI can only control a single LinkerHand dexterous hand at a time, you need to configure the file so that it matches the specific LinkerHand dexterous hand you are using.


## LinkerHand Dexterous Hand Example 100
LinkerHand dexterous hand example 100 provides many example cases and source code, fully demonstrating the capabilities of the LinkerHand dexterous hand.

- Preparation – start the SDK

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch


- #### 0001 – Get current LinkerHand dexterous hand state; the state values include range and radian values

Open a new terminal

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    # If _loop is True, the terminal will continuously print the current state values of the LinkerHand dexterous hand.
    # If False, it will print the current state values only once.
    $ rosrun L20_get_linker_hand_state L20_get_linker_hand_state.py _loop:=True

![STATE](../doc/state.png)


- #### 0002 – GUI control

Using the GUI, you can control each joint of the LinkerHand dexterous hand L10/L20 independently with sliders.  
You can also use the add button to record the current values of all sliders, saving the current motion state of each joint.  
Then you can replay those actions through function buttons.

To use `gui_control` to control the LinkerHand dexterous hand:  
The `gui_control` interface needs `linker_hand_sdk_ros` to be running, and it operates on the LinkerHand dexterous hand via topics.

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal and start the GUI control:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ rosrun gui_control gui_control.py

After launching, a UI window will pop up. You can control the corresponding LinkerHand dexterous hand joints using the sliders.  
You can also use the add button on the right side to save the current slider data, so it can be used for replay later.

![START_SDK](../doc/gui_control.png) 

- #### 0003 – Get LinkerHand dexterous hand force sensor data  
  Note: supported only in versions after V2.1.4; no longer supported here and should be obtained via topics instead.

After starting the SDK, open a new terminal:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    # If _loop is True, the terminal will continuously print the current state values of the LinkerHand dexterous hand.
    # If False, it will print the current state values only once.
    $ rosrun get_linker_hand_force get_linker_hand_force.py _loop:=False
    #2025-01-15 15:43:16  No data for left hand
    #2025-01-15 15:43:16  Right hand five-finger normal force: [0.0, 0.0, 0.0, 0.0, 0.0]
    #2025-01-15 15:43:16  Right hand five-finger tangential force: [0.0, 0.0, 0.0, 0.0, 0.0]
    #2025-01-15 15:43:16  Right hand five-finger tangential force direction: [255.0, 255.0, 255.0, 255.0, 255.0]
    #2025-01-15 15:43:16  Right hand five-finger proximity sensing: [0.0, 0.0, 0.0, 0.0, 0.0]


- #### 0004 – Get current LinkerHand dexterous hand speed

After starting the SDK, open a new terminal:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    # If _loop is True, the terminal will continuously print the current state values of the LinkerHand dexterous hand.
    # If False, it will print the current state values only once.
    $ rosrun get_linker_hand_speed get_linker_hand_speed.py _loop:=False
    #2025-01-15 15:57:17  No data for left hand
    #2025-01-15 15:57:17  Current right hand five-finger speeds: [180, 250, 250, 250, 250]


- #### 0005 – Get current LinkerHand dexterous hand current

After starting the SDK, open a new terminal:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    # If _loop is True, the terminal will continuously print the current state values of the LinkerHand dexterous hand.
    # If False, it will print the current state values only once.
    $ rosrun get_linker_hand_current get_linker_hand_current.py _loop:=False
    #2025-01-15 16:25:29  No data for left hand
    #2025-01-15 16:25:29  Current right hand five-finger currents: [42, 42, 42, 42, 42]


- #### 0009 – Interactive example: rock–paper–scissors game  
  Note: requires an RGB camera

After starting the SDK, open a new terminal:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ rosrun finger_guessing finger_guessing.py

---

- #### 0101 – Tactile sensor with dexterous hand for pinching/grasping

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal to run the demo example:

    python ./<your_file_path>/lipcontroller.py

![Start demo](../doc/开始演示.png)

- If the terminal prints “__Start demo__”, it is running normally.  
  If the hand configuration is correct, the index and middle fingers should start a pinching motion.  
  When an object is pinched, the hand will stop. When the object is removed, the hand will try to pinch again, until it either pinches an object or reaches its motion limit.  
  The limit state is shown below:

![Limit position](../doc/极限位置.png)

-  __lipcontroller.py__ is a demo developed for the version 7 hand.  
   When using it as a demo on other versions, you need to adjust the alignment posture between thumb and index finger; otherwise it cannot perform the action “__index finger and thumb pinch together__” correctly.

- #### 0102 – Use Python to control the hand to make an OK gesture

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal to run the demo example:

    python ./<your_file_path>/gesture-Show-OK.py
    # After starting, the terminal will print "testing".
    # The hand will start making an OK gesture, bending and straightening the middle, ring, and little fingers.

<img src="../doc/20250221-135722.jpeg" width="300" height="300" /><img src="../doc/20250221-135706.jpeg" width="300" height="300" />

- #### 0103 – Use Python to control the hand to do a rotating index finger motion

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal to run the demo example:

    python ./<your_file_path>/gesture-Show-Surround-Index-Finger.py
    # After starting, the terminal will print "testing".
    # The hand will form a fist and extend the index finger, and the index finger will repeatedly rotate.

- #### 0104 – Use Python to control the hand to do a waving motion

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal to run the demo example:

    python ./<your_file_path>/gesture-Show-Wave.py
    # After starting, the terminal will print "testing".
    # The thumb will extend outward and remain still, while the other four fingers perform a wave-like motion.

- #### 0105 – Use Python to control the hand to perform a complex demonstration sequence

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

![START_SDK](../doc/start_sdk.png) 

After successful startup, information such as SDK version, CAN interface status, dexterous hand configuration, and current joint speed will be displayed.  

Open a new terminal to run the demo example:

    python ./<your_file_path>/gesture-Show-Ye.py
    # After starting, the terminal will print "testing".
    # The hand will perform a complex sequence of actions to showcase its dexterity.

- This example is a demo developed for the version 7 hand.  
  When using it as a demo on other versions, you need to adjust the alignment posture between thumb and index finger; otherwise it cannot perform the action “__index finger and thumb pinching or closing together__” correctly.

- #### 0106 – Use Python to control the hand to repeatedly grasp

To use this example, you need to start `linker_hand_sdk_ros`:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

Open a new terminal and run the Python file:

    $ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/gesture-show
    $ python gesture-Loop.py 


- #### 0201 – Set L25 dexterous hand to disabled mode

This disables the motors of the L25 dexterous hand so that you can freely move each joint manually.

First start LinkerHandSDKROS:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

Open a new terminal and run the disable function program:

    $ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
    $ python set_disability.py


- #### 0202 – Set L25 dexterous hand to enabled mode

This enables the motors of the L25 dexterous hand. Once enabled, you can control it with control programs.

First start LinkerHandSDKROS:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

Open a new terminal and run the enable function program:

    $ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
    $ python set_enable.py


- #### 0203 – Set L25 dexterous hand to teleoperation mode

If you have multiple L25 dexterous hands of the same version, you can use this example to control one enabled L25 dexterous hand with another disabled L25 dexterous hand of the same version.

First start LinkerHandSDKROS.  
Below is the configuration for the controlled L25 dexterous hand, using a right hand as an example.

First ensure that the two Ubuntu machines are on the same network and correctly configured as ROS master/slave, so that they can communicate via ROS.  
You can refer to the [official ROS documentation](https://wiki.ros.org/).

Controller machine A configuration:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

Open a new terminal and run the remote-control program:

    $ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
    $ python set_remote_control.py

Controlled machine B configuration:

    # Open a new terminal and start ROS
    $ roscore

Open a new terminal and start the ROS SDK:

    $ cd Linker_Hand_SDK_ROS/
    $ source ./devel/setup.bash
    $ roslaunch linker_hand_sdk_ros linker_hand.launch

At this point, manually moving the disabled L25 dexterous hand on machine A will control the enabled L25 dexterous hand on machine B.


- #### 1001 – Use LinkerHand dexterous hand for imitation learning training

To use this example, you need Ubuntu 20.04 with ROS Noetic.  
The default hardware is the LinkerRobot humanoid robot, but you can also use other robotic arms or robots for imitation learning, as long as you change the corresponding data topics.

[For detailed instructions, please refer to the human-dex project README.md](https://github.com/linkerbotai/human-dex)

1. Environment setup

        cd human-dex
        conda create -n human-dex python=3.8.10
        conda activate human-dex
        pip install torchvision
        pip install torch
        pip install -r requirements.txt

2. Installation

        mkdir -p your_ws/src
        cd your_ws/src
        git clone https://github.com/linkerbotai/human-dex.git
        cd ..
        catkin_make
        source ./devel/setup.bash

3. Run

        # Data collection
        roslaunch record_hdf5 record_hdf5.launch
        # Open a new terminal to send the collection command
        rostopic pub /record_hdf5 std_msgs/String "data: '{\"method\":\"start\",\"type\":\"humanplus\"}'"

4. Training

        cd humanplus/scripts/utils/HIT
        python3 imitate_episodes_h1_train.py --task_name data_cb_grasp --ckpt_dir cb_grasp/ --policy_class HIT --chunk_size 50 --hidden_dim 512 --batch_size 48 --dim_feedforward 512 --lr 1e-5 --seed 0 --num_steps 100000 --eval_every 1000 --validate_every 1000 --save_every 1000 --no_encoder --backbone resnet18 --same_backbones --use_pos_embd_image 1 --use_pos_embd_action 1 --dec_layers 6 --gpu_id 0 --feature_loss_weight 0.005 --use_mask --data_aug

5. Reproduction / evaluation

        cd humanplus/scripts
        python3 cb.py


---
- #### 1002 – Unidexgrasp dexterous hand grasping algorithm based on LinkerHand

The original Unidexgrasp algorithm uses the Shadow Hand; here we provide the code for implementing the Unidexgrasp algorithm on LinkerHand.

[For the detailed process, please refer to the linker_unidexgrasp project](https://github.com/linkerbotai/linker_unidexgrasp)

## Grasp pose generation

The grasp pose generation part uses a mapping scheme to map the model’s output Shadow Hand pose to the LinkerHand L20 pose, for subsequent development.

1. Environment setup

        conda create -n unidexgrasp python=3.8
        conda activate unidexgrasp
        conda install -y pytorch==1.10.0 torchvision==0.11.0 torchaudio==0.10.0 cudatoolkit=11.3 -c pytorch -c conda-forge
        conda install -y https://mirrors.bfsu.edu.cn/anaconda/cloud/pytorch3d/linux-64/pytorch3d-0.6.2-py38_cu113_pyt1100.tar.bz2
        pip install -r requirements.txt
        cd thirdparty/pytorch_kinematics
        pip install -e .
        cd ../nflows
        pip install -e .
        cd ../
        git clone https://github.com/wrc042/CSDF.git
        cd CSDF
        pip install -e .
        cd ../../

2. Training  

GraspIPDF:

        python ./network/train.py --config-name ipdf_config --exp-dir ./ipdf_train

GraspGlow:

        python ./network/train.py --config-name glow_config --exp-dir ./glow_train
        python ./network/train.py --config-name glow_joint_config --exp-dir ./glow_train

ContactNet:

        python ./network/train.py --config-name cm_net_config --exp-dir ./cm_net_train

3. Evaluation

        python ./network/eval.py --config-name eval_config --exp-dir=./eval

4. Mapping – result visualization

        python ./tests/visualize_result_l20_shadow.py --exp_dir 'eval' --num 3

Save the results for later use in reinforcement learning algorithm development:

        python ./tests/data_for_RL.py
