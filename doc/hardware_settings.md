# Using the LinkerHand Dexterous Hand ROS SDK on Desktop (Laptop), Raspberry Pi, or Jetson Devices

## Description
The LinkerHand dexterous hand hardware and ROS SDK software can be used on most x86 and arm64 devices.

- __The device must be running Ubuntu 20.04 with ROS Noetic and Python 3.8__
- __The device must have a standard 5V USB port__

## Usage

  __Before use, please modify the [setting.yaml](../linker_hand_sdk_ros/config/setting.yaml) configuration file according to your actual needs.__  

  __Insert the LinkerHand dexterous hand USB-to-CAN device into the Ubuntu machine.__  

  Make sure the current system environment is Ubuntu 20.04, ROS Noetic, Python 3.8.10.

- Download

    $ mkdir -p Linker_Hand_SDK_ROS/src  
    $ cd Linker_Hand_SDK_ROS/src  
    $ git clone https://github.com/linkerbotai/linker_hand_sdk.git  

- Build

    $ cd Linker_Hand_SDK_ROS  
    $ pip install -r requirements.txt  
    $ catkin_make  

- Set the `ip` command to NOPASSWORD mode

    $ sudo visudo  
    # Add the following lines  
    your_username ALL=(ALL) NOPASSWD: /sbin/ip  
    your_username ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can0 up type $ $ can bitrate 1000000  
    # Save and exit  

- Configure ROS master/slave (only effective in this terminal; ignore if you do not need ROS master/slave communication)

    $ source /opt/ros/noetic/setup.bash  

    $ export ROS_MASTER_URI=http://<ROS Master IP>:11311  

    $ export ROS_IP=<local IP>  

    $ export ROS_HOSTNAME=<local IP>  

- Start the SDK

    # Enable CAN port  
    $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000  # USB-to-CAN device blue LED stays on  
    $ cd ~/Linker_Hand_SDK_ROS/  
    $ source ./devel/setup.bash  
    $ roslaunch linker_hand_sdk_ros linker_hand.launch  

## Motion Capture Glove Teleoperation

- First start the ROS2 to ROS1 bridge on this machine

    # After installing ROS2 Foxy on this machine, run the following commands in a new terminal  
    $ source /opt/ros/foxy/setup.bash  
    $ export ROS_DOMAIN_ID=11  
    $ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics  

- After starting the above, you can receive motion capture glove teleoperation data.  
  On the mini PC, the Ethernet port **with a router label** has IP: `192.168.11.222`,  
  and the Ethernet port **without a label** has IP: `192.168.11.221`.
