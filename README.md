# ARS408_ros

## Dependence
* Ubuntu 18.04
* ROS melodic

* opencv-python
* pyqt5 (version 5.9.2 for best effect for GUI)
## Guideline
* ars408_msg
    * Msg define, for showing something we want.
* ars408_ros
    * `cantopic2data.cpp`: Decode can message.
    * `visualRadar.cpp`: Show msg on rviz.
    * `easyCamToRos.py`: Camera to ros example.
* ars408_ros setting
    * Run with python.

## Usage
```bash
# bulid
cd <Your ROS Workspace>\src
git clone https://github.com/YoYo860224/ars408_ros.git
cd ..
rosdep install -i --from-paths src --os=ubuntu:bionic
catkin_make

# run
roslaunch ars408_ros lan.launch
```

## Plugin
* ars408_setting

## command
```bash
# Setup (can_setup.sh)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Show
candump can0

# Send
cansend can0 <id>#<msg>
# Example: Object
cansend can0 200#0800000008000000

# rosbag
rosbag record --duration=120 -O zoo /received_messages /camImg
rosbag play zoo.bag
```

## Referance
* [CAN-BUS](https://hackmd.io/@yoyo860224/HkkAS9F88)
