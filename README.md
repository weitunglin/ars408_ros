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
* ars408_setting 
    * Set radar.
* ars408_srv
    * Some filter funtion.

## Usage
```bash
# bulid
cd <Your ROS Workspace>\src
git clone https://github.com/YoYo860224/ars408_ros.git
cd ars408_ros
python -m pip install -r requirement.txt
cd ../..
rosdep install -i --from-paths src --os=ubuntu:bionic
catkin_make

# run
roslaunch ars408_ros lan.launch
roslaunch ars408_ros lan.launch replay:=true
```

## command
```bash
# Setup (can_setup.sh)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Radar setting
rosrun ars408_setting ARS200_Setting.py 

# Show
candump can0

# Send
cansend can0 <id>#<msg>
# Example: Object
cansend can0 200#0800000008000000

# rosbag
rosbag record --duration=120 -o <PREFIX> /received_messages /rgbImg /thermalImg /speed /zaxis
rosbag play <name.bag>
```

## Arduino
```bash
# 開啟Arduino IDE
sudo ~/Documents/arduino-1.8.13/arduino

# ROS
roscore

# 更改權限 並執行serial_node
sudo chmod 666 /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```

## Referance
* [CAN-BUS](https://hackmd.io/@yoyo860224/HkkAS9F88)
