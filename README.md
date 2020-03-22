# ARS408_ros

## Dependence
* Ubuntu 18.04
* ROS melodic

## Usage
```bash
# bulid
cd <Your ROS Workspace>\src
git clone https://github.com/YoYo860224/ars408_ros.git
cd ..
rosdep install -i ars408_ros --os=ubuntu:bionic
catkin_make

# run
roslaunch ars408_ros lan.launch
```