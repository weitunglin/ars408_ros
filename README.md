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
rosdep install -i --from-paths src --os=ubuntu:bionic
catkin_make

# run
roslaunch ars408_ros lan.launch
```

## cansend
```bash
# Object
cansend can0 200#38000000080C0000
# Cluster
cansend can0 200#38000000100C0000
```


point      白色
car        紅色
truck      紫色
reserverd  藍色
motocycle  綠色
bicycle    黃色
wide       青色
reserved   藍色