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
    * more and more ...
* ars408_setting 
    * Set radar.
* ars408_srv
    * Some filter funtion (now for RCS).

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

## Arti command
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
```

# AI command
```bash
# Start
roslaunch ars408_ros record.launch
roslaunch ars408_ros replay.launch
roslaunch ars408_ros dualV.launch

# Radar setting
rosrun ars408_setting ARS200_Setting.py

# Set RCS filter
rosrun ars408_srv filter_client 0

# Record, no duration
rosbag record -o <PREFIX> /received_messages /rgbImg /thermalImg /GPSinfo
rosbag record -o <PREFIX> /rgbImg /thermalImg
# Record, duration
rosbag record --duration=120 -o <PREFIX> /received_messages /rgbImg /thermalImg /GPSinfo /velodyne_points
rosbag record --duration=120 -o <PREFIX> /received_messages /rgbImg /thermalImg /GPSinfo
rosbag record --duration=120 -o <PREFIX> /rgbImg /thermalImg
rosbag record --duration=60 -o <PREFIX> /velodyne_points
rosbag record -o Ultrasonic /rgbImg /Ultrasonic
rosbag record -o multiRGB /first/rgbImg /second/rgbImg /third/rgbImg
rosbag record -o output /rgbImg2 /rgbImg4 /rgbImg6
# Play
rosbag play <name.bag>
rosbag play --clock <name.bag>    # 要回放 lidar，rosparam 也要設置 use_sim_time (因為用到 tf 的關係)
# To Video --clock 的話 bag 跑完之後 toVideo.py 才會自動結束
roscore
rosparam set use_sim_time true
rosrun ars408_ros toVideo.py -o filename
rosbag play --clock -l <name.bag> # -l 表示loop

# Pure Record
rosrun ars408_ros toVideo.py -s -r ~/outDir -o seq1
```

## Referance
* [CAN-BUS](https://hackmd.io/@yoyo860224/HkkAS9F88)
* [yolo](https://github.com/a888999a/yolov3fusion1#-to-080)
* [yolo_weight](https://drive.google.com/file/d/1XEIJP14Q6jJK5dum3lXq_rAyU9FOuI3J/view)
* [yolo_origin](https://github.com/YunYang1994/tensorflow-yolov3)
* [LidarPlot-Ref](https://github.com/darylclimb/cvml_project/tree/master/projections/lidar_camera_projection)
* [yolo_torch_github](https://github.com/BalinLin/yolov4_torch.git)
* [yolo_torch_google](https://drive.google.com/file/d/1Em5QUIsBIcCpu2wv0WzKuOuj68FpXoNl/view)
* [yolo_torch_weight](https://drive.google.com/drive/folders/1sEFwu-2b_vfedT-6qHVaarRrpErIzSY9)
* [yolo_torch_weight_20210108](https://drive.google.com/drive/folders/1vEs-OFp6tTEUtk2I6XadkwAc9hNM9kMG?usp=sharing)
* [yolo_torch_origin](https://github.com/Tianxiaomo/pytorch-YOLOv4)