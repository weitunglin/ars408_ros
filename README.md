# ARS408_ros

# ncsist

## install repo

```
1. mkdir -p catkin_ws/src
2. git clone -b ncsist https://github.com/weitunglin/ars408_ros.git ARS408_ros
```

## install dependencies

- [ ] TODO

## build project

```
1. cd catkin_ws
2. catkin_make
3. source devel/setup.bash
```

## run param optimization


prepare [param.yaml](https://github.com/weitunglin/ars408_ros/blob/ncsist/ars408_ros/param.yaml) file.

example param.yaml for 1201_17d_1.bag
```
checkpoints:
  - - 337
    - 990
  - - 508
    - 574
  - - 603
    - 354
  - - 657
    - 213
  - - 691
    - 117
  - - 710
    - 47
interval_dist: 10
optimizer_step: 3
roi_region:
  - - 300
    - 0
  - - 1100
    - 156
speed_threshold: 5
start_dist: 15
```

```
(first terminal)
- roscore
(second terminal) (needed if replaying bag file) 
- rosbag play <bagfile>
(third terminal)
- roslaunch ars408_ros optimize.launch param_path:=/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/param/0410_1.yaml model_path:=/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/weights/yolov8m.pt
(fourth terminal)
- rostopic pub /optimizer_start std_msgs/Empty -1
```

after optimization finished `/optimizer` topic will display the results

## run traffic analysis

```
(first terminal)
- roscore
(second terminal) (needed if replaying bag file) 
- rosbag play <bagfile>
(third terminal)
- roslaunch ars408_ros replay_custom.launch
```

`/mot_image` will display the results

## record bag file

```
(first terminal)
- roscore
(second terminal)
- roslaunch ars408_ros record_custom.launch
(third terminal)
- rosbag record --duration 30 -O test.bag /camera/image_raw/compressed /rgb/front_center/decoded_messages
```

after recorded, you can validate the bag using `rosbag info test.bag`
