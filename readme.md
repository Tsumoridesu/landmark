# mcl_with_landmark

## Requirements
theta_simple_stitching

https://github.com/open-rdc/theta_simple_stitching

yolov5_pytorch_ros

https://github.com/Tsumoridesu/yolov5_pytorch_ros

orne_navigation(branch: landmark)

https://github.com/open-rdc/orne_navigation

## Installation

for ubuntu 20.04

### yolov5_pytorch_ros
```
git clone https://github.com/Tsumoridesu/yolov5_pytorch_ros.git
```
### landmark
```
git clone https://github.com/Tsumoridesu/landmark.git
```
### theta_simple_stitching
```
git clone https://github.com/open-rdc/theta_simple_stitching.git
```

### build
```
catkin build yolov5_pytorch_ros landmark theta_simple_stitching
# adds package to your path
source ~/catkin_ws/devel/setup.bash 
```

## Usage

```commandline  
# run your mcl first

# run theta_simple_stitching
sudo chmod 666 /dev/bus/sub/001/*
roslaunch theta_simple_stitching theta.launch

# run landmark
python test_fot_alg.py

# run yolov5_pytorch_ros
roslaunch yolov5_pytorch_ros detector.launch
```

## Topics
### Subscribed topics
* **`particlecloud`** (geometry_msgs::PoseArray)

  subscribed to the particlecloud topic published by the particle filter

* **`detected_objects_in_imag`** (yolov5_pytorch_ros::BoundingBoxes)

    subscribed to the bounding boxes topic published by the yolov5_pytorch_ros node
### Published topics
* **`vision_weight`** (std_msgs::Float64MultiArray)
    
    published to the vision_weight for your particle filter

## TODO
* add launch file for mcl_with_landmark
* rewrite test_fot_alg.py
