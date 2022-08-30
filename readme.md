# mcl_with_landmark

## Requirements
theta_simple_stitching

https://github.com/open-rdc/theta_simple_stitching

yolov5_pytorch_ros

https://github.com/Tsumoridesu/yolov5_pytorch_ros

orne_navigation(branch: landmark)

https://github.com/open-rdc/orne_navigation

## Installation

### for ubuntu 20.04

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

### extra for ubuntu 18.04

```
# ros melodic should be install cv_brdige first
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-kinetic-cv-bridge
# Create catkin workspace
mkdir catkin_workspace
cd catkin_workspace
catkin init
# Instruct catkin to set cmake variables(be careful python3 version)
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# Instruct catkin to install built packages into install place. It is $CATKIN_WORKSPACE/install folder
catkin config --install
# Clone cv_bridge src
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
# Find version of cv_bridge in your repository
apt-cache show ros-kinetic-cv-bridge | grep Version
    Version: 1.12.8-0xenial-20180416-143935-0800
# Checkout right version in git repo. In our case it is 1.12.8
cd src/vision_opencv/
git checkout 1.12.8
cd ../../
# Build
catkin build cv_bridge
# Extend environment with new package
source install/setup.bash --extend
```
### requirements
```
pip3 install pyyaml
pip3 install numpy
```


## Usage
mcl_with_landmark baseed the YOLOv5 to give the weight for your particlecloud.

you should add the landmark information to ```/landmark_cfg/landmark_list.yaml``` first.

the landmark pose is based on your map.



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
