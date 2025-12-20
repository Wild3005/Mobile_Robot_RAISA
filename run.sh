#!/bin/bash

. install/setup.bash 
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=10
ros2 launch ros2_utils all.launch.py 

