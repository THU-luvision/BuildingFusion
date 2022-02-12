#!/bin/bash

source /opt/ros/kinetic/setup.bash 
source devel/setup.bash 

roscore &>/dev/null &
rosrun collaborative_fusion collaborative_fusion_server &>/dev/null &
rosrun collaborative_fusion collaborative_fusion_client /data
