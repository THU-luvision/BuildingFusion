#!/bin/bash

source /opt/ros/kinetic/setup.bash 
source devel/setup.bash 

roscore &>/dev/null &
sleep 5
rosrun collaborative_fusion collaborative_fusion_server &>/dev/null &
sleep 5
rosrun collaborative_fusion collaborative_fusion_client /data
