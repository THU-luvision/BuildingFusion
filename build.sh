#!/bin/bash
source /opt/ros/kinetic/setup.bash

cd src/collaborative_fusion/src/scn_cpp/sparseconvnet/build
rm -rf*
cmake ..
make -j4

cd ../../..
cd ../../..

catkin_make -j8 -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
