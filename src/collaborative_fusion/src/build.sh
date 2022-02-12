#!/bin/bash


cd ./CHISEL
mkdir build
cd build
cmake ../src
make -j
cd ../../
mkdir build
cd build
cmake ..
make -j

