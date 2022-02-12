#!/bin/bash

# Expose the X server on the host.
sudo xhost +local:root

docker run \
  --rm \
  -it \
  --gpus all \
  --ipc=host \
  -w /workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v <path_to_data>:/data \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  luvisionsigma/buildingfusion:dev \
  bash run.sh

  # -v /home/zheng/Desktop/opensourcing/data/1203-1206:/data \
  