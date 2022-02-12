#!/bin/bash

# Expose the X server on the host.
sudo xhost +local:root# --rm: Make the container ephemeral (delete on exit).

docker run \
  --rm \
  -it \
  --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  luvisionsigma/buildingfusion:dev \
  -v <path_to_data>:/data \
  -w /workspace \
  bash run.sh