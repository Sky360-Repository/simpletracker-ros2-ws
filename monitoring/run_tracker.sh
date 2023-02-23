#!/bin/bash

docker run -it \
  --user ros:ros \
  --privileged \
  --network=host \
  --cap-add=SYS_PTRACE \
  --security-opt=seccomp:unconfined \
  --security-opt=apparmor:unconfined \
  --device=/dev/video0:/dev/video0 \
  --volume=/dev:/dev \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  -p 8080:8080 \
  -e DISPLAY=$DISPLAY \
  sky360/simpletracker-ros2:1.0.2 \
  bash