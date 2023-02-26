#!/bin/bash

#docker container rm /sky360-tracker

#docker network rm sky360

#docker network create \
#  --driver=bridge \
#  --attachable \
#  --scope local \
#  --subnet=10.0.43.0/24 \
#  --ip-range=10.0.43.128/25 \
#  sky360

#--network sky360 \

docker run -it \
  --rm \
  --user ros:ros \
  --privileged \
  --name sky360-tracker \
  --network=host \
  --cap-add=SYS_PTRACE \
  --security-opt=seccomp:unconfined \
  --security-opt=apparmor:unconfined \
  --device=/dev/video0:/dev/video0 \
  --volume=/dev:/dev \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  -p 8000:8000 \
  -p 8080:8080 \
  -p 8081:8081 \  
  -e DISPLAY=$DISPLAY \
  sky360/simpletracker-ros2:1.0.2 \
  bash
