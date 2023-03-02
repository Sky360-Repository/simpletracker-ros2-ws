docker compose up

docker network create \
  --driver=bridge \
  --attachable \
  --scope local \
  --subnet=10.0.0.0/16 \
  --ip-range=10.0.0.0/24 \
  --gateway=10.0.0.1 \
  sky360 || true

docker run -it \
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
  -p 8080:8080 \
  -p 8081:8081 \
  -p 8082:8082 \
  -e DISPLAY=$DISPLAY \
  sky360/simpletracker-ros2:1.0.3 \
  bash

https://grafana.com/grafana/dashboards/12486-node-exporter-full/

https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/topic_names.html

docker network connect sky360 sky360-tracker

#### Detach from a running container
Ctrl-p + Ctrl-q

#### Attach to a running container
docker attach sky360-tracker


https://www.ross-robotics.co.uk/news/w79zhjoey8k0univkzvr1qyqorgqbf
https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
https://github.com/foxglove/tutorials/blob/main/ros2/rosbridge/index.html