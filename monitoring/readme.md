## To get the monitoring stuff up and running, do the following

You will need to have docker compose installed and working

In the first terminal window navigate to this monitoring folder and then the command to stat the containers is:
```bash
docker compose up
```
If you would like to exit the docker compose application type **Ctrl-c**


In the second terminal window, launch the sky360/simpletracker-ros2:1.0.3 docker container interactively
```bash
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
```
In the interactive terminal run the follow commands in sequence
```bash
./setup.sh
./build.sh
./launch_monitoring.sh
```

If you would like to exit the application type **Ctrl-c** The ROS2Prometheus application does not exit smoothly so please open System Monitor (Ubuntu) and kill the ROS2Prometheus process as it opens and occupies the 8080 port.

ROS2Prometheus is a sample application to export DDS metrics to Prometheus. More info regarding this application can be found [here](https://docs.vulcanexus.org/en/latest/rst/tutorials/tools/prometheus/prometheus.html). Here is the full list of [DDS Statistics Topic names](https://fast-dds.docs.eprosima.com/en/latest/fastdds/statistics/dds_layer/topic_names.html)

#### Dashboards

To verify that Prometheus is running open the [Prometheus Targets](http://localhost:9090/targets) page. You should see 4 targets and they should all be up.

In [Grafana](http://localhost:3000) you will need to install the [Node Exporter Full](https://grafana.com/grafana/dashboards/12486-node-exporter-full/) dashboard.

Currently suggested dashboards are:
- [Node Exporter Full](https://grafana.com/grafana/dashboards/12486-node-exporter-full/)
- More to follow

Sky360 Prometheus Metrics End Points are:
- [Sky360-DDS](http://localhost:8080/metrics)
- [sky360-tracker](http://localhost:8082/metrics)

## This section is here for reference, please ignore

docker network create \
  --driver=bridge \
  --attachable \
  --scope local \
  --subnet=10.0.0.0/16 \
  --ip-range=10.0.0.0/24 \
  --gateway=10.0.0.1 \
  sky360 || true

docker network connect sky360 sky360-tracker

#### Detach from a running container
Ctrl-p + Ctrl-q

#### Attach to a running container
docker attach sky360-tracker
