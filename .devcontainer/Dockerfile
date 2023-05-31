FROM sky360/ros2-humble-dev:latest
#FROM sky360/ros2-humble-cuda-dev:1.0.1
# ** [Optional] Uncomment this section to install additional packages. **

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-numpy libboost-python-dev \
   # Install colcon build tools
   && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
   # ROS2 build related stuff
   && pip3 install setuptools==58.2.0 \
   && export PYTHONWARNINGS=ignore:::setuptools.command.install \
   && apt-get autoremove -y \
   && apt-get clean -y
   # && rm -rf /var/lib/apt/lists/*

RUN pip3 install tensorflow

# RUN apt-get update && apt-get -y install vulcanexus-humble-base vulcanexus-humble-base

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f /opt/vulcanexus/humble/setup.bash ]; then source /opt/vulcanexus/humble/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# ARG USERNAME=ros
# RUN chown -R $USERNAME:$USERNAME /opt/Fast-DDS-statistics-backend