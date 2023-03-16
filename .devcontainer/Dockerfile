FROM sky360/ros2-humble-tf:1.0.0
#FROM sky360/ros2-humble-cuda-dev:1.0.1
# ** [Optional] Uncomment this section to install additional packages. **

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
   && apt-get -y install --no-install-recommends python3-numpy libboost-python-dev \
   # ROS2 build related stuff
   && pip3 install setuptools==58.2.0 \
   && export PYTHONWARNINGS=ignore:::setuptools.command.install \
   && apt-get autoremove -y \
   && apt-get clean -y
   # && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get -y install vulcanexus-humble-base vulcanexus-humble-base

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f /opt/vulcanexus/humble/setup.bash ]; then source /opt/vulcanexus/humble/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
