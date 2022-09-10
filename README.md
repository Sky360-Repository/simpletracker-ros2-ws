#### The command below does not need to be run again, this was used by me to setup the project.
`ros2 pkg create --build-type ament_python cv_basics --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs python3-opencv`

### The commands below should be used to get this ROS package up and running in VSCode and using the ROS dev container

#### The following 2 ROS dependency commands should be included in the Dockerfile so that they don't have to be run manually, but I don't know how to do that yet

**NOTE:** Run all the following commands using the terminal in VSCode and from the **simpletracker-ros2-ws** folder. This terminal should be in the context of the container so should execute commands in the **development container** and not the local machine.

## If you would like to, just run the `setup.bash` file and skip all the gubbins below. Only thing to do would be to "Update web cam details", see below

### Gubbins
* Add the ROS2 apt repository to your system. `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`
* Then add the repository to your sources list. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
* Update your apt repository caches after setting up the repositories. `sudo apt update`
* Update ROS dependency repositories `rosdep update`
* Install ROS dependencies `rosdep install -i --from-path src --rosdistro humble -y`
* Build the ROS package `colcon build`
* Source the built package `source install/setup.bash`
* Run both the image publisher and the image subscriber:
  * `ros2 run cv_basics img_publisher` 
  * `ros2 run cv_basics img_subscriber`

### Update web cam details
**NOTE:** Update line 37 of the webcam_pub.py file in the cv_basics package to supply either:
**NOTE:** Update line 27 of the app_settings.py file in the simple_tracker_configuration package to supply either:
* a camera index i.e. 0
* or a uri e.g.: 'rtsp://[user]:[password]@192.168.0.43:554/cam/realmonitor?channel=1&subtype=0'

### I have made some more progress with regards to ROS2, follow steps above to ensure everything is working

* You can now run some additional packages
  * `ros2 run simple_tracker_configuration configuration_service` 
  * `ros2 run simple_tracker frame_publisher`
  * `ros2 run simple_tracker_visualisers resized_frame_visualiser`
  * `ros2 run simple_tracker_visualisers full_frame_visualiser`

Alternatively you can run Simple Tracker using the launch facility
* `ros2 launch simple_tracker_launch simple_tracker_launch.py` 
