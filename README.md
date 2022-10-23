#### The command below does not need to be run again, this was used by me to setup the project.
`ros2 pkg create --build-type ament_python cv_basics --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs python3-opencv`

### The commands below should be used to get this ROS package up and running in VSCode and using the ROS dev container

#### The following 2 ROS dependency commands should be included in the Dockerfile so that they don't have to be run manually, but I don't know how to do that yet

**NOTE:** Run all the following commands using the terminal in VSCode and from the **simpletracker-ros2-ws** folder. This terminal should be in the context of the container so should execute commands in the **development container** and not the local machine.

## If you would like to, just run the `setup.sh` file and skip all the gubbins below. Only thing to do would be to "Update web cam details", see below

### Gubbins
* Add the ROS2 apt repository to your system. `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`
* Then add the repository to your sources list. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
* Update your apt repository caches after setting up the repositories. `sudo apt update`
* Update ROS dependency repositories `rosdep update`
* Install ROS dependencies `rosdep install -i --from-path src --rosdistro humble -y`
* Build the ROS package `colcon build`
* Source the built package `source install/setup.bash`
* Launch the tracker:
  * `ros2 launch simple_tracker_launch simple_tracker_launch.py` 

### Update web cam details

**NOTE:** Update the params.yaml file in the sime_tracker_launch/config directory to change camera details

### I have made some more progress with regards to ROS2, follow steps above to ensure everything is working

* You can run Simple Tracker by running the build file
  * `./build.sh` 

### Updating configuration from the Visualiser window

* When the visualiser is up and running, a number of key presses are currently handled as to allow you to change the configuration dynamically:
  * **+** This will increase the size of the image
  * **-** This will decrease the size of the image
  * **m** This will switch masks
  * **c** This will switch controllers, however you need to have configured your camera details as per above for this to work

### Updating configuration from command line example:

* This will update the "frame_provider_resize_dimension_h" configuration entry to a value of 400

`ros2 service call '/sky360/config/entry/update/v1' 'simple_tracker_interfaces/srv/ConfigEntryUpdate' '{entries:[{key: frame_provider_resize_dimension_h, type: int, value: 400}]}'`

`ros2 service call '/sky360/config/entry/update/v1' 'simple_tracker_interfaces/srv/ConfigEntryUpdate' '{entries:[{key: controller_type, type: str, value: video}]}'`

### Tracker status via command line

* To keep an eye on the state of the tracker, using the following echo command

`ros2 topic echo /sky360/tracker/tracking_state/v1`

### View more windows

* Uncomment some of the remappings in the "simple_tracker_launch/launch/simple_tracker_launch.py" visualiser node to enable additional windows like optical flow etc
