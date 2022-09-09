#### The command below does not need to be run again, this was used by me to setup the project.
`ros2 pkg create --build-type ament_python cv_basics --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs python3-opencv`

### The commands below should be used to get this ROS package up and running in VSCode and using the ROS dev container

#### The following 2 ROS dependency commands should be included in the Dockerfile so that they don't have to be run manually, but I don't know how to do that yet

**NOTE:** Run all the following commands using the terminal in VSCode and from the **simpletracker-ros2-ws** folder. This terminal should be in the context of the container so should execute commands in the **development container** and not the local machine.

* Update ROS dependency repositories `rosdep update`
* Install ROS dependencies `rosdep install -i --from-path src --rosdistro humble -y`
* Build the ROS package `colcon build`
* Source the built package `source install/setup.bash`
* Run both the image publisher and the image subscriber:
  * `ros2 run cv_basics img_publisher` 
  * `ros2 run cv_basics img_subscriber`

**NOTE:** Update line 37 of the webcam_pub.py file in the cv_basics package to supply either:
**NOTE:** Update line 27 of the app_settings.py file in the simple_tracker_configuration package to supply either:
* a camera index i.e. 0
* or a uri e.g.: 'rtsp://[user]:[password]@192.168.0.43:554/cam/realmonitor?channel=1&subtype=0'

### I have made some more progress with regards to ROS2, follow steps above to ensure everything is working

* You can now run some additional packages
  * `ros2 run simple_tracker_configuration configuration_service` 
  * `ros2 run simple_tracker resized_frame_publisher`
  * `ros2 run simple_tracker_visualisers resized_frame_visualiser`
  * `ros2 run simple_tracker full_frame_publisher`
  * `ros2 run simple_tracker_visualisers full_frame_visualiser`