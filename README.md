### The command below does not need to be run again, this was used by me to setup the project.
ros2 pkg create --build-type ament_python cv_basics --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2

# The command below should be used to get this ROS package in the container environment

### The following 2 ROS dependency commands should be included in the Dockerfile so that they don't have to be run manually, but I don't know hwo to do that yet

Update ROS dependency repositories `rosdep update`
Install ROS dependencies `rosdep install -i --from-path src --rosdistro humble -y`

**NOTE:** Update line 37 of the webcam_pub.py file to supply either:
* a camera index i.e. 0
* or a uri e.g.: 'rtsp://[user]:[<password]@192.168.0.43:554/cam/realmonitor?channel=1&subtype=0'
* 

Build the ROS package `colcon build`
Source the built package `source install/setup.bash`
Run both the image publisher and the image subscriber `ros2 run cv_basics img_publisher` and `ros2 run cv_basics img_subscriber`
