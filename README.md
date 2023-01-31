## SimpleTracker-ros2-ws

### The commands below should be used to get this ROS package up and running in VSCode and using the ROS dev container

#### Setup 
Run the setup script to setup dev container
`./setup.sh`

#### Update web cam details

**NOTE:** Update the params.yaml file in the sime_tracker_launch/config directory to change camera details

#### Build

Run the build script to build the project
`./build.sh`

#### Launch

Run the launch script to launch the project. This will source the setup.bash file and launch the application
`./launch.sh`

Run the simulation launch script to launch the project in simulation mode. This will source the setup.bash file and launch the application
`./launch_simulation.sh`

#### Note: If you run into the following error:

`[frame_provider-4] [INFO] [1675115712.958444773] [frame_provider]: frame_provider node is up and running.Simple visualiser-1 at.apa.xcb: could not connect to display 1 [simple visualiser-101 at.apa.plugin: Could not load the Ot platform plugin "xcb" in "" even though it was found`

Then run the folllowing commands:

1. `export DISPLAY=:0`
2. `xhost +`

##### Replay a rosbag recording

* `ros2 bag play 2023_01_25-16_48_14/` where 2023_01_25-16_48_14/ is a folder that contains a rosbag recording
* `./launch_rosbag_replay.sh`

### Updating configuration from the Visualiser window

* When the visualiser is up and running, a number of key presses are currently handled as to allow you to change the configuration dynamically:
  * **+** This will increase the size of the image
  * **-** This will decrease the size of the image
  * **m** This will switch masks
  * **c** This will switch controllers, however you need to have configured your camera details as per above for this to work

### Updating configuration from command line example:

* This will update the "frame_provider_resize_dimension_h" configuration entry to a value of 400

`ros2 service call '/sky360/config/entry/update' 'simple_tracker_interfaces/srv/ConfigEntryUpdate' '{entries:[{key: frame_provider_resize_dimension_h, type: int, value: 400}]}'`

`ros2 service call '/sky360/config/entry/update' 'simple_tracker_interfaces/srv/ConfigEntryUpdate' '{entries:[{key: controller_type, type: str, value: video}]}'`

### Tracker status via command line

* To keep an eye on the state of the tracker, using the following echo command

`ros2 topic echo /sky360/tracker/tracking_state`

### View more windows

* Uncomment some of the remappings in the "simple_tracker_launch/launch/simple_tracker_launch.py" visualiser node to enable additional windows like optical flow etc
