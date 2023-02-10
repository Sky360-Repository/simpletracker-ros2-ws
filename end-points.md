## Please Note: 
These end points are very much work in progress and have not been finalised yet. Therefore they are subject to change at any point in time. Before they are finalised, they will be versioned so as to not interfere with downstream developments. The versioning approach is again, very much still in the process of being 

## Configuration

### sky360/config/entries - ConfigEntryArray.srv
Service end point for retrieval of an array of config entries

### sky360/config/entry/update - ConfigEntryUpdate.srv
Service end point for the update of a single config entry

### sky360/config/updated - ConfigEntryUpdateArray.msg
Message end point to provide a notification event that the config entry has been updated.

## Camera

### sky360/camera/original - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the original frame received from OpenCV

## Frames

### sky360/frames/original - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the original frame after resizing

### sky360/frames/masked - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the original frame after processing i.e. resize, masked etc

### sky360/frames/grey - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the grey frame after processing i.e. resize, blurred, masked etc

### sky360/frames/dense_optical_flow - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing the dense optical flow frame

### sky360/frames/foreground_mask - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the foreground masked frame

### sky360/frames/masked_background - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing of the masked background frame

### sky360/frames/annotated - [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
Message end point for the publishing the annotated frame i.e. the frame with status and tracks etc

## Detection

### sky360/detector/bgs/bounding_boxes - [vision_msgs/BoundingBox2DArray](https://github.com/ros-perception/vision_msgs)
Message end point for the publishing of bounding box array for a frame

### sky360/detector/canny/bounding_boxes - [vision_msgs/BoundingBox2DArray](https://github.com/ros-perception/vision_msgs)
Message end point for the publishing of bounding box array for a frame

## Tracker

### sky360/tracker/detections - [vision_msgs/Detection2DArray](https://github.com/ros-perception/vision_msgs)
Message end point for the publishing of detections

### sky360/tracker/trajectory - TrackTrajectoryArray.msg
Message end point for the publishing of detection trajectories

### sky360/tracker/prediction - TrackTrajectoryArray.msg
Message end point for the publishing of predicted trajectories

### sky360/tracker/tracking_state - TrackingState.msg
Message end point for the publishing whether the tracker is tracking

## Observer

### sky360/observer/day_night_classifier - ObserverDayNight.msg
Message end point for the publishing of a day night classifier result

### sky360/observer/cloud_estimation - ObserverCloudEstimation.msg
Message end point for the publishing of a cloud cover estimation result
