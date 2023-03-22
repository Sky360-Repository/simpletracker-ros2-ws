# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import rclpy.logging
from rclpy.time import Time
from datetime import datetime
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Classification
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray

class RosbagRecorder():

  def __init__(self, settings):
    self.settings = settings
    self.logger = rclpy.logging.get_logger('rosbag_recorder')

    # setup the rosbag so that we can record into it
    db_uri = f'sky360_recordings/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}'
    self.logger.info(f'Creating database {db_uri}.')

    self.writer = rosbag2_py.SequentialWriter()
    
    storage_options = rosbag2_py._storage.StorageOptions(uri=db_uri, storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    self.writer.open(storage_options, converter_options)

    image_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/frames/masked', type='sensor_msgs/msg/Image', serialization_format='cdr')
    tracking_state_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/tracking_state', type='simple_tracker_interfaces/msg/TrackingState', serialization_format='cdr')
    detection_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/detections', type='vision_msgs/msg/Detection2DArray', serialization_format='cdr')
    trajectory_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/trajectory', type='simple_tracker_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')
    prediction_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/tracker/prediction', type='simple_tracker_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')
    classification_topic_info = rosbag2_py._storage.TopicMetadata(name='sky360/classification', type='vision_msgs/msg/Classification', serialization_format='cdr')

    self.writer.create_topic(image_topic_info)
    self.writer.create_topic(detection_topic_info)
    self.writer.create_topic(tracking_state_topic_info)
    self.writer.create_topic(trajectory_topic_info)
    self.writer.create_topic(prediction_topic_info)
    self.writer.create_topic(classification_topic_info)

  def record(self, masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:Detection2DArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray, msg_classification:Classification):
     
    ns = Time.from_msg(masked_frame.header.stamp).nanoseconds
    self.writer.write('sky360/frames/masked', serialize_message(masked_frame), ns)
    self.writer.write('sky360/tracker/tracking_state', serialize_message(msg_tracking_state), ns)
    self.writer.write('sky360/tracker/detections', serialize_message(msg_detection_array), ns)
    self.writer.write('sky360/tracker/trajectory', serialize_message(msg_trajectory_array), ns)
    self.writer.write('sky360/tracker/prediction', serialize_message(msg_prediction_array), ns)
    self.writer.write('sky360/classification', serialize_message(msg_classification), ns)

    status_message = f"(Sky360) Tracker Status: trackable:{msg_tracking_state.trackable}, alive:{msg_tracking_state.alive}, started:{msg_tracking_state.started}, ended:{msg_tracking_state.ended}"
    self.logger.debug(status_message)
