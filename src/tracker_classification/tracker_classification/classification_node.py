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

import traceback as tb
import rclpy
import message_filters
from rclpy.qos import QoSProfile, QoSPresetProfiles
from typing import List
import os

import numpy as np
import tensorflow as tf

from cv_bridge import CvBridge
from vision_msgs.msg import ObjectHypothesis, Detection2DArray, Classification
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory

from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.enumerations import TrackingStateEnum
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from simple_tracker_shared.utils import decode_bbox_msg

class ClassificationNode(ConfiguredNode):

    def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('classification_provider')

        self.pub_classification = self.create_publisher(Classification, 'sky360/classification', publisher_qos_profile)

        self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
        self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)

        # setup the time synchronizer and register the subscriptions and callback
        self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracker_detections], 10)
        self.time_synchronizer.registerCallback(self.synced_callback)

        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def synced_callback(self, msg_masked_frame:Image, msg_detection_array:Detection2DArray):

        if msg_masked_frame is not None and msg_detection_array is not None:

            fixed_size = 64

            try:
                masked_frame = self.br.imgmsg_to_cv2(msg_masked_frame)

                predictions = []
                for detection in msg_detection_array.detections:

                    id_arr = detection.id.split("-")

                    id = id_arr[0]
                    tracking_state = TrackingStateEnum(int(id_arr[1]))

                    if tracking_state == TrackingStateEnum.ActiveTarget:
                        (x, y, w, h) = decode_bbox_msg(detection.bbox)

                        if w <= fixed_size and h <= fixed_size:

                            (x1, y1, w1, h1) = (int(x+(w/2)) - int(fixed_size/2), int(y+(h/2)) - int(fixed_size/2), fixed_size, fixed_size)

                            roi = np.ascontiguousarray(masked_frame[y1:y1+h1, x1:x1+w1], dtype=np.uint8)

                            #start time
                            #start_time = self.get_clock().now()

                            predictions.append(self.classify(roi))

                            #end time
                            #elapsed_time = self.get_clock().now() - start_time
                            #elapsed_time_ms = elapsed_time.nanoseconds / 1000000
                            #self.get_logger().debug(f'Classification took: {elapsed_time_ms} milliseconds')


                classification_msg = Classification()
                classification_msg.header = msg_masked_frame.header
                classification_msg.results = []

                for i in range(len(predictions)):
                    hypotesis = ObjectHypothesis()
                    hypotesis.class_id = f'{id}-{str(np.argmax(predictions[i]))}'
                    hypotesis.score = float(np.max(predictions[i]))
                    classification_msg.results.append(hypotesis)

                self.pub_classification.publish(classification_msg)

            except Exception as e:
                self.get_logger().error(f'Exception during object classification. Error: {e}.')
                self.get_logger().error(tb.format_exc())

    def classify(self, image_np):

        predictions = []
        #convert the image to array
        img_array = tf.keras.preprocessing.image.img_to_array(image_np)
        #add a dimension to the array
        img_array = tf.expand_dims(img_array, 0)
        #predict the image
        predictions.append(self.model.predict(img_array))

        return predictions

    def warmup(self):

        image_np = np.uint8(np.random.randint(0, 255, size=(64, 64, 3)))

        self.classify(image_np)

        self.get_logger().info('Warmup completed! Ready to receive real images!')

    def config_list(self) -> List[str]:
        return ['classification_model']

    def validate_config(self) -> bool:
        valid = True

        if self.app_configuration['classification_model'] == None:
           self.get_logger().error('No classification_model defined')
           valid = False
        else:
           model = self.app_configuration['classification_model']
           self.model_path = os.path.join(get_package_share_directory('tracker_classification'), 'models', model)
           valid = os.path.isfile(self.model_path)

        return valid

    def on_config_loaded(self, init: bool):

        if init:
            self.br = CvBridge()
            
            self.model = tf.keras.models.load_model(self.model_path)
            self.warmup()

        self.counter = 0

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = ClassificationNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()