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

import numpy as np

from ros2_tf_core import img_conversion as img_utils
from ros2_tf_core import models as models_utils
from ros2_tf_core.tensorflow_node import TensorflowNode
from tf_interfaces.srv import ImageClassification as ImageClassificationSrv
from vision_msgs.msg import ObjectHypothesis
from tf_classification_py.models import IMAGENET_INCEPTION # This will need to change to our model

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2DArray

from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.enumerations import TrackingStateEnum
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile

from simple_tracker_shared.utils import decode_bbox_msg

class ClassificationNode(TensorflowNode):

    def __init__(self, tf_model, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('classification_provider')

        self.br = CvBridge()  

        # ROS parameters
        self.num_predictions_p = self.declare_parameter('num_predictions', 5)

        # Prepare the Tensorflow network
        self.startup(tf_model)

        # Advertise info about the Tensorflow network
        self.publish_vision_info(tf_model)

        self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
        self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)

        # setup the time synchronizer and register the subscriptions and callback
        self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracker_detections], 10)
        self.time_synchronizer.registerCallback(self.synced_callback)

        self.counter = 0

        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def synced_callback(self, msg_masked_frame:Image, msg_detection_array:Detection2DArray):

        if msg_masked_frame is not None and msg_detection_array is not None:

            self.counter += 1

            try:

                masked_frame = self.br.imgmsg_to_cv2(msg_masked_frame)
                #(sh, sw) = masked_frame.shape[:2]

                #n_channels = 3
                #dtype = 'uint8'
                #img_buf = np.asarray(msg_masked_frame.data, dtype=dtype)
                #image_np = np.ndarray(shape=(sh, sw, n_channels), dtype=dtype, buffer=img_buf)

                for detection in msg_detection_array.detections:

                    id_arr = detection.id.split("-")

                    id = id_arr[0]
                    tracking_state = TrackingStateEnum(int(id_arr[1]))

                    if tracking_state == TrackingStateEnum.ActiveTarget:
                        (x, y, w, h) = decode_bbox_msg(detection.bbox)
                        fixed_size = 64

                        if w <= fixed_size and h <= fixed_size:

                            (x1, y1, w1, h1) = (int(x+(w/2)) - int(fixed_size/2), int(y+(h/2)) - int(fixed_size/2), fixed_size, fixed_size)
                            # we need to classify this target to determine if we should ignore it
                            #self.get_logger().info(f"Found detection that needs classification --> id:{id}, x:{x1}, y:{y1}, w:{w1} h:{h1}.")

                            #(h, w) = masked_frame.shape[:2]

                            roi = np.ascontiguousarray(masked_frame[y1:y1+h1, x1:x1+w1], dtype=np.uint8)
                            #output_dict = self.classify(roi)
                            #roi = image_np[y1:y1+h1, x1:x1+w1]
                            #self.get_logger().info(f"roi.flags: {roi.flags}")

                            #self.get_logger().info(f"roi shape: {roi.shape}")

                            img_buf = np.asarray(roi, dtype=np.uint8)
                            #self.get_logger().info(f"img_buf.flags: {img_buf.flags}")
                            image_np = np.ndarray(shape=(64, 64, 3), dtype=np.uint8, buffer=img_buf)
                            #self.get_logger().info(f"image_np.flags: {image_np.flags}")

                            output_dict = self.classify(image_np)

                            classes = output_dict['classification_classes']
                            scores = output_dict['classification_scores']

                            for i in range(len(classes)):
                                self.get_logger().info(f"{self.counter} Class found --> id : {str(classes[i].item())}, score : {scores[i].item()}.")                        

                #classification.header.stamp = self.get_clock().now().to_msg()
                #classification.results = []
                #for i in range(len(classes)):
                #    hypotesis = ObjectHypothesis()
                #    hypotesis.class_id = str(classes[i].item())
                #    hypotesis.score = scores[i].item()
                #    classification.results.append(hypotesis)

            except Exception as e:
                self.get_logger().error(f"Exception during object classification. Error: {e}.")
                self.get_logger().error(tb.format_exc())

    def startup(self, tf_model):

        if tf_model.save_load_format != models_utils.SaveLoadFormat.FROZEN_MODEL:
            raise ValueError('Classification node currently supports only FROZEN MODELS')

        # Load model
        model_path = tf_model.compute_model_path()
        self.graph, self.session = models_utils.load_frozen_model(model_path)
        self.get_logger().info('Load model completed!')

        # Define input tensor
        self.input_image_tensor = self.graph.get_tensor_by_name('DecodeJpeg:0')

        # Define output tensor
        self.output_softmax_tensor = self.graph.get_tensor_by_name('softmax:0')

        self.warmup()

    def classify(self, image_np):

        start_time = self.get_clock().now()

        scores = self.session.run(self.output_softmax_tensor, feed_dict={self.input_image_tensor: image_np})

        elapsed_time = self.get_clock().now() - start_time
        elapsed_time_ms = elapsed_time.nanoseconds / 1000000
        self.get_logger().debug(f'Image classification took: {elapsed_time_ms} milliseconds')

        # Get top indices from softmax
        scores = np.squeeze(scores)
        top_classes = scores.argsort()[-self.num_predictions_p.value:][::-1]

        output_dict = {}
        output_dict['classification_classes'] = top_classes
        output_dict['classification_scores'] = [scores[i] for i in top_classes]

        return output_dict

    def warmup(self):

        image_np = np.uint8(np.random.randint(0, 255, size=(480, 640, 3)))

        self.classify(image_np)

        self.get_logger().info('Warmup completed! Ready to receive real images!')

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = get_topic_subscriber_qos_profile()
  publisher_qos_profile = get_topic_publisher_qos_profile()

  node = ClassificationNode(IMAGENET_INCEPTION, subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()