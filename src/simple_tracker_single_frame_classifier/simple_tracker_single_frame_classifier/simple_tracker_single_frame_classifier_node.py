# Subscribe to topic: ros2 topic echo /sky360/classification
# TensorFlow version 2.12.0 worked

import traceback as tb
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles
from typing import List
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import ObjectHypothesis, Detection2DArray, Classification
from cv_bridge import CvBridge
from simple_tracker_interfaces.msg import TrackingState, TrackTrajectoryArray
# from simple_tracker_interfaces.msg import ClassifiedObject, ClassifiedObjectsArray
from simple_tracker_shared.configured_node import ConfiguredNode
from simple_tracker_shared.node_runner import NodeRunner
from simple_tracker_shared.qos_profiles import get_topic_publisher_qos_profile, get_topic_subscriber_qos_profile
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from simple_tracker_annotated_frame_provider.annotated_frame_creator import AnnotatedFrameCreator
from tensorflow.keras.preprocessing import image as kimage
from tensorflow.keras.models import load_model
from PIL import Image as ImageROI
from simple_tracker_configuration.app_settings import AppSettings
import cv2
import json  # import the json module
from datetime import datetime, timedelta

class SingleFrameClassifierNode(Node):
    def __init__(self, model_path):
        super().__init__('single_frame_classifier')

         # Get settings from the app settings file
        app_settings = AppSettings.Get(self)

        annotatedFrameCreatorSettings = {
                'visualiser_bbox_line_thickness': app_settings['visualiser_bbox_line_thickness'],
                'visualiser_frame_source': app_settings['visualiser_frame_source'],
                'visualiser_show_cropped_tracks': app_settings['visualiser_show_cropped_tracks'],
                'visualiser_cropped_zoom_factor': app_settings['visualiser_cropped_zoom_factor'],
                'visualiser_bbox_size': app_settings['visualiser_bbox_size'],
        }

        self.creator = AnnotatedFrameCreator(annotatedFrameCreatorSettings)
        self.bridge = CvBridge()

        # Load the Keras model
        self.model = load_model(model_path)

        # Get QoS Profiles
        subscriber_qos_profile = get_topic_subscriber_qos_profile()
        publisher_qos_profile = get_topic_publisher_qos_profile()

        # Add a publisher for the classification results
        self.pub_classification = self.create_publisher(String, 'sky360/classification', publisher_qos_profile)

        self.sub_masked_frame = message_filters.Subscriber(self, Image, 'sky360/frames/masked', qos_profile=subscriber_qos_profile)
        self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'sky360/tracker/tracking_state', qos_profile=subscriber_qos_profile)
        self.sub_tracker_detections = message_filters.Subscriber(self, Detection2DArray, 'sky360/tracker/detections', qos_profile=subscriber_qos_profile)
        self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/trajectory', qos_profile=subscriber_qos_profile)
        self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'sky360/tracker/prediction', qos_profile=subscriber_qos_profile)

        # TimeSynchronizer to ensure all messages have the same timestamp
        self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state,
        self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
        self.time_synchronizer.registerCallback(self.synced_callback)

    def synced_callback(self, msg_masked_frame, msg_tracking_state, msg_detection_array, msg_trajectory_array, msg_prediction_array):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg_masked_frame, "bgr8")

        # Extract the timestamp from the original frame
        frame_timestamp = msg_masked_frame.header.stamp.sec + msg_masked_frame.header.stamp.nanosec * 1e-9

        # Convert the ROS timestamp to a Python datetime object
        frame_datetime = datetime.fromtimestamp(frame_timestamp)

        # Convert the datetime object to a string in ISO 8601 format
        frame_timestamp_str = frame_datetime.isoformat()

        # Start by creating an empty list to hold the data for all detected objects
        all_objects_data = []

        for detection in msg_detection_array.detections:

            # Get the Bounding Box Info for Detected Object
            bbox = self.creator._get_sized_bbox(detection.bbox)

            # Convert the bounding box  center coordinates, width and height to integers
            x, y, w, h = map(int, bbox)

            # This line first crops a region of interest (ROI) from cv_image defined by the rectangle starting at (x, y) with width w and height h.
            # Then, it converts the color space of the cropped image from BGR (Blue, Green, Red) to RGB (Red, Green, Blue).
            # OpenCV reads images in BGR format by default, but many other libraries (including Matplotlib and PIL) use the RGB format.
            img = cv2.cvtColor(cv_image[y:y+h, x:x+w], cv2.COLOR_BGR2RGB)

            # This line converts the NumPy array img into a PIL Image object.
            # PIL stands for Python Imaging Library, which provides the Image class for manipulating images.
            # The fromarray method creates an image memory from an object exporting the array interface.
            img = ImageROI.fromarray(img)

            # This line resizes the image to a fixed size of 64x64 pixels.
            # This is a common operation when preparing images for input to a neural network, which often requires images of a fixed size.
            img = img.resize((64, 64))

            # This line converts the PIL Image object back into a NumPy array.
            # This is done using the img_to_array function from Keras' image module.
            # It's useful to have the image data as a NumPy array for further processing, especially if you're going to feed the image to a machine learning model.
            img_array = kimage.img_to_array(img)

            # This line adds an extra dimension to the beginning of the NumPy array.
            # If img_array had a shape of (64, 64, 3), after this operation it will have a shape of (1, 64, 64, 3).
            # This is typically done when preparing a single image for input to a neural network model, as the model usually expects a batch of images as input (even if the batch only contains one image).
            img_array = np.expand_dims(img_array, axis=0)

            # Classify the region defined by Bounding Box
            predictions = self.model.predict(img_array)

            # Log the classification
            # self.get_logger().info('Model predictions: ' + str(predictions))

            # Now, let's create a dictionary to hold our data
            object_data = {
                'bbox': {
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h
                },
                'classification': predictions.tolist()  # Numpy arrays are not JSON serializable, but Python lists are
            }

            # Add the object data to the list
            all_objects_data.append(object_data)

        # Create a dictionary to hold the entire frame data
        frame_data = {
                'timestamp': frame_timestamp_str,
                'objects': all_objects_data
        }

        # Convert the entire frame data to a JSON string
        frame_data_json = json.dumps(frame_data)

        # Convert the JSON string to a ROS message
        msg_classification = String(data=frame_data_json)

        # Publish the message
        self.pub_classification.publish(msg_classification)

def main(args=None):
    rclpy.init(args=args)
    # Get current working directory
    workspace_root = os.getcwd()
    # Construct the path to your model
    model_path = os.path.join(workspace_root, 'src', 'simple_tracker_single_frame_classifier', 'models', 'cnn-mlp-05.keras')
    node = SingleFrameClassifierNode(model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()