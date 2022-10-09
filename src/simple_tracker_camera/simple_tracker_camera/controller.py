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

import cv2
import os
from simple_tracker_shared.configured_node import ConfiguredNode
from ament_index_python.packages import get_package_share_directory
from .camera import get_camera

######################################################################################################################################
# Base class for various controller implementations. The idea here is that we have a controller that drives a camera or video replay #
# process as they are likely to be slightly different.                                                                               #
# This is the main entry point into the object tracking and frame processing logic.                                                  #
######################################################################################################################################
class Controller():

    def __init__(self, node: ConfiguredNode, app_configuration: dict):
        self.node = node
        self.app_configuration = app_configuration
        self.capture = None
        pass

    def read(self):
        return self.capture.read()

    # Static factory select method to determine what masking implementation to use
    @staticmethod
    def Select(node: ConfiguredNode, app_configuration: dict):

        controller_type = app_configuration['controller_type']

        if controller_type == 'camera':
            return Controller.Camera(node, app_configuration)

        if controller_type == 'video':
            return Controller.Video(node, app_configuration)

    @staticmethod
    def Camera(node: ConfiguredNode, app_configuration: dict):
        return CameraController(node, app_configuration)

    @staticmethod
    def Video(node: ConfiguredNode, app_configuration: dict):
        return VideoController(node, app_configuration)


##########################################################################################################################
# Specialised implementation of the controller class for consuming and dealing with the video inpuit from a live camera. #
##########################################################################################################################
class CameraController(Controller):

    def __init__(self, node: ConfiguredNode, app_configuration: dict):
        super().__init__(node, app_configuration)

        self.capture = get_camera(app_configuration)

########################################################################################################################
# Specialised implementation of the controller class for consuming and daling with the video inpuit from a video file. #
########################################################################################################################
class VideoController(Controller):

    def __init__(self, node: ConfiguredNode, app_configuration: dict):
        super().__init__(node, app_configuration)

        self.videos_folder = os.path.join(get_package_share_directory('simple_tracker_camera'), 'videos')
        self.video_file_path = os.path.join(self.videos_folder, app_configuration['camera_video_file'])
        self.capture = cv2.VideoCapture(self.video_file_path)
        if not self.capture.isOpened():
            self.node.get_logger().error(f'Could not open video {self.video_file_path}.')
