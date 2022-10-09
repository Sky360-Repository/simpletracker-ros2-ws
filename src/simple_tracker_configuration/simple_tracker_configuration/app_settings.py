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

from rclpy.node import Node, Parameter

##################################################################################################
# This class provides a central area for populating and validating the application configuration #
##################################################################################################
class AppSettings():

    # Method to populate a configuration dictionary for use throughout the simple tracker application
    @staticmethod
    def Get(node: Node):

        # https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/

        node.declare_parameters(
            namespace='',
            parameters=[
                ('controller_type', 'camera'),
                ('camera_video_file', 'plane_flying_past2.mkv'),
                ('tracker_type', 'CSRT'),
                ('background_subtractor_type', 'KNN'),
                ('mask_type', 'overlay_inverse'),
                ('mask_pct', 10),
                ('mask_overlay_image_file_name', 'mask-shrubs-inverse-overlay.jpg'),
                ('visualiser_frame_source', 'original')
            ]
        )

        app_settings = {}

        # Controller section
        app_settings['controller_type'] = node.get_parameter('controller_type').value

        # Camera node section
        app_settings['camera_mode'] = 'rtsp'
        app_settings['camera_uri'] = 'rtsp://sky360:Sky360Sky!@192.168.0.43:554/cam/realmonitor?channel=1&subtype=2'
        app_settings['camera_resize_frame'] = True
        app_settings['camera_resize_dimension_h'] = 960
        app_settings['camera_resize_dimension_w'] = None
        app_settings['camera_cuda_enable'] = False
        #app_settings['camera_video_file'] = 'plane_flying_past.mkv'        
        app_settings['camera_video_file'] = node.get_parameter('camera_video_file').value
        app_settings['camera_video_loop'] = True

        # Frame Provider node section
        app_settings['frame_provider_resize_frame'] = True
        app_settings['frame_provider_resize_dimension_h'] = 960
        #app_settings['frame_provider_resize_dimension_h'] = 400
        app_settings['frame_provider_resize_dimension_w'] = None
        app_settings['frame_provider_blur'] = True
        app_settings['frame_provider_blur_radius'] = 3
        app_settings['frame_provider_cuda_enable'] = False

        # Visualiser node section
        app_settings['visualiser_font_size'] = 0.5
        app_settings['visualiser_font_thickness'] = 1
        app_settings['visualiser_bbox_line_thickness'] = 1
        app_settings['visualiser_bbox_size'] = 64
        app_settings['visualiser_log_status_to_console'] = False
        app_settings['visualiser_cuda_enable'] = False
        app_settings['visualiser_frame_source'] = node.get_parameter('visualiser_frame_source').value

        # Video Tracker section
        app_settings['tracker_type'] = node.get_parameter('tracker_type').value
        app_settings['tracker_stopwatch_enable'] = False
        app_settings['tracker_active_only'] = True
        app_settings['tracker_detection_mode'] = 'background_subtraction'
        app_settings['tracker_detection_sensitivity'] = 1
        app_settings['tracker_max_active_trackers'] = 10        
        app_settings['tracker_wait_seconds_threshold'] = 0
        app_settings['tracker_cuda_enable'] = False        

        # Background subtractor section
        app_settings['background_subtractor_type'] = node.get_parameter('background_subtractor_type').value
        app_settings['background_subtractor_learning_rate'] = 0.05
        app_settings['background_subtractor_cuda_enable'] = False

        # Track Plotting section
        app_settings['track_path_plotting_enabled'] = True
        app_settings['track_plotting_type'] = 'line'
        app_settings['track_validation_enable'] = True
        app_settings['track_stationary_threshold'] = 5
        app_settings['track_orphaned_threshold'] = 20
        app_settings['track_prediction_enabled'] = True
        app_settings['track_cuda_enable'] = False

        # Mask section
        #app_settings['mask_type'] = 'fish_eye'
        #app_settings['mask_type'] = 'no_op'
        app_settings['mask_type'] = node.get_parameter('mask_type').value
        #app_settings['mask_type'] = 'overlay'
        app_settings['mask_pct'] = node.get_parameter('mask_pct').value
        #app_settings['mask_overlay_image_file_name'] = 'mikes-camera-mask-overlay.jpg'
        app_settings['mask_overlay_image_file_name'] = node.get_parameter('mask_overlay_image_file_name').value
        app_settings['mask_overlay_image'] = None
        app_settings['mask_cuda_enable'] = False

        # Dense optical flow
        app_settings['dense_optical_cuda_enable'] = False
        app_settings['dense_optical_flow_h'] = 400
        app_settings['dense_optical_flow_w'] = 480

        # MOT_STF section
        app_settings['motstf_write_original'] = True
        app_settings['motstf_write_annotated'] = True
        app_settings['motstf_write_images'] = False

        return app_settings