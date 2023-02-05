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
                ('camera_mode', 'rtsp'),
                ('camera_uri', 'rtsp://sky360:Sky360Sky!@192.168.0.43:554/cam/realmonitor?channel=1&subtype=2'),
                ('camera_video_file', 'plane_flying_past2.mkv'),
                ('camera_video_loop', True),

                ('frame_provider_resize_frame', True),
                ('frame_provider_resize_dimension_h', 960),
                ('frame_provider_resize_dimension_w', 960),
                ('frame_provider_blur', True),
                ('frame_provider_blur_radius', 3),
                ('frame_provider_cuda_enable', False),

                ('visualiser_bbox_line_thickness', 1),
                ('visualiser_bbox_size', 64),
                ('visualiser_log_status_to_console', False),
                ('visualiser_frame_source', 'original'),
                ('visualiser_resize_frame', True),
                ('visualiser_resize_dimension_h', 960),
                ('visualiser_resize_dimension_w', 960),
                ('visualiser_show_cropped_tracks', True),
                ('visualiser_cropped_zoom_factor', 2),

                ('tracker_type', 'CSRT'),
                ('tracker_stopwatch_enable', False),
                ('tracker_active_only', True),
                ('tracker_detection_mode', 'background_subtraction'),
                ('tracker_detection_sensitivity', 1),
                ('tracker_max_active_trackers', 10),
                ('tracker_min_centre_point_distance_between_bboxes', 64),

                ('background_subtractor_type', 'KNN'),
                ('background_subtractor_sensitivity', 1),
                ('background_subtractor_learning_rate', 0.05),
                ('background_subtractor_cuda_enable', False),

                ('blob_detector_type', 'sky360'),
                ('blob_detector_min_distance_between_blobs', 64),

                ('track_path_plotting_enabled', True),
                ('track_plotting_type', 'line'),
                ('track_validation_enable', True),
                ('track_stationary_threshold', 5),
                ('track_orphaned_threshold', 20),
                ('track_prediction_enabled', True),

                ('mask_type', 'overlay_inverse'),
                ('mask_pct', 10),
                ('mask_overlay_image_file_name', 'mask-shrubs-inverse-overlay.jpg'),
                
                ('dense_optical_flow_h', 400),
                ('dense_optical_flow_w', 480),
                ('dense_optical_cuda_enable', False),

                ('observer_timer_interval', 30),
                ('observer_day_night_brightness_threshold', 95)
            ]
        )

        app_settings = {}

        # Controller section
        app_settings['controller_type'] = node.get_parameter('controller_type').value

        # Camera node section
        app_settings['camera_mode'] = node.get_parameter('camera_mode').value
        app_settings['camera_uri'] = node.get_parameter('camera_uri').value
        app_settings['camera_video_file'] = node.get_parameter('camera_video_file').value
        app_settings['camera_video_loop'] = node.get_parameter('camera_video_loop').value

        # Frame Provider node section
        app_settings['frame_provider_resize_frame'] = node.get_parameter('frame_provider_resize_frame').value
        app_settings['frame_provider_resize_dimension_h'] = node.get_parameter('frame_provider_resize_dimension_h').value
        app_settings['frame_provider_resize_dimension_w'] = node.get_parameter('frame_provider_resize_dimension_w').value
        app_settings['frame_provider_blur'] = node.get_parameter('frame_provider_blur').value
        app_settings['frame_provider_blur_radius'] = node.get_parameter('frame_provider_blur_radius').value
        app_settings['frame_provider_cuda_enable'] = node.get_parameter('frame_provider_cuda_enable').value

        # Visualiser node section
        app_settings['visualiser_bbox_line_thickness'] = node.get_parameter('visualiser_bbox_line_thickness').value
        app_settings['visualiser_bbox_size'] = node.get_parameter('visualiser_bbox_size').value
        app_settings['visualiser_log_status_to_console'] = node.get_parameter('visualiser_log_status_to_console').value
        app_settings['visualiser_frame_source'] = node.get_parameter('visualiser_frame_source').value
        app_settings['visualiser_resize_frame'] = node.get_parameter('visualiser_resize_frame').value
        app_settings['visualiser_resize_dimension_h'] = node.get_parameter('visualiser_resize_dimension_h').value
        app_settings['visualiser_resize_dimension_w'] = node.get_parameter('visualiser_resize_dimension_w').value
        app_settings['visualiser_show_cropped_tracks'] = node.get_parameter('visualiser_show_cropped_tracks').value
        app_settings['visualiser_cropped_zoom_factor'] = node.get_parameter('visualiser_cropped_zoom_factor').value

        # Video Tracker section
        app_settings['tracker_type'] = node.get_parameter('tracker_type').value
        app_settings['tracker_stopwatch_enable'] = node.get_parameter('tracker_stopwatch_enable').value
        app_settings['tracker_active_only'] = node.get_parameter('tracker_active_only').value
        app_settings['tracker_detection_mode'] = node.get_parameter('tracker_detection_mode').value
        app_settings['tracker_detection_sensitivity'] = node.get_parameter('tracker_detection_sensitivity').value
        app_settings['tracker_max_active_trackers'] = node.get_parameter('tracker_max_active_trackers').value
        app_settings['tracker_min_centre_point_distance_between_bboxes'] = node.get_parameter('tracker_min_centre_point_distance_between_bboxes').value
        
        # Background subtractor section
        app_settings['background_subtractor_type'] = node.get_parameter('background_subtractor_type').value
        app_settings['background_subtractor_sensitivity'] = node.get_parameter('background_subtractor_sensitivity').value
        app_settings['background_subtractor_learning_rate'] = node.get_parameter('background_subtractor_learning_rate').value
        app_settings['background_subtractor_cuda_enable'] = node.get_parameter('background_subtractor_cuda_enable').value

        # Blob detector section
        app_settings['blob_detector_type'] = node.get_parameter('blob_detector_type').value
        app_settings['blob_detector_min_distance_between_blobs'] = node.get_parameter('blob_detector_min_distance_between_blobs').value

        # Track Plotting section
        app_settings['track_path_plotting_enabled'] = node.get_parameter('track_path_plotting_enabled').value
        app_settings['track_plotting_type'] = node.get_parameter('track_plotting_type').value
        app_settings['track_validation_enable'] = node.get_parameter('track_validation_enable').value
        app_settings['track_stationary_threshold'] = node.get_parameter('track_stationary_threshold').value
        app_settings['track_orphaned_threshold'] = node.get_parameter('track_orphaned_threshold').value
        app_settings['track_prediction_enabled'] = node.get_parameter('track_prediction_enabled').value

        # Mask section
        app_settings['mask_type'] = node.get_parameter('mask_type').value
        app_settings['mask_pct'] = node.get_parameter('mask_pct').value
        app_settings['mask_overlay_image_file_name'] = node.get_parameter('mask_overlay_image_file_name').value
        app_settings['mask_overlay_image'] = None # Don't configure this as it's used as temporary storage

        # Dense optical flow
        app_settings['dense_optical_cuda_enable'] = node.get_parameter('dense_optical_cuda_enable').value
        app_settings['dense_optical_flow_h'] = node.get_parameter('dense_optical_flow_h').value
        app_settings['dense_optical_flow_w'] = node.get_parameter('dense_optical_flow_w').value

        # Observer section
        app_settings['observer_timer_interval'] = node.get_parameter('observer_timer_interval').value
        app_settings['observer_day_night_brightness_threshold'] = node.get_parameter('observer_day_night_brightness_threshold').value

        return app_settings