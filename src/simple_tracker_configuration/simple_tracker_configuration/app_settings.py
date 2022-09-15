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

##################################################################################################
# This class provides a central area for populating and validating the application configuration #
##################################################################################################
class AppSettings():

    # Method to populate a configuration dictionary for use throughout the simple tracker application
    @staticmethod
    def Get():

        app_settings = {}

        # Controller section
        app_settings['controller_type'] = 'camera'
        app_settings['controller_iteration_interval'] = 10

        # Camera node section
        app_settings['camera_mode'] = 'rtsp'
        app_settings['camera_uri'] = 'rtsp://sky360:Sky360Sky!@192.168.0.43:554/cam/realmonitor?channel=1&subtype=0'
        app_settings['camera_resize_frame'] = True
        app_settings['camera_resize_dimension_h'] = 1080
        app_settings['camera_resize_dimension_w'] = 1400
        app_settings['camera_cuda_enable'] = False

        # Frame Provider node section
        app_settings['frame_provider_resize_frame'] = True
        app_settings['frame_provider_resize_dimension_h'] = 1080
        app_settings['frame_provider_resize_dimension_w'] = 1400
        app_settings['frame_provider_blur'] = True
        app_settings['frame_provider_blur_radius'] = 3
        app_settings['frame_provider_cuda_enable'] = False

        # Visualiser node section
        app_settings['visualiser_font_size'] = 0.75
        app_settings['visualiser_font_thickness'] = 2
        app_settings['visualiser_cuda_enable'] = False

        # Video Tracker section
        app_settings['tracker_type'] = 'CSRT'
        app_settings['tracker_stopwatch_enable'] = False
        app_settings['tracker_detection_mode'] = 'background_subtraction'
        app_settings['tracker_detection_sensitivity'] = 1
        app_settings['tracker_calculate_optical_flow'] = False
        app_settings['tracker_max_active_trackers'] = 10        
        app_settings['tracker_wait_seconds_threshold'] = 0
        app_settings['tracker_cuda_enable'] = False        

        # Background subtractor section
        app_settings['background_subtractor_type'] = 'KNN'
        app_settings['background_subtractor_learning_rate'] = 0.05
        app_settings['background_subtractor_cuda_enable'] = False

        # Track Plotting section
        app_settings['track_plotting_enabled'] = False
        app_settings['track_plotting_type'] = 'line'
        app_settings['track_validation_enable'] = True
        app_settings['track_stationary_threshold'] = 5
        app_settings['track_orphaned_threshold'] = 20
        app_settings['track_prediction_enabled'] = False
        app_settings['track_cuda_enable'] = False

        # BBox section
        app_settings['bbox_size'] = 64

        # Mask section
        #app_settings['mask_type'] = 'no_op'
        app_settings['mask_type'] = 'fish_eye'
        app_settings['mask_pct'] = 5
        app_settings['mask_overlay_image_path'] = None
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