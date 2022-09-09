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

        app_settings['controller'] = 'camera'

        app_settings['camera_mode'] = 'rtsp'
        app_settings['camera_uri'] = 'rtsp://sky360:Sky360Sky!@192.168.0.43:554/cam/realmonitor?channel=1&subtype=0'
        app_settings['camera_iteration_interval'] = 10

        #Visualisers
        app_settings['font_size'] = 0.75
        app_settings['font_thickness'] = 2

        # Video Tracker section
        app_settings['enable_stopwatch'] = False
        app_settings['enable_cuda'] = False
        app_settings['detection_mode'] = 'background_subtraction'
        app_settings['detection_sensitivity'] = 2
        app_settings['noise_reduction'] = True
        app_settings['resize_frame'] = False
        app_settings['resize_dimension'] = 1024
        app_settings['blur_radius'] = 3
        app_settings['calculate_optical_flow'] = False
        app_settings['max_active_trackers'] = 10
        app_settings['tracker_type'] = 'CSRT'
        app_settings['background_subtractor_type'] = 'KNN'
        app_settings['background_subtractor_learning_rate'] = 0.05
        app_settings['tracker_wait_seconds_threshold'] = 0

        # Tracker section
        app_settings['enable_track_validation'] = True
        app_settings['stationary_track_threshold'] = 5
        app_settings['orphaned_track_threshold'] = 20

        # BBox section
        app_settings['bbox_fixed_size'] = True
        app_settings['bbox_size'] = 64

        # Track Plotting section
        app_settings['track_plotting_enabled'] = False
        app_settings['track_plotting_type'] = 'line'

        # Track Prediction section
        app_settings['track_prediction_enabled'] = False

        # Mask section
        app_settings['mask_type'] = 'fish_eye'
        app_settings['mask_pct'] = 10
        app_settings['mask_overlay_image_path'] = None

        # Dense optical flow
        app_settings['dense_optical_flow_height'] = 480
        app_settings['dense_optical_flow_width'] = 480

        # MOT_STF section
        app_settings['motstf_write_original'] = True
        app_settings['motstf_write_annotated'] = True
        app_settings['motstf_write_images'] = False

        return app_settings