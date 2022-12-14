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
import sys

def get_camera(app_configuration: dict):
    camera_mode = app_configuration['camera_mode']
    camera_uri = app_configuration['camera_uri']
    print(f"Connecting to {camera_mode} camera at {camera_uri}")
    camera = None
    if camera_mode == 'ffmpeg':
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
        camera = cv2.VideoCapture(
            camera_uri,
            cv2.CAP_FFMPEG
        )
    elif camera_mode == 'rtsp':
        camera = cv2.VideoCapture(
            camera_uri
        )
    elif camera_mode == 'local':
        for i in range(-1, 100):
            try:
                camera = cv2.VideoCapture(i)
                if camera:
                    break
            except cv2.error as e:
                print(e)
            except Exception as e:
                print(e)
    ##
    # Apparently this works for firewire, but I couldn't get it to
    ## camera = cv2.VideoCapture(3, cv2.CAP_DC1394)

    if not camera:
        print(cv2.getBuildInformation())
        print(f"Unable to find camera using config: {app_configuration}")
        sys.exit(1)
    return camera
