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

# Utility function to clip out the center part of a frame. This is mainly used by the fish-eye mask
# to remove masked ("black") parts of the frame
def clip_at_center(frame, center, width, height, new_width, new_height):
    x, y = center
    half_width = int(new_width/2)
    half_height = int(new_height/2)

    left = max(0, x-half_width)
    right = min(x+half_width, width)
    right = max(new_width, right)

    top = max(0, y-half_height)
    bottom = min(y+half_height, height)
    bottom = max(new_height, bottom)

    return frame[top:bottom, left:right]

# Utility function to determine if the installed version of open cv is supported
# We support v4.1 and above
def frame_resize(frame, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the frame to be resized and
    # grab the frame size
    dim = None
    (h, w) = frame.shape[:2]

    # if both the width and height are None, then return the
    # original frame
    if width is None and height is None:
        return frame

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the frame
    resized = cv2.resize(frame, dim, interpolation = inter)

    # return the resized frame
    return resized    