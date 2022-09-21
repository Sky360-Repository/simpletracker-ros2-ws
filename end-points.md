## Please Note: 
These end points are very much work in progress and have not been finalised yet. Therefore they are subject to change at any point in time. Before they are finalised, they will be versioned so as to not interfere with downstream developments. The versioning approach is again, very much still in the process of being 

## Configuration

### sky360/config/entries/v1 - ConfigEntryArray.srv
Service end point for retrieval of an array of config entries

### sky360/config/entry/update/v1 - ConfigEntryUpdate.srv
Service end point for the update of a single config entry

### sky360/config/updated/v1 - ConfigEntryUpdateArray.msg
Message end point to provide a notification event that the config entry has been updated.

## Camera

### sky360/camera/original/v1 - CameraFrame.msg
Message end point for the publishing of the original frame received from OpenCV

## Frames

### sky360/frames/original/v1 - Frame.msg
Message end point for the publishing of the original frame after resizing

### sky360/frames/masked/v1 - Frame.msg
Message end point for the publishing of the original frame after processing i.e. resize, masked etc

### sky360/frames/grey/v1 - Frame.msg
Message end point for the publishing of the grey frame after processing i.e. resize, blurred, masked etc

### sky360/frames/dense_optical_flow/v1 - Frame.msg
Message end point for the publishing the dense optical flow frame

### sky360/frames/foreground_mask/v1 - Frame.msg
Message end point for the publishing of the foreground masked frame

### sky360/frames/masked_background/v1 - Frame.msg
Message end point for the publishing of the masked background frame

### sky360/frames/annotated/v1 - Frame.msg
Message end point for the publishing the annotated frame i.e. the frame with status and tracks etc

## Detection

### sky360/detector/key_points/v1 - KeyPointArray.msg
Message end point for the publishing of key point detection array for a frame

### sky360/detector/bounding_boxes/v1 - BoundingBoxArray.msg
Message end point for the publishing of bounding box array for a frame

## Tracker

### sky360/tracker/tracking_state/v1 - TrackingState.msg
Message end point for the publishing whether the tracker is tracking

### sky360/tracker/tracks/v1 - TrackArray.msg
Message end point for the publishing of tracks
