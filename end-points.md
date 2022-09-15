## Please Note: 
These end points are very much work in progress and have not been finalised yet. Therefore they are subject to change at any point in time. Before they are finalised, they will be versioned so as to not interfere with downstream developments. The versioning approach is again, very much still in the process of being 

## Configuration

### sky360/config/entries/v1
Service end point for retrieval of an array of config entries

### sky360/config/entry/update/v1
Service end point for the update of a single config entry

### sky360/config/updated/v1
Message end point to provide a notification event that the config entry has been updated.

## Camera

### sky360/camera/original/v1
Message end point for the publishing of the original frame received from OpenCV

## Frames

### sky360/frames/original/v1
Message end point for the publishing of the original frame after processing i.e. resize, blurred, masked etc

### sky360/frames/grey/v1
Message end point for the publishing of the grey frame after processing i.e. resize, blurred, masked etc

### sky360/frames/dense_optical_flow/v1
Message end point for the publishing the dense optical flow frame

### sky360/frames/foreground_mask/v1
Message end point for the publishing of the foreground masked frame

### sky360/frames/masked_background/v1

## Detection

### sky360/detecting/key_points/v1 -- TODO

### sky360/detecting/key_point/v1

### sky360/detecting/bounding_boxes/v1 -- TODO

### sky360/detecting/bounding_boxe/v1

### sky360/detecting/bounding_boxes/sized/v1 -- TODO

### sky360/detecting/bounding_box/sized/v1


### sky360/tracker -- TODO



