## Please Note: 
These end points are very much work in progress and have not been finalised yet. Therefore they are subject to change at any point in time. Before they are finalised, they will be versioned so as to not interfere with downstream developments. The versioning approach is again, very much still in the process of being 

## Configuration

### sky360/config/entry/v1
Service end point for retrieval of a single config entry

### sky360/config/entries/v1
Service end point for retrieval of an array of config entries

### sky360/config/entry/update/v1
Service end point for the update of a single config entry

### sky360/config/updated/v1
Message end point to provide a notification event that the config entry has been updated.

## Frames

### sky360/frame/original/v1
Message end point for the publishing of the original frame received from OpenCV

### sky360/frame/annotated/v1
Message end point for the publishing of the annotated frame

### sky360/frame/grey/v1
Message end point for the publishing of the grey frame

### sky360/frame/masked_background/v1
Message end point for the publishing of the masked_background frame

### sky360/frame/optical_flow/v1
Message end point for the publishing of the optical_flow frame

### sky360/frame/original/scaled/v1
Message end point for the publishing of a scaled version of the original frame received from OpenCV **Note** This will be deleted as its a temporary test message end point. If resizing is done, it will be done at source level accross all frames.


## Tracker

### sky360/tracker

## Tracks

### sky360/track

### sky360/tracks/prediction

### sky360/target

