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

import numpy as np
from threading import Thread
from simple_tracker_shared.utils import is_bbox_being_tracked, bbox_overlap, is_bbox_being_tracked
from .tracker import Tracker

################################################################################################
# This class is pretty much considered the application part of the Simple Tracker application. #
################################################################################################
class VideoTracker():

    def __init__(self, settings):
      
        self.settings = settings
        self.total_trackers_finished = 0
        self.total_trackers_started = 0
        self.live_trackers = []

    @property
    def is_tracking(self):
        return len(self.live_trackers) > 0

    def active_trackers(self):
        trackers = filter(lambda x: x.is_tracking(), self.live_trackers)
        if trackers is None:
            return []
        else:
            return trackers

    # function to create trackers from extracted keypoints
    def create_trackers_from_bboxes(self, tracker_type, bboxes, frame):
        for bbox in bboxes:
            # print(bbox)

            # Initialize tracker with first frame and bounding box
            if not is_bbox_being_tracked(self.live_trackers, bbox):
                self.create_and_add_tracker(tracker_type, frame, bbox)

    # function to create an add the tracker to the list of active trackers
    def create_and_add_tracker(self, frame, bbox):
        if not bbox:
            raise Exception("null bbox")

        self.total_trackers_started += 1

        tracker = Tracker(self.settings, self.total_trackers_started, frame, bbox)
        tracker.update(frame)
        self.live_trackers.append(tracker)

    # function to update existing trackers and and it a target is not tracked then create a new tracker to track the target
    #
    # trackers are run on seperate threads in the hope to speed up the applicaiton by taking advantage of parallelism
    def update_trackers(self, bboxes, frame):

        unmatched_bboxes = bboxes.copy()
        failed_trackers = []
        tracker_count = len(self.live_trackers)

        threads = [None] * tracker_count
        results = [None] * tracker_count

        # Mike: We can do the tracker updates in parallel
        for i in range(tracker_count):
            tracker = self.live_trackers[i]
            threads[i] = Thread(target=self.update_tracker_task, args=(tracker, frame, results, i))
            threads[i].start()

        # Mike: We got to wait for the threads to join before we proceed
        for i in range(tracker_count):
            threads[i].join()

        for i in range(tracker_count):
            tracker = self.live_trackers[i]
            ok, bbox = results[i]
            if not ok:
                # Tracking failure
                failed_trackers.append(tracker)

            # Try to match the new detections with this tracker
            for new_bbox in bboxes:
                if new_bbox in unmatched_bboxes:
                    overlap = bbox_overlap(bbox, new_bbox)
                    # print(f'Overlap: {overlap}; bbox:{bbox}, new_bbox:{new_bbox}')
                    if overlap > 0.2:
                        unmatched_bboxes.remove(new_bbox)

        # remove failed trackers from live tracking
        for tracker in failed_trackers:
            self.live_trackers.remove(tracker)
            self.total_trackers_finished += 1

        # Add new detections to live tracker
        for new_bbox in unmatched_bboxes:
            # Hit max trackers?
            if len(self.live_trackers) < self.settings['tracker_max_active_trackers']:
                if not is_bbox_being_tracked(self.live_trackers, new_bbox):
                    self.create_and_add_tracker(frame, new_bbox)

    # Mike: Identifying tasks that can be called on seperate threads to try and speed this sucker up
    def update_tracker_task(self, tracker, frame, results, index):
        results[index] = tracker.update(frame)
