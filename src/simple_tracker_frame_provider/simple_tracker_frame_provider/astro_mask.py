import cv2
import datetime
import dateutil
import math
import numpy as np
import ephem #pip install ephem
from .mask import Mask

#############################################################################################################
# NoOp masking implementations. It's just a passthrough and does not perform any sort of masking operation. #
# Its the fallback option and supports both CPU and GPU architectures.                                      #
#############################################################################################################
class AstroMask(Mask):

    def __init__(self, settings):
        super().__init__(settings)

    def initialise(self, init_frame, stream=None):
        super().initialise(init_frame, stream)
        self.shape = init_frame.shape[:2]
        self.height = self.shape[0]
        self.width = self.shape[1]

        # Location of camera
        self.latitude = 48.1
        self.longitude = 16.3

        # Fisheye info
        self.fisheye_fov = 180 # degrees
        self.fisheye_half_fov = self.fisheye_fov/2
        self.fisheye_radius = 1440 # pixels

        # Radius of solar mask 
        self.solar_mask_radius = 250 # pixels

        # Moon mask radius
        self.moon_mask_radius = 180 # pixels

        # Correct for angle (south is assumed to be at the top of the camera frame) 
        # In Sky360 dataset south is approx 9 degrees off the top
        self.theta = math.radians(-5)

        self.timezone = 'Europe/Vienna'

        return (self.width, self.height)

class SolarMask(AstroMask):

    def __init__(self, settings):
        super().__init__(settings)

    def apply(self, frame, stream=None):
        self.guard_initialise()
        
        tz = dateutil.tz.gettz(self.timezone)  
        observer = ephem.Observer()
        observer.lat = str(self.latitude)
        observer.lon = str(self.longitude)
        observer.date = datetime.datetime(2021, 5, 15, 13, 54, 17, tzinfo = tz) # <-- add time of frame here

        sun = ephem.Sun()
        sun.compute(observer)

        azimuth_rad = float(sun.az)
        zenith_rad = math.radians(90) - float(sun.alt)

        cx = self.width / 2
        cy = self.height / 2

        x_pos = cx - self.fisheye_radius * math.sin(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov) * -1
        y_pos = cy + self.fisheye_radius * math.cos(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov)

        # Correct for orientation of camera (south is assumed to be top of the frame)
        x_centered = x_pos - cx
        y_centered = y_pos - cy
        x_rotated = x_centered * math.cos(self.theta) - y_centered * math.sin(self.theta) + cx
        y_rotated = x_centered * math.sin(self.theta) + y_centered * math.cos(self.theta) + cy

        # Draw circle at sun's position on the video frame
        frame_masked = frame.copy()
        #frame_masked = frame
        cv2.circle(frame_masked, (int(x_rotated), int(y_rotated)), self.solar_mask_radius, (178, 110, 67), -1);

        # Indicate where south is on the frame
        x_text = int(cx + (self.fisheye_radius-150) * np.cos(np.radians(-90)+self.theta))
        y_text = int(cy + (self.fisheye_radius-150) * np.sin(np.radians(-90)+self.theta))
        cv2.putText(frame_masked, "S", (int(x_text), int(y_text)), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 2, cv2.LINE_AA)

        # Copy of frame but blurred
        blurred = cv2.GaussianBlur(frame_masked, (61,61), 11)

        mask = np.zeros((self.height,self.width), np.uint8)
        cv2.circle(mask, (int(x_rotated), int(y_rotated)), self.solar_mask_radius+10, (255, 255, 255), -1)
        mask = cv2.GaussianBlur(mask, (51,51),11 )

        blended = self.alphaBlend(frame_masked, blurred, mask)

        return blended

class LunarMask(AstroMask):

    def __init__(self, settings):
        super().__init__(settings)

    def apply(self, frame, stream=None):
        self.guard_initialise()
        
        cx = self.width / 2
        cy = self.height / 2

        tz = dateutil.tz.gettz('Europe/Vienna')  
        observer = ephem.Observer()
        observer.lat = str(self.latitude)
        observer.lon = str(self.longitude)
        observer.date = datetime.datetime(2022, 2, 7, 20, 50, 51, tzinfo = tz) # <-- add time of frame here

        moon = ephem.Moon()
        moon.compute(observer)

        azimuth_rad = float(moon.az)
        zenith_rad = math.radians(90) - float(moon.alt)

        x_pos = cx - self.fisheye_radius * math.sin(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov) * -1
        y_pos = cy + self.fisheye_radius * math.cos(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov)

        x_center = x_pos - cx
        y_center = y_pos - cy
        x_rotated = x_center * math.cos(self.theta) - y_center * math.sin(self.theta) + cx
        y_rotated = x_center * math.sin(self.theta) + y_center * math.cos(self.theta) + cy

        # Draw circle at moon's position on the video frame
        frame_masked = frame.copy()
        #frame_masked = frame
        cv2.circle(frame_masked, (int(x_rotated), int(y_rotated)), self.moon_mask_radius, (22,22,22), -1)

        # Indicate where south is on the frame
        x_text = int(cx + (self.fisheye_radius-150) * np.cos(np.radians(-90)+self.theta))
        y_text = int(cy + (self.fisheye_radius-150) * np.sin(np.radians(-90)+self.theta))
        cv2.putText(frame_masked, "S", (int(x_text), int(y_text)), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 2, cv2.LINE_AA)

        # Blur
        blured = cv2.GaussianBlur(frame_masked, (61,61), 11)
        mask = np.zeros((self.height,self.width), np.uint8)
        cv2.circle(mask, (int(x_rotated), int(y_rotated)), self.solar_mask_radius+10, (255, 255, 255), -1)
        mask = cv2.GaussianBlur(mask, (51,51),11 )

        blended = self.alphaBlend(frame_masked, blured, mask)

        return blended
