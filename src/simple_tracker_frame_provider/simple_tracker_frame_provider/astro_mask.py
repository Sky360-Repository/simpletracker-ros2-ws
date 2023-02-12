import cv2
import datetime
from datetime import datetime
import dateutil
from dateutil import tz
import math
import numpy as np
import ephem #pip install ephem
from .mask import Mask

#############################################################################################################
# NoOp masking implementations. It's just a passthrough and does not perform any sort of masking operation. #
# Its the fallback option and supports both CPU and GPU architectures.                                      #
#############################################################################################################
class AstroMask(Mask):

    @staticmethod
    def Lunar(settings):
        return LunarMask(settings)

    @staticmethod
    def Solar(settings):
        return SolarMask(settings)

    def __init__(self, settings):
        super().__init__(settings)

    def initialise(self, init_frame, stream=None):
        super().initialise(init_frame, stream)
        self.shape = init_frame.shape[:2]
        self.height = self.shape[0]
        self.width = self.shape[1]

        self.center_x = self.width / 2
        self.center_y = self.height / 2

        # Location of camera
        self.latitude = 51.7
        self.longitude = -4.3

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

        #self.timezone = 'Europe/Vienna'

        return (self.width, self.height)

    def alphaBlend(self, img1, img2, mask):
        if mask.ndim==3 and mask.shape[-1] == 3:
            alpha = mask/255.0
        else:
            alpha = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)/255.0
        blended = cv2.convertScaleAbs(img1*(1-alpha) + img2*alpha)
        return blended

    def drawCardinals(self, frame_masked):
        text_colour = (50, 170, 50)
        loc = self.fisheye_radius-150

        # Indicate where north is on the frame
        x_north = int(self.center_x + loc * np.cos(np.radians(+90)+self.theta))
        y_north = int(self.center_y + loc * np.sin(np.radians(+90)+self.theta))
        cv2.putText(frame_masked, "N", (int(x_north), int(y_north)), cv2.FONT_HERSHEY_SIMPLEX, 2, text_colour, 2)

        # Indicate where east is on the frame
        x_east = int(self.center_x + loc * np.cos(np.radians(+180)+self.theta))
        y_east = int(self.center_y + loc * np.sin(np.radians(+180)+self.theta))
        cv2.putText(frame_masked, "E", (int(x_east), int(y_east)), cv2.FONT_HERSHEY_SIMPLEX, 2, text_colour, 2)

        # Indicate where south is on the frame
        x_south = int(self.center_x + loc * np.cos(np.radians(-90)+self.theta))
        y_south = int(self.center_y + loc * np.sin(np.radians(-90)+self.theta))
        cv2.putText(frame_masked, "S", (int(x_south), int(y_south)), cv2.FONT_HERSHEY_SIMPLEX, 2, text_colour, 2)

        # Indicate where west is on the frame
        x_west = int(self.center_x + loc * np.cos(self.theta))
        y_west = int(self.center_y + loc * np.sin(self.theta))
        cv2.putText(frame_masked, "W", (int(x_west), int(y_west)), cv2.FONT_HERSHEY_SIMPLEX, 2, text_colour, 2)                

    @property
    def get_observer(self) -> ephem.Observer: 
        #tz = dateutil.tz.gettz(self.timezone)
        observer = ephem.Observer()
        observer.lat = str(self.latitude)
        observer.lon = str(self.longitude)
        #observer.date = datetime.datetime(2022, 2, 7, 20, 50, 51, tzinfo = tz) # <-- add time of frame here
        observer.date = datetime.now()

        return observer

class SolarMask(AstroMask):

    def __init__(self, settings):
        super().__init__(settings)

    def apply(self, frame, stream=None):
        self.guard_initialise()
        
        observer = self.get_observer

        sun = ephem.Sun()
        sun.compute(observer)

        azimuth_rad = float(sun.az)
        zenith_rad = math.radians(90) - float(sun.alt)

        x_pos = self.center_x - self.fisheye_radius * math.sin(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov) * -1
        y_pos = self.center_y + self.fisheye_radius * math.cos(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov)

        # Correct for orientation of camera (south is assumed to be top of the frame)
        x_centered = x_pos - self.center_x
        y_centered = y_pos - self.center_y
        x_rotated = x_centered * math.cos(self.theta) - y_centered * math.sin(self.theta) + self.center_x
        y_rotated = x_centered * math.sin(self.theta) + y_centered * math.cos(self.theta) + self.center_y

        # Draw circle at sun's position on the video frame
        #frame_masked = frame.copy()
        frame_masked = frame
        cv2.circle(frame_masked, (int(x_rotated), int(y_rotated)), self.solar_mask_radius, (178, 110, 67), -1);

        # draw cardinal directions
        self.drawCardinals(frame_masked)

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
        
        observer = self.get_observer

        moon = ephem.Moon()
        moon.compute(observer)

        azimuth_rad = float(moon.az)
        zenith_rad = math.radians(90) - float(moon.alt)

        x_pos = self.center_x - self.fisheye_radius * math.sin(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov) * -1
        y_pos = self.center_y + self.fisheye_radius * math.cos(azimuth_rad) * zenith_rad / math.radians(self.fisheye_half_fov)

        x_center = x_pos - self.center_x
        y_center = y_pos - self.center_y
        x_rotated = x_center * math.cos(self.theta) - y_center * math.sin(self.theta) + self.center_x
        y_rotated = x_center * math.sin(self.theta) + y_center * math.cos(self.theta) + self.center_y

        # Draw circle at moon's position on the video frame
        #frame_masked = frame.copy()
        frame_masked = frame
        cv2.circle(frame_masked, (int(x_rotated), int(y_rotated)), self.moon_mask_radius, (22,22,22), -1)

        # draw cardinal directions
        self.drawCardinals(frame_masked)

        # Blur
        blured = cv2.GaussianBlur(frame_masked, (61,61), 11)
        mask = np.zeros((self.height,self.width), np.uint8)
        cv2.circle(mask, (int(x_rotated), int(y_rotated)), self.solar_mask_radius+10, (255, 255, 255), -1)
        mask = cv2.GaussianBlur(mask, (51,51),11 )

        blended = self.alphaBlend(frame_masked, blured, mask)

        return blended
