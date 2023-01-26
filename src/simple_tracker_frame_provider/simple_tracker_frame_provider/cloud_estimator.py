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
#import os
import numpy as np
#import seaborn as sns; sns.set()
import math

####################################################################################################################################
# Base class for various masking implementations. The idea here is that we have a standardised masking processing interface        #
# that is used by the frame processor. We currently support several types which in turn support both CPU and GPU architectures.    #
# If additonal architectures are to be supported in future, like VPI, then this is where the specialisation implementation will go.#
####################################################################################################################################
class CloudEstimator():

    @staticmethod
    def Day(settings):
        return DayTimeCloudEstimator(settings)

    @staticmethod
    def Night(settings):
        return NightTimeCloudEstimator(settings)

    def __init__(self, settings):
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):
        pass

    def br_norm(self, img, mask):
        b, g, r = cv2.split(img)
        r = np.where(r == 0, 1, r)
        r = r.astype(np.float32)
        b = b.astype(np.float32)
        lambda_n = np.where(mask, (b - r) / (b + r), 0)
        return lambda_n
    
    def find_threshold_mce(self, img):

        StArray = img.ravel()
        x = np.linspace(-1,1,201)
        y, bins = np.histogram(StArray, bins=x)

        MinValue = np.min(StArray)
        MaxValue = np.max(StArray)

        t_int_decimal= MinValue+((MaxValue-MinValue)/2)
        t_int = np.ceil(t_int_decimal*100)/100
        index_of_t_int = np.where(np.isclose(x, t_int))

        m0a = 0
        m1a = 0
        for i in range(index_of_t_int[0][0]):
            m0a += y[i]
            m1a += x[i] * y[i]
        
        m0b = 0
        m1b = 0
        for i in range(index_of_t_int[0][0], 200):
            m0b += y[i]
            m1b += x[i] * y[i]

        mu_a=(m1a/m0a)
        mu_b=(m1b/m0b)

        if mu_a < 0:
            mu_a = abs(mu_a)

        diff=5

        t_n_decimal=((mu_b-mu_a) /(np.log(mu_b)-np.log(mu_a)))
        t_n = np.ceil(t_n_decimal*100)/100; 

        iter = 1
        while True:
            # print("Present iteration:", iter)
            t_int = t_n
        
            # Finding index of t_int
            for i in range(201):
                if x[i] == t_int:
                    index_of_t_int = i
                    break
        
            m0a = 0
            m1a = 0
            for i in range(index_of_t_int[0][0]):
                m0a += y[i]
                m1a += x[i] * y[i]
            
            m0b = 0
            m1b = 0
            for i in range(index_of_t_int[0][0], 200):
                m0b += y[i]
                m1b += x[i] * y[i]
        
            mu_a = m1a/m0a
            mu_b = m1b/m0b

            if mu_a < 0:
                mu_a = abs(mu_a)
        
            t_nplus1_decimal = (mu_b - mu_a) / (np.log(mu_b) - np.log(mu_a))
            t_nplus1 = math.ceil(t_nplus1_decimal * 100) / 100
        
            diff = abs(t_nplus1 - t_n)
            t_n = t_nplus1
        
            if diff == 0:
                break
        
            iter += 1

        ThresholdValue = t_n

        return ThresholdValue

    def get_mask(self, frame):
        height, width, _ = frame.shape
        x, y = np.ogrid[:width, :height]
        center_x, center_y = width // 2, height // 2
        radius = 1070
        mask = (x - center_x)**2 + (y - center_y)**2 < radius**2
        return mask        

#############################################################################################################
# NoOp masking implementations. It's just a passthrough and does not perform any sort of masking operation. #
# Its the fallback option and supports both CPU and GPU architectures.                                      #
#############################################################################################################
class DayTimeCloudEstimator(CloudEstimator):

    def __init__(self, settings):
        super().__init__(settings)
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):

        mask = self.get_mask(frame)

        lambda_n = self.br_norm(frame, mask)

        std = np.std(lambda_n[mask])
        #print("Processing image: " + image_files[i])
        # print("Standard deviation: " + str(std))
        if std > 0.03: # magic number
            #print("Bimodal distribution")
            threshold = self.find_threshold_mce(frame)
            _, ratio_mask = cv2.threshold(frame, threshold, 255, cv2.THRESH_BINARY)
        else:
            #print("Unimodal distribution")
            _, ratio_mask = cv2.threshold(frame, 0.25, 255, cv2.THRESH_BINARY)

        #ax.imshow(ratio_mask, cmap='gray')
        #ax.axis('off')  
        #ax.set_title(image_files[i],fontsize=8)

        # Count the number of pixels in each part of the mask
        N_Cloud = np.count_nonzero(ratio_mask[mask] == 0)
        N_Sky = np.count_nonzero(ratio_mask[mask])

        # Cloud cover ratio
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100
        #print(f'Cloud cover: {round(ccr,2)}')
        #print(f'')
        return round(ccr,2)

class NightTimeCloudEstimator(CloudEstimator):

    def __init__(self, settings):
        super().__init__(settings)
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):

        mask = self.get_mask(frame)

        #print("Processing image: " + image_files[i])
        otsu_threshold, image_result = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU,)

        # Count the number of non zero pixels (white)
        N_Cloud = np.count_nonzero(image_result[mask])
        N_Sky = np.count_nonzero(image_result[mask] == 0)
    
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100
        #print(f'Cloud cover: {round(ccr,2)}%')
        #print(f'')
        return round(ccr,2)
