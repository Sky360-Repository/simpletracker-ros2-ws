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
from simple_tracker_shared.utils import frame_resize

####################################################################################################################################
# Base class for various frame processor implementations. The idea here is that we have a standardised frame processing interface  #
# that is used by the video tracker. We currently support both CPU and GPU so require a frame processor for each one of these      #
# architectures. If additonal architectures are to be supported in future, like VPI, then this is where the specialisation         #
# implementation will go.                                                                                                          #
####################################################################################################################################
class FrameProcessor():

    def __init__(self, settings):
        self.settings = settings

    # Static select method, used as a factory method for selecting the appropriate implementation based on configuration
    @staticmethod
    def Select(settings, cuda_enable_setting:str):
        if settings[cuda_enable_setting]:
            return FrameProcessor.GPU(settings)

        return FrameProcessor.CPU(settings)

    # Static method, used to instantiate the CPU implementation of the frame processor
    @staticmethod
    def CPU(settings):
        return CpuFrameProcessor(settings)

    # Static method, used to instantiate the GPU implementation (CUDA) of the frame processor
    @staticmethod
    def GPU(settings):
        return GpuFrameProcessor(settings)

    # noise reduction interface specification
    def reduce_noise(self, frame, blur_radius, stream):
        pass

    # interface specification for converting a frame from colour to grey
    def convert_to_grey(self, frame, stream):
        pass

    # interface specification for processing the frame for the frame provider
    def process_for_frame_provider(self, mask, camera_frame, stream):
        pass

    # interface specification for processing the background subtraction
    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        pass

    # interface specification for processing the optical flow
    def process_optical_flow(self, dense_optical_flow, frame_grey, stream):
        pass

######################################################################
# Specialised implementation of the frame processor specific to CPU. #
######################################################################
class CpuFrameProcessor(FrameProcessor):

    def __init__(self, settings):
        super().__init__(settings)

    def __enter__(self):
        #print('CPU.__enter__')
        return self

    def __exit__(self, type, value, traceback):
        pass
        #print('CPU.__exit__')

    def reduce_noise(self, frame, blur_radius, stream):
        # Overload this for a CPU specific implementation
        #print('CPU.noise_reduction')
        noise_reduced_frame = cv2.GaussianBlur(frame, (blur_radius, blur_radius), 0)
        # frame_grey = cv2.medianBlur(frame_grey, blur_radius)
        return noise_reduced_frame

    def convert_to_grey(self, frame, stream):
        # Overload this for a CPU specific implementation
        #print('CPU.convert_to_grey')
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def process_for_frame_provider(self, mask, camera_frame, stream):

        frame_original = camera_frame

        if self.settings['frame_provider_resize_frame']:
            frame_original = frame_resize(frame_original, 
                width=self.settings['frame_provider_resize_dimension_w'], 
                height=self.settings['frame_provider_resize_dimension_h'])

        if not mask.initialised:
            mask.initialise(frame_original, stream)

        # apply mask
        frame_masked = mask.apply(frame_original, stream)

        #grey
        frame_grey = self.convert_to_grey(frame_masked, stream)

        # blur
        if self.settings['frame_provider_blur']:
            frame_grey = self.reduce_noise(frame_grey, self.settings['frame_provider_blur_radius'], stream)

        return frame_original, frame_grey, frame_masked

    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        foreground_mask_frame = background_subtractor.apply(frame_grey)
        frame_masked_background = cv2.bitwise_and(frame_grey, frame_grey, mask=foreground_mask_frame)
        return foreground_mask_frame, frame_masked_background

    def process_optical_flow(self, dense_optical_flow, frame_grey, stream):
        dof_frame = dense_optical_flow.process_grey_frame(frame_grey)
        #return self.resize(dof_frame, self.settings['dense_optical_flow_w'], self.settings['dense_optical_flow_h'], stream)
        return dof_frame

################################################################################
# Specialised implementation of the frame processor specific to GPU i.e. CUDA. #
################################################################################
class GpuFrameProcessor(FrameProcessor):
    # https://jamesbowley.co.uk/accelerating-opencv-with-cuda-streams-in-python/
    # Mike: NOTE The cuda implementation is terrible, it runs at about 1/3 the speed of the CPU implementation on my laptop.
    # I think this might have something to do with pararllel streams (link above) but am not very confident in that statement
    # as I am still very much trying to get a better understanding of it all.
    def __init__(self, settings):
        super().__init__(settings)

    def __enter__(self):
        #print('GPU.__enter__')
        return self

    def __exit__(self, type, value, traceback):
        pass
        #print('GPU.__exit__')

    def reduce_noise(self, gpu_frame, blur_radius, stream):
        # Overload this for a GPU specific implementation
        #print('GPU.noise_reduction')
        gpuFilter = cv2.cuda.createGaussianFilter(cv2.CV_8UC1, cv2.CV_8UC1, (blur_radius, blur_radius), 0)
        gpu_noise_reduced_frame = cv2.cuda_Filter.apply(gpuFilter, gpu_frame, stream=stream)
        return gpu_noise_reduced_frame

    def convert_to_grey(self, gpu_frame, stream):
        # Overload this for a GPU specific implementation
        #print('GPU.convert_to_grey')
        return cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY, stream=stream)

    def process_for_frame_provider(self, mask, camera_frame, stream):

        frame_original = camera_frame

        if self.settings['frame_provider_resize_frame']:
            frame_original = frame_resize(frame_original, 
                width=self.settings['frame_provider_resize_dimension_w'], 
                height=self.settings['frame_provider_resize_dimension_h'])

        if not mask.initialised:
            mask.initialise(frame_original, stream)

        gpu_frame_original = cv2.cuda_GpuMat()
        gpu_frame_original.upload(frame_original, stream=stream) 

        # apply mask
        gpu_frame_masked = mask.apply(gpu_frame_original, stream)

        #grey
        gpu_frame_grey = self.convert_to_grey(gpu_frame_masked, stream)

        # blur
        if self.settings['frame_provider_blur']:
            gpu_frame_grey = self.reduce_noise(gpu_frame_grey, self.settings['frame_provider_blur_radius'], stream)

        frame_masked = gpu_frame_masked.download()
        frame_grey = gpu_frame_grey.download()

        return frame_grey, frame_masked

    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        # Overload this for a GPU specific implementation
        # print('GPU.keypoints_from_bg_subtraction')

        gpu_frame_grey = cv2.cuda_GpuMat()
        gpu_frame_grey.upload(frame_grey, stream=stream) 

        gpu_foreground_mask = background_subtractor.apply(gpu_frame_grey, learningRate=self.settings['background_subtractor_learning_rate'], stream=stream)
        gpu_masked_background = cv2.cuda.bitwise_and(gpu_frame_grey, gpu_frame_grey, mask=gpu_foreground_mask, stream=stream)

        foreground_mask = gpu_foreground_mask.download()
        masked_background = gpu_masked_background.download()

        return foreground_mask, masked_background

    def process_optical_flow(self, dense_optical_flow, frame_grey, stream):
        # Overload this for a GPU specific implementation
        #print('GPU.process_optical_flow')

        gpu_frame_grey = cv2.cuda_GpuMat()
        gpu_frame_grey.upload(frame_grey, stream=stream) 

        gpu_dof_frame = dense_optical_flow.process_grey_frame(gpu_frame_grey, stream)
        dof_frame = gpu_dof_frame.download()
        return dof_frame
