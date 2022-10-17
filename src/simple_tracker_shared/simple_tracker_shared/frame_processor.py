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

    # frame resize interface specification
    def resize(self, frame, w, h, stream):
        pass

    # interface specification for extracting keypoints from background subtracted frame
    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        pass

    # interface specification for extracting keypoints from background subtracted frame
    def process_dense_optical_flow(self, dense_optical_flow, frame_grey, stream):
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

    def resize(self, frame, w, h, stream):
        # Overload this for a CPU specific implementation
        #print(f'CPU.resize_frame w:{w}, h:{h}')
        return cv2.resize(frame, (w, h))

    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        return background_subtractor.apply(frame_grey)

    def process_dense_optical_flow(self, dense_optical_flow, frame_grey, stream):
        dof_frame = dense_optical_flow.process_grey_frame(frame_grey)
        return self.resize(dof_frame, self.settings['dense_optical_flow_w'], self.settings['dense_optical_flow_h'], stream)

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

    def resize(self, gpu_frame, w, h, stream):
        # Overload this for a GPU specific implementation
        #print(f'GPU.resize_frame w:{w}, h:{h}')
        return cv2.cuda.resize(gpu_frame, (w, h), stream=stream)

    def process_bg_subtraction(self, background_subtractor, frame_grey, stream):
        # Overload this for a GPU specific implementation
        # print('GPU.keypoints_from_bg_subtraction')

        gpu_frame_grey = cv2.cuda_GpuMat()
        gpu_frame_grey.upload(frame_grey, stream=stream) 

        gpu_foreground_mask = background_subtractor.apply(gpu_frame_grey, learningRate=self.settings['background_subtractor_learning_rate'], stream=stream)
        foreground_mask = gpu_foreground_mask.download()
        return foreground_mask

    def process_optical_flow(self, dense_optical_flow, frame_grey, stream):
        # Overload this for a GPU specific implementation
        #print('GPU.process_optical_flow')

        gpu_frame_grey = cv2.cuda_GpuMat()
        gpu_frame_grey.upload(frame_grey, stream=stream) 

        gpu_dof_frame = dense_optical_flow.process_grey_frame(gpu_frame_grey, stream)
        gpu_dof_frame = self.resize(gpu_dof_frame, self.settings['dense_optical_flow_w'], self.settings['dense_optical_flow_h'], stream)
        dof_frame = gpu_dof_frame.download()
        return dof_frame
