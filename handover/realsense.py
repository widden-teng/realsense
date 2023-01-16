import pyrealsense2 as rs
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time
import cv2

# Resolution of camera streams
RESOLUTION_X = 1280
RESOLUTION_Y = 720


class VideoStreamer:
    """
    Video streamer that takes advantage of multi-threading, and continuously is reading frames.
    Frames are then ready to read when program requires.
    """

    def __init__(self, video_file=None):
        """
        When initialised, VideoStreamer object should be reading frames
        """
        self.setup_image_config(video_file)
        self.configure_streams()
        self.stopped = False

    def start(self):
        """
        Initialise thread, update method will run under thread
        """
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """
        Constantly read frames until stop() method is introduced
        """
        while True:

            if self.stopped:
                return

            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            # Convert image to numpy array and initialise images
            self.color_image = np.asanyarray(color_frame.get_data())
            self.depth_image = np.asanyarray(depth_frame.get_data())

    def stop(self):
        self.pipeline.stop()
        self.stopped = True

    def read(self):
        return (self.color_image, self.depth_image)

    def setup_image_config(self, video_file=None):
        """
        Setup config and video steams. If --file is specified as an argument, setup
        stream from file. The input of --file is a .bag file in the bag_files folder.
        .bag files can be created using d435_to_file in the tools folder.
        video_file is by default None, and thus will by default stream from the 
        device connected to the USB.
        """
        config = rs.config()

        if video_file is None:

            config.enable_stream(rs.stream.depth, RESOLUTION_X,
                                 RESOLUTION_Y, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, RESOLUTION_X,
                                 RESOLUTION_Y, rs.format.bgr8, 30)
        else:
            try:
                config.enable_device_from_file(
                    "bag_files/{}".format(video_file))
            except:
                print("Cannot enable device from: '{}'".format(video_file))

        self.config = config

    def configure_streams(self):
        # Configure video streams
        self.pipeline = rs.pipeline()

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

    def get_depth_scale(self):
        return self.profile.get_device().first_depth_sensor().get_depth_scale()


if __name__ == "__main__":
    # Initialise video streams from D435
    video_streamer = VideoStreamer()
    depth_scale = video_streamer.get_depth_scale()
    video_streamer.start()
    time.sleep(1)
    while True:
        # time_start = time.time()
        color_image, depth_image = video_streamer.read()
        # t1 = time.time()

        cv2.imshow('RGB image', color_image)
        cv2.imshow('Depth image', depth_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    video_streamer.stop()
    cv2.destroyAllWindows()
