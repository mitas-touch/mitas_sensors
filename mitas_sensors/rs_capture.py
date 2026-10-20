import numpy as np
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API


class RSCapture:
    @staticmethod
    def get_device_serial_numbers():
        devices = rs.context().devices
        return [d.get_info(rs.camera_info.serial_number) for d in devices]

    def __init__(self, name, serial_number, dim=(640, 480), fps=30, depth=False):
        self.name = name
        avail_serials = self.get_device_serial_numbers()
        print(f"available serialnumbers: {avail_serials}")
        assert serial_number in avail_serials
        self.serial_number = serial_number
        self.depth = depth
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_device(self.serial_number)
        self.cfg.enable_stream(rs.stream.color, dim[0], dim[1], rs.format.bgr8, fps)
        if self.depth:
            self.cfg.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, fps)
        self.profile = self.pipe.start(self.cfg)

        # info intrsinsics 
        video_stream = self.profile.get_stream(rs.stream.color)
        self._intrinsics = video_stream.as_video_stream_profile().get_intrinsics()


        # only needed if depth is used!
        self.do_align = False

        self.last_frame = None 

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        if self.do_align:
            align_to = rs.stream.color
            self.align = rs.align(align_to)

    def read(self):
        # frames = self.pipe.wait_for_frames()
        # above is not getting frames 
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=5000)
        except RuntimeError as e:
            print(f"Using old frame, due to {str(e)}")
            frames = self.last_frame
        

        if self.do_align:
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if self.depth:
                depth_frame = aligned_frames.get_depth_frame()
        else:
            color_frame = frames.get_color_frame()


        if color_frame.is_video_frame():
            image = np.asarray(color_frame.get_data())
            if self.depth and depth_frame.is_depth_frame():
                depth = np.expand_dims(np.asarray(depth_frame.get_data()), axis=2)
                return True, np.concatenate((image, depth), axis=-1)
            else:
                return True, image
        else:
            return False, None

    def close(self):
        self.pipe.stop()
        self.cfg.disable_all_streams()


    def get_intrinsics(self):
        return self._intrinsics