import threading

import ffmpegcv
from ffmpegcv.ffmpeg_reader_camera import query_camera_devices

import numpy as np
import cv2


class GelSightSensorError(RuntimeError):
    pass


class GelSight:
    """
    Class to connect to the GelSight sensor using the ffmpegcv package.
    Credit to Janis Lenz, Alina Böhm, Inga Pfenning for the code.
    """

    def __init__(
        self,
        img_shape: tuple[int, int] = (480, 320),
        camera_name: str = "GelSight",
    ):
        """
        Initialize connection to the GelSight sensor using the ffmpegcv package.

        :param img_shape: shape of the image (width, height) passed to ffmpegcv resize
        :param camera_name: substring matched against V4L device names.
            Default "GelSight" selects the first matching device (single-sensor behavior).
            Pass a longer substring of the device name to select a specific unit.
        :return: None
        """

        self.__camera_name: str = camera_name
        self.__img_shape = img_shape
        self.__latest_img = np.zeros(tuple(reversed(img_shape)) + (3,), dtype=np.uint8)
        self.__img_before_latest = np.zeros(
            tuple(reversed(img_shape)) + (3,), dtype=np.uint8
        )
        self.__camera = ffmpegcv.VideoCaptureCAM(
            self._get_camera_id(), pix_fmt="rgb24", resize=self.__img_shape
        )
        if not self.connected:
            pass

        self.terminate_thread = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(
            target=self._image_thread, name=f"{self.__camera_name}_thread"
        )
        self.thread.start()

    def __del__(self):
        self.close()

    def close(self):
        """Close the connection to the GelSight."""
        self.terminate_thread = True
        self.thread.join()
        self.__camera.close()

    def _image_thread(self):
        """Thread function to receive images asynchronously."""
        try:
            while not self.terminate_thread:
                success, img = self.__camera.read()
                if not success:
                    print(f"{self.__camera_name} thread closed!")
                    self.terminate_thread = True
                    continue
                with self.lock:
                    self.__img_before_latest = self.__latest_img.copy()
                    self.__latest_img = img
        except Exception:
            pass

    def _get_camera_id(self):
        """
        Determine camera ID of the GelSight by device name.

        :raises GelSightSensorError: if no matching camera found
        :return: int camera ID
        """
        devices = query_camera_devices()
        for id, info in devices.items():
            name = str(info[0])
            if self.__camera_name.lower() in name.lower():
                print(f"Found {self.__camera_name}: {info}, id: {id}")
                return id
        raise GelSightSensorError(f"Could not find {self.__camera_name}!")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def camera_name(self) -> str:
        return self.__camera_name

    @property
    def camera(self) -> ffmpegcv.FFmpegReaderCAM:
        return self.__camera

    @property
    def connected(self) -> bool:
        return self.__camera is not None and self.__camera.isOpened()

    @property
    def img_size(self) -> tuple[int, int]:
        return self.__img_shape

    @property
    def latest_img(self):
        """
        Latest captured image.

        :raises GelSightSensorError: if connection is lost
        """
        if not self.connected and not self.thread.is_alive():
            raise GelSightSensorError(f"Connection to {self.__camera_name} lost!")
        return self.__latest_img

    @property
    def second_latest_img(self):
        """
        Second latest captured image.

        :raises GelSightSensorError: if connection is lost
        """
        if not self.connected and not self.thread.is_alive():
            raise GelSightSensorError(f"Connection to {self.__camera_name} lost!")
        return self.__img_before_latest
