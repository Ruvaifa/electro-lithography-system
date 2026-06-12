"""
Basler camera live-view support using the pypylon SDK.

This module is intentionally independent from main.py so it can be tested on
the camera bench now and imported into the motion-control flow later.
"""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import cv2
except ImportError as exc:  # pragma: no cover - depends on local install
    raise ImportError(
        "OpenCV is required for live display. Install it with: pip install opencv-python"
    ) from exc

try:
    from pypylon import pylon
except ImportError as exc:  # pragma: no cover - depends on local install
    raise ImportError(
        "pypylon is required for Basler camera control. Install it with: pip install pypylon"
    ) from exc


FrameCallback = Callable[[object], None]


@dataclass
class BaslerCameraSettings:
    """Optional camera settings used during initialization."""

    exposure_time_us: Optional[float] = None
    gain: Optional[float] = None
    width: Optional[int] = None
    height: Optional[int] = None
    center_x: bool = True
    center_y: bool = True


class BaslerCamera:
    """
    Small wrapper around pypylon for live display and future integration.

    Example:
        camera = BaslerCamera()
        camera.open()
        camera.show_live()
        camera.close()
    """

    def __init__(
        self,
        camera_index: int = 0,
        settings: Optional[BaslerCameraSettings] = None,
    ) -> None:
        self.camera_index = camera_index
        self.settings = settings or BaslerCameraSettings()
        self.camera: Optional[pylon.InstantCamera] = None
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        self._stop_event = threading.Event()
        self._stream_thread: Optional[threading.Thread] = None

    def open(self) -> None:
        """Open the selected Basler camera and apply configured settings."""
        if self.camera and self.camera.IsOpen():
            return

        factory = pylon.TlFactory.GetInstance()
        devices = factory.EnumerateDevices()
        if not devices:
            raise RuntimeError("No Basler camera found.")
        if self.camera_index < 0 or self.camera_index >= len(devices):
            raise IndexError(
                f"Camera index {self.camera_index} is invalid. "
                f"{len(devices)} camera(s) detected."
            )

        self.camera = pylon.InstantCamera(factory.CreateDevice(devices[self.camera_index]))
        self.camera.Open()
        self._apply_settings()

    def close(self) -> None:
        """Stop streaming and close the camera connection."""
        self.stop_stream()
        if self.camera:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera.IsOpen():
                self.camera.Close()
        self.camera = None

    def get_frame(self, timeout_ms: int = 5000):
        """
        Grab and return one frame as an OpenCV BGR image.

        This is useful for future integration where main.py needs snapshots or
        image processing without opening a display window.
        """
        self._require_open_camera()
        assert self.camera is not None

        was_grabbing = self.camera.IsGrabbing()
        if not was_grabbing:
            self.camera.StartGrabbingMax(1)

        grab_result = self.camera.RetrieveResult(
            timeout_ms,
            pylon.TimeoutHandling_ThrowException,
        )
        try:
            if not grab_result.GrabSucceeded():
                raise RuntimeError(f"Frame grab failed: {grab_result.ErrorDescription}")
            return self.converter.Convert(grab_result).GetArray()
        finally:
            grab_result.Release()

    def show_live(
        self,
        window_name: str = "Basler Camera Live",
        timeout_ms: int = 5000,
        exit_key: str = "q",
        frame_callback: Optional[FrameCallback] = None,
    ) -> None:
        """
        Display a blocking live video stream.

        Press the exit key, default 'q', or Esc to close the stream. The optional
        callback receives each OpenCV BGR frame and can be used later for
        overlays, saving, or live image processing.
        """
        self._require_open_camera()
        assert self.camera is not None

        self._stop_event.clear()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        try:
            while self.camera.IsGrabbing() and not self._stop_event.is_set():
                grab_result = self.camera.RetrieveResult(
                    timeout_ms,
                    pylon.TimeoutHandling_ThrowException,
                )
                try:
                    if not grab_result.GrabSucceeded():
                        print(f"[WARN] Frame grab failed: {grab_result.ErrorDescription}")
                        continue

                    frame = self.converter.Convert(grab_result).GetArray()
                    if frame_callback:
                        frame_callback(frame)

                    cv2.imshow(window_name, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord(exit_key), 27):
                        break
                finally:
                    grab_result.Release()
        finally:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            cv2.destroyWindow(window_name)

    def start_stream(
        self,
        window_name: str = "Basler Camera Live",
        frame_callback: Optional[FrameCallback] = None,
    ) -> None:
        """
        Start live view in a background thread.

        This is the integration-friendly path for future main.py usage. Call
        stop_stream() before program shutdown.
        """
        if self._stream_thread and self._stream_thread.is_alive():
            return

        self._stop_event.clear()
        self._stream_thread = threading.Thread(
            target=self.show_live,
            kwargs={
                "window_name": window_name,
                "frame_callback": frame_callback,
            },
            daemon=True,
        )
        self._stream_thread.start()

    def stop_stream(self) -> None:
        """Request the live-view thread to stop and wait briefly for it."""
        self._stop_event.set()
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=2)
        self._stream_thread = None

    def _apply_settings(self) -> None:
        assert self.camera is not None

        self._try_set_bool("CenterX", self.settings.center_x)
        self._try_set_bool("CenterY", self.settings.center_y)
        if self.settings.width is not None:
            self._try_set_int("Width", self.settings.width)
        if self.settings.height is not None:
            self._try_set_int("Height", self.settings.height)
        if self.settings.exposure_time_us is not None:
            self._try_set_float("ExposureTime", self.settings.exposure_time_us)
        if self.settings.gain is not None:
            self._try_set_float("Gain", self.settings.gain)

    def _try_set_bool(self, node_name: str, value: bool) -> None:
        node = getattr(self.camera, node_name, None)
        if node and pylon.IsWritable(node):
            node.SetValue(value)

    def _try_set_int(self, node_name: str, value: int) -> None:
        node = getattr(self.camera, node_name, None)
        if node and pylon.IsWritable(node):
            minimum = node.GetMin()
            maximum = node.GetMax()
            increment = max(node.GetInc(), 1) if hasattr(node, "GetInc") else 1
            clamped = max(minimum, min(maximum, int(value)))
            aligned = minimum + ((clamped - minimum) // increment) * increment
            node.SetValue(aligned)

    def _try_set_float(self, node_name: str, value: float) -> None:
        node = getattr(self.camera, node_name, None)
        if node and pylon.IsWritable(node):
            minimum = node.GetMin()
            maximum = node.GetMax()
            node.SetValue(max(minimum, min(maximum, float(value))))

    def _require_open_camera(self) -> None:
        if not self.camera or not self.camera.IsOpen():
            raise RuntimeError("Camera is not open. Call open() first.")

    def __enter__(self) -> "BaslerCamera":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def list_basler_cameras() -> list[str]:
    """Return display names for all connected Basler cameras."""
    factory = pylon.TlFactory.GetInstance()
    return [device.GetFriendlyName() for device in factory.EnumerateDevices()]


def main() -> None:
    parser = argparse.ArgumentParser(description="Show live video from a Basler camera.")
    parser.add_argument("--camera-index", type=int, default=0, help="Basler camera index.")
    parser.add_argument("--exposure", type=float, help="Exposure time in microseconds.")
    parser.add_argument("--gain", type=float, help="Camera gain.")
    parser.add_argument("--width", type=int, help="Requested image width.")
    parser.add_argument("--height", type=int, help="Requested image height.")
    parser.add_argument("--list", action="store_true", help="List connected cameras and exit.")
    args = parser.parse_args()

    if args.list:
        cameras = list_basler_cameras()
        if not cameras:
            print("No Basler cameras found.")
            return
        for index, name in enumerate(cameras):
            print(f"[{index}] {name}")
        return

    settings = BaslerCameraSettings(
        exposure_time_us=args.exposure,
        gain=args.gain,
        width=args.width,
        height=args.height,
    )

    with BaslerCamera(camera_index=args.camera_index, settings=settings) as camera:
        print("Live stream started. Press 'q' or Esc in the video window to stop.")
        try:
            camera.show_live()
        except KeyboardInterrupt:
            print("\nStopping camera stream...")
            time.sleep(0.1)


if __name__ == "__main__":
    main()
