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
    from pypylon import pylon, genicam
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
        """Apply camera settings via the proper pypylon NodeMap API.

        Strategy:
          1. Reset camera to factory defaults so stale settings from a previous
             run (e.g. a crashed server session) don't leave the image black.
          2. Set ExposureAuto = Continuous so brightness adjusts automatically.
          3. As a safety net, also set a 20 ms fallback ExposureTime — this is
             only visible if ExposureAuto=Continuous cannot be written (rare).
          4. Apply any explicit overrides from BaslerCameraSettings.
        """
        assert self.camera is not None
        node_map = self.camera.GetNodeMap()

        # --- Step 1: Reset to factory defaults ---
        # This clears any bad state left by a previous crashed session.
        try:
            user_set_sel = node_map.GetNode("UserSetSelector")
            user_set_load = node_map.GetNode("UserSetLoad")
            if (user_set_sel and genicam.IsAvailable(user_set_sel) and genicam.IsWritable(user_set_sel)
                    and user_set_load and genicam.IsAvailable(user_set_load)):
                try:
                    user_set_sel.SetValue("Default")
                except Exception:
                    entry = user_set_sel.GetEntryByName("Default")
                    if entry and genicam.IsAvailable(entry):
                        user_set_sel.SetIntValue(entry.GetValue())
                user_set_load.Execute()
                print("[CAMERA] Reset to factory defaults (UserSetLoad=Default).")
        except Exception as e:
            print(f"[CAMERA] UserSetLoad reset skipped: {e}")

        # --- Step 2: Enable continuous auto-exposure ---
        # Do NOT set ExposureMode=Timed first — it can lock cameras in
        # hardware-trigger modes. Let the factory default set it.
        try:
            exp_auto = node_map.GetNode("ExposureAuto")
            if exp_auto and genicam.IsAvailable(exp_auto) and genicam.IsWritable(exp_auto):
                try:
                    exp_auto.SetValue("Continuous")
                except Exception:
                    entry = exp_auto.GetEntryByName("Continuous")
                    if entry and genicam.IsAvailable(entry):
                        exp_auto.SetIntValue(entry.GetValue())
                print("[CAMERA] ExposureAuto = Continuous.")
            else:
                print("[CAMERA] ExposureAuto node not writable or available.")
        except Exception as e:
            print(f"[CAMERA] ExposureAuto=Continuous failed: {e}")

        # --- Step 3: Safety-net fallback exposure (20 ms) ---
        # If ExposureAuto is Off or unsupported, 20 ms gives a visible image
        # in typical lab lighting without being over/underexposed.
        try:
            exp_auto_val = ""
            try:
                exp_auto_node = node_map.GetNode("ExposureAuto")
                if exp_auto_node:
                    try:
                        exp_auto_val = exp_auto_node.GetValue()
                    except AttributeError:
                        exp_auto_val = exp_auto_node.GetCurrentEntry().GetSymbolic()
            except Exception:
                pass
            if exp_auto_val != "Continuous":
                exp_time = node_map.GetNode("ExposureTime")
                if exp_time and genicam.IsWritable(exp_time):
                    fallback = max(exp_time.GetMin(), min(exp_time.GetMax(), 20000.0))
                    exp_time.SetValue(fallback)
                    print(f"[CAMERA] Fallback ExposureTime = {fallback:.0f} µs.")
        except Exception as e:
            print(f"[CAMERA] Fallback ExposureTime skipped: {e}")

        # --- Step 4: Apply explicit user overrides ---
        if self.settings.center_x is not None:
            self._nm_set_bool(node_map, "CenterX", self.settings.center_x)
        if self.settings.center_y is not None:
            self._nm_set_bool(node_map, "CenterY", self.settings.center_y)
        if self.settings.width is not None:
            self._nm_set_int(node_map, "Width", self.settings.width)
        if self.settings.height is not None:
            self._nm_set_int(node_map, "Height", self.settings.height)
        if self.settings.exposure_time_us is not None:
            # Manual exposure: turn off auto first, then set value
            self._nm_set_enum(node_map, "ExposureAuto", "Off")
            self._nm_set_float(node_map, "ExposureTime", self.settings.exposure_time_us)
            print(f"[CAMERA] Manual ExposureTime = {self.settings.exposure_time_us:.0f} µs (ExposureAuto=Off).")
        if self.settings.gain is not None:
            self._nm_set_float(node_map, "Gain", self.settings.gain)
            print(f"[CAMERA] Gain = {self.settings.gain}.")

    # ------------------------------------------------------------------
    # NodeMap helpers — use genicam.IsWritable() (correct pypylon API)
    # ------------------------------------------------------------------

    def _nm_set_enum(self, node_map, node_name: str, value: str) -> None:
        """Set an enumeration node by string value."""
        try:
            node = node_map.GetNode(node_name)
            if node and genicam.IsWritable(node):
                try:
                    node.SetValue(value)
                except Exception:
                    entry = node.GetEntryByName(value)
                    if entry and genicam.IsAvailable(entry):
                        node.SetIntValue(entry.GetValue())
        except Exception as e:
            print(f"[CAMERA] Could not set {node_name}={value}: {e}")

    def _nm_set_bool(self, node_map, node_name: str, value: bool) -> None:
        """Set a boolean node."""
        try:
            node = node_map.GetNode(node_name)
            if node and genicam.IsWritable(node):
                node.SetValue(value)
        except Exception as e:
            print(f"[CAMERA] Could not set {node_name}={value}: {e}")

    def _nm_set_int(self, node_map, node_name: str, value: int) -> None:
        """Set an integer node, clamped and aligned to valid range."""
        try:
            node = node_map.GetNode(node_name)
            if node and genicam.IsWritable(node):
                minimum = node.GetMin()
                maximum = node.GetMax()
                increment = node.GetInc() if node.GetInc() > 0 else 1
                clamped = max(minimum, min(maximum, int(value)))
                aligned = minimum + ((clamped - minimum) // increment) * increment
                node.SetValue(aligned)
        except Exception as e:
            print(f"[CAMERA] Could not set {node_name}={value}: {e}")

    def _nm_set_float(self, node_map, node_name: str, value: float) -> None:
        """Set a float node, clamped to valid range."""
        try:
            node = node_map.GetNode(node_name)
            if node and genicam.IsWritable(node):
                minimum = node.GetMin()
                maximum = node.GetMax()
                node.SetValue(max(minimum, min(maximum, float(value))))
        except Exception as e:
            print(f"[CAMERA] Could not set {node_name}={value}: {e}")

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
#test