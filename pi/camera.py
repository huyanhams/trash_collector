"""camera.py

Thin Picamera2 capture wrapper for the trash-collecting catamaran.

Public surface
--------------

    cam = Camera()
    cam.open()                # configure and start the camera
    frame = cam.capture()     # block until next frame, return as np.ndarray
    cam.close()               # stop and release

Design notes
------------

* One Camera instance per program. Owns the underlying Picamera2
  object for the lifetime of the program.

* `capture()` returns an RGB numpy array at CAMERA_RESOLUTION (the
  resolution the YOLO model expects). Picamera2 natively produces
  RGB888, which is exactly what the YOLOv5n model expects — we pass
  frames through unmodified, with no color conversion. There is no
  other consumer of these frames in this repository (the separate
  dashboard project is out of scope per docs/agent_rules.md and
  handles its own color handling). This avoids the classic
  silent-failure mode of "the model trained on RGB but receives BGR
  and produces subtly wrong detections with no error."

* No I/O on import. Picamera2 is imported lazily inside `open()` so
  that other modules (e.g. the dashboard project, or unit tests) can
  `import camera` on a machine without a camera attached.

* No global state. Everything is on the instance.

This module does NOT do detection, does NOT send commands, and does
NOT touch the serial port. Those are separate modules' jobs.
"""

from __future__ import annotations

import numpy as np

import config


class Camera:
    """Wrapper around Picamera2 for the robot's forward-facing camera.

    Use:
        cam = Camera()
        cam.open()
        try:
            frame = cam.capture()   # np.ndarray, BGR, shape (H, W, 3)
            ...
        finally:
            cam.close()
    """

    def __init__(self) -> None:
        self._picam = None
        self._width = config.CAMERA_RESOLUTION[0]
        self._height = config.CAMERA_RESOLUTION[1]

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        """Configure and start the camera. Idempotent: calling open() on
        an already-open Camera is a no-op (returns silently).
        """
        if self._picam is not None:
            return

        # Lazy import so this module can be imported on machines without
        # Picamera2 / libcamera installed (e.g. the dashboard, or a
        # developer laptop for unit tests).
        from picamera2 import Picamera2  # type: ignore

        self._picam = Picamera2()

        # Build a configuration that matches the resolution and
        # framerate from config.py. We use the main RGB stream only;
        # no separate low-res stream, since the rest of the pipeline
        # only consumes full-res frames.
        video_config = self._picam.create_video_configuration(
            main={"size": (self._width, self._height),
                  "format": "RGB888"},
            controls={"FrameRate": float(config.CAMERA_FRAMERATE)},
        )
        self._picam.configure(video_config)
        self._picam.start()

    def close(self) -> None:
        """Stop the camera and release the underlying Picamera2 object.
        Safe to call multiple times; safe to call if open() was never
        called.
        """
        if self._picam is None:
            return
        try:
            self._picam.stop()
        except Exception:
            # Don't let a teardown error mask whatever the caller is
            # actually trying to do (e.g. logging a real exception).
            pass
        try:
            self._picam.close()
        except Exception:
            pass
        self._picam = None

    # -- capture -----------------------------------------------------------

    def capture(self) -> np.ndarray:
        """Block until the next frame is available and return it as an
        RGB numpy array with shape (H, W, 3), dtype uint8.

        Picamera2 returns RGB natively, which is exactly what the
        YOLOv5n model expects. We pass the frame through unmodified
        — no color conversion, no channel flip. This is deliberate
        (see the module docstring): a wrong color conversion here is
        a silent-failure mode where detections look plausible but
        are subtly incorrect.
        """
        if self._picam is None:
            raise RuntimeError("Camera.capture() called before Camera.open()")

        # Picamera2's capture_array() already returns RGB at the
        # resolution and format we requested in open(). No conversion.
        return self._picam.capture_array()  # shape (H, W, 3), uint8, RGB

    # -- introspection -----------------------------------------------------

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def is_open(self) -> bool:
        return self._picam is not None
