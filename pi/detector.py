"""detector.py

YOLOv5n single-class "trash" detector for the trash-collecting catamaran.

The model file (PyTorch .pt weights) is loaded exactly once at startup
and reused for every frame. Inference results are filtered by confidence
and reduced to a small, plain-Python list of dicts — no torch tensors,
no pandas dataframes, no autograd graphs — so the downstream `decision`
module can stay framework-light and easy to test.

Public surface
--------------

    det = Detector()
    det.warmup()                       # optional: one zero-image pass
    detections = det.infer(frame_rgb)  # list[dict], see below

    # Each detection dict has exactly these keys:
    #   {
    #     "xmin": float,    # absolute pixels, in frame coordinates
    #     "ymin": float,
    #     "xmax": float,
    #     "ymax": float,
    #     "confidence": float,    # 0..1
    #     "class_name": "trash",  # always this; single-class model
    #   }

Design notes
------------

* The model is a YOLOv5n custom-weights detector (see
  docs/model_asset_info.md), loaded via `torch.hub.load` with the
  'custom' path. We rely on the AutoShape wrapper returned by that
  call to do letterbox / NMS / confidence filtering automatically;
  the wrapper's output is a Detections object that we walk to build
  our plain-dict list.

* `infer()` is the only hot-path method. It must be safe to call
  repeatedly with different frames. No state is mutated between
  calls.

* `decision.py` is the only intended consumer of `infer()`. Because
  `decision.py` is a pure function (no torch, no camera), we strip
  every torch-specific thing from the return value: pure Python
  floats, plain dicts, no tensors. This is the boundary between the
  ML side and the decision side.

* Input frames are RGB, as natively produced by Picamera2 and as
  expected by the YOLOv5 model. There is no color conversion in
  this module or in `camera.py` — getting this wrong is a
  silent-failure mode (plausible-looking but subtly wrong
  detections), so we keep the chain explicit: Picamera2 → RGB →
  detector (RGB). There is no other consumer of camera frames in
  this repository (the separate dashboard project is out of scope
  per docs/agent_rules.md and handles its own color).

* No I/O on import. torch and the model are loaded lazily inside
  the first `infer()` call (or explicitly via `warmup()`), so
  `import detector` succeeds on a machine without torch installed.
  This is the same pattern as `camera.py`.
"""

from __future__ import annotations

from typing import Any, Dict, List

import numpy as np

import config


class Detector:
    """Single-class trash detector backed by a YOLOv5n .pt file.

    Use:
        det = Detector()
        det.warmup()                      # optional
        detections = det.infer(frame)     # list of detection dicts
    """

    def __init__(self) -> None:
        self._model = None  # type: ignore[var-annotated]

    # -- lifecycle ---------------------------------------------------------

    def _load(self) -> None:
        """Lazy model load. Idempotent: a second call is a no-op.
        All torch / hub imports happen here, not at module import.
        """
        if self._model is not None:
            return

        # Imported here so the module is import-clean on machines
        # without torch (e.g. the dashboard project, or a CI runner).
        import torch  # type: ignore

        # The exact loader recipe from docs/model_asset_info.md:
        #   torch.hub.load('ultralytics/yolov5', 'custom', path=...)
        # This returns an AutoShape-wrapped model that handles
        # letterbox, NMS, and confidence filtering automatically.
        self._model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=config.MODEL_PATH,
        )

        # Apply the runtime confidence threshold from config. The
        # AutoShape wrapper exposes model.conf as the post-NMS
        # confidence threshold; setting it once is enough.
        try:
            self._model.conf = float(config.YOLO_CONFIDENCE_THRESHOLD)
        except AttributeError:
            # Some ultralytics versions name this differently. The
            # explicit per-detection filter below is a safety net.
            pass

    def warmup(self) -> None:
        """Force-load the model and run a single zero-image pass.

        Useful to pay the model-load cost at startup rather than on
        the first real frame, so the first loop iteration's latency
        matches steady-state. Safe to call multiple times.
        """
        self._load()
        if self._model is None:
            return
        # Build a tiny zero-image at the configured camera resolution
        # so the warmup allocation matches what the real frames will
        # look like.
        h, w = config.CAMERA_RESOLUTION[1], config.CAMERA_RESOLUTION[0]
        zeros = np.zeros((h, w, 3), dtype=np.uint8)
        # Discard the result; we just want the load + first-pass
        # overhead out of the way.
        _ = self._infer_raw(zeros)

    # -- inference ---------------------------------------------------------

    def infer(self, frame_rgb: np.ndarray) -> List[Dict[str, Any]]:
        """Run inference on a single RGB frame and return a list of
        detection dicts (see module docstring for the dict shape).

        Detections with confidence below YOLO_CONFIDENCE_THRESHOLD
        are filtered out here (in addition to the threshold being
        applied inside the model) — this is the safety net for
        ultralytics versions where model.conf doesn't take effect.

        Returns an empty list if nothing is detected, or if the
        input frame is empty / wrong shape. Never raises on bad
        input; returns [] instead so the main loop is never
        crashed by a single bad frame.
        """
        if frame_rgb is None or frame_rgb.size == 0:
            return []
        if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            return []

        self._load()  # idempotent
        if self._model is None:
            return []

        raw = self._infer_raw(frame_rgb)
        return self._extract_detections(raw)

    def _infer_raw(self, frame_rgb: np.ndarray):
        """Call into the AutoShape model and return the raw Detections
        object. Kept private — the public surface returns plain dicts.
        """
        # The AutoShape wrapper accepts a numpy RGB array directly.
        # We don't pass a conf argument here because we set model.conf
        # once in _load() (where supported) and we double-filter in
        # _extract_detections regardless. The model itself still
        # does its own NMS; this is just defense in depth.
        return self._model(frame_rgb)

    def _extract_detections(self, raw) -> List[Dict[str, Any]]:
        """Convert a YOLOv5 Detections object into a list of plain
        dicts, filtering by confidence.

        Why strip to plain dicts:
          - decision.py is a pure function and must not import torch.
          - the dashboard / future tools can consume plain dicts
            without a torch dependency.
          - plain dicts are JSON-serializable, which the logger
            module will appreciate.
        """
        out: List[Dict[str, Any]] = []
        if raw is None:
            return out

        # YOLOv5's Detections object exposes a .pandas().xyxy method
        # that gives a clean DataFrame of [xmin, ymin, xmax, ymax,
        # confidence, class, name] per row. We use it because it
        # handles tensor->python conversion and NMS-index slicing
        # for us. If .xyxy isn't available (older API), we fall back
        # to walking the .xyxy tensor directly.
        rows = None
        try:
            df = raw.pandas().xyxy[0]  # type: ignore[attr-defined]
            rows = df.to_dict("records")
        except Exception:
            # Fallback path: walk the raw tensor.
            try:
                rows = self._walk_xyxy_tensor(raw)
            except Exception:
                rows = []

        if not rows:
            return out

        threshold = float(config.YOLO_CONFIDENCE_THRESHOLD)
        for row in rows:
            # Normalize field names — pandas may return 'confidence'
            # or 'Confidence' depending on ultralytics version.
            conf = float(row.get("confidence", row.get("Confidence", 0.0)))
            if conf < threshold:
                continue

            # The model is single-class; we still copy whatever
            # 'name' the model produced for debugging visibility,
            # but the contract is that it will be 'trash' in
            # practice. We override to the constant anyway so the
            # downstream API is stable across ultralytics versions.
            out.append({
                "xmin": float(row.get("xmin", 0.0)),
                "ymin": float(row.get("ymin", 0.0)),
                "xmax": float(row.get("xmax", 0.0)),
                "ymax": float(row.get("ymax", 0.0)),
                "confidence": conf,
                "class_name": config.MODEL_CLASS_NAME,
            })

        return out

    def _walk_xyxy_tensor(self, raw) -> list:
        """Fallback path for older ultralytics versions without
        .pandas().xyxy. Walks raw.xyxy[0] (a tensor of shape (N, 6)
        with columns [xmin, ymin, xmax, ymax, conf, class]) and
        returns a list of dicts in the same shape as the pandas path.
        """
        rows = []
        try:
            xyxy = raw.xyxy[0]  # tensor
        except Exception:
            return rows

        # Convert tensor to a numpy array on CPU.
        try:
            arr = xyxy.cpu().numpy()
        except Exception:
            try:
                arr = xyxy.detach().cpu().numpy()
            except Exception:
                return rows

        for r in arr:
            if len(r) < 6:
                continue
            xmin, ymin, xmax, ymax, conf, cls = r[:6]
            rows.append({
                "xmin": float(xmin),
                "ymin": float(ymin),
                "xmax": float(xmax),
                "ymax": float(ymax),
                "confidence": float(conf),
                "class": int(cls),
                "name": config.MODEL_CLASS_NAME,
            })
        return rows
