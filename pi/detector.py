from __future__ import annotations

from typing import Any, Dict, List

import numpy as np

import config


class Detector:

    def __init__(self) -> None:
        self._model = None  # type: ignore[var-annotated]


    def _load(self) -> None:
        if self._model is not None:
            return

        import torch  

        self._model = torch.hub.load(
            "ultralytics/yolov5",
            "custom",
            path=config.MODEL_PATH,
        )

        try:
            self._model.conf = float(config.YOLO_CONFIDENCE_THRESHOLD)
        except AttributeError:
            pass

    def warmup(self) -> None:
        self._load()
        if self._model is None:
            return
        h, w = config.CAMERA_RESOLUTION[1], config.CAMERA_RESOLUTION[0]
        zeros = np.zeros((h, w, 3), dtype=np.uint8)
        _ = self._infer_raw(zeros)


    def infer(self, frame_rgb: np.ndarray) -> List[Dict[str, Any]]:
        if frame_rgb is None or frame_rgb.size == 0:
            return []
        if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            return []

        self._load() 
        if self._model is None:
            return []

        raw = self._infer_raw(frame_rgb)
        return self._extract_detections(raw)

    def _infer_raw(self, frame_rgb: np.ndarray):
        return self._model(frame_rgb)

    def _extract_detections(self, raw) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        if raw is None:
            return out

        rows = None
        try:
            df = raw.pandas().xyxy[0]  
            rows = df.to_dict("records")
        except Exception:
            try:
                rows = self._walk_xyxy_tensor(raw)
            except Exception:
                rows = []

        if not rows:
            return out

        threshold = float(config.YOLO_CONFIDENCE_THRESHOLD)
        for row in rows:
            conf = float(row.get("confidence", row.get("Confidence", 0.0)))
            if conf < threshold:
                continue

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
        rows = []
        try:
            xyxy = raw.xyxy[0]  # tensor
        except Exception:
            return rows

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
