from __future__ import annotations

import signal
import sys
import time
import traceback
from typing import Any, Dict, List, Optional

import config
import camera
import detector as detector_mod
import decision
import serial_link
import logger as logger_mod


TARGET_LOOP_HZ = 10.0


_should_stop = False


def _on_sigint(signum, frame):
    global _should_stop
    _should_stop = True


def _build_cycle(
    *,
    t_s: float,
    detections: List[Dict[str, Any]],
    command: Dict[str, Any],
    status: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    best: Optional[Dict[str, Any]] = None
    if detections:
        best = max(detections, key=lambda d: float(d.get("confidence", 0.0)))

    cycle: Dict[str, Any] = {
        "t_s": t_s,
        "state": command.get("state", ""),
        "left_motor_pct": command.get("left_motor_pct", ""),
        "right_motor_pct": command.get("right_motor_pct", ""),
        "rudder_angle_deg": command.get("rudder_angle_deg", ""),
        "n_detections": len(detections),
        "best_confidence": "",
        "best_class": "",
        "best_cx_norm": "",
        "best_cy_norm": "",
        "best_h_norm": "",
        "override_active": "",
        "dist_front_cm": "",
        "dist_left_cm": "",
        "dist_right_cm": "",
    }

    if best is not None:
        cycle["best_confidence"] = float(best.get("confidence", 0.0))
        cycle["best_class"] = best.get("class_name", config.MODEL_CLASS_NAME)
        # Compute normalized geometry. config.CAMERA_RESOLUTION is
        # (width, height); we use the configured resolution rather
        # than the live frame dimensions to keep the CSV stable
        # across resolution changes.
        frame_w, frame_h = config.CAMERA_RESOLUTION
        xmin = float(best.get("xmin", 0.0))
        xmax = float(best.get("xmax", 0.0))
        ymin = float(best.get("ymin", 0.0))
        ymax = float(best.get("ymax", 0.0))
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        cycle["best_cx_norm"] = (cx - 0.5 * frame_w) / (0.5 * frame_w)
        cycle["best_cy_norm"] = (cy - 0.5 * frame_h) / (0.5 * frame_h)
        cycle["best_h_norm"] = (ymax - ymin) / float(frame_h)

    if status is not None:
        cycle["override_active"] = bool(status.get("override_active", False))
        cycle["dist_front_cm"] = int(status.get("dist_front_cm", 0))
        cycle["dist_left_cm"]  = int(status.get("dist_left_cm", 0))
        cycle["dist_right_cm"] = int(status.get("dist_right_cm", 0))

    return cycle

def main() -> int:
    global _should_stop

    signal.signal(signal.SIGINT, _on_sigint)

    cam = camera.Camera()
    det = detector_mod.Detector()
    link = serial_link.SerialLink()
    log = logger_mod.Logger()
    state = decision.RobotState() 
    run_start_mono = time.monotonic()

    try:
        cam.open()
    except Exception as e:
        print(f"[main] FATAL: Camera.open() failed: {e}", file=sys.stderr)
        traceback.print_exc()
        return 1

    try:
        det.warmup()
    except Exception as e:
        print(f"[main] FATAL: Detector.warmup() failed: {e}", file=sys.stderr)
        traceback.print_exc()
        cam.close()
        return 1

    try:
        link.open()
    except Exception as e:
        print(f"[main] FATAL: SerialLink.open() failed: {e}", file=sys.stderr)
        traceback.print_exc()
        cam.close()
        return 1

    print(
        f"[main] Serial link open. Waiting "
        f"{config.ARDUINO_BOOT_WAIT_S:.1f} s for Arduino boot + ESC arming...",
        file=sys.stderr,
    )
    time.sleep(float(config.ARDUINO_BOOT_WAIT_S))
    print("[main] Arduino boot wait complete; entering main loop.",
          file=sys.stderr)

    log.open()

    print(f"[main] Run started. CSV -> {log.run_csv_path}", file=sys.stderr)

    target_period_s = 1.0 / TARGET_LOOP_HZ
    try:
        while not _should_stop:
            loop_t0 = time.monotonic()

            try:
                frame = cam.capture()
            except Exception as e:
                print(f"[main] WARNING: capture failed: {e}", file=sys.stderr)
                _sleep_to_period(loop_t0, target_period_s)
                continue

            detections = det.infer(frame)

            state, command = decision.step(
                state,
                detections=detections,
                frame_width=cam.width,
                frame_height=cam.height,
                now_s=time.monotonic(),
            )

            link.send_cmd(
                int(command["left_motor_pct"]),
                int(command["right_motor_pct"]),
                int(command["rudder_angle_deg"]),
            )

            status = link.maybe_recv_status()

            t_s = time.monotonic() - run_start_mono
            cycle = _build_cycle(
                t_s=t_s,
                detections=detections,
                command=command,
                status=status,
            )
            log.write_cycle(cycle)

            _sleep_to_period(loop_t0, target_period_s)

    finally:
        print("[main] Shutting down...", file=sys.stderr)
        try:
            log.close()
        except Exception as e:
            print(f"[main] WARNING: log.close() failed: {e}", file=sys.stderr)
        try:
            link.close()
        except Exception as e:
            print(f"[main] WARNING: link.close() failed: {e}", file=sys.stderr)
        try:
            cam.close()
        except Exception as e:
            print(f"[main] WARNING: cam.close() failed: {e}", file=sys.stderr)

    print(f"[main] Run ended after {log.n_cycles} cycles.", file=sys.stderr)
    return 0


def _sleep_to_period(loop_t0: float, period_s: float) -> None:
    """Sleep until `period_s` seconds have elapsed since loop_t0.
    If we're already past the deadline, return immediately (so
    the loop runs as fast as YOLO allows under heavy load).
    """
    elapsed = time.monotonic() - loop_t0
    remaining = period_s - elapsed
    if remaining > 0.0:
        time.sleep(remaining)

if __name__ == "__main__":
    sys.exit(main())
