"""main.py

Orchestration entry point for the Pi side of the trash-collecting
catamaran robot. Wires together Camera, Detector, decision.step,
SerialLink, and Logger, and runs the main loop.

THIS FILE IS ORCHESTRATION ONLY. The actual behavior lives in the
paired modules:

    camera       — Picamera2 capture wrapper
    detector     — YOLOv5n single-class "trash" detector
    decision     — pure state machine (SEARCH / APPROACH / COLLECT)
    serial_link  — Pi <-> Arduino serial link
    logger       — per-cycle CSV + per-run JSON run summary

The main loop, per docs/design_spec.md §4 and §5, is roughly:

    loop:
        1. Capture a frame from the camera.
        2. Run inference -> list of detection dicts.
        3. Run one step of the state machine.
        4. Send the resulting CMD to the Arduino (rate-limited).
        5. Drain any inbound STATUS (optional, for logging).
        6. Log the cycle to CSV.
        7. Honor a target loop rate so we don't spin.

Every step here is a delegation; no behavior is implemented inline
in this file. The Arduino owns the FAILSAFE/AVOID safety chain
(see arduino/robot/src/SafetyLogic.cpp); this file only sends
intent.

Run as:

    python3 -m pi.main

or simply:

    python3 pi/main.py

from the repository root. Requires that the Arduino is plugged in
and the user has access to the serial device (typically /dev/ttyACM0
on Raspberry Pi OS; see config.SERIAL_PORT).

Press Ctrl-C to stop. Shutdown is clean: every opened resource is
closed in a finally block, the logger writes its JSON summary, and
the robot's Arduino is left in its failsafe state (no CMD within
1 s -> motors stop, rudder center).
"""

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


# ----------------------------------------------------------------------------
# Module instances (one of each, lifetime = program lifetime).
#
# We construct them at import time? No — we construct them inside
# main() so that `import main` is a no-op (lets the dashboard or a
# test script import the module without side effects).
# ----------------------------------------------------------------------------

# Target loop rate. The YOLO inference is the bottleneck (~5-10 Hz
# on Pi 4 with YOLOv5n). We don't try to push faster than the
# camera+model can deliver; the loop just naturally slows down
# under load. This constant is a ceiling, not a target.
TARGET_LOOP_HZ = 10.0


# Global flag set by SIGINT handler. The main loop checks this
# between cycles and exits cleanly.
_should_stop = False


def _on_sigint(signum, frame):
    """SIGINT handler: ask the main loop to stop at the next
    iteration boundary. We do NOT raise or call sys.exit() here —
    raising inside a signal handler is unsafe, and sys.exit()
    skips our finally blocks. The main loop polls _should_stop
    once per cycle and exits cleanly.
    """
    global _should_stop
    _should_stop = True


# ----------------------------------------------------------------------------
# Cycle construction
# ----------------------------------------------------------------------------


def _build_cycle(
    *,
    t_s: float,
    detections: List[Dict[str, Any]],
    command: Dict[str, Any],
    status: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Build the cycle dict the logger expects. Maps the outputs of
    decision.step() and serial_link.maybe_recv_status() into the
    flat column-oriented shape the logger writes to CSV.
    """
    # Detection summary: pick the same "best" rule decision.py uses
    # (highest confidence), so the logged "best_confidence" matches
    # what the state machine acted on this cycle. If there are no
    # detections, all "best_*" fields are blank / None.
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


# ----------------------------------------------------------------------------
# main()
# ----------------------------------------------------------------------------


def main() -> int:
    """Run the main loop. Returns the process exit code:
        0 on clean shutdown (Ctrl-C or normal end of run),
        1 on a fatal error that prevented startup.
    """
    global _should_stop

    # Wire SIGINT to a flag-based handler so the loop exits cleanly
    # at the next iteration boundary (not mid-iteration).
    signal.signal(signal.SIGINT, _on_sigint)

    # Construct the module instances. We do this here (not at
    # import time) so importing main is side-effect-free.
    cam = camera.Camera()
    det = detector_mod.Detector()
    link = serial_link.SerialLink()
    log = logger_mod.Logger()
    state = decision.RobotState()  # pure-function state object
    run_start_mono = time.monotonic()

    # -- Startup sequence -------------------------------------------------

    # Open the camera. If this fails, we abort — without a camera
    # there is no useful work to do.
    try:
        cam.open()
    except Exception as e:
        print(f"[main] FATAL: Camera.open() failed: {e}", file=sys.stderr)
        traceback.print_exc()
        return 1

    # Warm up the detector (load the model + run one zero-image
    # pass). This pays the ~2 s model-load cost up front so the
    # first real cycle's latency matches steady-state. If warmup
    # fails, abort — without a model the state machine has no input.
    try:
        det.warmup()
    except Exception as e:
        print(f"[main] FATAL: Detector.warmup() failed: {e}", file=sys.stderr)
        traceback.print_exc()
        cam.close()
        return 1

    # Open the serial link. If this fails, abort — without the
    # serial link the boat is uncontrollable. (We could fall back
    # to running the decision logic dry for offline debugging, but
    # that's a separate code path; for the live boat, fail fast.)
    #
    # NOTE: opening the serial port triggers an Arduino UNO
    # auto-reset (DTR toggle on connect), which restarts the
    # Arduino's setup() from scratch — including its bootloader
    # window and its own ESC_ARM_DELAY_MS hold. The Pi has no
    # way to know exactly when the Arduino's clock restarted, so
    # it sleeps a conservative ARDUINO_BOOT_WAIT_S before sending
    # the first real CMD. See pi/config.py for the derivation.
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

    # Open the logger LAST so its start time corresponds to "we're
    # actually running," not "we imported the module."
    log.open()

    print(f"[main] Run started. CSV -> {log.run_csv_path}", file=sys.stderr)

    # -- Main loop --------------------------------------------------------
    #
    # We sleep at the bottom of the loop to honor TARGET_LOOP_HZ.
    # The sleep is a best-effort ceiling; if YOLO is the bottleneck
    # the loop will naturally run slower, and the sleep will
    # collapse to zero or a tiny value. We never sleep BEFORE
    # inference — that would just add latency.
    target_period_s = 1.0 / TARGET_LOOP_HZ
    try:
        while not _should_stop:
            loop_t0 = time.monotonic()

            # 1. Capture a frame. Camera.capture() blocks until the
            #    next frame is available, so the loop rate is
            #    naturally bounded by the camera framerate too.
            try:
                frame = cam.capture()
            except Exception as e:
                # A single bad frame shouldn't kill the run. Log
                # and continue; the next frame may be fine.
                print(f"[main] WARNING: capture failed: {e}", file=sys.stderr)
                _sleep_to_period(loop_t0, target_period_s)
                continue

            # 2. Inference. A bad frame (wrong shape) returns [].
            detections = det.infer(frame)

            # 3. State machine step. Pure function over the input
            #    state + detections + the current monotonic time.
            state, command = decision.step(
                state,
                detections=detections,
                frame_width=cam.width,
                frame_height=cam.height,
                now_s=time.monotonic(),
            )

            # 4. Send the CMD. Throttled internally to
            #    CMD_SEND_INTERVAL_MS; returns False if rate-limited.
            link.send_cmd(
                int(command["left_motor_pct"]),
                int(command["right_motor_pct"]),
                int(command["rudder_angle_deg"]),
            )

            # 5. Optional inbound STATUS poll. Non-blocking; returns
            #    None if no full line is available this cycle.
            status = link.maybe_recv_status()

            # 6. Log the cycle to CSV.
            t_s = time.monotonic() - run_start_mono
            cycle = _build_cycle(
                t_s=t_s,
                detections=detections,
                command=command,
                status=status,
            )
            log.write_cycle(cycle)

            # 7. Honor the target loop rate.
            _sleep_to_period(loop_t0, target_period_s)

    finally:
        # Clean shutdown. The Arduino will FAILSAFE on its own when
        # we stop sending CMD (within FAILSAFE_TIMEOUT_MS = 1000 ms
        # per docs/technical_spec.md §2), so the boat is safe
        # before this finally block even runs.
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


# ----------------------------------------------------------------------------
# Internal helpers
# ----------------------------------------------------------------------------


def _sleep_to_period(loop_t0: float, period_s: float) -> None:
    """Sleep until `period_s` seconds have elapsed since loop_t0.
    If we're already past the deadline, return immediately (so
    the loop runs as fast as YOLO allows under heavy load).
    """
    elapsed = time.monotonic() - loop_t0
    remaining = period_s - elapsed
    if remaining > 0.0:
        time.sleep(remaining)


# Module entry point. We guard with `__name__ == "__main__"` so
# `import main` is side-effect-free (important for any future
# tooling that wants to import this file without launching a run).
if __name__ == "__main__":
    sys.exit(main())
