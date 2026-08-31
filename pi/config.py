# config.py
# ============================================================================
# Pi-side tunable constants for the trash-collecting catamaran robot.
#
# Mirrors the role of arduino/robot/src/Config.h on the Arduino side.
# Every numeric value used by the Pi logic lives here as a named
# constant — never as a hardcoded literal inside logic code (see
# docs/agent_rules.md §3).
#
# Source of truth ordering (per README.md):
#   - docs/technical_spec.md §6 for the initial tunable defaults below
#   - docs/design_spec.md for protocol and behavior
#   - docs/model_asset_info.md for the detection model
#
# Values marked TUNABLE DEFAULT below are starting points expected to
# be adjusted after real hardware testing — that is the intended
# workflow, not a sign anything here is "wrong."
# ============================================================================

import os

# ----------------------------------------------------------------------------
# Paths
# ----------------------------------------------------------------------------

# Repository layout assumes this file lives at pi/config.py. The
# detection model is at pi/models/best.pt (see docs/repo_structure.md).
# We resolve the model path relative to THIS file, not relative to the
# process CWD, so the program works regardless of how it is launched
# (systemd unit, ssh session, VS Code run config, etc.).
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_THIS_DIR, "models", "best.pt")

# Runtime output (CSV per-cycle + JSON run summary). See logger.py. Not
# committed to version control per docs/repo_structure.md.
LOGS_DIR = os.path.join(_THIS_DIR, "..", "logs")

# ----------------------------------------------------------------------------
# Serial link to the Arduino (FIXED — must match docs/technical_spec.md §2
# and arduino/robot/src/Config.h)
# ----------------------------------------------------------------------------

SERIAL_BAUD_RATE = 115200
# Pi <-> Arduino USB serial device. On Raspberry Pi OS this is typically
# /dev/ttyACM0. If you see "permission denied," add your user to the
# "dialout" group, or set up a udev rule for the Arduino.
SERIAL_PORT = "/dev/ttyACM0"
# Seconds to wait for the serial port to appear at startup (e.g. while
# the Arduino is finishing its bootloader/reset).
SERIAL_OPEN_TIMEOUT_S = 5.0

# ----------------------------------------------------------------------------
# Arduino boot / ESC arming timing
# ----------------------------------------------------------------------------

# Arduino UNO boards auto-reset when a serial connection is opened
# (the DTR toggle on connect triggers a hardware reset). That means
# SerialLink.open() effectively restarts the Arduino's setup() from
# scratch, including its bootloader window and its own
# ESC_ARM_DELAY_MS hold. The Pi cannot know exactly when the
# Arduino's clock started, so it has to wait conservatively for the
# worst case BEFORE sending the first real CMD.
#
# This value must cover, with a small buffer:
#   1. The Arduino UNO bootloader window (typically ~1-2 s after
#      reset, variable by board revision).
#   2. The Arduino's setup() arming hold. The Arduino's
#      ESC_ARM_DELAY_MS is 2000 (see arduino/robot/src/Config.h §3).
#   3. A safety margin so a slow Pi or busy USB doesn't cost us the
#      boot wait.
#
# We express the value in pure seconds (the Arduino's constant lives
# in its own Config.h, not in this file) and add a generous 1.5 s
# buffer above the 2 s arming hold so a slower-than-expected Pi
# doesn't miss the arming window. If the bench check ever shows the
# Arduino isn't ready after this sleep, RAISE this constant — never
# lower it. Lowering it would race against the Arduino's setup().
ARDUINO_BOOT_WAIT_S = 2.0 + 1.5  # ESC_ARM_DELAY_MS (2000 ms) + 1.5 s margin

# ----------------------------------------------------------------------------
# Command cadence (FIXED protocol cadence, TUNABLE target rate)
# ----------------------------------------------------------------------------

# Target command rate to the Arduino (~10 Hz). The actual rate is
# capped by the YOLO loop, which runs at ~5-10 Hz on a Pi 4 with
# YOLOv5n. CMD_SEND_INTERVAL_MS is the *minimum* gap between CMDs —
# the main loop will not send faster than this, but may send slower
# when YOLO is the bottleneck.
CMD_SEND_INTERVAL_MS = 100  # 10 Hz

# ----------------------------------------------------------------------------
# Camera (TUNABLE DEFAULT — see docs/technical_spec.md §6 and
# docs/model_asset_info.md)
# ----------------------------------------------------------------------------

# Input resolution the YOLOv5n model expects. Matches the camera
# capture resolution on purpose, so we do not pay for a software resize
# on every frame. The model_asset_info doc notes that the AutoShape
# wrapper letterboxes whatever it receives, so an exact match is "good
# practice but not strictly required" — we do it anyway because it's
# free at capture time.
CAMERA_RESOLUTION = (640, 640)  # (width, height) in pixels
CAMERA_FRAMERATE  = 30          # Picamera2 requested fps; actual fps
                                # may be lower under load, which is
                                # fine — we sample whatever YOLO
                                # returns.

# ----------------------------------------------------------------------------
# Detection model (FIXED contract — see docs/model_asset_info.md)
# ----------------------------------------------------------------------------

# Single-class detector (class: "trash"). All detections are treated
# identically — no class-index-to-name mapping is needed or used.
MODEL_CLASS_NAME = "trash"

# Minimum confidence to treat a detection as valid. Detections below
# this are dropped before the decision logic sees them. TUNABLE.
YOLO_CONFIDENCE_THRESHOLD = 0.5

# ----------------------------------------------------------------------------
# SEARCH state (TUNABLE DEFAULT — docs/technical_spec.md §6)
# ----------------------------------------------------------------------------

SEARCH_CRUISE_SPEED_PCT        = 40   # Forward speed while patrolling
SEARCH_NO_DETECTION_TIMEOUT_S   = 5    # Seconds with no detection -> scan turn
SEARCH_SCAN_TURN_SPEED_PCT      = 30   # Active-motor speed during a scan pivot
SEARCH_SCAN_TURN_DURATION_S    = 2    # How long the scan pivot lasts

# ----------------------------------------------------------------------------
# APPROACH state (TUNABLE DEFAULT — docs/technical_spec.md §6)
# ----------------------------------------------------------------------------

# Rudder = RUDDER_CENTER_DEG + APPROACH_RUDDER_KP * horizontal_offset
# where horizontal_offset is the bbox center's signed distance from
# the frame's horizontal center, normalized to [-1.0, +1.0].
#
# Sign convention (CHOSEN 2026-07-10, pending hardware bench check):
#   - positive horizontal_offset  = target is to the RIGHT of frame center
#   - positive rudder deflection  = larger angle (toward RUDDER_MAX_DEG)
#   - This MUST match the Arduino-side RUDDER_MIN/MAX convention
#     (currently assumed: RUDDER_MIN=LEFT, RUDDER_MAX=RIGHT, NOT YET
#     VERIFIED — see Config.h). If the bench check flips the rudder
#     convention, flip the sign of APPROACH_RUDDER_KP here too.
#   - With Kp = 30, the output range exactly fills the 60..120 deg
#     clamp at the bbox-edge extremes — see docs/technical_spec.md §6.
APPROACH_RUDDER_KP = 30.0

# Speed scaling: linear map from bbox size (proxy for proximity) to
# motor percent. Small bbox (far) -> APPROACH_MAX_SPEED_PCT; bbox near
# the COLLECT threshold (very close) -> APPROACH_MIN_SPEED_PCT. The
# mapping uses bbox_height_ratio (bbox height / frame height) as the
# independent variable, monotonically mapped onto [MAX, MIN] across
# [0.0, COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD].
APPROACH_MAX_SPEED_PCT = 50
APPROACH_MIN_SPEED_PCT = 20

# If the detection briefly disappears mid-APPROACH, wait this long
# before reverting to SEARCH. Absorbs single-frame dropouts from YOLO
# jitter without flapping the state machine.
DETECTION_LOST_GRACE_PERIOD_S = 1.5

# ----------------------------------------------------------------------------
# COLLECT state (TUNABLE DEFAULT — docs/technical_spec.md §6)
# ----------------------------------------------------------------------------

# Bbox height (as a fraction of frame height) above which the Pi
# transitions from APPROACH to COLLECT.
COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD = 0.6
COLLECT_SPEED_PCT                   = 25   # Straight-line speed during COLLECT
COLLECT_DURATION_S                  = 3     # Hold COLLECT this long, then SEARCH

# ----------------------------------------------------------------------------
# Rudder constants (must match Arduino's RUDDER_MIN/MAX/CENTER_DEG in
# Config.h)
# ----------------------------------------------------------------------------

# Pi never sends a rudder angle outside this range; SerialProtocol on
# the Arduino also clamps, but clamping on the Pi side avoids
# transmitting values we know will be rejected.
RUDDER_MIN_DEG    = 60
RUDDER_MAX_DEG    = 120
RUDDER_CENTER_DEG = 90

# ----------------------------------------------------------------------------
# Status (debug telemetry) cadence
# ----------------------------------------------------------------------------

# We do not currently consume Arduino -> Pi STATUS on the Pi side (the
# Arduino emits it for any external log/dashboard). The Arduino emits
# at ~2 Hz internally. This constant is reserved in case we later
# want a Pi-side listener thread; it has no effect on the main loop
# for now.
STATUS_LOG_HZ = 2
