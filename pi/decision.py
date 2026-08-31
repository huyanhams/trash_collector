"""decision.py

Pure decision logic for the Pi side. The single source of truth for
the SEARCH / APPROACH / COLLECT state machine on the robot.

This module is a PURE FUNCTION with no I/O. It does not import torch,
does not import the camera, does not touch the serial port, and does
not read the clock. Time-dependent logic (durations, timeouts) is
driven by the caller-supplied `now_s` parameter, so decision.py is
trivially testable on recorded data and trivially reusable by the
separate dashboard project (see docs/agent_rules.md §6 and
docs/repo_structure.md for the rationale).

Public surface
--------------

    state = RobotState()                         # initial state
    state, command = step(state, detections, frame_width, frame_height,
                          now_s)

    # `command` is a plain dict:
    #   {
    #     "left_motor_pct":  int (0..100),
    #     "right_motor_pct": int (0..100),
    #     "rudder_angle_deg": int (RUDDER_MIN_DEG..RUDDER_MAX_DEG),
    #     "state":            "SEARCH" | "APPROACH" | "COLLECT",
    #   }

Design notes
------------

* The internal state (current phase, time-since-last-detection, the
  scan-turn / COLLECT-end timers) lives in the `RobotState`
  dataclass. The caller owns one instance and threads it through
  `step()` each loop. This keeps `step()` a pure function (no
  module-level state, no globals, no singletons) and makes it easy
  to replay or unit-test.

* "Best" detection rule (CHOSEN 2026-07-10): the detection with the
  highest confidence among all detections above the
  YOLO_CONFIDENCE_THRESHOLD (which the caller has already filtered
  for). If the caller passes a pre-filtered list, this is just a
  max-by-confidence selection. See the initial design-review notes
  for the rationale.

* Horizontal offset sign convention (CHOSEN 2026-07-10, must match
  the Arduino-side rudder convention):
    - positive offset  = target is to the RIGHT of frame center
    - positive rudder deflection = larger angle (toward RUDDER_MAX_DEG)
    - APPROACH_RUDDER_KP in config.py is positive, and
      `rudder = center + Kp * offset` (clamped).
  This MUST match Config.h's RUDDER_MIN/MAX convention
  (currently assumed RUDDER_MIN=LEFT, RUDDER_MAX=RIGHT, NOT YET
  VERIFIED — see Config.h). If the bench check flips the rudder
  convention, flip the sign of APPROACH_RUDDER_KP here too.

* APPROACH speed scaling (CHOSEN 2026-07-10): linear map from
  bbox_height_ratio (bbox height / frame height) to motor percent.
  Smaller box (far) -> APPROACH_MAX_SPEED_PCT; larger box (close)
  -> APPROACH_MIN_SPEED_PCT. The mapping covers
  bbox_height_ratio in [0, COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD]:
      speed = MAX - (MAX - MIN) * (bbox_height_ratio / threshold)
  Bbox height is used (not area) because it is the cleanest proxy
  for "how close is the trash vertically" given the camera's forward
  pitch; area would mix in the less-relevant horizontal size.

* State transitions follow docs/design_spec.md §6:
    SEARCH  --(trash detected)--> APPROACH
    APPROACH--(bbox near bottom / large)--> COLLECT
    APPROACH--(detection lost for grace period)--> SEARCH
    COLLECT --(COLLECT_DURATION_S elapsed)--> SEARCH
    SEARCH  --(no detection for SEARCH_NO_DETECTION_TIMEOUT_S)--> scan-turn sub-state
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import config


# ----------------------------------------------------------------------------
# State enum and RobotState dataclass
# ----------------------------------------------------------------------------


class State(str, Enum):
    """The Pi-owned intent states. Values are strings (not ints) so
    they serialize cleanly to JSON for logging and so the wire `state`
    field in the command dict is human-readable.
    """
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    COLLECT = "COLLECT"


# Sub-state of SEARCH: when we've gone SEARCH_NO_DETECTION_TIMEOUT_S
# without seeing anything, we initiate a scan turn (a brief pivot in
# place) before resuming normal cruise. This is a sub-state of SEARCH,
# not a new top-level state, because the actuation during the scan
# turn is part of "searching" — we just do it with a different motion
# profile for a short time. We track it via the time-remaining field
# on RobotState (see below) rather than as a separate enum, so the
# state machine remains a single dispatch.
#
#   _scan_turn_remaining_s is None       -> not in a scan turn
#   _scan_turn_remaining_s is a float>=0 -> scan turn in progress


@dataclass
class RobotState:
    """The Pi-side internal state. Caller-owned, threaded through
    `step()` on every loop iteration.

    Fields:
        state:                  current top-level state (SEARCH/APPROACH/COLLECT)
        last_detection_time_s:  `now_s` value at the most recent frame that
                                had at least one valid detection. Used to
                                drive DETECTION_LOST_GRACE_PERIOD_S in
                                APPROACH. Initialized to -infinity so
                                the very first frame (which will almost
                                certainly not detect anything) doesn't
                                look like "stale detection" to the
                                grace-period check.
        collect_end_time_s:     `now_s` at which to leave COLLECT and
                                return to SEARCH. Set when entering
                                COLLECT. Meaningful only when state==COLLECT.
        scan_turn_remaining_s:  None when not in a scan turn; otherwise
                                the seconds left in the current scan
                                pivot. Counted down each loop by the
                                estimated dt between calls. When it
                                reaches 0, the scan turn is over and
                                normal SEARCH cruise resumes.
        last_now_s:             `now_s` value at the previous `step()`
                                call, used to estimate the elapsed
                                time between calls for the scan-turn
                                countdown. 0.0 on the first call (no
                                previous timestamp), which makes the
                                first dt estimate 0 — the safe
                                default (don't artificially advance
                                the timer on the first iteration).
    """
    state: State = State.SEARCH
    last_detection_time_s: float = field(default=-math.inf)
    collect_end_time_s: float = 0.0
    scan_turn_remaining_s: Optional[float] = None
    last_now_s: float = 0.0


# ----------------------------------------------------------------------------
# Top-level entry point
# ----------------------------------------------------------------------------


def step(
    state: RobotState,
    detections: List[Dict[str, Any]],
    frame_width: int,
    frame_height: int,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    """One iteration of the decision state machine.

    Args:
        state:           The caller's current RobotState. Mutated in
                         place AND returned (Python's dataclass
                         semantics), so callers may use either style.
        detections:      Pre-filtered list of detection dicts from
                         Detector.infer(). The caller is responsible
                         for confidence filtering — this function
                         only does the "best detection" selection
                         and bbox geometry. (Passing [] means "no
                         valid detections this frame.")
        frame_width:     Width of the frame, in pixels. Must be > 0.
        frame_height:    Height of the frame, in pixels. Must be > 0.
        now_s:           Current monotonic time in seconds. Caller-
                         supplied (this module does not call
                         time.time() etc.) so the function is pure
                         and testable.

    Returns:
        (state, command) where state is the (possibly mutated) input
        and command is a dict with the four fields documented in the
        module docstring.
    """
    if frame_width <= 0 or frame_height <= 0:
        # Defensive: a zero-sized frame would make bbox ratios
        # divide-by-zero. Return a safe stop command and don't
        # advance any timers.
        return state, _stop_command(state)

    # Always advance scan-turn countdown, regardless of state, so
    # the timer is consistent with the wall clock.
    if state.scan_turn_remaining_s is not None:
        state.scan_turn_remaining_s = max(
            0.0, state.scan_turn_remaining_s - _dt_estimate(state, now_s)
        )
    # Update the dt baseline for the next call. We do this AFTER the
    # countdown so the first call's dt is 0.0 (we have no prior
    # timestamp), which is the safe default — never advance the
    # timer on the first iteration.
    state.last_now_s = now_s

    best = _select_best_detection(detections)
    if best is not None:
        state.last_detection_time_s = now_s

    # Dispatch by current state. Each branch may transition and
    # return its own command.
    if state.state == State.SEARCH:
        return _step_search(state, best, frame_width, frame_height, now_s)
    if state.state == State.APPROACH:
        return _step_approach(state, best, frame_width, frame_height, now_s)
    # COLLECT
    return _step_collect(state, now_s)


# ----------------------------------------------------------------------------
# Per-state logic
# ----------------------------------------------------------------------------


def _step_search(
    state: RobotState,
    best: Optional[Dict[str, Any]],
    frame_width: int,
    frame_height: int,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    if best is not None:
        # Trash found — leave SEARCH for APPROACH. Reset scan-turn
        # state since we're no longer interested in pivoting to
        # look for new things.
        state.state = State.APPROACH
        state.scan_turn_remaining_s = None
        # Recurse into APPROACH for the actual command, so the
        # transition is handled in a single place.
        return _step_approach(state, best, frame_width, frame_height, now_s)

    # No detection. Two sub-cases:
    if state.scan_turn_remaining_s is not None and state.scan_turn_remaining_s > 0:
        # Mid-scan-turn: pivot using differential thrust + full
        # rudder deflection. We always pivot the same direction
        # (left) for simplicity — alternating adds complexity
        # without obvious benefit for a pool-test prototype.
        # Direction can be flipped in post-pool-tuning if needed.
        return state, _scan_turn_command()

    # Normal cruise: forward at SEARCH_CRUISE_SPEED_PCT, rudders
    # centered. We also need to decide whether to START a scan
    # turn. The trigger is: it's been SEARCH_NO_DETECTION_TIMEOUT_S
    # since the last detection (or since program start).
    if (now_s - state.last_detection_time_s) >= config.SEARCH_NO_DETECTION_TIMEOUT_S:
        # Start a scan turn. Clear the "last detection" timestamp
        # so we don't immediately trigger another scan turn the
        # moment this one ends; the search has effectively been
        # "reset" by the scan.
        state.last_detection_time_s = now_s
        state.scan_turn_remaining_s = float(config.SEARCH_SCAN_TURN_DURATION_S)
        return state, _scan_turn_command()

    return state, _search_cruise_command()


def _step_approach(
    state: RobotState,
    best: Optional[Dict[str, Any]],
    frame_width: int,
    frame_height: int,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    if best is None:
        # Lost detection. Use the grace period before flipping back
        # to SEARCH, to absorb single-frame YOLO dropouts.
        if (now_s - state.last_detection_time_s) >= config.DETECTION_LOST_GRACE_PERIOD_S:
            state.state = State.SEARCH
            return _step_search(state, None, frame_width, frame_height, now_s)
        # Still inside the grace period. Hold the last approach
        # command — speed slowly decays toward MIN to be safe.
        return state, _approach_decay_command(state, frame_height, best=None)

    # We have a detection. Compute rudder from horizontal offset.
    h_offset = _horizontal_offset(best, frame_width)  # in [-1, +1]
    rudder = _rudder_from_offset(h_offset)
    rudder = _clamp_rudder(rudder)

    # Compute speed from bbox height ratio.
    bbox_h_ratio = _bbox_height_ratio(best, frame_height)  # in [0, +inf)
    speed = _approach_speed_from_ratio(bbox_h_ratio)

    # Transition to COLLECT if the bbox is "very close" (height
    # ratio at or above the threshold). The threshold is a fraction
    # of frame height — the trash now fills that much of the
    # vertical view, meaning we're nearly on top of it.
    if bbox_h_ratio >= config.COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD:
        state.state = State.COLLECT
        state.collect_end_time_s = now_s + float(config.COLLECT_DURATION_S)
        # Return the COLLECT command for this same iteration. We
        # don't try to do "one last approach frame" — the trash is
        # about to leave the bottom of the frame anyway.
        return state, _collect_command()

    return state, {
        "state": State.APPROACH.value,
        "left_motor_pct": speed,
        "right_motor_pct": speed,
        "rudder_angle_deg": rudder,
    }


def _step_collect(
    state: RobotState,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    if now_s >= state.collect_end_time_s:
        # Done collecting. Return to SEARCH and let the next frame
        # produce the SEARCH command.
        state.state = State.SEARCH
        # Do not reset last_detection_time_s here — let SEARCH
        # decide whether to scan based on the actual time since
        # the last real detection. (If the trash was just collected,
        # there will likely be no detection immediately, and the
        # SEARCH timeout will fire normally.)
        return state, _search_cruise_command()
    return state, _collect_command()


# ----------------------------------------------------------------------------
# Detection selection and geometry helpers
# ----------------------------------------------------------------------------


def _select_best_detection(
    detections: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    """Return the highest-confidence detection, or None if the list
    is empty. Per the chosen rule (2026-07-10), "best" = highest
    confidence. The caller is responsible for pre-filtering by the
    confidence threshold; we don't re-filter here.
    """
    if not detections:
        return None
    return max(detections, key=lambda d: float(d.get("confidence", 0.0)))


def _horizontal_offset(detection: Dict[str, Any], frame_width: int) -> float:
    """Return the bbox center's signed horizontal offset from the
    frame's horizontal center, normalized to [-1, +1].

    Sign convention: positive = target to the right of center.
    """
    cx = 0.5 * (float(detection["xmin"]) + float(detection["xmax"]))
    # Normalized offset in [-1, +1]:
    return (cx - 0.5 * frame_width) / (0.5 * frame_width)


def _bbox_height_ratio(detection: Dict[str, Any], frame_height: int) -> float:
    """Return the bbox height as a fraction of frame height. This is
    the proximity proxy used for both speed scaling (APPROACH) and
    COLLECT-trigger threshold.
    """
    h = float(detection["ymax"]) - float(detection["ymin"])
    return h / float(frame_height)


def _rudder_from_offset(h_offset: float) -> float:
    """Proportional mapping from horizontal offset to rudder angle.

        rudder = RUDDER_CENTER_DEG + APPROACH_RUDDER_KP * h_offset

    Clamping to [RUDDER_MIN_DEG, RUDDER_MAX_DEG] is the caller's job
    (so this function stays a pure algebraic expression).
    """
    return float(config.RUDDER_CENTER_DEG) + float(config.APPROACH_RUDDER_KP) * h_offset


def _clamp_rudder(angle: float) -> int:
    return int(max(config.RUDDER_MIN_DEG, min(config.RUDDER_MAX_DEG, round(angle))))


def _approach_speed_from_ratio(bbox_h_ratio: float) -> int:
    """Linear map from bbox_height_ratio to motor percent, covering
    the range [0, COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD]:

        ratio = 0.0   (very far)  -> APPROACH_MAX_SPEED_PCT
        ratio = THRESH (very near) -> APPROACH_MIN_SPEED_PCT

    Beyond the threshold, the COLLECT state has already been
    entered, so we don't need to extrapolate.
    """
    if config.COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD <= 0:
        return int(config.APPROACH_MAX_SPEED_PCT)
    # Clamp the ratio into [0, threshold] before mapping.
    r = max(0.0, min(bbox_h_ratio, config.COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD))
    frac = r / config.COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD  # 0..1
    span = float(config.APPROACH_MAX_SPEED_PCT - config.APPROACH_MIN_SPEED_PCT)
    speed = float(config.APPROACH_MAX_SPEED_PCT) - span * frac
    return int(round(speed))


def _approach_decay_command(
    state: RobotState,
    frame_height: int,
    best: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Used when we're in APPROACH but the detection has been lost
    for less than the grace period. We hold the last computed
    approach speed/rudder, but bias speed toward MIN as a safety
    measure (slow down while uncertain). Pure-function-style: we
    don't have a "last command" cached, so we approximate by using
    the bbox-height ratio of None which yields MIN speed.
    """
    # Without "last command" state we'd have to thread another
    # field through RobotState; for the prototype we just command
    # the minimum speed and centered rudder. The Arduino's
    # motor/servo modules clamp on the way out, so this is safe.
    _ = frame_height  # not used in the simple fallback
    _ = best
    return {
        "state": State.APPROACH.value,
        "left_motor_pct": int(config.APPROACH_MIN_SPEED_PCT),
        "right_motor_pct": int(config.APPROACH_MIN_SPEED_PCT),
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }


# ----------------------------------------------------------------------------
# Command factories (one per motion profile)
# ----------------------------------------------------------------------------


def _search_cruise_command() -> Dict[str, Any]:
    return {
        "state": State.SEARCH.value,
        "left_motor_pct": int(config.SEARCH_CRUISE_SPEED_PCT),
        "right_motor_pct": int(config.SEARCH_CRUISE_SPEED_PCT),
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }


def _scan_turn_command() -> Dict[str, Any]:
    """Differential-thrust pivot + full-rudder deflection, per
    docs/design_spec.md §7 "sharp maneuvering" mode. The active
    motor is the LEFT motor (right motor at 0) and the rudder is
    at its full-RIGHT clamp — i.e. the robot pivots to the right.
    If a future tuning pass shows the robot should pivot left
    instead, swap left/right and RUDDER_MIN/RUDDER_MAX below.
    """
    return {
        "state": State.SEARCH.value,
        "left_motor_pct": int(config.SEARCH_SCAN_TURN_SPEED_PCT),
        "right_motor_pct": 0,
        "rudder_angle_deg": int(config.RUDDER_MAX_DEG),
    }


def _collect_command() -> Dict[str, Any]:
    return {
        "state": State.COLLECT.value,
        "left_motor_pct": int(config.COLLECT_SPEED_PCT),
        "right_motor_pct": int(config.COLLECT_SPEED_PCT),
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }


def _stop_command(state: RobotState) -> Dict[str, Any]:
    """Used only as a defensive fallback when the frame geometry is
    invalid. Reports the current state (so logging isn't confused)
    but commands a full stop on both motors.
    """
    return {
        "state": state.state.value,
        "left_motor_pct": 0,
        "right_motor_pct": 0,
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }


# ----------------------------------------------------------------------------
# Internal time helpers
# ----------------------------------------------------------------------------


def _dt_estimate(state: RobotState, now_s: float) -> float:
    """Estimate the elapsed time since the last `step()` call on
    this RobotState, used to count down the scan-turn timer.

    Implemented as a pure function of the input `state` and `now_s`
    — there is no module-level mutable state. Two independent
    `RobotState` instances (e.g. two simulation runs, or a dashboard
    and a live robot in the same process) cannot affect each other.

    Returns 0.0 on the first call (state.last_now_s == 0.0 by
    initialization), which is the safe default: never artificially
    advance the timer on the first iteration.
    """
    prev = state.last_now_s
    if prev == 0.0:
        return 0.0
    return max(0.0, now_s - prev)
