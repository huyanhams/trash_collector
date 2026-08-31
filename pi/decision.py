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
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    COLLECT = "COLLECT"


@dataclass
class RobotState:
    state: State = State.SEARCH
    last_detection_time_s: float = field(default=-math.inf)
    collect_end_time_s: float = 0.0
    scan_turn_remaining_s: Optional[float] = None
    last_now_s: float = 0.0



def step(
    state: RobotState,
    detections: List[Dict[str, Any]],
    frame_width: int,
    frame_height: int,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    if frame_width <= 0 or frame_height <= 0:
        return state, _stop_command(state)

    if state.scan_turn_remaining_s is not None:
        state.scan_turn_remaining_s = max(
            0.0, state.scan_turn_remaining_s - _dt_estimate(state, now_s)
        )

    state.last_now_s = now_s

    best = _select_best_detection(detections)
    if best is not None:
        state.last_detection_time_s = now_s

    if state.state == State.SEARCH:
        return _step_search(state, best, frame_width, frame_height, now_s)
    if state.state == State.APPROACH:
        return _step_approach(state, best, frame_width, frame_height, now_s)
    return _step_collect(state, now_s)




def _step_search(
    state: RobotState,
    best: Optional[Dict[str, Any]],
    frame_width: int,
    frame_height: int,
    now_s: float,
) -> Tuple[RobotState, Dict[str, Any]]:
    if best is not None:
        state.state = State.APPROACH
        state.scan_turn_remaining_s = None
        return _step_approach(state, best, frame_width, frame_height, now_s)

    if state.scan_turn_remaining_s is not None and state.scan_turn_remaining_s > 0:
        return state, _scan_turn_command()


    if (now_s - state.last_detection_time_s) >= config.SEARCH_NO_DETECTION_TIMEOUT_S:
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
        if (now_s - state.last_detection_time_s) >= config.DETECTION_LOST_GRACE_PERIOD_S:
            state.state = State.SEARCH
            return _step_search(state, None, frame_width, frame_height, now_s)
        return state, _approach_decay_command(state, frame_height, best=None)

    h_offset = _horizontal_offset(best, frame_width)  # in [-1, +1]
    rudder = _rudder_from_offset(h_offset)
    rudder = _clamp_rudder(rudder)

    bbox_h_ratio = _bbox_height_ratio(best, frame_height)  # in [0, +inf)
    speed = _approach_speed_from_ratio(bbox_h_ratio)

    if bbox_h_ratio >= config.COLLECT_BBOX_HEIGHT_RATIO_THRESHOLD:
        state.state = State.COLLECT
        state.collect_end_time_s = now_s + float(config.COLLECT_DURATION_S)
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
        state.state = State.SEARCH
        return state, _search_cruise_command()
    return state, _collect_command()



def _select_best_detection(
    detections: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if not detections:
        return None
    return max(detections, key=lambda d: float(d.get("confidence", 0.0)))


def _horizontal_offset(detection: Dict[str, Any], frame_width: int) -> float:
    cx = 0.5 * (float(detection["xmin"]) + float(detection["xmax"]))
    # Normalized offset in [-1, +1]:
    return (cx - 0.5 * frame_width) / (0.5 * frame_width)


def _bbox_height_ratio(detection: Dict[str, Any], frame_height: int) -> float:
    h = float(detection["ymax"]) - float(detection["ymin"])
    return h / float(frame_height)


def _rudder_from_offset(h_offset: float) -> float:
    return float(config.RUDDER_CENTER_DEG) + float(config.APPROACH_RUDDER_KP) * h_offset


def _clamp_rudder(angle: float) -> int:
    return int(max(config.RUDDER_MIN_DEG, min(config.RUDDER_MAX_DEG, round(angle))))


def _approach_speed_from_ratio(bbox_h_ratio: float) -> int:
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
    _ = frame_height
    _ = best
    return {
        "state": State.APPROACH.value,
        "left_motor_pct": int(config.APPROACH_MIN_SPEED_PCT),
        "right_motor_pct": int(config.APPROACH_MIN_SPEED_PCT),
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }



def _search_cruise_command() -> Dict[str, Any]:
    return {
        "state": State.SEARCH.value,
        "left_motor_pct": int(config.SEARCH_CRUISE_SPEED_PCT),
        "right_motor_pct": int(config.SEARCH_CRUISE_SPEED_PCT),
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }


def _scan_turn_command() -> Dict[str, Any]:
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
    return {
        "state": state.state.value,
        "left_motor_pct": 0,
        "right_motor_pct": 0,
        "rudder_angle_deg": int(config.RUDDER_CENTER_DEG),
    }



def _dt_estimate(state: RobotState, now_s: float) -> float:
    prev = state.last_now_s
    if prev == 0.0:
        return 0.0
    return max(0.0, now_s - prev)
