from __future__ import annotations

import csv
import datetime
import json
import os
import sys
import time as _time
from typing import Any, Dict, List, Optional

import config


_CSV_COLUMNS = [
    "t_s",             
    "state",           
    "left_motor_pct",
    "right_motor_pct",
    "rudder_angle_deg",
    "n_detections",    
    "best_confidence", 
    "best_class",      
    "best_cx_norm",    
    "best_cy_norm",    
    "best_h_norm",     
    "override_active", 
    "dist_front_cm",
    "dist_left_cm",
    "dist_right_cm",
]


class Logger:
    def __init__(self) -> None:
        self._csv_file = None
        self._csv_writer = None
        self._run_start_monotonic: Optional[float] = None
        self._run_start_wall: Optional[datetime.datetime] = None
        self._run_csv_path: Optional[str] = None
        self._run_summary_path: Optional[str] = None

        self._state_counts: Dict[str, int] = {}
        self._n_cycles: int = 0

    def open(self) -> None:
        """Open the CSV file for this run. Idempotent: a second call
        is a no-op (returns silently).
        """
        if self._csv_file is not None:
            return

        os.makedirs(config.LOGS_DIR, exist_ok=True)

        ts = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ"
        )
        self._run_csv_path = os.path.join(
            config.LOGS_DIR, f"run_{ts}.csv"
        )
        self._run_summary_path = os.path.join(
            config.LOGS_DIR, f"run_{ts}_summary.json"
        )

        try:
            self._csv_file = open(self._run_csv_path, "w", newline="", encoding="utf-8")
        except OSError as e:
            self._csv_file = None
            print(f"[logger] WARNING: could not open {self._run_csv_path}: {e}",
                  file=sys.stderr)
            return

        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(_CSV_COLUMNS)
        self._csv_file.flush()

        self._run_start_monotonic = _now_monotonic()
        self._run_start_wall = datetime.datetime.now(datetime.timezone.utc)
        self._state_counts = {}
        self._n_cycles = 0
    def close(self) -> None:
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
            except OSError:
                pass
            try:
                self._csv_file.close()
            except OSError:
                pass
            self._csv_file = None
            self._csv_writer = None

        self.write_run_summary()
        self._run_start_monotonic = None
        self._run_start_wall = None

    def write_cycle(self, cycle: Dict[str, Any]) -> None:
        if self._csv_writer is None:
            return

        row = _csv_row(cycle)
        try:
            self._csv_writer.writerow(row)
        except OSError as e:
            print(f"[logger] WARNING: write failed: {e}", file=sys.stderr)
            return

        self._n_cycles += 1
        state = cycle.get("state", "") or ""
        if state:
            self._state_counts[state] = self._state_counts.get(state, 0) + 1

        try:
            self._csv_file.flush()
        except OSError as e:
            print(f"[logger] WARNING: flush failed: {e}", file=sys.stderr)

    def write_run_summary(self) -> None:
        """Write the JSON run summary to disk. Best-effort: I/O
        failures are logged to stderr and otherwise ignored.
        """
        if self._run_summary_path is None:
            return

        now_monotonic = _now_monotonic()
        now_wall = datetime.datetime.now(datetime.timezone.utc)

        if self._run_start_monotonic is not None:
            duration_s = max(0.0, now_monotonic - self._run_start_monotonic)
        else:
            duration_s = 0.0

        summary = {
            "schema_version": 1,
            "csv_path": self._run_csv_path,
            "start_utc": (
                self._run_start_wall.isoformat()
                if self._run_start_wall is not None else None
            ),
            "end_utc": now_wall.isoformat(),
            "duration_s": duration_s,
            "n_cycles": self._n_cycles,
            "state_counts": dict(self._state_counts),
        }

        try:
            with open(self._run_summary_path, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2, sort_keys=True)
        except OSError as e:
            print(f"[logger] WARNING: could not write summary: {e}",
                  file=sys.stderr)


    @property
    def is_open(self) -> bool:
        return self._csv_file is not None

    @property
    def n_cycles(self) -> int:
        return self._n_cycles

    @property
    def run_csv_path(self) -> Optional[str]:
        return self._run_csv_path



def _csv_row(cycle: Dict[str, Any]) -> List[str]:
    def s(key: str) -> str:
        v = cycle.get(key, "")
        if v is None or v is False:
            if v is False:
                return "0"
            return ""
        if v is True:
            return "1"
        return str(v)

    return [s(c) for c in _CSV_COLUMNS]



def _now_monotonic() -> float:
    return _time.monotonic()
