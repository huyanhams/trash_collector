"""logger.py

Per-cycle run logging for the Pi side.

Two outputs, one Logger instance:

1. A CSV file: one row per loop iteration, append-only. Columns are
   the small set of fields needed to reconstruct what the robot
   did each cycle (state, command, detection summary, override).

2. A JSON run summary: a single object written at the end of the
   run (or on demand via `write_run_summary()`). Contains the
   start time, the run duration, the number of cycles, and a few
   useful aggregates (per-state cycle counts).

The CSV is the "high-frequency" stream: one row per cycle, written
as data arrives, never held in memory. The JSON is the "low-frequency"
stream: written at shutdown, summarizes the whole run.

File layout
-----------

The default output directory is `config.LOGS_DIR` (one level above
`pi/`, i.e. `<repo>/logs/`). Filenames include a UTC timestamp so
successive runs don't clobber each other:

    logs/run_2026-07-10T10-24-37Z.csv
    logs/run_2026-07-10T10-24-37Z_summary.json

We never raise on I/O failure mid-run: a transient disk-full or
permission error logs to stderr and continues. Losing a few rows
of telemetry is better than crashing the main loop and losing
control of the boat. The JSON summary at the end is best-effort;
if it fails, the CSV is still useful.

This module does NOT touch the camera, detector, decision, or
serial modules. It only consumes the dicts those modules produce.
"""

from __future__ import annotations

import csv
import datetime
import json
import os
import sys
import time as _time
from typing import Any, Dict, List, Optional

import config


# CSV column order. Defined as a module-level constant so the
# per-row writer and the run-summary writer agree on field names.
# Keep this list aligned with `_csv_row` below.
_CSV_COLUMNS = [
    "t_s",                  # seconds since run start (monotonic)
    "state",                # SEARCH | APPROACH | COLLECT
    "left_motor_pct",
    "right_motor_pct",
    "rudder_angle_deg",
    "n_detections",         # count of valid detections this cycle
    "best_confidence",      # confidence of the "best" detection, or "" if none
    "best_class",           # "trash" if there was a detection, else ""
    "best_cx_norm",         # best bbox center x, normalized to [-1, +1]
    "best_cy_norm",         # best bbox center y, normalized to [-1, +1]
    "best_h_norm",          # best bbox height, normalized to frame height
    "override_active",      # bool from Arduino STATUS, or "" if no status received
    "dist_front_cm",
    "dist_left_cm",
    "dist_right_cm",
]


class Logger:
    """Per-cycle CSV + per-run JSON logger.

    Use:
        log = Logger()
        log.open()
        try:
            for cycle in run_loop():
                log.write_cycle(cycle)
        finally:
            log.close()  # writes the JSON run summary

    `cycle` is a dict matching the keys in _CSV_COLUMNS (missing
    keys are written as empty strings; extra keys are ignored).
    """

    def __init__(self) -> None:
        self._csv_file = None
        self._csv_writer = None
        self._run_start_monotonic: Optional[float] = None
        self._run_start_wall: Optional[datetime.datetime] = None
        self._run_csv_path: Optional[str] = None
        self._run_summary_path: Optional[str] = None

        # Per-state cycle counts, accumulated in memory and written
        # to the JSON summary at close(). Bounded by the number of
        # states (3), so the memory cost is trivial.
        self._state_counts: Dict[str, int] = {}
        self._n_cycles: int = 0

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        """Open the CSV file for this run. Idempotent: a second call
        is a no-op (returns silently).
        """
        if self._csv_file is not None:
            return

        os.makedirs(config.LOGS_DIR, exist_ok=True)

        # Filenames include a UTC timestamp so successive runs don't
        # clobber each other. We use a Z suffix and a colon-free
        # timestamp (colons are not safe in some filesystems) for
        # portability. Example: run_2026-07-10T10-24-37Z.csv
        ts = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H-%M-%SZ"
        )
        self._run_csv_path = os.path.join(
            config.LOGS_DIR, f"run_{ts}.csv"
        )
        self._run_summary_path = os.path.join(
            config.LOGS_DIR, f"run_{ts}_summary.json"
        )

        # newline="" is the csv module's recommended mode on all
        # platforms; it prevents the writer from emitting extra
        # "\r" on Windows when used with the csv module.
        try:
            self._csv_file = open(self._run_csv_path, "w", newline="", encoding="utf-8")
        except OSError as e:
            # If we can't open the CSV, fall back to /dev/null (or
            # NUL on Windows) and log a warning. The run still
            # proceeds — losing telemetry is better than crashing.
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
        """Flush the CSV, close it, and write the JSON run summary.
        Safe to call multiple times; safe to call if open() was
        never called or failed.
        """
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

        # Write the run summary regardless of CSV success — if the
        # CSV failed, we still have in-memory state counts and we
        # can produce a partial summary that says "0 cycles logged"
        # or similar. The summary is best-effort.
        self.write_run_summary()
        self._run_start_monotonic = None
        self._run_start_wall = None

    # -- per-cycle write ---------------------------------------------------

    def write_cycle(self, cycle: Dict[str, Any]) -> None:
        """Write one cycle's worth of telemetry as a CSV row.

        Missing keys are written as empty strings. Extra keys are
        ignored. I/O errors are caught and logged to stderr — the
        main loop should never crash because of a logger failure.
        """
        if self._csv_writer is None:
            return

        row = _csv_row(cycle)
        # _csv_row returns a list of strings aligned with
        # _CSV_COLUMNS, so this is a one-line write.
        try:
            self._csv_writer.writerow(row)
        except OSError as e:
            print(f"[logger] WARNING: write failed: {e}", file=sys.stderr)
            return

        # Update the in-memory aggregates for the run summary.
        self._n_cycles += 1
        state = cycle.get("state", "") or ""
        if state:
            self._state_counts[state] = self._state_counts.get(state, 0) + 1

        # Flush every cycle. The CSV is the operator's only
        # record of what the boat did; buffering it would mean
        # losing recent rows on a hard crash. Per-cycle flushes
        # are cheap (a few hundred bytes per row).
        try:
            self._csv_file.flush()
        except OSError as e:
            print(f"[logger] WARNING: flush failed: {e}", file=sys.stderr)

    # -- run summary -------------------------------------------------------

    def write_run_summary(self) -> None:
        """Write the JSON run summary to disk. Best-effort: I/O
        failures are logged to stderr and otherwise ignored.
        """
        if self._run_summary_path is None:
            return

        # Compute the run duration. If we have a start time, use
        # it; otherwise report 0.0. We use monotonic time for the
        # duration (immune to wall-clock adjustments) and wall-
        # clock time for the human-readable start/end.
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

    # -- introspection -----------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._csv_file is not None

    @property
    def n_cycles(self) -> int:
        return self._n_cycles

    @property
    def run_csv_path(self) -> Optional[str]:
        return self._run_csv_path


# ----------------------------------------------------------------------------
# CSV row builder
# ----------------------------------------------------------------------------


def _csv_row(cycle: Dict[str, Any]) -> List[str]:
    """Build a CSV row from a cycle dict, aligned with _CSV_COLUMNS.
    Missing values become empty strings (which the csv module writes
    as empty fields, not the literal "None" or "null").
    """
    def s(key: str) -> str:
        v = cycle.get(key, "")
        if v is None or v is False:
            # False is the natural "no override" value; we still
            # want it to be present in the CSV as 0, not blank.
            # Empty string means "absent," False means "explicit no."
            if v is False:
                return "0"
            return ""
        if v is True:
            return "1"
        return str(v)

    return [s(c) for c in _CSV_COLUMNS]


# ----------------------------------------------------------------------------
# Internal helpers
# ----------------------------------------------------------------------------


def _now_monotonic() -> float:
    """Monotonic seconds, current value. We call time.monotonic()
    every time, so the value is always current — there is no
    import-time snapshot. This is the right thing for a logger,
    which only ever exists in one process and has no purity
    requirement on its helpers.
    """
    return _time.monotonic()
