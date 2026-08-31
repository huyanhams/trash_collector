"""serial_link.py

Pi-side serial communication with the Arduino UNO.

Public surface
--------------

    link = SerialLink()
    link.open()                        # open and configure the port
    link.send_cmd(left, right, rudder) # format + write a CMD line
    link.maybe_recv_status() -> dict   # optional: drain any STATUS line
    link.close()                       # close the port

Wire format (FIXED — see docs/design_spec.md §4 and the Arduino side
SerialProtocol.cpp):

    Pi  -> Arduino:  CMD,<left 0-100>,<right 0-100>,<rudder 60-120>\n
    Arduino -> Pi:   STATUS,<front>,<left>,<right>,<overrideActive>\n

Design notes
------------

* Sends are throttled to CMD_SEND_INTERVAL_MS (default 10 Hz). Calls
  to `send_cmd()` more frequent than the interval are dropped — we
  do NOT queue, because the most recent command is the right one
  to apply (the Pi is the "advisor," the Arduino is the supervisor).

* The CMD line is built deterministically from integers; we do
  not use any third-party formatting library. The line length is
  bounded (16 chars + '\n'), well under any buffer concern.

* The optional `maybe_recv_status()` is a non-blocking poll. It
  drains whatever bytes are available, looks for one newline-
  terminated STATUS line, parses it, and returns the dict. If no
  complete line is available, it returns None. We do not run a
  background thread — `main.py` polls whenever it polls other
  things (i.e. once per loop, cheap).

* The port is opened with a short retry loop (SERIAL_OPEN_TIMEOUT_S)
  to handle the case where the Arduino is still in its bootloader
  window when the Pi starts up. After open() returns successfully,
  the port is ready for use.

* No I/O on import. The pyserial import is deferred to open() so
  that `import serial_link` works on machines without pyserial
  installed (e.g. the dashboard project, or a developer laptop for
  unit tests).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

import config


# Wire format constants (kept here, not in config, because they're
# part of the protocol contract — Config.h and SerialProtocol.cpp
# on the Arduino side use the same characters).
_CMD_PREFIX = "CMD,"
_STATUS_PREFIX = "STATUS,"
_TERMINATOR = "\n"


class SerialLink:
    """Pi <-> Arduino serial link.

    Use:
        link = SerialLink()
        link.open()
        try:
            link.send_cmd(40, 40, 90)
            status = link.maybe_recv_status()
            ...
        finally:
            link.close()
    """

    def __init__(self) -> None:
        self._ser = None
        self._last_send_ms = 0
        # Reentrancy guard for the port. pyserial is not generally
        # thread-safe, and even though main.py is single-threaded,
        # this is cheap defense-in-depth.
        self._lock = threading.Lock()
        # Reusable bytearray buffer for inbound STATUS parsing.
        # Sized for the longest possible STATUS line:
        # "STATUS,200,200,200,1\n" = 21 chars; we round up to 64.
        self._rx_buf = bytearray(64)
        self._rx_len = 0

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        """Open the configured serial port. Idempotent. Retries until
        SERIAL_OPEN_TIMEOUT_S elapses, then raises the last error
        (so the operator sees the real failure mode, e.g. permission
        denied vs. device not found).
        """
        if self._ser is not None:
            return

        # Lazy import so the module is import-clean on machines
        # without pyserial (e.g. the dashboard project).
        import serial  # type: ignore

        deadline = time.monotonic() + float(config.SERIAL_OPEN_TIMEOUT_S)
        last_err: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                self._ser = serial.Serial(
                    port=config.SERIAL_PORT,
                    baudrate=config.SERIAL_BAUD_RATE,
                    # We do not enable any flow control — both ends
                    # are running locally over USB, and the
                    # PROTOCOL_TERMINATOR ('\n') is our framing.
                    rtscts=False,
                    xonxoff=False,
                    dsrdtr=False,
                    # 50 ms read timeout makes maybe_recv_status()
                    # effectively non-blocking at the syscall level.
                    # We still drive it from a poll rather than a
                    # blocking read, so the timeout is just a
                    # safety net.
                    timeout=0.05,
                )
                return
            except Exception as e:  # noqa: BLE001 — surface any open error
                last_err = e
                self._ser = None
                time.sleep(0.2)

        # Out of retries. Re-raise the last error so the operator
        # sees the real failure (device missing, permission denied,
        # busy, etc.) rather than a generic "could not open."
        assert last_err is not None
        raise last_err

    def close(self) -> None:
        """Close the serial port. Safe to call multiple times; safe
        to call if open() was never called or failed.
        """
        if self._ser is None:
            return
        try:
            with self._lock:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._rx_len = 0

    # -- outbound (Pi -> Arduino) ------------------------------------------

    def send_cmd(
        self,
        left_motor_pct: int,
        right_motor_pct: int,
        rudder_angle_deg: int,
    ) -> bool:
        """Format and send a CMD line. Returns True if a line was
        actually written, False if the call was rate-limited
        (CMD_SEND_INTERVAL_MS not yet elapsed since the last send).

        Inputs are clamped to the protocol's allowed ranges before
        being sent. A bad value is silently clamped, not rejected —
        the protocol is one-way (no acks), and clamping is always
        safer than dropping the command entirely.
        """
        if self._ser is None:
            return False

        now_ms = _now_ms()
        if (now_ms - self._last_send_ms) < int(config.CMD_SEND_INTERVAL_MS):
            return False  # rate-limited; the most recent command is
                          # still being applied by the Arduino

        # Clamp to the protocol's allowed ranges. These mirror the
        # Arduino's SerialProtocol validation, but doing it here too
        # means a misbehaving caller can never send out-of-range.
        l = _clamp(int(left_motor_pct), 0, 100)
        r = _clamp(int(right_motor_pct), 0, 100)
        rudder = _clamp(int(rudder_angle_deg),
                        config.RUDDER_MIN_DEG, config.RUDDER_MAX_DEG)

        # "CMD,40,40,90\n" — 12 chars at typical values. The Arduino
        # parser (SerialProtocol.cpp) uses strtol with explicit base
        # 10, so we send the rudder as a plain variable-width
        # integer (not zero-padded) to avoid any future risk of a
        # parser change interpreting "090" as octal. This matches
        # how the motor percentages are sent.
        line = f"{_CMD_PREFIX}{l},{r},{rudder}{_TERMINATOR}".encode("ascii")
        try:
            with self._lock:
                self._ser.write(line)
            self._last_send_ms = now_ms
            return True
        except Exception:
            # Write failed (port closed, USB glitch, etc.). Drop
            # this command. The Arduino's FAILSAFE will kick in
            # after 1 s of no valid CMD, which is the correct
            # safe response to "I can't talk to you right now."
            return False

    # -- inbound (Arduino -> Pi) -------------------------------------------

    def maybe_recv_status(self) -> Optional[Dict[str, Any]]:
        """Non-blocking poll for an inbound STATUS line. Returns a
        dict with keys "dist_front_cm", "dist_left_cm",
        "dist_right_cm", "override_active" (booleans for the last)
        if a complete line is available, or None otherwise.

        We do NOT buffer partial lines across calls — if the caller
        polls and there's no complete line, the bytes are kept in
        our internal rx buffer for the next call. If a partial
        line grows past the buffer size, we discard it (the
        Arduino will resend the next cycle).
        """
        if self._ser is None:
            return None

        try:
            with self._lock:
                # Read whatever is available, up to the buffer's
                # free space. We never block.
                n = self._ser.in_waiting
                if n <= 0:
                    return None
                chunk = self._ser.read(min(n, len(self._rx_buf) - self._rx_len))
        except Exception:
            return None

        if not chunk:
            return None

        # Append to the rx buffer, scanning for the terminator.
        for b in chunk:
            if self._rx_len >= len(self._rx_buf):
                # Buffer full with no terminator. Discard the whole
                # thing — the Arduino will resend next cycle.
                self._rx_len = 0
                break
            if b == ord(_TERMINATOR) or b == ord("\n"):
                # End of line. Try to parse whatever we have.
                line_bytes = bytes(self._rx_buf[:self._rx_len])
                self._rx_len = 0
                return _parse_status_line(line_bytes)
            self._rx_buf[self._rx_len] = b
            self._rx_len += 1

        return None

    # -- introspection -----------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    @property
    def last_send_ms(self) -> int:
        return self._last_send_ms


# ----------------------------------------------------------------------------
# Internal helpers
# ----------------------------------------------------------------------------


def _clamp(v: int, lo: int, hi: int) -> int:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _now_ms() -> int:
    """Monotonic milliseconds. We use time.monotonic() so the value
    is not affected by wall-clock adjustments during a run.
    """
    return int(time.monotonic() * 1000.0)


def _parse_status_line(line_bytes: bytes) -> Optional[Dict[str, Any]]:
    """Parse one STATUS,... line. Returns the dict on success, or
    None on any parse failure (so the main loop can keep going on
    noisy lines).

    Per the design spec and SerialProtocol.cpp, distances are
    already clamped to SENSOR_MAX_DISTANCE_CM (200) on the wire,
    so a value of 200 here is "no echo / out of range," not a real
    measurement. We do not enforce that here — the consumer of
    this dict decides how to interpret distances.
    """
    if not line_bytes:
        return None
    try:
        line = line_bytes.decode("ascii", errors="ignore").strip()
    except Exception:
        return None
    if not line.startswith(_STATUS_PREFIX):
        return None
    body = line[len(_STATUS_PREFIX):]
    parts = body.split(",")
    if len(parts) != 4:
        return None
    try:
        dist_front = int(parts[0])
        dist_left  = int(parts[1])
        dist_right = int(parts[2])
        override   = bool(int(parts[3]))
    except ValueError:
        return None
    return {
        "dist_front_cm": dist_front,
        "dist_left_cm":  dist_left,
        "dist_right_cm": dist_right,
        "override_active": override,
    }
