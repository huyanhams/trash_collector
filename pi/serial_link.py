from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

import config

_CMD_PREFIX = "CMD,"
_STATUS_PREFIX = "STATUS,"
_TERMINATOR = "\n"


class SerialLink:
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


    def open(self) -> None:
        if self._ser is not None:
            return

        import serial  

        deadline = time.monotonic() + float(config.SERIAL_OPEN_TIMEOUT_S)
        last_err: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                self._ser = serial.Serial(
                    port=config.SERIAL_PORT,
                    baudrate=config.SERIAL_BAUD_RATE,
                    rtscts=False,
                    xonxoff=False,
                    dsrdtr=False,
                    timeout=0.05,
                )
                return
            except Exception as e:
                last_err = e
                self._ser = None
                time.sleep(0.2)

        assert last_err is not None
        raise last_err

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            with self._lock:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._rx_len = 0


    def send_cmd(
        self,
        left_motor_pct: int,
        right_motor_pct: int,
        rudder_angle_deg: int,
    ) -> bool:
        if self._ser is None:
            return False

        now_ms = _now_ms()
        if (now_ms - self._last_send_ms) < int(config.CMD_SEND_INTERVAL_MS):
            return False 
        l = _clamp(int(left_motor_pct), 0, 100)
        r = _clamp(int(right_motor_pct), 0, 100)
        rudder = _clamp(int(rudder_angle_deg),
                        config.RUDDER_MIN_DEG, config.RUDDER_MAX_DEG)

        line = f"{_CMD_PREFIX}{l},{r},{rudder}{_TERMINATOR}".encode("ascii")
        try:
            with self._lock:
                self._ser.write(line)
            self._last_send_ms = now_ms
            return True
        except Exception:
            return False


    def maybe_recv_status(self) -> Optional[Dict[str, Any]]:
        if self._ser is None:
            return None

        try:
            with self._lock:
                n = self._ser.in_waiting
                if n <= 0:
                    return None
                chunk = self._ser.read(min(n, len(self._rx_buf) - self._rx_len))
        except Exception:
            return None

        if not chunk:
            return None

        for b in chunk:
            if self._rx_len >= len(self._rx_buf):
                self._rx_len = 0
                break
            if b == ord(_TERMINATOR) or b == ord("\n"):
                line_bytes = bytes(self._rx_buf[:self._rx_len])
                self._rx_len = 0
                return _parse_status_line(line_bytes)
            self._rx_buf[self._rx_len] = b
            self._rx_len += 1

        return None


    @property
    def is_open(self) -> bool:
        return self._ser is not None

    @property
    def last_send_ms(self) -> int:
        return self._last_send_ms



def _clamp(v: int, lo: int, hi: int) -> int:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _now_ms() -> int:
    return int(time.monotonic() * 1000.0)


def _parse_status_line(line_bytes: bytes) -> Optional[Dict[str, Any]]:
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
