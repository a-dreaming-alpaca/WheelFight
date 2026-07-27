"""Validated, reconnecting WheelFight Mega 2560 sensor receiver."""

from __future__ import annotations

import glob
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Optional, Union


PROTOCOL_MARKER = "WF1"
ANALOG_CHANNEL_COUNT = 14
DIGITAL_CHANNEL_COUNT = 3
EXPECTED_FIELD_COUNT = 3 + ANALOG_CHANNEL_COUNT + DIGITAL_CHANNEL_COUNT
MAX_UINT32 = 0xFFFFFFFF
DEFAULT_BAUDRATE = 115200
DEFAULT_STALE_AFTER = 0.2


def _accumulate_serial_line(
    pending: bytearray,
    chunk: bytes,
    max_line_bytes: int,
) -> tuple[Optional[bytes], Optional[str]]:
    """Accumulate one newline-terminated frame without flushing later data."""
    if not chunk:
        return None, None

    pending.extend(chunk)
    if not pending.endswith(b"\n"):
        if len(pending) >= max_line_bytes:
            pending.clear()
            return None, "oversized serial line"
        return None, None

    raw = bytes(pending)
    pending.clear()

    # A Mega reset can truncate one old frame immediately before the first
    # complete reboot frame. The newest marker is safe to use because valid
    # payload fields contain only unsigned decimal text.
    latest_prefix = raw.rfind(b"WF1,")
    if latest_prefix > 0:
        return (
            raw[latest_prefix:],
            "discarded incomplete bytes before frame prefix",
        )
    return raw, None


class FrameError(ValueError):
    """Base class for invalid sensor frames."""


class FrameFormatError(FrameError):
    """The frame structure or a value is invalid."""


class FrameChecksumError(FrameError):
    """The transmitted checksum does not match the payload."""


@dataclass(frozen=True)
class SensorFrame:
    sequence: int
    mega_millis: int
    analog: tuple[int, ...]
    digital: tuple[int, ...]
    received_monotonic: float

    @property
    def infrared(self) -> tuple[int, ...]:
        """A0-A11, starting forward and increasing clockwise by 30 degrees."""
        return self.analog[:12]

    @property
    def grayscale_front(self) -> int:
        return self.analog[12]

    @property
    def grayscale_rear(self) -> int:
        return self.analog[13]

    @property
    def front_left_detected(self) -> bool:
        """Backward-compatible alias: nearby surface is detected (raw low)."""
        return self.digital[0] == 0

    @property
    def front_right_detected(self) -> bool:
        """Backward-compatible alias: nearby surface is detected (raw low)."""
        return self.digital[1] == 0

    @property
    def rear_fence_detected(self) -> bool:
        """Raw rear high-object detection; state determines whether it is fence."""
        return self.digital[2] == 0

    @property
    def front_left_edge(self) -> bool:
        return self.digital[0] == 1

    @property
    def front_right_edge(self) -> bool:
        return self.digital[1] == 1

    @property
    def rear_high_object(self) -> bool:
        return self.digital[2] == 0

    def as_dict(self) -> dict:
        return {
            "sequence": self.sequence,
            "mega_millis": self.mega_millis,
            "analog": list(self.analog),
            "digital": list(self.digital),
            "infrared": list(self.infrared),
            "grayscale_front": self.grayscale_front,
            "grayscale_rear": self.grayscale_rear,
            "front_left_detected": self.front_left_detected,
            "front_right_detected": self.front_right_detected,
            "rear_fence_detected": self.rear_fence_detected,
            "front_left_edge": self.front_left_edge,
            "front_right_edge": self.front_right_edge,
            "rear_high_object": self.rear_high_object,
            "received_monotonic": self.received_monotonic,
        }


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _parse_decimal(token: str, name: str, minimum: int, maximum: int) -> int:
    if not token or not token.isdecimal():
        raise FrameFormatError(f"{name} is not an unsigned decimal integer")
    value = int(token, 10)
    if not minimum <= value <= maximum:
        raise FrameFormatError(
            f"{name}={value} is outside {minimum}..{maximum}"
        )
    return value


def parse_frame(
    raw: Union[bytes, str], received_monotonic: Optional[float] = None
) -> SensorFrame:
    if isinstance(raw, bytes):
        try:
            text = raw.decode("ascii")
        except UnicodeDecodeError as exc:
            raise FrameFormatError("frame is not ASCII") from exc
    else:
        text = raw

    text = text.rstrip("\r\n")
    if text.count("*") != 1:
        raise FrameFormatError("frame must contain exactly one '*' separator")

    payload, crc_text = text.split("*", 1)
    if len(crc_text) != 4 or any(
        character not in "0123456789abcdefABCDEF" for character in crc_text
    ):
        raise FrameFormatError("CRC must contain exactly four hexadecimal digits")

    expected_crc = int(crc_text, 16)
    actual_crc = crc16_ccitt_false(payload.encode("ascii"))
    if actual_crc != expected_crc:
        raise FrameChecksumError(
            f"CRC mismatch: received {expected_crc:04X}, calculated {actual_crc:04X}"
        )

    fields = payload.split(",")
    if len(fields) != EXPECTED_FIELD_COUNT:
        raise FrameFormatError(
            f"expected {EXPECTED_FIELD_COUNT} payload fields, got {len(fields)}"
        )
    if fields[0] != PROTOCOL_MARKER:
        raise FrameFormatError(f"unsupported protocol marker {fields[0]!r}")

    sequence = _parse_decimal(fields[1], "sequence", 0, MAX_UINT32)
    mega_millis = _parse_decimal(fields[2], "mega_millis", 0, MAX_UINT32)

    analog_start = 3
    analog = tuple(
        _parse_decimal(fields[analog_start + index], f"A{index}", 0, 1023)
        for index in range(ANALOG_CHANNEL_COUNT)
    )
    digital_start = analog_start + ANALOG_CHANNEL_COUNT
    digital = tuple(
        _parse_decimal(fields[digital_start + index], f"DI{index}", 0, 1)
        for index in range(DIGITAL_CHANNEL_COUNT)
    )

    if received_monotonic is None:
        received_monotonic = time.monotonic()
    return SensorFrame(
        sequence=sequence,
        mega_millis=mega_millis,
        analog=analog,
        digital=digital,
        received_monotonic=received_monotonic,
    )


def encode_frame(
    sequence: int,
    mega_millis: int,
    analog: Iterable[int],
    digital: Iterable[int],
) -> bytes:
    """Build a protocol frame for tests and diagnostics."""
    analog_values = tuple(analog)
    digital_values = tuple(digital)
    if len(analog_values) != ANALOG_CHANNEL_COUNT:
        raise ValueError(f"analog must contain {ANALOG_CHANNEL_COUNT} values")
    if len(digital_values) != DIGITAL_CHANNEL_COUNT:
        raise ValueError(f"digital must contain {DIGITAL_CHANNEL_COUNT} values")

    numeric_fields = [
        str(_validate_integer(sequence, "sequence", 0, MAX_UINT32)),
        str(_validate_integer(mega_millis, "mega_millis", 0, MAX_UINT32)),
    ]
    numeric_fields.extend(
        str(_validate_integer(value, f"A{index}", 0, 1023))
        for index, value in enumerate(analog_values)
    )
    numeric_fields.extend(
        str(_validate_integer(value, f"DI{index}", 0, 1))
        for index, value in enumerate(digital_values)
    )
    payload = ",".join([PROTOCOL_MARKER, *numeric_fields]).encode("ascii")
    return payload + f"*{crc16_ccitt_false(payload):04X}\r\n".encode("ascii")


def _validate_integer(value: int, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name}={value} is outside {minimum}..{maximum}")
    return value


class MegaSensorReader:
    """Background serial receiver with validation and automatic reconnect."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_BAUDRATE,
        stale_after: float = DEFAULT_STALE_AFTER,
        read_timeout: float = 0.1,
        reconnect_interval: float = 1.0,
        frame_queue_size: int = 256,
        max_line_bytes: int = 256,
    ) -> None:
        if stale_after <= 0 or read_timeout <= 0 or reconnect_interval <= 0:
            raise ValueError("time intervals must be positive")
        if frame_queue_size <= 0 or max_line_bytes <= 0:
            raise ValueError("queue and line sizes must be positive")

        self.port = port
        self.baudrate = baudrate
        self.stale_after = stale_after
        self.read_timeout = read_timeout
        self.reconnect_interval = reconnect_interval
        self.max_line_bytes = max_line_bytes

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frames: queue.Queue[SensorFrame] = queue.Queue(frame_queue_size)
        self._receive_times: deque[float] = deque(maxlen=100)

        self._serial_connection = None
        self._latest: Optional[SensorFrame] = None
        self._connected_port: Optional[str] = None
        self._last_error = "not started"
        self._last_sequence: Optional[int] = None

        self._valid_frames = 0
        self._invalid_frames = 0
        self._checksum_errors = 0
        self._dropped_frames = 0
        self._duplicate_frames = 0
        self._out_of_order_frames = 0
        self._queue_overruns = 0
        self._reconnects = 0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._load_pyserial()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="mega-sensor-reader", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        connection = self._serial_connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout)

    def latest_frame(self) -> Optional[SensorFrame]:
        with self._lock:
            return self._latest

    def get_frame(self, timeout: Optional[float] = None) -> Optional[SensorFrame]:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty:
            return None

    def is_stale(self, now: Optional[float] = None) -> bool:
        frame = self.latest_frame()
        if frame is None:
            return True
        if now is None:
            now = time.monotonic()
        return now - frame.received_monotonic > self.stale_after

    def status(self) -> dict:
        with self._lock:
            receive_times = tuple(self._receive_times)
            if len(receive_times) >= 2 and receive_times[-1] > receive_times[0]:
                rate_hz = (len(receive_times) - 1) / (
                    receive_times[-1] - receive_times[0]
                )
            else:
                rate_hz = 0.0
            latest = self._latest
            return {
                "connected": self._connected_port is not None,
                "port": self._connected_port,
                "stale": latest is None
                or time.monotonic() - latest.received_monotonic > self.stale_after,
                "rate_hz": rate_hz,
                "last_error": self._last_error,
                "valid_frames": self._valid_frames,
                "invalid_frames": self._invalid_frames,
                "checksum_errors": self._checksum_errors,
                "dropped_frames": self._dropped_frames,
                "duplicate_frames": self._duplicate_frames,
                "out_of_order_frames": self._out_of_order_frames,
                "queue_overruns": self._queue_overruns,
                "reconnects": self._reconnects,
            }

    @staticmethod
    def available_ports() -> list[str]:
        _, list_ports = MegaSensorReader._load_pyserial()
        preferred = sorted(glob.glob("/dev/serial/by-id/*"))
        port_info = list(list_ports.comports())

        def looks_like_mega(port) -> bool:
            identity = " ".join(
                str(value or "")
                for value in (
                    port.description,
                    port.manufacturer,
                    port.product,
                    port.hwid,
                )
            ).lower()
            return "arduino" in identity or "mega 2560" in identity

        # Prefer a device identified as an Arduino. Other USB serial devices
        # remain candidates for clone boards, while built-in ttyS ports are not
        # selected accidentally.
        mega_ports = [port.device for port in port_info if looks_like_mega(port)]
        usb_ports = [port.device for port in port_info if port.vid is not None]
        fallback = sorted(
            path
            for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*")
            for path in glob.glob(pattern)
        )
        return list(
            dict.fromkeys([*mega_ports, *preferred, *usb_ports, *fallback])
        )

    @staticmethod
    def _load_pyserial():
        try:
            import serial
            from serial.tools import list_ports
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required; install 0_FinalSourceCode/sensor_requirements.txt"
            ) from exc
        return serial, list_ports

    def _select_port(self) -> Optional[str]:
        if self.port:
            return self.port
        ports = self.available_ports()
        return ports[0] if ports else None

    def _run(self) -> None:
        serial_module, _ = self._load_pyserial()
        while not self._stop_event.is_set():
            selected_port = self._select_port()
            if selected_port is None:
                self._set_disconnected("no USB serial device found")
                self._stop_event.wait(self.reconnect_interval)
                continue

            try:
                connection = serial_module.Serial(
                    selected_port,
                    self.baudrate,
                    timeout=self.read_timeout,
                    write_timeout=self.read_timeout,
                )
                self._serial_connection = connection
                with self._lock:
                    self._connected_port = selected_port
                    self._last_error = ""
                    self._last_sequence = None
                    self._reconnects += 1

                pending = bytearray()
                while not self._stop_event.is_set():
                    remaining = self.max_line_bytes - len(pending)
                    chunk = connection.read_until(b"\n", remaining)
                    raw, line_error = _accumulate_serial_line(
                        pending,
                        chunk,
                        self.max_line_bytes,
                    )
                    if line_error:
                        self._record_invalid(line_error)
                    if raw is None:
                        continue

                    try:
                        frame = parse_frame(raw, time.monotonic())
                    except FrameChecksumError as exc:
                        self._record_invalid(str(exc), checksum=True)
                        continue
                    except FrameError as exc:
                        self._record_invalid(str(exc))
                        continue
                    self._record_valid(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._set_disconnected(
                        f"serial connection failed on {selected_port}: {exc}"
                    )
            finally:
                connection = self._serial_connection
                self._serial_connection = None
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                with self._lock:
                    self._connected_port = None

            self._stop_event.wait(self.reconnect_interval)

    def _record_valid(self, frame: SensorFrame) -> None:
        with self._lock:
            if self._last_sequence is not None:
                delta = (frame.sequence - self._last_sequence) & MAX_UINT32
                if delta == 0:
                    self._duplicate_frames += 1
                elif delta < 0x80000000:
                    self._dropped_frames += max(0, delta - 1)
                else:
                    self._out_of_order_frames += 1
            self._last_sequence = frame.sequence
            self._latest = frame
            self._valid_frames += 1
            self._receive_times.append(frame.received_monotonic)
            self._last_error = ""

        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            self._frames.put_nowait(frame)
            with self._lock:
                self._queue_overruns += 1

    def _record_invalid(self, error: str, checksum: bool = False) -> None:
        with self._lock:
            self._invalid_frames += 1
            if checksum:
                self._checksum_errors += 1
            self._last_error = error

    def _set_disconnected(self, error: str) -> None:
        with self._lock:
            self._connected_port = None
            self._last_error = error


__all__ = [
    "ANALOG_CHANNEL_COUNT",
    "DIGITAL_CHANNEL_COUNT",
    "FrameChecksumError",
    "FrameError",
    "FrameFormatError",
    "MegaSensorReader",
    "SensorFrame",
    "crc16_ccitt_false",
    "encode_frame",
    "parse_frame",
]
