"""Live console monitor and CSV recorder for the WheelFight Mega bridge."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, TextIO

from mega_sensor_reader import MegaSensorReader, SensorFrame


CSV_HEADER = [
    "host_time_utc",
    "sequence",
    "mega_millis",
    *[f"A{index}" for index in range(14)],
    "DI0",
    "DI1",
    "DI2",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor and record WheelFight Mega sensor frames"
    )
    parser.add_argument(
        "--port",
        help="serial device; omit to auto-discover /dev/serial/by-id, ttyACM, or ttyUSB",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", type=Path, help="optional CSV output path")
    parser.add_argument(
        "--display-interval",
        type=float,
        default=0.2,
        help="seconds between console updates (default: 0.2)",
    )
    return parser.parse_args()


def open_csv(path: Optional[Path]) -> tuple[Optional[TextIO], Optional[csv.writer]]:
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    file_handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(file_handle)
    writer.writerow(CSV_HEADER)
    file_handle.flush()
    return file_handle, writer


def write_csv_row(
    writer: Optional[csv.writer], file_handle: Optional[TextIO], frame: SensorFrame
) -> None:
    if writer is None or file_handle is None:
        return
    writer.writerow(
        [
            datetime.now(timezone.utc).isoformat(),
            frame.sequence,
            frame.mega_millis,
            *frame.analog,
            *frame.digital,
        ]
    )
    file_handle.flush()


def print_frame(frame: SensorFrame, status: dict) -> None:
    infrared = " ".join(
        f"IR{index}={value:4d}" for index, value in enumerate(frame.infrared)
    )
    digital = (
        f"DI(raw)={frame.digital} "
        f"detected(FL/FR/R)="
        f"{int(frame.front_left_detected)}/"
        f"{int(frame.front_right_detected)}/"
        f"{int(frame.rear_fence_detected)}"
    )
    print(
        f"seq={frame.sequence} mega={frame.mega_millis}ms "
        f"port={status['port']} update_rate={status['rate_hz']:.1f}Hz "
        f"drop={status['dropped_frames']} bad={status['invalid_frames']}"
    )
    print(f"  {infrared}")
    print(
        f"  gray_front={frame.grayscale_front:4d} "
        f"gray_rear={frame.grayscale_rear:4d} {digital}"
    )


def main() -> int:
    args = parse_args()
    if args.display_interval <= 0:
        print("--display-interval must be positive", file=sys.stderr)
        return 2

    reader = MegaSensorReader(port=args.port, baudrate=args.baud)
    csv_file, csv_writer = open_csv(args.csv)
    last_display = 0.0
    last_wait_message = 0.0

    try:
        reader.start()
        print("Waiting for WheelFight Mega sensor frames. Press Ctrl+C to stop.")
        while True:
            frame = reader.get_frame(timeout=0.5)
            now = time.monotonic()
            if frame is None:
                if now - last_wait_message >= 2.0:
                    status = reader.status()
                    print(
                        f"waiting: connected={status['connected']} "
                        f"port={status['port']} error={status['last_error']}"
                    )
                    last_wait_message = now
                continue

            write_csv_row(csv_writer, csv_file, frame)
            if now - last_display >= args.display_interval:
                print_frame(frame, reader.status())
                last_display = now
    except KeyboardInterrupt:
        print("Stopping sensor monitor.")
        return 0
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        reader.stop()
        if csv_file is not None:
            csv_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
