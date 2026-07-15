"""Low-speed, operator-confirmed motor and shovel calibration helper."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace

from motion_controller import MotionController
from robot_config import DEFAULT_CONFIG


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WheelFight actuator bench test")
    parser.add_argument(
        "--run",
        action="store_true",
        help="actually send commands; without this flag only print the plan",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    drive = subparsers.add_parser("drive", help="test differential drive")
    drive.add_argument("--left", type=int, required=True)
    drive.add_argument("--right", type=int, required=True)
    drive.add_argument("--seconds", type=float, default=0.30)

    shovel = subparsers.add_parser("shovel", help="test one shovel pose")
    shovel.add_argument("--left-angle", type=int, required=True)
    shovel.add_argument("--right-angle", type=int, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.command == "drive":
        if args.seconds <= 0 or args.seconds > 2.0:
            raise SystemExit("--seconds must be greater than 0 and at most 2.0")
        limit = DEFAULT_CONFIG.hardware.motor_limit
        if abs(args.left) > limit or abs(args.right) > limit:
            raise SystemExit(f"drive values must be within {-limit}..{limit}")
        plan = (
            f"drive left={args.left}, right={args.right}, "
            f"duration={args.seconds:.2f}s"
        )
    else:
        for name, angle in (
            ("left-angle", args.left_angle),
            ("right-angle", args.right_angle),
        ):
            if not 0 <= angle <= 1023:
                raise SystemExit(f"--{name} must be within 0..1023")
        plan = f"shovel left={args.left_angle}, right={args.right_angle}"

    print(f"Plan: {plan}")
    if not args.run:
        print("Dry run only. Add --run after raising the wheels/unloading the shovel.")
        return

    answer = input("Type RUN to confirm the mechanism is safe: ").strip()
    if answer != "RUN":
        print("Cancelled.")
        return

    hardware = replace(
        DEFAULT_CONFIG.hardware,
        shovel_motion_enabled=args.command == "shovel",
    )
    controller = MotionController(config=hardware)
    try:
        if args.command == "drive":
            controller.move_cmd(args.left, args.right)
            time.sleep(args.seconds)
            controller.stop(force=True)
        else:
            controller.set_shovel(args.left_angle, args.right_angle)
            input("Press Enter after observing the pose; bus will then close...")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
