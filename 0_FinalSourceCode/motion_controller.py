"""Hardware motion boundary for the four-motor differential chassis."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from project_paths import add_project_root_to_path
from robot_config import DEFAULT_CONFIG, HardwareConfig


@dataclass(frozen=True)
class DriveCommand:
    left_speed: int = 0
    right_speed: int = 0
    label: str = "stop"


class MotionController:
    """Owns all writes to the CDS motor/servo bus.

    The physical chassis has two motors per side, wired as one left command and
    one right command. Positive left/right values mean chassis-forward. The
    right CDS command is inverted because the motors are mirrored physically.
    """

    def __init__(
        self,
        uptech=None,
        config: HardwareConfig = DEFAULT_CONFIG.hardware,
        open_bus: bool = True,
    ) -> None:
        if uptech is None:
            add_project_root_to_path()

            # Import lazily so perception/state-machine tests do not require
            # libuptech.so on the development computer.
            from uptech import UpTech

            uptech = UpTech()

        self.uptech = uptech
        self.config = config
        self._last_drive: Optional[tuple[int, int]] = None
        self._shovel_pose = "unknown"
        self._closed = False

        if open_bus:
            self.uptech.CDS_Open()
        self._configure_servo_modes()
        self.stop(force=True)

    def _configure_servo_modes(self) -> None:
        # The CDS motor channels are driven directly with CDS_SetSpeed. The
        # controller only requires an explicit mode command for positional
        # servos, matching the original robot hardware interface.
        self.uptech.CDS_SetMode(
            self.config.left_shovel_servo_id, self.config.servo_mode
        )
        self.uptech.CDS_SetMode(
            self.config.right_shovel_servo_id, self.config.servo_mode
        )

    def _clamp_speed(self, value: int) -> int:
        limit = self.config.motor_limit
        return max(-limit, min(limit, int(round(value))))

    def move_cmd(
        self, left_speed: int = 0, right_speed: int = 0, force: bool = False
    ) -> None:
        if self._closed:
            return
        left = self._clamp_speed(left_speed)
        right = self._clamp_speed(right_speed)
        if not force and self._last_drive == (left, right):
            return

        self.uptech.CDS_SetSpeed(self.config.left_motor_id, left)
        self.uptech.CDS_SetSpeed(self.config.right_motor_id, -right)
        self._last_drive = (left, right)

    def apply(self, command: DriveCommand, force: bool = False) -> None:
        self.move_cmd(command.left_speed, command.right_speed, force=force)

    def stop(self, force: bool = False) -> None:
        self.move_cmd(0, 0, force=force)

    def set_shovel(self, left_angle: int, right_angle: int) -> bool:
        if self._closed or not self.config.shovel_motion_enabled:
            return False
        speed = self.config.servo_speed
        self.uptech.CDS_SetAngle(
            self.config.left_shovel_servo_id, int(left_angle), speed
        )
        self.uptech.CDS_SetAngle(
            self.config.right_shovel_servo_id, int(right_angle), speed
        )
        return True

    def raise_shovel(self) -> bool:
        moved = self.set_shovel(
            self.config.shovel_raised_left, self.config.shovel_raised_right
        )
        self._shovel_pose = "raised" if moved else "disabled"
        return moved

    def lower_shovel(self) -> bool:
        moved = self.set_shovel(
            self.config.shovel_lowered_left, self.config.shovel_lowered_right
        )
        self._shovel_pose = "lowered" if moved else "disabled"
        return moved

    @property
    def last_drive(self) -> tuple[int, int]:
        return self._last_drive or (0, 0)

    @property
    def shovel_pose(self) -> str:
        return self._shovel_pose

    def close(self) -> None:
        if self._closed:
            return
        self.stop(force=True)
        try:
            self.uptech.CDS_Close()
        finally:
            self._closed = True


__all__ = ["DriveCommand", "MotionController"]
