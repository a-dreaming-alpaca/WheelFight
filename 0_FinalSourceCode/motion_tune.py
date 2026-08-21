"""Run one manually selected MotionController action for bench tuning."""

from __future__ import annotations

from motion_controller import MotionController
from robot_config import DEFAULT_CONFIG
import time

def run_selected_action(motion: MotionController) -> None:
    """Edit only the call below; tune all numeric values in robot_config.py."""

    # Safe default. Replace this one line with the action currently being
    # tested, for example:
    motion.move_cmd(600, 350)
    
    '''motion.raise_shovel()
    input('lower')
    motion.lower_shovel()
    input('raise')
    motion.raise_shovel()'''

    

   


def main() -> None:
    motion = MotionController(config=DEFAULT_CONFIG.hardware)
    try:
        run_selected_action(motion)
        input("Action active. Press Enter to stop: ")
    finally:
        motion.stop(force=True)
        motion.close()


if __name__ == "__main__":
    main()
