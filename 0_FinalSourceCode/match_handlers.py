from match_detection import LD, RD


class MatchHandlersMixin:
    def _left_align_ready(self):
        io_0 = self.uptech.ADC_IO_GetInputLevel(0)
        io_3 = self.uptech.ADC_IO_GetInputLevel(3)
        ad_1 = self.uptech.ADC_Get_Channel(1)
        return io_0 == 0 and ad_1 < RD and io_3 == 1

    def _right_align_ready(self):
        io_0 = self.uptech.ADC_IO_GetInputLevel(0)
        io_1 = self.uptech.ADC_IO_GetInputLevel(1)
        ad_3 = self.uptech.ADC_Get_Channel(3)
        return io_0 == 0 and ad_3 < LD and io_1 == 1

    def _handle_left_fence_align(self, freeSpeed):
        self._clear_action_sequence()
        self._set_state(self.STATE_FENCE_ALIGN_LEFT, "align platform")
        if self._left_align_ready():
            self._start_action_sequence(
                self.STATE_FENCE_RECOVER,
                "align-left-approach",
                [(freeSpeed, freeSpeed, 0.3)],
                "left align ready",
            )
            self._run_action_sequence()
        else:
            self.motion_controller.move_cmd(-freeSpeed + 50, freeSpeed - 50)

    def _handle_right_fence_align(self, freeSpeed):
        self._clear_action_sequence()
        self._set_state(self.STATE_FENCE_ALIGN_RIGHT, "align platform")
        if self._right_align_ready():
            self._start_action_sequence(
                self.STATE_FENCE_RECOVER,
                "align-right-approach",
                [(freeSpeed, freeSpeed, 0.3)],
                "right align ready",
            )
            self._run_action_sequence()
        else:
            self.motion_controller.move_cmd(freeSpeed - 50, -freeSpeed + 50)

    def _handle_off_platform(self, freeSpeed, turn):
        fence = self.fence_detect()
        if fence != 101:
            self._stun_time = 0

        if fence == 1:
            self._run_blocking_recovery(
                self.STATE_CLIMB,
                "climb-behind",
                self.motion_controller.go_up_behind_platform,
                "behind platform",
            )
            return
        if fence == 3:
            self._run_blocking_recovery(
                self.STATE_CLIMB,
                "climb-ahead",
                self.motion_controller.go_up_ahead_platform,
                "ahead platform",
            )
            return

        if self.state == self.STATE_FENCE_ALIGN_LEFT:
            self._handle_left_fence_align(freeSpeed)
            return
        if self.state == self.STATE_FENCE_ALIGN_RIGHT:
            self._handle_right_fence_align(freeSpeed)
            return

        if self._run_action_sequence():
            return

        if fence in (2, 16, 18):
            self._handle_left_fence_align(freeSpeed)
        elif fence in (4, 15, 17):
            self._handle_right_fence_align(freeSpeed)
        elif fence in (5, 6):
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                f"fence:{fence}",
                [(-freeSpeed, -freeSpeed, 0.4)],
                "front fence",
            )
        elif fence in (7, 8, 10):
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                f"fence:{fence}",
                [(freeSpeed, freeSpeed, 0.4)],
                "back fence or side enemy",
            )
        elif fence == 9:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:9",
                [(freeSpeed, -freeSpeed, turn), (freeSpeed, freeSpeed, 0.4)],
                "front/back target below platform",
            )
        elif fence == 11:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:11",
                [(-freeSpeed, -freeSpeed, 0.5), (-freeSpeed, freeSpeed, turn)],
                "three-side fence",
            )
        elif fence == 12:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:12",
                [(300, 600, turn)],
                "front-right-back fence",
            )
        elif fence == 13:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:13",
                [(600, 300, turn)],
                "front-left-back fence",
            )
        elif fence == 14:
            self._ensure_sequence(
                self.STATE_FENCE_RECOVER,
                "fence:14",
                [(-freeSpeed, freeSpeed, 0.2), (freeSpeed, freeSpeed, 0.3)],
                "side-back fence",
            )
        elif fence == 101:
            self._set_state(self.STATE_FENCE_RECOVER, "search platform")
            self.motion_controller.move_cmd(freeSpeed, -freeSpeed)
            self._stun_time += self.CONTROL_DT
            if self._stun_time > 3.3:
                self._stun_time = 0
                self._start_action_sequence(
                    self.STATE_FENCE_RECOVER,
                    "fence:unstick",
                    [(freeSpeed, freeSpeed, 0.5)],
                    "unstick platform search",
                )
        else:
            self._set_state(self.STATE_FENCE_RECOVER, f"unknown fence {fence}")
            self.motion_controller.move_cmd(freeSpeed, -freeSpeed)

    def _edge_recover_steps(self, edge, freeSpeed, turn):
        edge_steps = {
            1: [(-freeSpeed, -freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            2: [(-freeSpeed, -freeSpeed, 0.8), (-freeSpeed, freeSpeed, turn)],
            3: [(freeSpeed, freeSpeed, 0.8), (-freeSpeed, freeSpeed, turn)],
            4: [(freeSpeed, freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            5: [(-freeSpeed, -freeSpeed, 0.8), (freeSpeed, -freeSpeed, turn)],
            6: [(freeSpeed, freeSpeed, 0.5)],
            7: [(freeSpeed + 100, -freeSpeed, 0.5), (freeSpeed, freeSpeed, 0.3)],
            8: [(-freeSpeed, freeSpeed + 100, 0.5), (freeSpeed, freeSpeed, 0.3)],
            102: [(freeSpeed, -freeSpeed, self.CONTROL_DT)],
        }
        return edge_steps.get(edge)

    def _handle_edge_recover(self, edge, freeSpeed, turn):
        self._stun_time = 0

        steps = self._edge_recover_steps(edge, freeSpeed, turn)
        if steps is None:
            steps = [(freeSpeed, -freeSpeed, self.CONTROL_DT)]
        self._ensure_sequence(
            self.STATE_EDGE_RECOVER,
            f"edge:{edge}",
            steps,
            f"edge {edge}",
        )

    def _handle_on_platform(self, freeSpeed, enemySpeed, turn, turn_180):
        edge = self.edge_detect()
        if edge != 0:
            self._handle_edge_recover(edge, freeSpeed, turn)
            return

        enemy = self.enemy_detect()
        if enemy in (1, 11):
            self._clear_action_sequence()
            speed = enemySpeed if enemy == 11 else 700
            self._set_state(self.STATE_ATTACK, "front target")
            self.motion_controller.move_cmd(speed, speed)
            return

        if enemy == 5:
            self._ensure_sequence(
                self.STATE_AVOID_OWN_BLOCK,
                "avoid-own-block",
                [
                    (-freeSpeed, -freeSpeed, 0.5),
                    (-freeSpeed, freeSpeed, turn_180),
                    (freeSpeed, freeSpeed, 0.5),
                ],
                "own block",
            )
            return

        if self._run_action_sequence():
            return

        if enemy == 0:
            self._set_state(self.STATE_SEARCH, "patrol")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)
        elif enemy == 2:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-right",
                [(-freeSpeed, -freeSpeed, 0.3), (freeSpeed, -freeSpeed, 0.5)],
                "target right",
            )
        elif enemy == 3:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-back",
                [(freeSpeed, -freeSpeed, turn_180)],
                "target back",
            )
        elif enemy == 4:
            self._ensure_sequence(
                self.STATE_TURN_TO_TARGET,
                "target-left",
                [(-freeSpeed, -freeSpeed, 0.3), (-freeSpeed, freeSpeed, 0.3)],
                "target left",
            )
        else:
            self._set_state(self.STATE_SEARCH, f"unknown enemy {enemy}")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)

    def _handle_slip(self, freeSpeed):
        self._clear_action_sequence()
        slip = self.slip_detect()
        if slip == 0:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-right",
                self.motion_controller.slip_right,
                "left side slipped",
            )
        elif slip == 1:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-left",
                self.motion_controller.slip_left,
                "right side slipped",
            )
        elif slip == 2:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-back",
                self.motion_controller.slip_back,
                "front side slipped",
            )
        elif slip == 3:
            self._run_blocking_recovery(
                self.STATE_SLIP_RECOVER,
                "slip-front",
                self.motion_controller.slip_front,
                "back side slipped",
            )
        else:
            self._set_state(self.STATE_SLIP_RECOVER, f"unknown slip {slip}")
            self.motion_controller.move_cmd(freeSpeed, freeSpeed)
