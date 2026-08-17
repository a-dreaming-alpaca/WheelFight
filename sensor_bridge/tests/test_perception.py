import sys
import unittest
from dataclasses import replace
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from mega_sensor_reader import SensorFrame  # noqa: E402
from perception import PerceptionEngine, PlatformState  # noqa: E402
from robot_config import SensorConfig  # noqa: E402


def frame(sequence, analog, digital, timestamp=1.0):
    return SensorFrame(
        sequence=sequence,
        mega_millis=sequence * 20,
        analog=tuple(analog),
        digital=tuple(digital),
        received_monotonic=timestamp,
    )


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.config = SensorConfig(
            analog_filter_window=1,
            disabled_ir_indices=(),
            ir_detect_enter=550,
            ir_detect_exit=500,
            ir_a6_detect_enter=550,
            ir_a6_detect_exit=500,
            start_hand_enter=700,
            start_hand_exit=600,
            gray_on_is_high=False,
            gray_on_enter=500,
            gray_off_exit=700,
            edge_clear_frames=2,
            rear_high_confirm_frames=1,
            rear_high_clear_frames=2,
            platform_confirm_frames=1,
        )
        self.engine = PerceptionEngine(self.config)

    def test_default_config_maps_measured_grayscale_values(self):
        config = SensorConfig(analog_filter_window=1, platform_confirm_frames=1)
        engine = PerceptionEngine(config)

        on = engine.update(
            frame(1, [100] * 12 + [300, 300], (0, 0, 1)), now=1.0
        )
        off = engine.update(
            frame(2, [100] * 12 + [900, 900], (0, 0, 1), 1.02), now=1.02
        )

        self.assertEqual(on.platform_state, PlatformState.ON)
        self.assertEqual(off.platform_state, PlatformState.OFF)

    def test_default_rear_high_policy_asserts_once_and_clears_after_three(self):
        config = SensorConfig(analog_filter_window=1, platform_confirm_frames=1)
        engine = PerceptionEngine(config)
        analog = [100] * 12 + [300, 300]

        states = [
            engine.update(
                frame(sequence, analog, digital, 1.0 + sequence * 0.02),
                now=1.0 + sequence * 0.02,
            ).rear_high_object
            for sequence, digital in enumerate(
                ((0, 0, 0), (0, 0, 1), (0, 0, 1), (0, 0, 1)),
                start=1,
            )
        ]

        self.assertEqual(states, [True, True, True, False])

    def test_invalid_grayscale_hysteresis_order_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "grayscale hysteresis"):
            PerceptionEngine(
                SensorConfig(
                    gray_on_is_high=False,
                    gray_on_enter=700,
                    gray_off_exit=500,
                )
            )

    def test_platform_edge_and_rear_high_semantics(self):
        analog = [100] * 12 + [300, 300]
        first = self.engine.update(frame(1, analog, (0, 0, 0)), now=1.0)
        self.assertEqual(first.platform_state, PlatformState.ON)
        self.assertFalse(first.front_left_edge)
        self.assertTrue(first.rear_high_object)

        second = self.engine.update(frame(2, analog, (1, 0, 0), 1.02), now=1.02)
        self.assertTrue(second.front_left_edge)
        self.assertTrue(second.rear_high_object)

        third = self.engine.update(frame(3, analog, (0, 0, 1), 1.04), now=1.04)
        self.assertTrue(third.front_left_edge)
        self.assertTrue(third.rear_high_object)
        fourth = self.engine.update(frame(4, analog, (0, 0, 1), 1.06), now=1.06)
        self.assertFalse(fourth.front_left_edge)
        self.assertFalse(fourth.rear_high_object)

    def test_rear_high_detection_resets_the_clear_streak(self):
        analog = [100] * 12 + [300, 300]

        detected = self.engine.update(
            frame(1, analog, (0, 0, 0)), now=1.0
        )
        first_clear = self.engine.update(
            frame(2, analog, (0, 0, 1), 1.02), now=1.02
        )
        detected_again = self.engine.update(
            frame(3, analog, (0, 0, 0), 1.04), now=1.04
        )
        clear_after_reset = self.engine.update(
            frame(4, analog, (0, 0, 1), 1.06), now=1.06
        )
        released = self.engine.update(
            frame(5, analog, (0, 0, 1), 1.08), now=1.08
        )

        self.assertTrue(detected.rear_high_object)
        self.assertTrue(first_clear.rear_high_object)
        self.assertTrue(detected_again.rear_high_object)
        self.assertTrue(clear_after_reset.rear_high_object)
        self.assertFalse(released.rear_high_object)

    def test_rear_high_assertion_threshold_remains_independently_configurable(self):
        engine = PerceptionEngine(
            replace(self.config, rear_high_confirm_frames=2)
        )
        analog = [100] * 12 + [300, 300]

        first = engine.update(frame(1, analog, (0, 0, 0)), now=1.0)
        second = engine.update(
            frame(2, analog, (0, 0, 0), 1.02), now=1.02
        )

        self.assertFalse(first.rear_high_object)
        self.assertTrue(second.rear_high_object)

    def test_front_rear_platform_transitions(self):
        front_off = [100] * 12 + [900, 300]
        rear_off = [100] * 12 + [300, 900]
        self.assertEqual(
            self.engine.update(frame(1, front_off, (0, 0, 1)), now=1.0).platform_state,
            PlatformState.FRONT_TRANSITION,
        )
        self.assertEqual(
            self.engine.update(frame(2, rear_off, (0, 0, 1), 1.02), now=1.02).platform_state,
            PlatformState.REAR_TRANSITION,
        )

    def test_low_grayscale_platform_hysteresis(self):
        on = [100] * 12 + [300, 300]
        middle = [100] * 12 + [600, 600]
        off = [100] * 12 + [900, 900]

        self.assertEqual(
            self.engine.update(frame(1, on, (0, 0, 1)), now=1.0).platform_state,
            PlatformState.ON,
        )
        self.assertEqual(
            self.engine.update(
                frame(2, middle, (0, 0, 1), 1.02), now=1.02
            ).platform_state,
            PlatformState.ON,
        )
        self.assertEqual(
            self.engine.update(
                frame(3, off, (0, 0, 1), 1.04), now=1.04
            ).platform_state,
            PlatformState.OFF,
        )
        self.assertEqual(
            self.engine.update(
                frame(4, middle, (0, 0, 1), 1.06), now=1.06
            ).platform_state,
            PlatformState.OFF,
        )
        self.assertEqual(
            self.engine.update(
                frame(5, on, (0, 0, 1), 1.08), now=1.08
            ).platform_state,
            PlatformState.ON,
        )

    def test_circular_ir_cluster_wraps_across_a11_a0(self):
        analog = [100] * 14
        analog[11] = 800
        analog[0] = 900
        snapshot = self.engine.update(frame(1, analog, (0, 0, 1)), now=1.0)
        self.assertEqual(len(snapshot.clusters), 1)
        self.assertEqual(set(snapshot.clusters[0].indices), {0, 11})
        self.assertLess(abs(snapshot.clusters[0].bearing_deg), 30.0)

    def test_disabled_a10_is_retained_raw_but_excluded_from_targets(self):
        engine = PerceptionEngine(
            replace(self.config, disabled_ir_indices=(10,))
        )
        analog = [100] * 14
        analog[10] = 900

        snapshot = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)

        self.assertEqual(snapshot.raw_analog[10], 900)
        self.assertEqual(snapshot.filtered_analog[10], 900)
        self.assertEqual(snapshot.disabled_ir_indices, (10,))
        self.assertFalse(snapshot.infrared_active[10])
        self.assertEqual(snapshot.clusters, ())

    def test_cluster_bridges_only_across_explicitly_disabled_a10(self):
        analog = [100] * 14
        analog[9] = 800
        analog[11] = 800

        degraded = PerceptionEngine(
            replace(self.config, disabled_ir_indices=(10,))
        ).update(frame(1, analog, (0, 0, 1)), now=1.0)
        normal = self.engine.update(frame(1, analog, (0, 0, 1)), now=1.0)

        self.assertEqual(len(degraded.clusters), 1)
        self.assertEqual(degraded.clusters[0].indices, (9, 11))
        self.assertAlmostEqual(degraded.clusters[0].bearing_deg, -60.0)
        self.assertEqual(degraded.clusters[0].strength, 600.0)
        self.assertEqual(degraded.clusters[0].representative_value, 800)
        self.assertEqual(
            tuple(cluster.indices for cluster in normal.clusters),
            ((9,), (11,)),
        )

    def test_no_disabled_channels_preserve_cyclic_cluster_order(self):
        analog = [100] * 14
        analog[0] = 800
        analog[10] = 800

        snapshot = self.engine.update(frame(1, analog, (0, 0, 1)), now=1.0)

        self.assertEqual(
            tuple(cluster.indices for cluster in snapshot.clusters),
            ((10,), (0,)),
        )

    def test_disabled_a0_bridges_a11_and_a1_across_ring_boundary(self):
        engine = PerceptionEngine(
            replace(self.config, disabled_ir_indices=(0,))
        )
        analog = [100] * 14
        analog[1] = 800
        analog[11] = 800

        snapshot = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)

        self.assertEqual(len(snapshot.clusters), 1)
        self.assertEqual(set(snapshot.clusters[0].indices), {1, 11})
        self.assertAlmostEqual(snapshot.clusters[0].bearing_deg, 0.0)

    def test_consecutive_disabled_channels_are_not_bridged(self):
        engine = PerceptionEngine(
            replace(self.config, disabled_ir_indices=(9, 10))
        )
        analog = [100] * 14
        analog[8] = 800
        analog[11] = 800

        snapshot = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)

        self.assertEqual(
            tuple(cluster.indices for cluster in snapshot.clusters),
            ((8,), (11,)),
        )

    def test_disabled_channel_noise_does_not_change_feature_signature(self):
        engine = PerceptionEngine(
            replace(self.config, disabled_ir_indices=(10,))
        )
        analog = [100] * 14
        first = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)
        analog[10] = 1023
        second = engine.update(
            frame(2, analog, (0, 0, 1), 1.02), now=1.02
        )

        self.assertEqual(first.feature_signature(), second.feature_signature())

    def test_disabled_ir_indices_reject_invalid_or_duplicate_values(self):
        for disabled in ((-1,), (12,), (True,), (10, 10)):
            with self.subTest(disabled=disabled):
                with self.assertRaisesRegex(ValueError, "disabled_ir_indices"):
                    PerceptionEngine(
                        replace(self.config, disabled_ir_indices=disabled)
                    )

    def test_a6_uses_calibrated_hysteresis_without_changing_a5(self):
        config = SensorConfig(
            analog_filter_window=1,
            ir_detect_enter=350,
            ir_detect_exit=300,
            ir_a6_detect_enter=300,
            ir_a6_detect_exit=250,
            platform_confirm_frames=1,
        )
        engine = PerceptionEngine(config)
        analog = [0] * 14
        analog[5] = 320
        analog[6] = 300

        entered = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)
        self.assertFalse(entered.infrared_active[5])
        self.assertTrue(entered.infrared_active[6])

        analog[6] = 250
        retained = engine.update(
            frame(2, analog, (0, 0, 1), 1.02), now=1.02
        )
        self.assertTrue(retained.infrared_active[6])

        analog[6] = 249
        exited = engine.update(
            frame(3, analog, (0, 0, 1), 1.04), now=1.04
        )
        self.assertFalse(exited.infrared_active[6])

    def test_a6_threshold_keeps_rear_cluster_connected_and_sets_weight(self):
        config = SensorConfig(
            analog_filter_window=1,
            ir_detect_enter=350,
            ir_detect_exit=300,
            ir_a6_detect_enter=300,
            ir_a6_detect_exit=250,
            platform_confirm_frames=1,
        )
        engine = PerceptionEngine(config)
        analog = [0] * 14
        analog[5:8] = [360, 320, 360]

        connected = engine.update(frame(1, analog, (0, 0, 1)), now=1.0)
        self.assertEqual(connected.infrared_active[5:8], (True, True, True))
        self.assertEqual(len(connected.clusters), 1)
        self.assertEqual(connected.clusters[0].indices, (5, 6, 7))
        self.assertEqual(connected.clusters[0].strength, 190.0)

        analog[6] = 260
        retained = engine.update(
            frame(2, analog, (0, 0, 1), 1.02), now=1.02
        )
        self.assertEqual(len(retained.clusters), 1)
        self.assertEqual(retained.clusters[0].indices, (5, 6, 7))
        self.assertEqual(retained.clusters[0].strength, 130.0)

        analog[6] = 249
        split = engine.update(
            frame(3, analog, (0, 0, 1), 1.04), now=1.04
        )
        self.assertEqual(split.infrared_active[5:8], (True, False, True))
        self.assertEqual(
            tuple(cluster.indices for cluster in split.clusters),
            ((5,), (7,)),
        )

    def test_start_hands_use_a9_left_and_a3_right(self):
        analog = [100] * 14
        analog[3] = 800
        analog[9] = 810
        snapshot = self.engine.update(frame(1, analog, (0, 0, 1)), now=1.0)
        self.assertTrue(snapshot.start_left_hand_near)
        self.assertTrue(snapshot.start_right_hand_near)


if __name__ == "__main__":
    unittest.main()
