import sys
import unittest
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from mega_sensor_reader import (  # noqa: E402
    FrameChecksumError,
    FrameFormatError,
    MegaSensorReader,
    SensorFrame,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
)
from robot_config import DEFAULT_CONFIG  # noqa: E402


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.analog = tuple(range(100, 114))
        self.digital = (0, 1, 0)

    def test_crc_standard_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_encode_parse_round_trip(self):
        raw = encode_frame(42, 123456, self.analog, self.digital)
        self.assertEqual(
            raw,
            b"WF1,42,123456,100,101,102,103,104,105,106,107,108,109,"
            b"110,111,112,113,0,1,0*D1DC\r\n",
        )
        frame = parse_frame(raw, received_monotonic=10.5)

        self.assertEqual(frame.sequence, 42)
        self.assertEqual(frame.mega_millis, 123456)
        self.assertEqual(frame.analog, self.analog)
        self.assertEqual(frame.digital, self.digital)
        self.assertEqual(frame.received_monotonic, 10.5)

    def test_semantic_channel_mapping_and_active_low(self):
        frame = parse_frame(
            encode_frame(1, 2, self.analog, self.digital),
            received_monotonic=3.0,
        )
        self.assertEqual(frame.infrared, self.analog[:12])
        self.assertEqual(frame.grayscale_front, 112)
        self.assertEqual(frame.grayscale_rear, 113)
        self.assertTrue(frame.front_left_detected)
        self.assertFalse(frame.front_right_detected)
        self.assertTrue(frame.rear_fence_detected)

    def test_rejects_bad_checksum(self):
        raw = bytearray(encode_frame(42, 123456, self.analog, self.digital))
        raw[5] = ord("9")
        with self.assertRaises(FrameChecksumError):
            parse_frame(bytes(raw))

    def test_rejects_out_of_range_analog_with_valid_checksum(self):
        fields = ["WF1", "1", "2", "1024", *["0"] * 13, "0", "1", "1"]
        payload = ",".join(fields).encode("ascii")
        raw = payload + f"*{crc16_ccitt_false(payload):04X}\r\n".encode("ascii")
        with self.assertRaises(FrameFormatError):
            parse_frame(raw)

    def test_rejects_wrong_field_count(self):
        payload = b"WF1,1,2,0"
        raw = payload + f"*{crc16_ccitt_false(payload):04X}\r\n".encode("ascii")
        with self.assertRaises(FrameFormatError):
            parse_frame(raw)


class ReaderStateTests(unittest.TestCase):
    @staticmethod
    def frame(sequence, received_monotonic):
        return SensorFrame(
            sequence=sequence,
            mega_millis=sequence * 20,
            analog=(0,) * 14,
            digital=(1, 1, 1),
            received_monotonic=received_monotonic,
        )

    def test_default_linux_sensor_timings_leave_scheduling_margin(self):
        timing = DEFAULT_CONFIG.timing
        reader = MegaSensorReader()

        self.assertEqual(timing.sensor_warning_after, 0.10)
        self.assertEqual(timing.sensor_stop_after, 0.20)
        self.assertEqual(timing.sensor_read_timeout, 0.03)
        self.assertEqual(reader.stale_after, 0.20)
        self.assertEqual(reader.read_timeout, 0.03)

    def test_tracks_rate_drops_and_staleness(self):
        reader = MegaSensorReader(stale_after=0.2)
        reader._record_valid(self.frame(10, 1.00))
        reader._record_valid(self.frame(12, 1.02))

        status = reader.status()
        self.assertEqual(status["valid_frames"], 2)
        self.assertEqual(status["dropped_frames"], 1)
        self.assertAlmostEqual(status["rate_hz"], 50.0)
        self.assertFalse(reader.is_stale(now=1.21))
        self.assertTrue(reader.is_stale(now=1.23))

    def test_partial_frame_survives_empty_reads_and_is_completed_later(self):
        reader = MegaSensorReader()
        raw = encode_frame(20, 400, (100,) * 14, (0, 1, 0))
        split = len(raw) // 2

        self.assertIsNone(reader._consume_serial_bytes(raw[:split], 1.0))
        self.assertIsNone(reader._consume_serial_bytes(b"", 1.1))
        self.assertEqual(reader.status()["invalid_frames"], 0)
        self.assertIsNone(reader.latest_frame())

        newest = reader._consume_serial_bytes(raw[split:], 1.2)

        self.assertEqual(newest.sequence, 20)
        self.assertEqual(reader.latest_frame().sequence, 20)
        self.assertEqual(reader.status()["invalid_frames"], 0)

    def test_reconnect_reset_prevents_old_partial_frame_from_splicing(self):
        reader = MegaSensorReader()
        old = encode_frame(21, 420, (100,) * 14, (0, 1, 0))
        new = encode_frame(22, 440, (101,) * 14, (0, 1, 0))
        split = len(old) // 2

        self.assertIsNone(reader._consume_serial_bytes(old[:split], 1.3))
        reader._reset_serial_buffer()
        self.assertIsNone(reader._consume_serial_bytes(old[split:], 1.4))
        self.assertIsNone(reader.latest_frame())

        newest = reader._consume_serial_bytes(new, 1.5)

        self.assertEqual(newest.sequence, 22)
        self.assertEqual(reader.latest_frame().sequence, 22)

    def test_backlog_updates_all_statistics_but_publishes_only_newest(self):
        reader = MegaSensorReader()
        backlog = b"".join(
            encode_frame(sequence, sequence * 20, (sequence,) * 14, (1, 1, 1))
            for sequence in (30, 31, 32)
        )

        newest = reader._consume_serial_bytes(backlog, 2.0)
        status = reader.status()

        self.assertEqual(newest.sequence, 32)
        self.assertEqual(reader.latest_frame().sequence, 32)
        self.assertEqual(status["valid_frames"], 3)
        self.assertEqual(status["dropped_frames"], 0)
        self.assertEqual(reader.get_frame(timeout=0).sequence, 32)
        self.assertIsNone(reader.get_frame(timeout=0))

    def test_bad_checksum_discards_only_that_line_before_valid_frame(self):
        reader = MegaSensorReader()
        damaged = bytearray(encode_frame(40, 800, (100,) * 14, (1, 1, 1)))
        damaged[5] = ord("9")
        good = encode_frame(41, 820, (101,) * 14, (1, 1, 1))

        newest = reader._consume_serial_bytes(bytes(damaged) + good, 3.0)
        status = reader.status()

        self.assertEqual(newest.sequence, 41)
        self.assertEqual(status["valid_frames"], 1)
        self.assertEqual(status["invalid_frames"], 1)
        self.assertEqual(status["checksum_errors"], 1)
        self.assertEqual(status["last_error"], "")

    def test_valid_frame_before_bad_line_is_published_without_hiding_error(self):
        reader = MegaSensorReader()
        good = encode_frame(45, 900, (101,) * 14, (1, 1, 1))
        damaged = bytearray(encode_frame(46, 920, (102,) * 14, (1, 1, 1)))
        damaged[5] = ord("9")

        newest = reader._consume_serial_bytes(good + bytes(damaged), 3.5)
        status = reader.status()

        self.assertEqual(newest.sequence, 45)
        self.assertEqual(reader.latest_frame().sequence, 45)
        self.assertEqual(status["valid_frames"], 1)
        self.assertEqual(status["invalid_frames"], 1)
        self.assertEqual(status["checksum_errors"], 1)
        self.assertIn("CRC mismatch", status["last_error"])

    def test_oversized_line_resynchronizes_without_clearing_following_frame(self):
        reader = MegaSensorReader(max_line_bytes=128)
        good = encode_frame(50, 1000, (102,) * 14, (1, 1, 1))

        self.assertIsNone(reader._consume_serial_bytes(b"x" * 128, 4.0))
        self.assertEqual(reader.status()["invalid_frames"], 1)

        newest = reader._consume_serial_bytes(b"discarded\n" + good, 4.1)
        status = reader.status()

        self.assertEqual(newest.sequence, 50)
        self.assertEqual(status["invalid_frames"], 1)
        self.assertEqual(status["valid_frames"], 1)
        self.assertEqual(reader.latest_frame().sequence, 50)

    def test_complete_oversized_line_does_not_hide_same_chunk_valid_frame(self):
        reader = MegaSensorReader(max_line_bytes=128)
        good = encode_frame(60, 1200, (103,) * 14, (1, 1, 1))

        newest = reader._consume_serial_bytes(
            b"x" * 129 + b"\n" + good,
            5.0,
        )
        status = reader.status()

        self.assertEqual(newest.sequence, 60)
        self.assertEqual(status["invalid_frames"], 1)
        self.assertEqual(status["valid_frames"], 1)

    def test_sequence_gap_inside_one_backlog_is_counted(self):
        reader = MegaSensorReader()
        backlog = b"".join(
            encode_frame(sequence, sequence * 20, (100,) * 14, (1, 1, 1))
            for sequence in (70, 72)
        )

        reader._consume_serial_bytes(backlog, 6.0)

        self.assertEqual(reader.status()["dropped_frames"], 1)
        self.assertEqual(reader.latest_frame().sequence, 72)


if __name__ == "__main__":
    unittest.main()
