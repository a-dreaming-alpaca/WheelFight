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


if __name__ == "__main__":
    unittest.main()
