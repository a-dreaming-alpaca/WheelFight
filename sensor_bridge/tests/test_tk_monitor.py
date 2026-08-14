import json
import sys
import tempfile
import time
import unittest
from pathlib import Path


sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "0_FinalSourceCode")
)

from tk_monitor import HTML_PAGE, StatusReader  # noqa: E402


class StatusReaderTests(unittest.TestCase):
    def test_page_marks_disabled_ir_channels_without_hiding_raw_values(self):
        self.assertIn("sensor.disabled_ir_indices", HTML_PAGE)
        self.assertIn("DISABLED", HTML_PAGE)
        self.assertIn("raw[i]", HTML_PAGE)

    def test_reads_controller_status_without_opening_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "match_status.json"
            status_path.write_text(
                json.dumps({"timestamp": time.time(), "state": "ARENA_SEARCH"}),
                encoding="utf-8",
            )
            status = StatusReader(str(status_path)).read()
            self.assertEqual(status["state"], "ARENA_SEARCH")
            self.assertTrue(status["monitor"]["file_ok"])
            self.assertFalse(status["monitor"]["stale"])

    def test_missing_status_file_is_reported(self):
        status = StatusReader("definitely-not-present.json").read()
        self.assertFalse(status["monitor"]["file_ok"])
        self.assertTrue(status["monitor"]["stale"])


if __name__ == "__main__":
    unittest.main()
