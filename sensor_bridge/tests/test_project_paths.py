import subprocess
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_DIR = PROJECT_ROOT / "0_FinalSourceCode"


class ProjectPathTests(unittest.TestCase):
    def test_source_directory_launch_can_locate_repository_uptech_module(self):
        script = """
import importlib.util
from pathlib import Path
from project_paths import PROJECT_ROOT, add_project_root_to_path

add_project_root_to_path()
add_project_root_to_path()
spec = importlib.util.find_spec("uptech")
assert spec is not None
assert Path(spec.origin).resolve() == (PROJECT_ROOT / "uptech.py").resolve()
"""
        result = subprocess.run(
            [sys.executable, "-B", "-c", script],
            cwd=SOURCE_DIR,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
