"""Stable repository paths shared by executable entry points."""

from __future__ import annotations

import os
import sys
from pathlib import Path


SOURCE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SOURCE_DIR.parent


def add_project_root_to_path() -> Path:
    """Make the repository-level hardware modules importable.

    In particular, the vendor wrapper ``uptech.py`` lives in the repository
    root while the executable control scripts live in ``0_FinalSourceCode``.
    Resolve paths from this file rather than from the caller's working
    directory so both supported launch locations behave identically.
    """

    root_text = str(PROJECT_ROOT)
    normalized_root = os.path.normcase(os.path.abspath(root_text))
    normalized_paths = {
        os.path.normcase(os.path.abspath(path or os.curdir)) for path in sys.path
    }
    if normalized_root not in normalized_paths:
        sys.path.insert(0, root_text)
    return PROJECT_ROOT


__all__ = ["PROJECT_ROOT", "SOURCE_DIR", "add_project_root_to_path"]
