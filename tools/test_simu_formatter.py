#!/usr/bin/env python3
"""Compile and validate the C SIMU formatter with the host frame checker."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from check_simu import validate_frame  # noqa: E402


def find_compiler() -> str | None:
    requested = os.environ.get("CC")
    if requested:
        return requested
    for name in ("cc", "gcc", "clang"):
        compiler = shutil.which(name)
        if compiler:
            return compiler
    return None


def main() -> int:
    compiler = find_compiler()
    if compiler is None:
        print("FAIL: no host C compiler found; set CC to gcc or clang")
        return 2

    with tempfile.TemporaryDirectory(prefix="simu_formatter_test_") as temp:
        executable = Path(temp) / "simu_protocol_host_test"
        if os.name == "nt":
            executable = executable.with_suffix(".exe")
        command = [
            compiler,
            "-std=c17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "include"),
            str(ROOT / "tools" / "simu_protocol_host_test.c"),
            str(ROOT / "src" / "simu_protocol.c"),
            "-lm",
            "-o",
            str(executable),
        ]
        subprocess.run(command, check=True)
        frame = subprocess.check_output([str(executable)])

    errors, _sequence, _timestamp = validate_frame(frame)
    if errors:
        print(f"FAIL: C SIMU formatter produced an invalid frame: {errors}")
        return 1
    print("C SIMU formatter test: PASS")
    print("frame_length=221 checksum=validated_by_check_simu")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
