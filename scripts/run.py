#!/usr/bin/env python3
import subprocess
from pathlib import Path

def run_tests():
    build_dir = Path("build")
    test_exe = build_dir / "tests" / "ControllerTests"

    if not test_exe.exists():
        print("Test executable missing. Building first...")
        subprocess.run(["python ./scripts/build.py"], check=True)

    print("\nRunning tests...")
    try:
        subprocess.run([test_exe, "--gtest_output=xml:test_results.xml"], check=True)
    except subprocess.CalledProcessError:
        print("\nTests failed!")
        exit(1)

if __name__ == "__main__":
    run_tests()
