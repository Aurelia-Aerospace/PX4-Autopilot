#!/usr/bin/env python3
"""
test_ota_loop.py
Run update_rid_firmware.py N times and report stats.

Usage:
  python3 test_ota_loop.py [--runs 100] [--operator-key ...] [--firmware ...] [--device ...]
  Any extra args after --runs are forwarded to update_rid_firmware.py.
"""

import sys
import re
import time
import subprocess
import statistics
from argparse import ArgumentParser
from pathlib import Path

SCRIPT = Path(__file__).parent.parent / "update_rid_firmware.py"

parser = ArgumentParser(description="OTA loop tester")
parser.add_argument("--runs", type=int, default=100, help="Number of OTA attempts (default: 100)")
args, forward = parser.parse_known_args()

speeds = []
results = []

for i in range(1, args.runs + 1):
    cmd = [sys.executable, str(SCRIPT)] + forward
    print(f"\n{'='*60}\nRun {i}/{args.runs}\n{'='*60}", flush=True)

    lines = []
    # ponytail: stream output line-by-line so it appears in real time
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as proc:
        for line in proc.stdout:
            # replace \r with \n so progress updates print as separate lines
            line = line.replace('\r', '\n').rstrip('\n') + '\n'
            print(line, end="", flush=True)
            lines.append(line)
        proc.wait()

    out = "".join(lines)
    success = "Firmware validated" in out
    speed_match = re.findall(r'(\d+(?:\.\d+)?)\s+KB/s', out)
    run_speed = float(speed_match[-1]) if speed_match else None

    results.append(success)
    if run_speed is not None:
        speeds.append(run_speed)

    status = "OK" if success else "FAIL"
    speed_str = f"{run_speed:.1f} KB/s" if run_speed else "n/a"
    print(f"[{status}] run {i}: {speed_str}", flush=True)

    if i < args.runs:
        print(f"Waiting 10s before next run...", flush=True)
        time.sleep(10)

ok   = sum(results)
fail = len(results) - ok

print(f"""
{'='*60}
RESULTS  ({args.runs} runs)
  Success : {ok}
  Failure : {fail}
  Rate    : {ok/len(results)*100:.1f}%
""", end="")

if speeds:
    print(f"  Speed (KB/s)\n"
          f"    mean : {statistics.mean(speeds):.1f}\n"
          f"    min  : {min(speeds):.1f}\n"
          f"    max  : {max(speeds):.1f}")

print("=" * 60)
