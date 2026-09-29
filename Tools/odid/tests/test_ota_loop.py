#!/usr/bin/env python3
"""
test_ota_loop.py
Run update_rid_firmware.py N times and report stats.

Usage:
  python3 test_ota_loop.py [--runs 100] [--operator-key ...] [--firmware ...] [--device ...]
  Any extra args after --runs are forwarded to update_rid_firmware.py.
"""

import csv
import sys
import re
import time
import subprocess
import statistics
from argparse import ArgumentParser
from datetime import datetime, timezone
from pathlib import Path

SCRIPT = Path(__file__).parent.parent / "update_rid_firmware.py"
RESULTS_DIR = Path(__file__).parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

parser = ArgumentParser(description="OTA loop tester")
parser.add_argument("--runs", type=int, default=100, help="Number of OTA attempts (default: 100)")
args, forward = parser.parse_known_args()

session_ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
result_file = RESULTS_DIR / f"ota_{session_ts}.csv"
csv_fh = result_file.open("w", newline="")
writer = csv.writer(csv_fh)
writer.writerow(["timestamp", "run", "status", "speed_kbs"])
csv_fh.flush()

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
    speed_match = re.findall(r'(\d+(?:\.\d+)?)\s+KB/s', out)
    run_speed = float(speed_match[-1]) if speed_match else None

    if "Firmware validated" in out:
        status = "OK"
    elif any(f": {r}" in out for r in ("DENIED", "UNSUPPORTED", "FAILED")):
        status = "MODULE_REJECTED"
    else:
        status = "PYTHON_FAIL"

    results.append(status)
    if run_speed is not None:
        speeds.append(run_speed)

    speed_str = f"{run_speed:.1f} KB/s" if run_speed else "n/a"
    print(f"[{status}] run {i}: {speed_str}", flush=True)

    writer.writerow([datetime.now(timezone.utc).isoformat(), i, status, run_speed or ""])
    csv_fh.flush()

    if i < args.runs:
        print(f"Waiting 30s before next run...", flush=True)
        time.sleep(30)

ok       = results.count("OK")
rejected = results.count("MODULE_REJECTED")
pyfail   = results.count("PYTHON_FAIL")
total    = len(results)

print(f"""
{'='*60}
RESULTS  ({total} runs)
  OK             : {ok}
  Module rejected: {rejected}
  Python fail    : {pyfail}
  Success rate   : {ok/total*100:.1f}%
""", end="")

if speeds:
    print(f"  Speed (KB/s)\n"
          f"    mean : {statistics.mean(speeds):.1f}\n"
          f"    min  : {min(speeds):.1f}\n"
          f"    max  : {max(speeds):.1f}")

print("=" * 60)

csv_fh.close()
print(f"Results saved to {result_file}")
