#!/usr/bin/env python3
"""
test_set_param_loop.py
Run set_param_secure.py N times and report stats.

Usage:
  python3 test_set_param_loop.py [--runs 100] [--delay 10] [--key ...] [--param ...] [--value ...] [--device ...]
  Any extra args after --runs/--delay are forwarded to set_param_secure.py.
"""

import sys
import re
import time
import subprocess
import statistics
from argparse import ArgumentParser
from pathlib import Path

SCRIPT = Path(__file__).parent.parent / "set_param_secure.py"

parser = ArgumentParser(description="set_param_secure loop tester")
parser.add_argument("--runs",  type=int,   default=100, help="Number of attempts (default: 100)")
parser.add_argument("--delay", type=float, default=10,  help="Seconds between runs (default: 10)")
args, forward = parser.parse_known_args()

durations = []
results = []

for i in range(1, args.runs + 1):
    cmd = [sys.executable, str(SCRIPT)] + forward
    print(f"\n{'='*60}\nRun {i}/{args.runs}\n{'='*60}", flush=True)

    t0 = time.time()
    lines = []
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as proc:
        for line in proc.stdout:
            line = line.replace('\r', '\n').rstrip('\n') + '\n'
            print(line, end="", flush=True)
            lines.append(line)
        proc.wait()
    elapsed = time.time() - t0

    out = "".join(lines)
    success = "OK —" in out and "set to" in out

    results.append(success)
    durations.append(elapsed)

    status = "OK" if success else "FAIL"
    print(f"[{status}] run {i}: {elapsed:.1f}s", flush=True)

    if i < args.runs:
        print(f"Waiting {args.delay:.0f}s before next run...", flush=True)
        time.sleep(args.delay)

ok   = sum(results)
fail = len(results) - ok

print(f"""
{'='*60}
RESULTS  ({args.runs} runs)
  Success : {ok}
  Failure : {fail}
  Rate    : {ok/len(results)*100:.1f}%
  Time (s)
    mean  : {statistics.mean(durations):.1f}
    min   : {min(durations):.1f}
    max   : {max(durations):.1f}
{'='*60}""")
