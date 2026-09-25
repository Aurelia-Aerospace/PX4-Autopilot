#!/usr/bin/env python3
"""
update_rid_firmware.py
OTA firmware update for the ArduRemoteID module via MAVLink SECURE_COMMAND.

Flow:
  1. GET_SESSION_KEY (op=0): ECDH with operator key → session_key (same as write_rdct.py)
  2. OTA_BEGIN (op=11): BLAKE2b-16 MAC over "ota_begin" || fw_size
  3. OTA_CHUNK (op=12): raw chunks [flags(1)] [offset(4)] [data], no signature
     FLAG_FIRST=0x01 on first chunk, FLAG_LAST=0x02 on last chunk
     Retry on TEMPORARILY_REJECTED (FC waiting for DroneCAN ACK from ESP32)

Usage:
  python3 update_rid_firmware.py --operator-key keys/operator_key.json \
      --firmware rid_firmware.bin --device udpin:0.0.0.0:14551
"""

import sys
import struct
import random
import hashlib
import json
import time
from argparse import ArgumentParser

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pip install pymavlink")

try:
    from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey, X25519PublicKey
except ImportError:
    sys.exit("pip install cryptography")

try:
    import pyblake2
    def blake2b_derive(data): return pyblake2.blake2b(data, digest_size=32).digest()
    def blake2b_mac(key, msg): return pyblake2.blake2b(msg, digest_size=16, key=key[:32]).digest()
except ImportError:
    def blake2b_derive(data): return hashlib.blake2b(data, digest_size=32).digest()
    def blake2b_mac(key, msg): return hashlib.blake2b(msg, digest_size=16, key=key[:32]).digest()


SECURE_COMMAND_GET_SESSION_KEY = 0
SECURE_COMMAND_OTA_BEGIN       = 11
SECURE_COMMAND_OTA_CHUNK       = 12
MAV_RESULT_ACCEPTED            = 0
TIMEOUT                        = 15.0

CHUNK_SIZE = 200  # max firmware bytes per DroneCAN transfer (header is 5 bytes: flags+offset; 220-5=215 max)

FLAG_FIRST = 0x01
FLAG_LAST  = 0x02

RESULTS = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED"}


def ed25519_seed_to_x25519(seed_hex: str) -> bytes:
    seed = bytes.fromhex(seed_hex)
    h = hashlib.sha512(seed).digest()
    scalar = bytearray(h[:32])
    scalar[0]  &= 248
    scalar[31] &= 127
    scalar[31] |= 64
    return bytes(scalar)


def send_secure_command(mav, operation: int, data: bytes = b"", sequence: int = 0):
    payload = bytearray(220)
    payload[:len(data)] = data
    mav.mav.secure_command_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        sequence=sequence,
        operation=operation,
        sig_length=0,
        data_length=len(data),
        data=list(payload),
    )


def wait_reply(mav, operation: int, timeout: float = TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type="SECURE_COMMAND_REPLY", blocking=True, timeout=1.0)
        if msg and msg.operation == operation:
            return msg
    return None


parser = ArgumentParser(description='OTA firmware update for ArduRemoteID module')
parser.add_argument("--operator-key", required=True, help="keys/operator_key.json")
parser.add_argument("--firmware",     required=True, help="Firmware binary (.bin)")
parser.add_argument("--device",       default="udpin:0.0.0.0:14551")
parser.add_argument("--baud",         type=int, default=115200)
parser.add_argument("--timeout",      type=float, default=TIMEOUT,
                    help="Timeout per command in seconds (default: 15)")
parser.add_argument("--begin-timeout", type=float, default=60.0,
                    help="Timeout for OTA_BEGIN in seconds — waits for ESP32 partition erase (default: 60)")
parser.add_argument("--retry-delay",  type=float, default=0.0,
                    help="Delay between retries on TEMPORARILY_REJECTED (default: 0.0s)")
parser.add_argument("--chunk-reply-timeout", type=float, default=5.0,
                    help="Timeout waiting for reply per chunk attempt (default: 5.0s)")
parser.add_argument("--max-retries", type=int, default=10,
                    help="Max retry attempts per chunk before aborting (default: 10)")
parser.add_argument("--max-size",     type=int, default=2 * 1024 * 1024,
                    help="Maximum firmware size in bytes (default: 2MB)")
args = parser.parse_args()


def send_chunk(mav, sequence, data_bytes):
    payload = bytearray(220)
    payload[:len(data_bytes)] = data_bytes
    mav.mav.secure_command_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        sequence=sequence,
        operation=SECURE_COMMAND_OTA_CHUNK,
        sig_length=0,
        data_length=len(data_bytes),
        data=list(payload),
    )


def send_with_retry(mav, sequence, data_bytes, timeout, retry_delay, label, reply_timeout):
    deadline = time.time() + timeout
    attempts = 0
    while time.time() < deadline:
        if attempts >= args.max_retries:
            print(f"  {label}: max retries ({args.max_retries}) exceeded")
            return False
        send_chunk(mav, sequence, data_bytes)
        attempts += 1
        reply = wait_reply(mav, SECURE_COMMAND_OTA_CHUNK, timeout=reply_timeout)
        if reply is None:
            print(f"  {label}: timeout (attempt {attempts}/{args.max_retries}), retrying...")
            time.sleep(0.2)
            continue
        if reply.result == MAV_RESULT_ACCEPTED:
            return True
        if reply.result == 1:  # TEMPORARILY_REJECTED — FC waiting for DroneCAN ACK
            attempts -= 1  # don't count waiting as a real attempt
            time.sleep(retry_delay)
            continue
        print(f"  {label}: {RESULTS.get(reply.result, f'unknown({reply.result})')}")
        return False
    print(f"  {label}: overall timeout exceeded")
    return False


def main():
    with open(args.operator_key) as f:
        op_key = json.load(f)
    x25519_scalar = ed25519_seed_to_x25519(op_key["private"])

    firmware = open(args.firmware, 'rb').read()
    fw_size = len(firmware)
    print(f"Firmware: {args.firmware} ({fw_size} bytes)")

    if fw_size > args.max_size:
        sys.exit(f"ERROR: firmware ({fw_size} bytes) exceeds max size ({args.max_size} bytes)")

    chunks = [firmware[i:i + CHUNK_SIZE] for i in range(0, fw_size, CHUNK_SIZE)]
    n_chunks = len(chunks)
    print(f"Chunks: {n_chunks} x {CHUNK_SIZE} bytes")

    print(f"Connecting to {args.device} ...")
    mav = mavutil.mavlink_connection(args.device, baud=args.baud, dialect='ardupilotmega')
    mav.wait_heartbeat(timeout=10)
    print(f"Heartbeat: system {mav.target_system} component {mav.target_component}")

    # Step 1: GET_SESSION_KEY — ECDH (same as write_rdct.py)
    print("Requesting session key...")
    send_secure_command(mav, SECURE_COMMAND_GET_SESSION_KEY, sequence=1)
    reply = wait_reply(mav, SECURE_COMMAND_GET_SESSION_KEY)
    if not reply or reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"GET_SESSION_KEY failed: {reply.result if reply else 'timeout'}")

    eph_pub_bytes = bytes(reply.data[:32])
    x25519_key  = X25519PrivateKey.from_private_bytes(x25519_scalar)
    peer_pub    = X25519PublicKey.from_public_bytes(eph_pub_bytes)
    shared      = x25519_key.exchange(peer_pub)
    session_key = blake2b_derive(shared)
    print(f"Session key: {session_key.hex()}")

    # Step 2: OTA_BEGIN — retry on TEMPORARILY_REJECTED while ESP32 erases partition
    fw_size_bytes = struct.pack("<I", fw_size)
    mac  = blake2b_mac(session_key, b"ota_begin" + fw_size_bytes)
    data = mac + fw_size_bytes  # 16 + 4 = 20 bytes

    print("Sending OTA_BEGIN (waiting for partition erase)...")
    deadline = time.time() + args.begin_timeout
    while True:
        send_secure_command(mav, SECURE_COMMAND_OTA_BEGIN, data=data, sequence=2)
        reply = wait_reply(mav, SECURE_COMMAND_OTA_BEGIN)
        if not reply:
            sys.exit("Timed out waiting for OTA_BEGIN reply")
        if reply.result == MAV_RESULT_ACCEPTED:
            break
        if reply.result == 1:  # TEMPORARILY_REJECTED — still erasing
            if time.time() >= deadline:
                sys.exit("OTA_BEGIN timed out waiting for erase")
            time.sleep(0.5)
            continue
        sys.exit(f"OTA_BEGIN rejected: result={reply.result} (0=ok,2=denied,4=failed)")
    print("OTA partition ready")

    # Step 3: firmware chunks
    start_time = time.time()
    for i, chunk in enumerate(chunks):
        flags = 0
        if i == 0:         flags |= FLAG_FIRST
        if i == n_chunks - 1: flags |= FLAG_LAST
        offset = i * CHUNK_SIZE

        chunk_data = bytearray([flags]) + struct.pack("<I", offset) + bytearray(chunk)
        label = f"chunk {i + 1}/{n_chunks} offset={offset}"

        chunk_timeout  = args.timeout
        reply_timeout  = args.timeout if (flags & FLAG_LAST) else args.chunk_reply_timeout
        ok = send_with_retry(mav, i + 3, chunk_data, chunk_timeout,
                             args.retry_delay, label, reply_timeout)
        if not ok:
            sys.exit(f"OTA failed at {label}")

        pct     = (i + 1) * 100 // n_chunks
        elapsed = time.time() - start_time
        rate    = (offset + len(chunk)) / elapsed if elapsed > 0 else 0
        print(f"\r  {pct}% ({i + 1}/{n_chunks}) {rate / 1024:.1f} KB/s  ", end='', flush=True)

    elapsed = time.time() - start_time
    print(f"\nFirmware validated — module rebooting ({elapsed:.1f}s)")


if __name__ == '__main__':
    main()
