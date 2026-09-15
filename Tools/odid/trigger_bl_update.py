#!/usr/bin/env python3
"""Trigger a bootloader update on the FC via MAVLink SecureCommand (op 11).

The firmware tries paths in order:
  1. /etc/extras/cubepilot_cubeorange_bootloader.bin   (ROMFS, always present)
  2. /etc/extras/cubepilot_cubeorangeplus_bootloader.bin
  3. /fs/microsd/bootloader.bin                        (SD fallback, custom bootloader)

No SD card needed for a standard bootloader update.

Usage:
  python3 trigger_bl_update.py --key operator_key.json

Requires: pip install pymavlink cryptography
"""
import argparse
import hashlib
import json
import sys
import time

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


SECURE_COMMAND_GET_SESSION_KEY  = 0
SECURE_COMMAND_TRIGGER_BL_UPDATE = 13
MAV_RESULT_ACCEPTED             = 0
TIMEOUT                         = 10.0


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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--key",    required=True, help="operator_key.json")
    ap.add_argument("--device", default="udpin:0.0.0.0:14550")
    ap.add_argument("--baud",   type=int, default=57600)
    args = ap.parse_args()

    with open(args.key) as f:
        key_data = json.load(f)

    x25519_scalar = ed25519_seed_to_x25519(key_data["private"])

    print(f"Connecting to {args.device} ...")
    mav = mavutil.mavlink_connection(args.device, baud=args.baud)
    mav.wait_heartbeat(timeout=10)
    print(f"Heartbeat: system {mav.target_system} component {mav.target_component}")

    # Step 1: GET_SESSION_KEY
    print("Requesting session key...")
    send_secure_command(mav, SECURE_COMMAND_GET_SESSION_KEY, sequence=1)
    reply = wait_reply(mav, SECURE_COMMAND_GET_SESSION_KEY)
    if not reply or reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"GET_SESSION_KEY failed: {reply.result if reply else 'timeout'}")

    eph_pub_bytes = bytes(reply.data[:32])

    # Step 2: Derive session key
    x25519_key  = X25519PrivateKey.from_private_bytes(x25519_scalar)
    peer_pub    = X25519PublicKey.from_public_bytes(eph_pub_bytes)
    shared      = x25519_key.exchange(peer_pub)
    session_key = blake2b_derive(shared)

    # Step 3: TRIGGER_BL_UPDATE with MAC
    mac  = blake2b_mac(session_key, b"bl_update")
    data = mac   # 16 bytes

    print("Sending TRIGGER_BL_UPDATE — board will run bl_update and reboot...")
    send_secure_command(mav, SECURE_COMMAND_TRIGGER_BL_UPDATE, data=data, sequence=2)
    reply = wait_reply(mav, SECURE_COMMAND_TRIGGER_BL_UPDATE)
    if not reply:
        sys.exit("Timed out — board may have rebooted before replying")
    if reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"TRIGGER_BL_UPDATE failed: result={reply.result} (0=ok,2=denied,4=failed)")

    print("OK — bl_update launched. Board will reboot with new bootloader.")


if __name__ == "__main__":
    main()
