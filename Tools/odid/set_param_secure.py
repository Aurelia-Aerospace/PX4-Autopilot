#!/usr/bin/env python3
"""Set a @SECURE PX4 parameter via MAVLink SecureCommand (op 8).

Usage:
  python3 set_param_secure.py --key operator_key.json --param FW_LOCK --value 1

Protocol:
  1. GET_SESSION_KEY (op 0) — board returns ephemeral X25519 pub
  2. Derive shared session key: X25519 ECDH + BLAKE2b-32
  3. SET_PARAM (op 8) — data: name[16] | value[4] | MAC[16]
     MAC = BLAKE2b-16(key=session_key, msg="set_param" | name[16] | value[4])

Only @SECURE params (FW_LOCK, FW_SN) are accepted by the board.
"""
import argparse
import hashlib
import json
import struct
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
    def blake2b_derive(shared):
        return pyblake2.blake2b(shared, digest_size=32).digest()
    def blake2b_mac(key, msg):
        return pyblake2.blake2b(msg, digest_size=16, key=key[:32]).digest()
except ImportError:
    def blake2b_derive(shared):
        return hashlib.blake2b(shared, digest_size=32).digest()
    def blake2b_mac(key, msg):
        return hashlib.blake2b(msg, digest_size=16, key=key[:32]).digest()


SECURE_COMMAND_GET_SESSION_KEY = 0
SECURE_COMMAND_SET_PARAM       = 8
MAV_RESULT_ACCEPTED            = 0
TIMEOUT                        = 10.0


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
    ap.add_argument("--key",    required=True,  help="operator_key.json")
    ap.add_argument("--param",  required=True,  help="parameter name (e.g. FW_LOCK)")
    ap.add_argument("--value",  required=True,  type=int, help="integer value to set")
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
    print(f"Ephemeral pub: {eph_pub_bytes.hex()}")

    # Step 2: Derive session key
    x25519_key = X25519PrivateKey.from_private_bytes(x25519_scalar)
    peer_pub    = X25519PublicKey.from_public_bytes(eph_pub_bytes)
    shared      = x25519_key.exchange(peer_pub)
    session_key = blake2b_derive(shared)
    print(f"Session key: {session_key.hex()}")

    # Step 3: SET_PARAM with MAC
    name_bytes  = args.param.encode()[:15].ljust(16, b"\x00")  # null-padded to 16
    value_bytes = struct.pack("<i", args.value)                 # little-endian int32

    mac_input = b"set_param" + name_bytes + value_bytes
    mac       = blake2b_mac(session_key, mac_input)

    data = name_bytes + value_bytes + mac  # 16 + 4 + 16 = 36 bytes
    print(f"Setting {args.param} = {args.value} ...")
    send_secure_command(mav, SECURE_COMMAND_SET_PARAM, data=data, sequence=2)
    reply = wait_reply(mav, SECURE_COMMAND_SET_PARAM)
    if not reply:
        sys.exit("Timed out waiting for SET_PARAM reply")
    if reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"SET_PARAM failed: result={reply.result} (0=ok,2=denied,3=unsupported,4=failed)")

    print(f"OK — {args.param} set to {args.value}")


if __name__ == "__main__":
    main()
