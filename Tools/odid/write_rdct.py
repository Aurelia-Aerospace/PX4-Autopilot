#!/usr/bin/env python3
"""Write a signed RDCT recovery certificate to the FC via MAVLink SecureCommand (op 10).

Allows booting unsigned firmware until the bootloader is DFU-reflashed (cert is
one-time-write — lives in the last 512 bytes of bootloader flash sector 0).

Usage:
  python3 write_rdct.py --operator-key operator_key.json --recovery-key recovery_key.json

The operator key (key[0]) authenticates the MAVLink session.
The recovery key (key[1]) signs the RDCT cert itself (embedded in the bootloader).
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
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey, Ed25519PublicKey
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
SECURE_COMMAND_WRITE_RDCT      = 10
MAV_RESULT_ACCEPTED            = 0
TIMEOUT                        = 10.0

# image_cert_t layout (packed, no padding):
#   device_uuid[16]   uint8
#   caps[4]           uint32 LE
#   creator_info[16]  uint8
#   customer_info[16] uint8
#   creation_date     uint64 LE
#   valid_until       uint64 LE
# total data = 80 bytes, then Ed25519 signature (64 bytes)
CERT_DATA_FMT = "<16s4I16s16sQQ"   # 80 bytes
CERT_DATA_SIZE = struct.calcsize(CERT_DATA_FMT)  # 80
CERT_FULL_SIZE = CERT_DATA_SIZE + 64             # 144

RDCT_CAPS0_ALLOW_UNSIGNED_BOOT = 0x1


def ed25519_seed_to_x25519(seed_hex: str) -> bytes:
    seed = bytes.fromhex(seed_hex)
    h = hashlib.sha512(seed).digest()
    scalar = bytearray(h[:32])
    scalar[0]  &= 248
    scalar[31] &= 127
    scalar[31] |= 64
    return bytes(scalar)


def build_rdct_cert(recovery_priv_hex: str, valid_seconds: int = 3600) -> bytes:
    now = int(time.time())
    cert_data = struct.pack(
        CERT_DATA_FMT,
        b"\x00" * 16,                                    # device_uuid (zeros = any device)
        RDCT_CAPS0_ALLOW_UNSIGNED_BOOT, 0, 0, 0,          # caps[4]
        b"px4-odid-recovery\x00\x00\x00\x00\x00\x00"[:16], # creator_info
        b"\x00" * 16,                                    # customer_info
        now,                                             # creation_date
        now + valid_seconds,                             # valid_until
    )
    assert len(cert_data) == CERT_DATA_SIZE

    priv = Ed25519PrivateKey.from_private_bytes(bytes.fromhex(recovery_priv_hex))
    signature = priv.sign(cert_data)                     # 64 bytes
    return cert_data + signature


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
    ap.add_argument("--operator-key",  required=True, help="operator_key.json (session auth)")
    ap.add_argument("--recovery-key",  required=True, help="recovery_key.json (cert signing)")
    ap.add_argument("--valid",         type=int, default=3600, help="cert validity in seconds (default 3600)")
    ap.add_argument("--device",        default="udpin:0.0.0.0:14550")
    ap.add_argument("--baud",          type=int, default=57600)
    args = ap.parse_args()

    with open(args.operator_key)  as f: op_key  = json.load(f)
    with open(args.recovery_key) as f: rec_key = json.load(f)

    x25519_scalar = ed25519_seed_to_x25519(op_key["private"])

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
    print(f"Session key: {session_key.hex()}")

    # Step 3: Build and sign RDCT cert
    cert_bytes = build_rdct_cert(rec_key["private"], valid_seconds=args.valid)
    print(f"RDCT cert ({len(cert_bytes)} bytes), valid for {args.valid}s")

    # Step 4: WRITE_RDCT with MAC
    label    = b"write_rdct"
    mac      = blake2b_mac(session_key, label + cert_bytes)
    data     = mac + cert_bytes          # 16 + 144 = 160 bytes

    if len(data) > 220:
        sys.exit(f"Cert too large for SecureCommand payload ({len(data)} > 220)")

    print("Sending WRITE_RDCT...")
    send_secure_command(mav, SECURE_COMMAND_WRITE_RDCT, data=data, sequence=2)
    reply = wait_reply(mav, SECURE_COMMAND_WRITE_RDCT)
    if not reply:
        sys.exit("Timed out waiting for WRITE_RDCT reply")
    if reply.result == 5:
        sys.exit("WRITE_RDCT: cert already written (result=5). To clear it, DFU-reflash the bootloader.")
    if reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"WRITE_RDCT failed: result={reply.result} (0=ok,2=denied,4=failed,5=already_written)")

    print("OK — RDCT cert written. Reboot the board, then flash unsigned firmware.")
    print("Note: cert persists until bootloader is DFU-reflashed (clears sector 0).")


if __name__ == "__main__":
    main()
