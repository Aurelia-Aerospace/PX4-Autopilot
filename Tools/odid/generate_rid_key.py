#!/usr/bin/env python3
"""Generate a RID key pair on a PX4 ODID board via MAVLink SecureCommand.

Usage:
  python3 generate_rid_key.py --key operator_key.json [--device /dev/ttyUSB0] [--baud 57600]

Protocol:
  1. GET_SESSION_KEY  -> board returns ephemeral X25519 pub (32 bytes)
  2. Derive shared session key via X25519 + BLAKE2b
  3. GENERATE_RID_KEY -> board generates Ed25519 RID key pair, returns public key

Requires: pip install pymavlink cryptography pyblake2
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
    from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey
except ImportError:
    sys.exit("pip install cryptography")

try:
    import pyblake2
    def blake2b_32(key, msg):
        h = pyblake2.blake2b(digest_size=16, key=key[:32])
        h.update(msg)
        return h.digest()
    def blake2b_derive(shared):
        h = pyblake2.blake2b(digest_size=32)
        h.update(shared)
        return h.digest()
except ImportError:
    # fallback: hashlib blake2b (Python 3.6+, but no key= on all platforms)
    def blake2b_32(key, msg):
        return hashlib.blake2b(msg, digest_size=16, key=key[:32]).digest()
    def blake2b_derive(shared):
        return hashlib.blake2b(shared, digest_size=32).digest()


SECURE_COMMAND_GET_SESSION_KEY        = 0
SECURE_COMMAND_GET_REMOTEID_SESSION_KEY = 1
SECURE_COMMAND_SET_PARAM              = 8
SECURE_COMMAND_GENERATE_RID_KEY       = 9

MAV_RESULT_ACCEPTED = 0
TIMEOUT = 10.0


def ed25519_seed_to_x25519(seed_hex: str) -> bytes:
    """Convert Ed25519 private seed to X25519 scalar (RFC 8032 clamping)."""
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
    print(f"Heartbeat from system {mav.target_system} component {mav.target_component}")

    # Step 1: GET_SESSION_KEY
    print("Requesting session key...")
    send_secure_command(mav, SECURE_COMMAND_GET_SESSION_KEY, sequence=1)
    reply = wait_reply(mav, SECURE_COMMAND_GET_SESSION_KEY)
    if not reply:
        sys.exit("Timed out waiting for session key")
    if reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"GET_SESSION_KEY failed: result={reply.result}")

    eph_pub = bytes(reply.data[:32])
    print(f"Ephemeral pub: {eph_pub.hex()}")

    # Step 2: Derive session key
    x25519_key = X25519PrivateKey.from_private_bytes(x25519_scalar)
    from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PublicKey
    peer_pub   = X25519PublicKey.from_public_bytes(eph_pub)
    shared     = x25519_key.exchange(peer_pub)
    session_key = blake2b_derive(shared)
    print(f"Session key derived: {session_key.hex()}")

    # Step 3: GENERATE_RID_KEY authenticated with MAC
    mac = blake2b_32(session_key, b"gen_rid_key")
    print("Generating RID key pair on board...")
    send_secure_command(mav, SECURE_COMMAND_GENERATE_RID_KEY, data=mac, sequence=2)
    reply = wait_reply(mav, SECURE_COMMAND_GENERATE_RID_KEY)
    if not reply:
        sys.exit("Timed out waiting for GENERATE_RID_KEY reply")
    if reply.result != MAV_RESULT_ACCEPTED:
        sys.exit(f"GENERATE_RID_KEY failed: result={reply.result}")

    rid_pub = bytes(reply.data[:32])
    print(f"\nRID public key (Ed25519): {rid_pub.hex()}")
    with open("rid_pubkey.hex", "w") as f:
        f.write(rid_pub.hex() + "\n")
    print("Saved to rid_pubkey.hex")


if __name__ == "__main__":
    main()




#!/bin/bash
  set -e

  # 1. Compilar bootloader seguro
  make cubepilot_cubeorange-odid_bootloader

  # 2. Reemplazar el binario pre-compilado en extras con el tuyo
  cp build/cubepilot_cubeorange-odid_bootloader/cubepilot_cubeorange-odid_bootloader
  .bin \
     boards/cubepilot/cubeorange-odid/extras/cubepilot_cubeorange_bootloader.bin

  # 3. Compilar firmware (ahora embebe tu bootloader seguro en ROMFS)
  make cubepilot_cubeorange-odid_default

  # 4. Firmar firmware
  python3 Tools/odid/sign_firmware.py \
    --key keys/operator_key.json \
    --input
  build/cubepilot_cubeorange-odid_default/cubepilot_cubeorange-odid_default.bin \
    --output firmware_signed.bin

  # 5. Flashear firmware firmado (via QGC o uploader) ← aquí el usuario conecta el
  FC
  # ...

  # 6. Desde firmware corriendo: promover bootloader seguro
  python3 Tools/odid/trigger_bl_update.py --key keys/operator_key.json

  Esto funciona porque:
  - Paso 5 funciona con el bootloader ACTUAL (no seguro) — acepta cualquier firmware
  - El firmware firmado ya embebe el bootloader seguro en ROMFS
  - Paso 6 instala el bootloader seguro desde ROMFS y el board reinicia
  - Después del reboot, el bootloader seguro verifica el firmware firmado → arranca

  ---
  Comparación de enfoques

  Column 1: Herramienta extra
  DFU una vez: Cable DFU / BOOT pin
  Script sin DFU: Solo QGC
  ────────────────────────────────────────
  Column 1: Complejidad
  DFU una vez: Una vez, luego olvidar
  Script sin DFU: Cuidar el extras/ en git
  ────────────────────────────────────────
  Column 1: Riesgo
  DFU una vez: Mínimo
  Script sin DFU: Si commit del .bin viejo, pierdes trazabilidad
  ────────────────────────────────────────
  Column 1: Para updates futuros
  DFU una vez: Solo firmar + flashear
  Script sin DFU: Mismo script

  Recomendación: usa DFU para el bootloader inicial una sola vez. Es el mecanismo
  correcto — flashing del bootloader es una operación de bajo nivel. Después de ese
  setup, todo lo demás (firmware updates) es sign + QGC, sin DFU nunca más.
