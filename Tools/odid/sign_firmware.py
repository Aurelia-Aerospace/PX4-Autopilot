#!/usr/bin/env python3
"""Sign a PX4 firmware binary for secure boot.

The build embeds a TOC (table of contents) in the .main_toc linker section,
placed just after the bootdelay signature (right after the vector table).
This script scans the first 4KB for the TOC magic, signs the full firmware
with Ed25519, and appends the 64-byte signature.

Usage:
  python3 sign_firmware.py --key keys/operator_key.json \
      --input build/cubepilot_cubeorange-odid_default/cubepilot_cubeorange-odid_default.bin \
      --output firmware_signed.bin

Requires: pip install cryptography
"""
import argparse
import json
import struct
import sys

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
except ImportError:
    sys.exit("pip install cryptography")

TOC_START_MAGIC = 0x00434f54   # "TOC\0"
TOC_SEARCH_MAX  = 0x1000       # search within first 4KB
SIGNATURE_SIZE  = 64


def find_toc_offset(firmware: bytes) -> int:
    for off in range(0, min(TOC_SEARCH_MAX, len(firmware) - 4), 4):
        if struct.unpack_from("<I", firmware, off)[0] == TOC_START_MAGIC:
            return off
    return -1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--key",    required=True, help="operator_key.json")
    ap.add_argument("--input",  required=True, help="unsigned firmware .bin")
    ap.add_argument("--output", required=True, help="signed firmware .bin")
    args = ap.parse_args()

    with open(args.key) as f:
        key_data = json.load(f)

    with open(args.input, "rb") as f:
        firmware = f.read()

    toc_offset = find_toc_offset(firmware)
    if toc_offset < 0:
        sys.exit(
            f"TOC magic 0x{TOC_START_MAGIC:08x} not found in first 0x{TOC_SEARCH_MAX:x} bytes.\n"
            "Build with toc.c included (check board CMakeLists.txt)."
        )

    print(f"TOC:    found at binary offset 0x{toc_offset:x}")

    priv = Ed25519PrivateKey.from_private_bytes(bytes.fromhex(key_data["private"]))
    signature = priv.sign(firmware)

    with open(args.output, "wb") as f:
        f.write(firmware)
        f.write(signature)

    print(f"Input:  {args.input} ({len(firmware)} bytes)")
    print(f"Output: {args.output} ({len(firmware) + SIGNATURE_SIZE} bytes)")
    print(f"Sig:    {signature.hex()}")


if __name__ == "__main__":
    main()
