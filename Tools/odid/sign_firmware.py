#!/usr/bin/env python3
"""Sign a PX4 firmware for secure boot. Accepts .px4 or .bin input.

Usage:
  python3 sign_firmware.py --key keys/operator_key.json \
      --input build/cubepilot_cubeorange-odid_default/cubepilot_cubeorange-odid_default.px4 \
      --output firmware_signed.px4

Requires: pip install cryptography
"""
import argparse
import base64
import json
import struct
import sys
import zlib

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
except ImportError:
    sys.exit("pip install cryptography")

TOC_START_MAGIC = 0x00434f54
TOC_SEARCH_MAX  = 0x1000
SIGNATURE_SIZE  = 64


def find_toc_offset(firmware: bytes) -> int:
    for off in range(0, min(TOC_SEARCH_MAX, len(firmware) - 4), 4):
        if struct.unpack_from("<I", firmware, off)[0] == TOC_START_MAGIC:
            return off
    return -1


def load_firmware(path: str) -> tuple[bytes, dict | None]:
    """Returns (raw binary, px4_pkg or None)."""
    with open(path, "rb") as f:
        header = f.read(4)
    if header[:1] == b"{":
        with open(path) as f:
            pkg = json.load(f)
        firmware = zlib.decompress(base64.b64decode(pkg["image"]))
        return firmware, pkg
    with open(path, "rb") as f:
        return f.read(), None


def save_output(path: str, signed: bytes, pkg: dict | None) -> None:
    if pkg is not None:
        pkg["image"]      = base64.b64encode(zlib.compress(signed, level=9)).decode()
        pkg["image_size"] = len(signed)
        with open(path, "w") as f:
            json.dump(pkg, f)
    else:
        with open(path, "wb") as f:
            f.write(signed)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--key",    required=True, help="operator_key.json")
    ap.add_argument("--input",  required=True, help="unsigned firmware (.px4 or .bin)")
    ap.add_argument("--output", required=True, help="signed firmware (.px4 or .bin)")
    args = ap.parse_args()

    with open(args.key) as f:
        key_data = json.load(f)

    firmware, pkg = load_firmware(args.input)

    toc_offset = find_toc_offset(firmware)
    if toc_offset < 0:
        sys.exit(
            f"TOC magic 0x{TOC_START_MAGIC:08x} not found in first 0x{TOC_SEARCH_MAX:x} bytes.\n"
            "Build with toc.c included (check board CMakeLists.txt)."
        )

    print(f"TOC:    found at binary offset 0x{toc_offset:x}")

    priv = Ed25519PrivateKey.from_private_bytes(bytes.fromhex(key_data["private"]))
    signed = firmware + priv.sign(firmware)

    save_output(args.output, signed, pkg)

    fmt = ".px4" if pkg else ".bin"
    print(f"Input:  {args.input} ({len(firmware)} bytes)")
    print(f"Output: {args.output} ({len(signed)} bytes, {fmt})")
    print(f"Sig:    {signed[-SIGNATURE_SIZE:].hex()}")


if __name__ == "__main__":
    main()
