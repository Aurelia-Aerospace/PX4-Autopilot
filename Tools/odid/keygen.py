#!/usr/bin/env python3
"""Generate an Ed25519 operator key pair for PX4 ODID SecureCommand.

Outputs:
  operator_key.json   -- private key (keep secret, used by generate_rid_key.py)
  operator_key.pub    -- public key in C hex format (embed as CONFIG_PUBLIC_KEY0)
"""
import os
import json
import sys

try:
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
except ImportError:
    sys.exit("pip install cryptography")

def generate_keypair(json_path: str, pub_path: str, comment: str) -> None:
    priv = Ed25519PrivateKey.generate()
    priv_bytes = priv.private_bytes_raw()
    pub_bytes  = priv.public_key().public_bytes_raw()
    with open(json_path, "w") as f:
        json.dump({"private": priv_bytes.hex(), "public": pub_bytes.hex()}, f, indent=2)
    with open(pub_path, "w") as f:
        f.write(f"//{comment}\n")
        for i in range(0, 32, 8):
            chunk = pub_bytes[i:i+8]
            f.write(", ".join(f"0x{b:02x}" for b in chunk) + ",\n")

generate_keypair("operator_key.json", "operator_key.pub",
                 "Operator public key — CONFIG_PUBLIC_KEY0 in bootloader.px4board")
generate_keypair("recovery_key.json", "recovery_key.pub",
                 "Recovery public key — CONFIG_PUBLIC_KEY1 in bootloader.px4board")

print("operator_key.json  <- keep secret, used for signing and MAVLink auth")
print("operator_key.pub   <- CONFIG_PUBLIC_KEY0 in bootloader.px4board")
print("recovery_key.json  <- keep secret, used for RDCT recovery certificates")
print("recovery_key.pub   <- CONFIG_PUBLIC_KEY1 in bootloader.px4board")
