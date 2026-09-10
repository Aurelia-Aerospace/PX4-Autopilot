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

priv = Ed25519PrivateKey.generate()
priv_bytes = priv.private_bytes_raw()
pub_bytes  = priv.public_key().public_bytes_raw()

key_json = {"private": priv_bytes.hex(), "public": pub_bytes.hex()}
with open("operator_key.json", "w") as f:
    json.dump(key_json, f, indent=2)

with open("operator_key.pub", "w") as f:
    f.write("//Public key to verify signed binaries\n")
    for i in range(0, 32, 8):
        chunk = pub_bytes[i:i+8]
        f.write(", ".join(f"0x{b:02x}" for b in chunk) + ",\n")

print("operator_key.json  <- private key (keep secret)")
print("operator_key.pub   <- add as CONFIG_PUBLIC_KEY0 in bootloader.px4board / default.px4board")
