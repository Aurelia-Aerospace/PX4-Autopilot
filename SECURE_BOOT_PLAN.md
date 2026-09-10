# Secure Boot & Recovery Plan — CubeOrange ODID

Target boards: `cubeorange-odid`, `cubeorangeplus-odid`  
Branch: `v1.17`

---

## Background

### PX4 vs ArduPilot bootloader flashing

ArduPilot has `flashbootloader` (a MAVLink command that flashes a new bootloader from within the running firmware). PX4's equivalent is **`bl_update`**, a NuttX shell command that:

1. Reads a `.bin` file from the filesystem (e.g., ROMFS or SD card)
2. Validates the image (stack pointer in RAM range, entrypoint in flash range)
3. Erases bootloader flash sectors with `up_progmem_eraseblock()`
4. Programs the new bootloader with `up_progmem_write()`
5. Verifies readback

`bl_update` is already enabled on ODID boards (`CONFIG_SYSTEMCMDS_BL_UPDATE=y`).

There is **no MAVLink interface** to the bootloader's serial protocol — it's a separate binary protocol for host tools (QGroundControl, uploader.py). The bootloader itself runs before PX4, so it cannot be reached while PX4 is running.

### Flash layout — STM32H7 (CubeOrange)

```
0x08000000 - 0x0801FFFF  Sector 0 (128 KB) — Bootloader
0x08020000 - 0x0803FFFF  Sector 1 (128 KB) — App start (APP_LOAD_ADDRESS)
...
```

The bootloader sector cannot be erased without losing the bootloader. However, the **blank area** at the end of sector 0 (e.g., `0x0801FE00`–`0x0801FFFF`) can be written in-place (STM32H7 allows writing to erased=0xFF cells without erasing the sector) — this is where the RDCT cert lives.

---

## What we are building

### 1. Secure Bootloader

The PX4 bootloader (`platforms/nuttx/src/bootloader/common/bl.c`) already has the code paths for:
- `BOOTLOADER_USE_TOC` — parse a Table of Contents (TOC) in flash
- `BOOTLOADER_USE_SECURITY` — enforce Ed25519 signature verification on the app
- `RDCT_CERT_ADDRESS` — read an R&D Certificate to conditionally allow unsigned boot

None of these are currently enabled for the ODID boards. We activate them.

### 2. RDCT Recovery

The **RDCT (R&D Certificate)** mechanism is a signed certificate written by the **running firmware** (not the bootloader) into blank flash inside the bootloader sector. On the next boot the bootloader reads and verifies this cert; if valid and `RDCT_CAPS0_ALLOW_UNSIGNED_BOOT` is set, it boots an unsigned (development) image.

This gives a DFU-less recovery path:
- Normal production: bootloader only boots signed firmware.
- Recovery: GCS issues `SECURE_COMMAND` → firmware writes RDCT cert → reboot → bootloader allows unsigned image → developer flashes normal firmware.

---

## Implementation plan

### Phase 1 — Key generation (offline, once per product)

Generate two Ed25519 key pairs:

| Key index | Purpose |
|-----------|---------|
| `key[0]` | Production signing — signs released firmware |
| `key[1]` | Recovery/RDCT signing — signs RDCT certs |

Keys are embedded in the bootloader as `CONFIG_PUBLIC_KEY0` and `CONFIG_PUBLIC_KEY1` in `bootloader.px4board`.

**Files to create/modify:**
- `boards/cubepilot/cubeorange-odid/bootloader.px4board` — add `CONFIG_PUBLIC_KEY0` and `CONFIG_PUBLIC_KEY1`
- `boards/cubepilot/cubeorangeplus-odid/bootloader.px4board` — same

```kconfig
CONFIG_PUBLIC_KEY0="<hex-encoded Ed25519 pubkey — production signing>"
CONFIG_PUBLIC_KEY1="<hex-encoded Ed25519 pubkey — RDCT/recovery signing>"
```

---

### Phase 2 — Bootloader activation

Enable secure boot and RDCT cert address in hw_config.h for both ODID boards.

**Files:** `boards/cubepilot/cubeorange-odid/src/hw_config.h` and `boards/cubepilot/cubeorangeplus-odid/src/hw_config.h`

Add at the end of each file:

```c
/* Secure boot */
#define BOOTLOADER_USE_SECURITY         1
#define BOOTLOADER_SIGNING_ALGORITHM    1   /* CRYPTO_ED25519 */

/* RDCT cert at the last 512 bytes of bootloader sector 0.
 * This area is blank (0xFF) from factory and can be written
 * without erasing the sector. */
#define RDCT_CERT_ADDRESS               0x0801FE00
```

**How bl.c uses these:**

When `BOOTLOADER_USE_SECURITY` is defined:
- `find_toc()` is called after the initial delay
- `verify_app(0, toc_entries)` checks the Ed25519 signature on the app with `key[0]`
- If signature fails → refuses to boot (unless RDCT allows it)

When `RDCT_CERT_ADDRESS` is defined (new code we add to bl.c):
- Before the signature check, read `image_cert_t` from `RDCT_CERT_ADDRESS`
- Verify the cert's own signature with `key[1]`
- Check `cert->caps[0] & RDCT_CAPS0_ALLOW_UNSIGNED_BOOT`
- If valid → skip signature check on app → boot unsigned image
- After boot: RDCT cert is **not** erased by the bootloader (firmware erases it after recovery is complete)

---

### Phase 3 — bl.c modification

**File:** `platforms/nuttx/src/bootloader/common/bl.c`

Add a helper function (gated on `#ifdef RDCT_CERT_ADDRESS`):

```c
#ifdef RDCT_CERT_ADDRESS
static bool check_rdct_allows_unsigned(void)
{
    const image_cert_t *cert = (const image_cert_t *)RDCT_CERT_ADDRESS;

    /* All-0xFF means empty — no cert present */
    if (cert->device_uuid[0] == 0xFF)
        return false;

    /* Verify cert signature with key[1] (recovery key) */
    /* cert data = everything before signature[] field */
    size_t data_len = offsetof(image_cert_t, signature);
    if (!crypto_signature_check(1, cert->signature, (const uint8_t *)cert, data_len))
        return false;

    return (cert->caps[0] & RDCT_CAPS0_ALLOW_UNSIGNED_BOOT) != 0;
}
#endif
```

In the main boot flow, before calling `verify_app()`:

```c
#ifdef RDCT_CERT_ADDRESS
    bool rdct_unsigned_ok = check_rdct_allows_unsigned();
#else
    bool rdct_unsigned_ok = false;
#endif

    if (!rdct_unsigned_ok && !verify_app(0, toc_entries)) {
        /* signature failed and no valid RDCT — refuse to boot */
        led_on(LED_BOOTLOADER);
        while (true) {}   /* or call bootloader() to wait for upload */
    }
```

---

### Phase 4 — Firmware: RDCT write via SecureCommand

The running firmware must be able to write an RDCT cert in response to a `SECURE_COMMAND` from the GCS.

**File:** `src/drivers/uavcan/remoteid.cpp`

Add a new case to the SecureCommand server callback:

```c
case SECURE_COMMAND_WRITE_RDCT: {
    /* Payload: raw image_cert_t bytes */
    if (req.data.size() != sizeof(image_cert_t)) {
        reply.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
        break;
    }

    /* Write to RDCT_CERT_ADDRESS — area must be blank (0xFF) */
    extern ssize_t up_progmem_write(size_t addr, const void *buf, size_t count);
    ssize_t written = up_progmem_write(RDCT_CERT_ADDRESS,
                                        req.data.data(),
                                        sizeof(image_cert_t));
    if (written != sizeof(image_cert_t)) {
        reply.result = dronecan::remoteid::SecureCommand::Response::RESULT_FAILED;
    } else {
        reply.result = dronecan::remoteid::SecureCommand::Response::RESULT_ACCEPTED;
    }
    break;
}
```

`RDCT_CERT_ADDRESS` must be defined for the firmware too — add it to `boards/cubepilot/cubeorange-odid/src/hw_config.h` (same value as bootloader).

Also add a new opcode to the DSDL:

**File:** `src/drivers/uavcan/libdronecan/dsdl/dronecan/remoteid/64.SecureCommand.uavcan`

```
uint32 SECURE_COMMAND_WRITE_RDCT = 10
```

---

### Phase 5 — RDCT cert erasure (post-recovery)

After a successful recovery boot, the firmware should erase the RDCT cert so it cannot be reused. Add to startup:

**File:** `src/drivers/uavcan/remoteid.cpp` (or a dedicated startup module)

```c
/* If RDCT cert is present at boot, erase it — it was consumed */
#ifdef RDCT_CERT_ADDRESS
const uint8_t *rdct = (const uint8_t *)RDCT_CERT_ADDRESS;
if (rdct[0] != 0xFF) {
    /* Write 0x00 over the signature field to invalidate */
    const image_cert_t *cert = (const image_cert_t *)RDCT_CERT_ADDRESS;
    static const uint8_t zeros[64] = {};
    up_progmem_write(RDCT_CERT_ADDRESS + offsetof(image_cert_t, signature),
                     zeros, sizeof(zeros));
}
#endif
```

Note: STM32H7 flash cannot be changed from 0→1 without a sector erase. Writing 0x00 over the signature field permanently invalidates the cert without needing a sector erase.

---

### Phase 6 — bl_update bootloader flashing

With secure boot active, **the bootloader itself can still be updated** via `bl_update` from within PX4. The new bootloader binary is embedded in ROMFS or fetched from SD card.

To trigger a bootloader update remotely:

1. GCS sends `SECURE_COMMAND_OTA_CHUNK` with the new bootloader binary (already defined in DSDL)
2. Firmware reassembles chunks, writes to a temp location (SD or spare flash)
3. Firmware calls `bl_update <path>` via `px4_system_exec()` or directly calls `bl_update_main()`
4. Reboot

This is already partially implemented in `remoteid.cpp` via `SECURE_COMMAND_OTA_CHUNK` handling. The missing piece is the final `bl_update` call.

**File:** `src/drivers/uavcan/remoteid.cpp`

After receiving all OTA chunks and verifying the new bootloader:

```c
/* Verify new bootloader Ed25519 signature before flashing */
/* Flash via bl_update mechanism */
px4_task_spawn_cmd("bl_update", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
                   2048, bl_update_main, (char *const[]){(char*)"bl_update",
                   (char*)"/fs/microsd/bootloader.bin", nullptr});
```

---

## File change summary

| File | Change |
|------|--------|
| `boards/cubepilot/cubeorange-odid/bootloader.px4board` | Add `CONFIG_PUBLIC_KEY0`, `CONFIG_PUBLIC_KEY1` |
| `boards/cubepilot/cubeorangeplus-odid/bootloader.px4board` | Same |
| `boards/cubepilot/cubeorange-odid/src/hw_config.h` | Add `BOOTLOADER_USE_SECURITY`, `BOOTLOADER_SIGNING_ALGORITHM`, `RDCT_CERT_ADDRESS` |
| `boards/cubepilot/cubeorangeplus-odid/src/hw_config.h` | Same |
| `platforms/nuttx/src/bootloader/common/bl.c` | Add `check_rdct_allows_unsigned()`, wire into boot flow |
| `src/drivers/uavcan/remoteid.cpp` | Add `SECURE_COMMAND_WRITE_RDCT` case, RDCT erasure on startup |
| `src/drivers/uavcan/libdronecan/dsdl/dronecan/remoteid/64.SecureCommand.uavcan` | Add `SECURE_COMMAND_WRITE_RDCT = 10` |

---

## Boot flow diagram

```
Power on
    │
    ▼
Bootloader starts
    │
    ├─ find_toc() → validate TOC in flash
    │
    ├─ RDCT_CERT_ADDRESS defined?
    │   ├─ YES → read image_cert_t from 0x0801FE00
    │   │         verify with key[1]
    │   │         if valid & ALLOW_UNSIGNED_BOOT → rdct_unsigned_ok = true
    │   └─ NO  → rdct_unsigned_ok = false
    │
    ├─ rdct_unsigned_ok?
    │   ├─ NO  → verify_app(key[0]) → fail → hang/upload mode
    │   └─ YES → skip signature check → boot unsigned app
    │
    ▼
App boots
    │
    ├─ On startup: RDCT cert present? → invalidate signature (write zeros)
    │
    └─ Normal operation
           │
           └─ SecureCommand WRITE_RDCT received?
                   → verify cert with key[1] (GCS-side)
                   → up_progmem_write() to 0x0801FE00
                   → reply ACCEPTED → operator reboots board
```

---

## What is NOT in scope

- Firmware encryption (`TOC_FLAG1_DECRYPT`) — stub in bl.c, not implementing now
- OTA firmware update (vs bootloader update) — separate feature
- Key revocation — out of scope for v1
- Secure storage of private keys — handled by the signing infrastructure, not PX4
