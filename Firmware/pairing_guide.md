# C.A.D.I. Security: XXTEA Pairing & Configuration Guide

This guide describes the process of "pairing" the **CADI_A** (Aircraft) and **CADI_G** (Ground) units using a shared XXTEA secret key stored in Non-Volatile Storage (NVS).

## Overview

To prevent unauthorized control of the aircraft, all command and telemetry packets are encrypted using the XXTEA algorithm. Both units must share the same 128-bit (16-byte) key. 

Previously, this key was hardcoded in the firmware. For improved security, it is now stored in the ESP32's NVS, allowing each drone to have a unique key without recompiling the code.

## Prerequisites

- ESP32-based CADI_A and CADI_G units.
- Serial Monitor (e.g., VS Code Serial Monitor, PuTTY, or Arduino Serial Monitor).
- Shared 16-character string (e.g., `XDF001_SECRET_KEY`).

## Pairing Procedure

### 1. Connect to Serial
Connect each unit to your computer via USB. Open a serial terminal at **115200 baud**.

### 2. Set the Key
Send the following command through the serial terminal:

```
SET_KEY:your_16_char_key
```

> [!IMPORTANT]
> The key MUST be exactly 16 characters long. If it is shorter, it will be padded with zeros. If it is longer, it will be truncated.

**Example:**
```
SET_KEY:XDF001_SEC_2026!
```

### 3. Verification
Upon receiving the command, the unit will respond with:
`[SEC] New key stored in NVS. Restarting...`

The unit will reboot and load the new key from NVS during the `setup()` phase.

### 4. Repeat for Both Units
Repeat this process for both the **AeroPart** and the **GrounPart**. Both must have the identical key to communicate.

## Technical Details

- **Algorithm:** XXTEA (Corrected Block TEA).
- **Storage:** `Preferences` library (Namespace: `"pairing"`, Key: `"shared_key"`).
- **Verification:** The system uses a `MAGIC_CMD` (0xAA) and `MAGIC_TELEM` (0xBB) header. If decryption failes (e.g., keys don't match), the CRC check will fail and the packet will be discarded.

## Troubleshooting

- **No communication:** Double-check that the keys match exactly on both units (case-sensitive).
- **Packet Loss:** Ensure the nRF24L01 antennas are properly connected and the `lossRate` in the GCS header is below 20%.
- **Resetting:** To clear a key, you must manually overwrite it with `SET_KEY` or perform an ESP32 flash erase.

---
*C.A.D.I. - Secure Unmanned Systems Integration*
