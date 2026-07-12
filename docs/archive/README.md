# Archived Documentation

This directory contains documentation that is **no longer current** but is kept
for **historical reference** and to understand the evolution of the project.

---

## Why these docs were archived

All docs here describe the **pre-Katapult** STM32 flashing approach (ROM bootloader,
AN3155 protocol, BOOT0+NRST hardware control), which was **replaced** by the
**Katapult bootloader** in late June 2026 (commits `b056943`–`29be8a0`).

Katapult made wireless/SD STM32 flashing **rock solid** and eliminated the need for:
- BOOT0/NRST GPIO control from the ESP32
- 8E1 parity switching (AN3155)
- Complex software-entry reset sequences
- ST-LINK dependency for field updates

---

## Archived documents

| Document | Why archived | Current equivalent |
|----------|-------------|-------------------|
| `WIRELESS_STM32_FLASH_EXPLAINED.md` | Detailed post-mortem of the ROM/AN3155/BOOT0+NRST approach that took 2 days to debug. Superseded wholesale by Katapult. | `docs/firmware/KATAPULT_BOOTLOADER_SETUP.md` |
| `SD_AND_STM32_FLASHING_FEASIBILITY.md` | Pre-implementation feasibility study for ROM-bootloader-based SD flashing. The feature was later implemented using Katapult instead. | `docs/firmware/KATAPULT_BOOTLOADER_SETUP.md` |

---

## Value of keeping these

These docs capture:
- **What didn't work** and why (debugging dead ends, silent failures, parity traps)
- **Design decisions** that led to choosing Katapult
- **Timeline context** for understanding git history (commits from 6/15–6/22)

If you're troubleshooting the *current* firmware, start with `docs/firmware/` — not here.
