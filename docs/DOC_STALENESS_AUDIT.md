# Documentation Staleness Audit

> Read-only audit — no doc changes made yet. Cross-references git commit history
> against current source to flag stale/superseded documentation.
>
> **Key event that dates the docs:** a **Katapult bootloader** was ported to the
> STM32G474 and made STM32 wireless/SD flashing "rock solid." Several docs were
> written *before* Katapult and describe the old (failed) ROM-bootloader reality.

---

## The timeline that matters (git commit dates)

| Date | Commit | Event |
|------|--------|-------|
| 6/15 | `0c0c279` | KNOWN_ISSUES.md written — "SD flash impossible, use ST-LINK" |
| 6/17 | `c79db17` | SD_AND_STM32_FLASHING_FEASIBILITY.md — "analysis only, nothing implemented" |
| 6/20 | `b65d428` | WIRELESS_STM32_FLASH_EXPLAINED.md — ROM/BOOT0 software-entry saga |
| 6/20 | `b056943`+ | **Katapult Phase 1 & 2** ported + implemented |
| 6/21 | `29be8a0` | "Fix all Katapult protocol bugs causing STM32 bricking" → **Katapult working** |
| 6/21 | `48e3164` | UART moved to UART3-IN header (GPIO 27/28) @ **576 kbaud** |
| 6/22 | `3311f60` | SD-card firmware flashing added for ESP32-P4 + STM32 |

**Proven current reality (from source, not docs):**
- App relocated: `STM32G474CETX_FLASH.ld` → `FLASH ORIGIN = 0x8002000` (8 KiB Katapult reserve)
- `main.c`: `SCB->VTOR = 0x08002000U;` + banner `BOOT,FW=KATAPULT-APP-v6-RELOCATED-0x08002000`
- ESP32-P4 link: `STM_APP_BAUD 576000` on GPIO 27/28 (UART3-IN); `KATAPULT_BAUD 250000`
- Katapult binaries committed in `STM32G474CE/katapult/`

---

## Verdicts

| Doc | Verdict | Why |
|-----|---------|-----|
| `docs/issues/KNOWN_ISSUES.md` | 🔴 **STALE — rewrite or archive** | Written 6/15, 6 days before Katapult. Whole premise (SD flash impossible, no BOOT0, ROM won't work via software, ST-LINK required, option-byte unverified) was **solved** by Katapult. |
| `docs/features/SD_AND_STM32_FLASHING_FEASIBILITY.md` | 🟠 **ARCHIVE (historical)** | Pure feasibility study that says "nothing implemented." It has since been implemented via Katapult. Still lists old GPIO17/18 wiring. Valuable as history, not as current guidance. |
| `docs/features/WIRELESS_STM32_FLASH_EXPLAINED.md` | 🟠 **ARCHIVE (historical)** | Documents the abandoned **ROM-bootloader / AN3155 / BOOT0+NRST** approach (GPIO29–32, 1 Mbaud, ROM @ 0x1FFF0000, 8E1 parity, breadcrumb). Superseded wholesale by Katapult. Great post-mortem, but not how it works now. |
| `docs/firmware/KATAPULT_BOOTLOADER_SETUP.md` | 🟡 **CURRENT — minor update** | This is the authoritative, correct doc. Two stale details: says app link is "1,000,000 baud" (now **576000**) and Katapult validate step shows `-b 250000` on `/dev/ttyUSB0` which is fine, but the app-baud/GPIO (27/28) should be noted. |
| `README.md` | 🟡 **UPDATE — stale sections** | Lines ~61–110: "STM32 update via ST-LINK," "SD-Card flashing requires ST-LINK," "experimental option-byte…unverified." All pre-Katapult. Should now point to Katapult wireless/SD flashing. |
| `docs/hardware/PIN_ASSIGNMENTS.md` | 🟢 **CURRENT** | STM32-side pinout (PA9/PA10, NRST) unchanged by Katapult. The GPIO move was on the ESP side. |
| `docs/hardware/BALANCER_SPECIFICATION.md` | 🟢 **CURRENT** | Hardware spec, no flash dependency. |
| `docs/hardware/SNUBBER_SPECIFICATION.md` | 🟢 **CURRENT** | Hardware spec. |
| `docs/hardware/BUCK_CONVERTER_PIN_ANALYSIS.md` | 🟢 **CURRENT** | Hardware analysis. |
| `docs/hardware/FIRMWARE_MEASUREMENT_ANALYSIS.md` | 🟢 **CURRENT** | Measurement analysis. |
| `docs/firmware/ESP32_ARCHITECTURE_ANALYSIS.md` | 🟢 **CURRENT** | Architecture; not flash-specific. |
| `docs/firmware/JOULE_ALGORITHM_FIX.md` | 🟢 **CURRENT** | Algorithm history. |
| `docs/firmware/MODE_SWITCHER_AND_JOULE_CLEANUP.md` | 🟢 **CURRENT** | Feature history. |
| `docs/firmware/STATUS_TAB_UI_IMPROVEMENTS.md` | 🟢 **CURRENT** | UI history. |
| `docs/firmware/CHANGELOG_P4_UI_STABLE.md` | 🟢 **CURRENT** | Changelog. |

---

## Proposed actions (for approval — nothing done yet)

1. **Create `docs/archive/`** and `git mv` the two superseded flash docs there:
   - `docs/features/WIRELESS_STM32_FLASH_EXPLAINED.md` → `docs/archive/`
   - `docs/features/SD_AND_STM32_FLASHING_FEASIBILITY.md` → `docs/archive/`
   - Add a one-line "SUPERSEDED BY KATAPULT_BOOTLOADER_SETUP.md" banner at top of each.

2. **KNOWN_ISSUES.md** — either rewrite the STM32-flash section to reflect Katapult
   (recommended) or archive it with a superseded note. Your call on which.

3. **KATAPULT_BOOTLOADER_SETUP.md** — fix the app baud (1,000,000 → 576000) and note
   the ESP link is GPIO 27/28 (UART3-IN).

4. **README.md** — replace the "ST-LINK required / SD unreliable / option-byte
   unverified" sections with the current Katapult wireless + SD flow.

5. Add a note in `docs/archive/README.md` explaining these are kept for history.
