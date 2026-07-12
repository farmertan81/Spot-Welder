# Documentation Index

Reference documentation for the Spot Welder project, organized by topic.

> Protocol and agent/build guidance live at the repository root:
> **[../PROTOCOL.md](../PROTOCOL.md)** (telemetry protocol reference) and
> **[../AGENTS.md](../AGENTS.md)** (build commands + workspace map).

## 📁 hardware/ — Hardware design specifications

- **[PIN_ASSIGNMENTS.md](hardware/PIN_ASSIGNMENTS.md)** — STM32G474CE pin map (connectors, functional groups, INA226 network, expansion pins)
- **[BALANCER_SPECIFICATION.md](hardware/BALANCER_SPECIFICATION.md)** — 4-cell passive balancer architecture and implementation
- **[SNUBBER_SPECIFICATION.md](hardware/SNUBBER_SPECIFICATION.md)** — RC snubber network specification (branches, calculations, layout, BOM)
- **[BUCK_CONVERTER_PIN_ANALYSIS.md](hardware/BUCK_CONVERTER_PIN_ANALYSIS.md)** — Buck converter pin analysis
- **[FIRMWARE_MEASUREMENT_ANALYSIS.md](hardware/FIRMWARE_MEASUREMENT_ANALYSIS.md)** — Firmware measurement/sensing analysis

## 📁 firmware/ — Firmware development history & notes

- **[ESP32_ARCHITECTURE_ANALYSIS.md](firmware/ESP32_ARCHITECTURE_ANALYSIS.md)** — ESP32 firmware architecture overview
- **[JOULE_ALGORITHM_FIX.md](firmware/JOULE_ALGORITHM_FIX.md)** — Joule-integration algorithm fix
- **[MODE_SWITCHER_AND_JOULE_CLEANUP.md](firmware/MODE_SWITCHER_AND_JOULE_CLEANUP.md)** — Mode switcher and joule cleanup
- **[KATAPULT_BOOTLOADER_SETUP.md](firmware/KATAPULT_BOOTLOADER_SETUP.md)** — Katapult bootloader setup
- **[STATUS_TAB_UI_IMPROVEMENTS.md](firmware/STATUS_TAB_UI_IMPROVEMENTS.md)** — Status tab UI improvements
- **[CHANGELOG_P4_UI_STABLE.md](firmware/CHANGELOG_P4_UI_STABLE.md)** — ESP32-P4 UI stable changelog

## 📁 features/ — Feature design documents

- **[WIRELESS_STM32_FLASH_EXPLAINED.md](features/WIRELESS_STM32_FLASH_EXPLAINED.md)** — Wireless STM32 flashing explained (also `.pdf`, `.docx`)
- **[SD_AND_STM32_FLASHING_FEASIBILITY.md](features/SD_AND_STM32_FLASHING_FEASIBILITY.md)** — SD card + STM32 flashing feasibility (also `.pdf`, `.docx`)

## 📁 issues/ — Known issues & troubleshooting

- **[KNOWN_ISSUES.md](issues/KNOWN_ISSUES.md)** — Known issues and workarounds (also `.pdf`, `.docx`)

## Related locations

- **`../Datasheets/`** — Component datasheets; STM32 reference manuals under `../Datasheets/STM/`
- **`../tools/`** — Katapult/flashing utility scripts
- **`../archive/`** — Retired artifacts (e.g. `old_firmware/` backup binary)
- **`../STM32G474CE/GPIO_BALANCER_ANALYSIS.md`** — GPIO analysis kept alongside STM32 source
