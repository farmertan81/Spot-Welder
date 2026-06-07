Import("env")

# ---------------------------------------------------------------------------
# patch_display.py  --  PlatformIO pre-build hook
#
# The Sunton ESP32-8048S043C (800x480 RGB / ST7262) suffers from a "scrambled"
# display when the RGB panel DMA streams pixels directly out of PSRAM while the
# CPU is also hammering PSRAM (LVGL rendering the redesigned UI). Two upstream
# library defaults make this worse:
#
#   1. The esp32_smartdisplay ST7262 panel driver does NOT set a bounce buffer
#      (bounce_buffer_size_px == 0 -> disabled). Without a bounce buffer the LCD
#      peripheral reads the framebuffer straight from PSRAM, which loses the bus
#      arbitration race and produces torn / shifted / scrambled scanlines.
#
#   2. The driver creates the LVGL draw buffer in PARTIAL render mode. Partial
#      mode flushes many small sub-areas; on this panel that increases the
#      number of PSRAM->LCD transfers competing for the bus and makes tearing
#      more visible. FULL render mode renders+flushes the whole frame at once.
#
# Because the library lives under .pio/libdeps (git-ignored and re-downloaded on
# every `pio run`), we cannot edit it durably by hand. This script re-applies
# the two fixes on every build, idempotently, after the dependency is fetched.
#
# Fix 1 (bounce buffer)  -> add  .bounce_buffer_size_px = 10 * DISPLAY_WIDTH
# Fix 2 (full refresh)   -> LV_DISPLAY_RENDER_MODE_PARTIAL -> ..._FULL
#
# IMPORTANT IDF-VERSION NOTE
# --------------------------
# `bounce_buffer_size_px` is a member of esp_lcd_rgb_panel_config_t ONLY on
# ESP-IDF >= 5.0. This project currently builds on Arduino core 2.0.17 /
# ESP-IDF 4.4.7, where that field does NOT exist, so inserting it unguarded
# breaks the build. We therefore wrap Fix 1 in an `#if ESP_IDF_VERSION >= 5.0`
# guard: it is a no-op on the current (4.4) toolchain and automatically
# activates if/when the framework is upgraded to Arduino core 3.x (IDF 5.x).
# Fix 2 (full refresh) is fully supported on the current toolchain and applies
# immediately.
# ---------------------------------------------------------------------------

import os

PANEL_SRC = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "esp32_smartdisplay",
    "src",
    "lvgl_panel_st7262_par.c",
)

# Markers
INCLUDE_ANCHOR = "#include <esp_lcd_panel_ops.h>\n"
INCLUDE_LINE = "#include <esp_idf_version.h>  // [patched] for ESP_IDF_VERSION guard below\n"

# Fix 1 is wrapped in an IDF>=5.0 guard (the field does not exist on IDF 4.4).
BOUNCE_BLOCK = (
    "#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)\n"
    "        // [patched] Fix 1: bounce buffer stages pixels through internal SRAM\n"
    "        // instead of streaming the framebuffer straight out of PSRAM, which\n"
    "        // is the root cause of the scrambled/torn display. IDF 5.0+ only.\n"
    "        .bounce_buffer_size_px = 10 * DISPLAY_WIDTH,\n"
    "#endif\n"
)
CLK_SRC_LINE = "        .clk_src = ST7262_PANEL_CONFIG_CLK_SRC,\n"
PARTIAL = "LV_DISPLAY_RENDER_MODE_PARTIAL"
FULL = "LV_DISPLAY_RENDER_MODE_FULL"


def patch_panel(*args, **kwargs):
    if not os.path.isfile(PANEL_SRC):
        print("[patch_display] panel source not found yet: %s" % PANEL_SRC)
        return

    with open(PANEL_SRC, "r", encoding="utf-8") as fh:
        src = fh.read()

    original = src
    changed = []

    # --- ensure esp_idf_version.h is included --------------------------------
    if "esp_idf_version.h" not in src and INCLUDE_ANCHOR in src:
        src = src.replace(INCLUDE_ANCHOR, INCLUDE_ANCHOR + INCLUDE_LINE, 1)

    # --- Fix 1: bounce buffer (IDF>=5.0, guarded) ----------------------------
    if "bounce_buffer_size_px" not in src:
        if CLK_SRC_LINE in src:
            src = src.replace(CLK_SRC_LINE, CLK_SRC_LINE + BOUNCE_BLOCK, 1)
            changed.append("Fix1 bounce_buffer_size_px = 10 * DISPLAY_WIDTH (guarded #if IDF>=5.0; no-op on current IDF 4.4)")
        else:
            print("[patch_display] WARNING: could not find clk_src anchor for bounce buffer insertion")
    else:
        changed.append("Fix1 bounce buffer already present (skipped)")

    # --- Fix 2: full refresh --------------------------------------------------
    if PARTIAL in src:
        src = src.replace(PARTIAL, FULL)
        changed.append("Fix2 LVGL render mode PARTIAL -> FULL")
    elif FULL in src:
        changed.append("Fix2 full refresh already present (skipped)")

    if src != original:
        with open(PANEL_SRC, "w", encoding="utf-8") as fh:
            fh.write(src)
        print("[patch_display] PATCHED %s" % PANEL_SRC)
    else:
        print("[patch_display] no changes needed (already patched) %s" % PANEL_SRC)

    for c in changed:
        print("[patch_display]   - %s" % c)


# Run immediately at script-load (pre: hook). The dependency has already been
# installed by the time extra_scripts run, so the file should exist.
patch_panel()
