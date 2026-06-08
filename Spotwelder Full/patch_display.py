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
# Fix 1 (bounce buffer)  -> add  .bounce_buffer_size_px = 20 * DISPLAY_WIDTH
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
    "        .bounce_buffer_size_px = 20 * DISPLAY_WIDTH,\n"
    "#endif\n"
)
# A larger bounce buffer gives the LCD peripheral more headroom before it has
# to fetch the next chunk from PSRAM, which further reduces the intermittent
# scrambled lines seen in the top-left corner. Bumped 10 -> 20 lines.
BOUNCE_OLD_VALUE = ".bounce_buffer_size_px = 10 * DISPLAY_WIDTH,"
BOUNCE_NEW_VALUE = ".bounce_buffer_size_px = 20 * DISPLAY_WIDTH,"
CLK_SRC_LINE = "        .clk_src = ST7262_PANEL_CONFIG_CLK_SRC,\n"
PARTIAL = "LV_DISPLAY_RENDER_MODE_PARTIAL"
FULL = "LV_DISPLAY_RENDER_MODE_FULL"

# --- ESP-IDF 5.x port markers ---------------------------------------------
# The esp32_smartdisplay 2.1.1 panel driver targets the ESP-IDF 4.x RGB-panel
# API. ESP-IDF 5.x changed esp_lcd_rgb_panel_config_t: it dropped the in-config
# on_frame_trans_done callback + user_ctx, dropped the relax_on_idle flag, and
# moved frame callbacks to esp_lcd_rgb_panel_register_event_callbacks(). The
# following patches make the driver build & run on IDF 5.x while still compiling
# unchanged on IDF 4.x (everything is wrapped in ESP_IDF_VERSION guards).

CONFIG_TRAILER_OLD = (
    "        .on_frame_trans_done = direct_io_frame_trans_done,\n"
    "        .user_ctx = display,\n"
    "        .flags = {.disp_active_low = ST7262_PANEL_CONFIG_FLAGS_DISP_ACTIVE_LOW, .relax_on_idle = ST7262_PANEL_CONFIG_FLAGS_RELAX_ON_IDLE, .fb_in_psram = ST7262_PANEL_CONFIG_FLAGS_FB_IN_PSRAM}};\n"
)
CONFIG_TRAILER_NEW = (
    "#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)\n"
    "        .on_frame_trans_done = direct_io_frame_trans_done,  // [patched IDF5] on_frame_trans_done (IDF4 only)\n"
    "        .user_ctx = display,\n"
    "        .flags = {.disp_active_low = ST7262_PANEL_CONFIG_FLAGS_DISP_ACTIVE_LOW, .relax_on_idle = ST7262_PANEL_CONFIG_FLAGS_RELAX_ON_IDLE, .fb_in_psram = ST7262_PANEL_CONFIG_FLAGS_FB_IN_PSRAM}};\n"
    "#else\n"
    "        // [patched IDF5] on_frame_trans_done/user_ctx/relax_on_idle were removed\n"
    "        // from esp_lcd_rgb_panel_config_t in ESP-IDF 5.x. The frame callback is\n"
    "        // registered after panel creation (see register_event_callbacks below).\n"
    "        .flags = {.disp_active_low = ST7262_PANEL_CONFIG_FLAGS_DISP_ACTIVE_LOW, .fb_in_psram = ST7262_PANEL_CONFIG_FLAGS_FB_IN_PSRAM}};\n"
    "#endif\n"
)

NEW_PANEL_ANCHOR = "    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_panel_config, &panel_handle));\n"
REGISTER_CB_BLOCK = (
    "#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)\n"
    "    // [patched IDF5] Register the frame-buffer-complete callback (replaces the\n"
    "    // on_frame_trans_done field of the IDF4 panel config). It signals LVGL that\n"
    "    // the draw buffer has been copied to the panel framebuffer -> flush ready.\n"
    "    const esp_lcd_rgb_panel_event_callbacks_t direct_io_cbs = {\n"
    "        .on_color_trans_done = (esp_lcd_rgb_panel_draw_buf_complete_cb_t)direct_io_frame_trans_done,\n"
    "    };\n"
    "    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &direct_io_cbs, display));\n"
    "#endif\n"
)


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
            changed.append("Fix1 bounce_buffer_size_px = 20 * DISPLAY_WIDTH (guarded #if IDF>=5.0; no-op on current IDF 4.4)")
        else:
            print("[patch_display] WARNING: could not find clk_src anchor for bounce buffer insertion")
    elif BOUNCE_OLD_VALUE in src:
        # Library was patched by a previous build with the old 10-line buffer.
        # The .pio/libdeps copy persists between builds, so upgrade it in place.
        src = src.replace(BOUNCE_OLD_VALUE, BOUNCE_NEW_VALUE)
        changed.append("Fix1 bounce buffer upgraded 10 -> 20 * DISPLAY_WIDTH")
    else:
        changed.append("Fix1 bounce buffer already present at 20 * DISPLAY_WIDTH (skipped)")

    # --- Fix 2: full refresh --------------------------------------------------
    if PARTIAL in src:
        src = src.replace(PARTIAL, FULL)
        changed.append("Fix2 LVGL render mode PARTIAL -> FULL")
    elif FULL in src:
        changed.append("Fix2 full refresh already present (skipped)")

    # --- IDF5 port A: config trailer -----------------------------------------
    # ESP-IDF 5.x removed on_frame_trans_done / user_ctx from the panel config
    # and the relax_on_idle flag. Guard the IDF4 form and add an IDF5 form.
    if CONFIG_TRAILER_OLD in src and "[patched IDF5] on_frame_trans_done" not in src:
        src = src.replace(CONFIG_TRAILER_OLD, CONFIG_TRAILER_NEW, 1)
        changed.append("IDF5: guarded on_frame_trans_done/user_ctx/relax_on_idle out of panel config")
    elif "[patched IDF5] on_frame_trans_done" in src:
        changed.append("IDF5: config trailer already guarded (skipped)")

    # --- IDF5 port B: register event callbacks after panel creation ----------
    if NEW_PANEL_ANCHOR in src and "esp_lcd_rgb_panel_register_event_callbacks" not in src:
        src = src.replace(NEW_PANEL_ANCHOR, NEW_PANEL_ANCHOR + REGISTER_CB_BLOCK, 1)
        changed.append("IDF5: registered on_color_trans_done via register_event_callbacks")
    elif "esp_lcd_rgb_panel_register_event_callbacks" in src:
        changed.append("IDF5: event callback registration already present (skipped)")

    # --- IDF5 port C: guard the debug log_d that references removed fields ----
    # The big log_d() prints rgb_panel_config.on_frame_trans_done / .user_ctx /
    # .flags.relax_on_idle, which do not exist on IDF5. Wrap that single line so
    # it only compiles on IDF < 5.0 (it is debug-only output).
    lines = src.split("\n")
    out = []
    wrapped_log = False
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith('log_d("rgb_panel_config:') and (i == 0 or "ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)" not in lines[i - 1]):
            out.append("#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)  // [patched IDF5] log references removed fields")
            out.append(line)
            out.append("#endif")
            wrapped_log = True
        else:
            out.append(line)
    if wrapped_log:
        src = "\n".join(out)
        changed.append("IDF5: guarded debug log_d that referenced removed config fields")
    elif any('log_d("rgb_panel_config:' in l for l in lines):
        changed.append("IDF5: debug log_d already guarded (skipped)")

    if src != original:
        with open(PANEL_SRC, "w", encoding="utf-8") as fh:
            fh.write(src)
        print("[patch_display] PATCHED %s" % PANEL_SRC)
    else:
        print("[patch_display] no changes needed (already patched) %s" % PANEL_SRC)

    for c in changed:
        print("[patch_display]   - %s" % c)


# ---------------------------------------------------------------------------
# Fix 3 (ESP-IDF 5.x build compatibility): the esp32_smartdisplay 2.1.1 sources
# use the non-standard type `uint` for the adaptive-brightness callback interval.
# The legacy Arduino core 2.x / ESP-IDF 4.4 sysroot pulled in a header that
# typedef'd `uint`; ESP-IDF 5.x no longer does, so the build fails with
# "unknown type name 'uint'". Replace the two `uint interval` occurrences with
# the standard `uint32_t`. Idempotent and harmless on IDF 4.4 too.
# ---------------------------------------------------------------------------
LIB_ROOT = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "esp32_smartdisplay",
)
UINT_FILES = [
    os.path.join(LIB_ROOT, "include", "esp32_smartdisplay.h"),
    os.path.join(LIB_ROOT, "src", "esp32_smartdisplay.c"),
]
UINT_OLD = "smartdisplay_lcd_adaptive_brightness_cb_t cb, uint interval"
UINT_NEW = "smartdisplay_lcd_adaptive_brightness_cb_t cb, uint32_t interval"


def patch_uint(*args, **kwargs):
    for path in UINT_FILES:
        if not os.path.isfile(path):
            print("[patch_display] uint-fix: file not found yet: %s" % path)
            continue
        with open(path, "r", encoding="utf-8") as fh:
            src = fh.read()
        if UINT_OLD in src:
            src = src.replace(UINT_OLD, UINT_NEW)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(src)
            print("[patch_display] PATCHED uint->uint32_t in %s" % path)
        elif UINT_NEW in src:
            print("[patch_display] uint-fix already present (skipped) %s" % path)
        else:
            print("[patch_display] uint-fix: pattern not found in %s" % path)


# Run immediately at script-load (pre: hook). The dependency has already been
# installed by the time extra_scripts run, so the files should exist.
patch_panel()
patch_uint()
