# WiFi Provisioning & QR Setup (ESP32 Spot Welder)

This document describes the on-device WiFi provisioning system added to the
ESP32 firmware (the controller that bridges the touch display + STM32 to the
Pi/web UI over TCP). It replaces the previous **hardcoded WiFi credentials** in
`src/main.cpp` with a runtime captive-portal + QR-code flow, plus a new
**Setup** tab on the touchscreen.

> **Status:** This feature is **compile-verified only** (PlatformIO `pio run`
> succeeds, RAM 55.0% / Flash 22.7%). It has **NOT been hardware-tested**.
> Please bench-test on the real display + ESP32 before relying on it.

---

## 1. What changed

| Area | Before | After |
| --- | --- | --- |
| WiFi credentials | Hardcoded `ssid` / `password` in `main.cpp` | Stored in NVS (`wificfg` namespace); none in source |
| First boot / no creds | N/A | Device starts a WiFi **Access Point** + **captive portal** and shows a **QR code** on screen |
| Joining your network | Re-flash firmware | Scan QR (or join AP) → pick network → enter password → Save |
| Connection failure | Stuck retrying | Falls back to AP/portal after a 20 s timeout |
| "Logs" tab (placeholder) | Empty placeholder | **Setup** tab: WiFi status, System Info, Maintenance |
| Status tab | — | Small WiFi indicator (green = connected, yellow = setup mode, grey = offline) |

### Files touched
- `src/main.cpp` — provisioning state machine, NVS load/save/clear, captive
  portal (DNS + HTTP), maintenance callbacks, `setup()` wiring.
- `src/ui.cpp` — Setup tab UI, QR provisioning overlay, confirm modal, WiFi
  indicator on the Status tab, 7 new `ui_*` functions.
- `include/ui.h` — new callback typedefs + function declarations.
- `include/lv_conf.h` — enabled `LV_USE_QRCODE` (LVGL's built-in QR widget).
- `.gitignore` — ignore secret header patterns.

---

## 2. How it works

### Boot flow
```
setup()
  └─ beginWifiProvisioning()
       ├─ load creds from NVS ("wificfg")
       ├─ creds present?  ── yes ─> startStaConnect()  (normal STA join)
       └─ creds empty?    ── yes ─> startApPortal()     (AP + portal + QR)
```

The provisioning state machine is driven every `loop()` by
`ensureWiFiAndServer()` and uses three states:

- `WIFI_PROV_STA_CONNECTING` — trying to join the saved network. On success →
  `WIFI_PROV_STA_CONNECTED`; on 20 s timeout → AP portal fallback.
- `WIFI_PROV_STA_CONNECTED` — connected; starts the TCP server on port 8888
  (the Pi/web bridge). Auto-reconnects if the link drops.
- `WIFI_PROV_AP_PORTAL` — soft-AP up; services DNS + HTTP for the captive
  portal. When the user saves creds, tears down the AP and switches to STA.

> **Cooperative, non-blocking by design.** We deliberately did **not** use the
> WiFiManager library because its portal loop blocks, which would freeze LVGL
> and the STM32 UART bridge. Instead `dnsServer.processNextRequest()` and
> `portalServer.handleClient()` are pumped from the main loop alongside
> `lv_timer_handler()`.

### The Access Point
- SSID: **`SpotWelder-XXXX`** (XXXX = last 2 bytes of the device MAC, so each
  unit is unique).
- **Open network (no password)** so a phone can auto-join straight from the QR
  code. The portal itself is only reachable on the local AP subnet.
- Portal IP: **`192.168.4.1`**. A wildcard DNS server resolves every hostname
  to this IP so the phone's "captive portal detected" prompt opens the page
  automatically.

### Captive-portal auto-detection (how the page pops up by itself)
When a phone/PC joins a WiFi network it immediately fetches an OS-specific
"is there internet?" probe URL. If the response isn't what it expects, the OS
declares a *captive portal* and auto-launches the sign-in page. We make this
fire reliably with three pieces:

1. **Wildcard DNS.** `dnsServer.start(53, "*", 192.168.4.1)` answers **every**
   DNS query with the ESP32's IP, with `setErrorReplyCode(NoError)` and
   `setTTL(0)` so nothing is cached and no query returns NXDOMAIN. Every probe
   hostname therefore resolves to us.

2. **Explicit probe-URL handlers.** The web server registers the known probe
   paths so they hit our code regardless of host:
   - Android: `/generate_204`, `/gen_204`
   - iOS / macOS: `/hotspot-detect.html`, `/library/test/success.html`
   - Windows: `/connecttest.txt`, `/ncsi.txt`, `/redirect`, `/fwlink`
   - Firefox / Linux: `/canonical.html`, `/success.txt`

3. **Response strategy — Method 3 (serve the portal page with HTTP 200).**
   Every probe handler *and* the catch-all `onNotFound` return the portal HTML
   with status **200** and no-cache headers. This is the most reliable strategy
   across OSes:
   - **iOS** renders the returned HTML directly inside its Captive Network
     Assistant popup (it expects the literal text "Success"; anything else →
     popup). It often *abandons* a 302 redirect to a bare IP, so we do **not**
     redirect.
   - **Android** expects HTTP 204; a 200-with-body → "Sign in to network".
   - **Windows** expects "Microsoft Connect Test"; any other body → browser.

   We considered Method 1 (302 redirect for all probes) and Method 2
   (return each OS's exact expected string) but Method 3 is both the simplest
   and the most broadly compatible, and is what fixed the earlier
   "had to type the IP manually" behaviour.

> **Performance note:** the network scan that fills the page's dropdown runs
> **once** when AP mode starts (cached in `g_scan_options`), *not* on every
> request. A blocking `WiFi.scanNetworks()` inside the request path was adding
> 1-2 s of latency to the probe response, which made the popup intermittent.
> A **"Rescan networks"** link (`/?rescan=1`) lets the user refresh the list.

> If a particular phone still doesn't pop the page (some vendor builds suppress
> it), browse to **http://192.168.4.1** manually — that always works.

### The QR code
Rendered on-screen using **LVGL's built-in `lv_qrcode`** widget (no external
library — just `LV_USE_QRCODE 1`). The payload uses the standard Wi-Fi
join format:
```
WIFI:T:nopass;S:<SpotWelder-XXXX>;;
```
Scanning it with a phone camera offers "Join network SpotWelder-XXXX"; once
joined, the captive portal pops up automatically.

### Saving credentials
The portal page (`/`) scans for nearby networks and renders a form (network
dropdown + password, plus a manual SSID field for hidden networks). On submit
(`/save`), the firmware:
1. writes SSID/password to NVS (`wificfg`),
2. shows a "saved, connecting…" confirmation page,
3. tears down the AP, hides the QR overlay, and switches to STA connect.

---

## 3. The Setup tab (touchscreen)

The old empty **Logs** tab is now **Setup** (`LV_SYMBOL_WIFI " Setup"`), with
three panels:

1. **Connection** — live WiFi status (Connected / Setup mode / Disconnected),
   network name, IP address, signal (dBm), and a **Reconfigure** button.
2. **System Info** — firmware version, chip model, flash size, total welds.
3. **Maintenance** — **Restart** and **Factory Reset** buttons, each guarded by
   a confirmation modal.

### Reconfigure WiFi
`Reconfigure` → `onWifiReconfigure()` → `startApPortal()`: drops back into AP +
portal + QR mode so you can move the welder to a different network without
re-flashing. Saving new creds returns it to normal operation.

### Restart
Reboots the ESP32 (`ESP.restart()`). Saved WiFi is retained, so it reconnects
normally.

### Factory Reset
Clears **all** NVS namespaces used by the firmware
(`wificfg`, `weldcfg`, `weldrecipe`, `spotwelder`, `weldstats`) and reboots.
The device comes back up in AP/portal mode because the WiFi creds are gone.
**This also erases your saved recipes, config and weld statistics.**

---

## 4. Testing instructions (bench)

> Flash the firmware as usual: `pio run -t upload` from the `Spotwelder Full`
> directory (board env `esp32-8048S043C`).

### A. First boot / no credentials
1. Erase NVS first to simulate a factory device:
   `pio run -t erase` (or use Factory Reset from the Setup tab on a running unit).
2. Power on. The screen should show the **WiFi Setup** overlay with a QR code
   and the AP name `SpotWelder-XXXX`.
3. On a phone, scan the QR → join the open AP → the setup page should open at
   `http://192.168.4.1`.
4. Pick your home WiFi, type the password, **Save**.
5. The device should reboot/connect; the Status-tab WiFi icon turns **green**
   and the Setup tab shows your IP.

### B. Normal reboot (creds saved)
- Power-cycle. It should connect to the saved network automatically (no QR),
  WiFi icon green within a few seconds.

### C. Wrong password / network down
- Provision a bad password (or power down your router). After ~20 s the device
  should fall back to AP/portal mode and re-show the QR.

### D. Reconfigure
- Setup tab → **Reconfigure** → confirm the QR overlay appears → join a
  different network → verify it reconnects.

### E. Restart & Factory Reset
- Setup tab → **Restart** → confirm → device reboots and reconnects.
- Setup tab → **Factory Reset** → confirm → device wipes NVS and returns to the
  first-boot QR flow. (Recipes/config/stats are also cleared.)

### F. TCP bridge unaffected
- Once connected (STA), confirm the Pi/web UI still reaches the welder on TCP
  port 8888 and telemetry flows as before. The TCP server is only started in
  the connected state, so it stays down during AP/portal mode.

---

## 5. Security considerations (READ THIS)

- **The old password is already public.** The previous hardcoded credentials
  (`ssid` / `password`) are still visible in this repository's **git history**
  on GitHub. Removing them from the current source does **not** remove them
  from history.
  - **Action: rotate / change that WiFi password now** if it is still in use.
  - Optionally purge history with [`git filter-repo`](https://github.com/newren/git-filter-repo)
    or the [BFG Repo-Cleaner](https://rtyley.github.io/bfg-repo-cleaner/), then
    force-push. (Coordinate with anyone who has cloned the repo.)
- **Credentials at rest:** WiFi creds are stored in ESP32 **NVS**, which is
  *not encrypted by default*. Anyone with physical access and a flash reader
  could extract them. For at-rest protection, enable **NVS encryption /
  flash encryption** at the build level (ESP-IDF feature). This is a hardware
  fuse-based step — do it deliberately, it is largely irreversible.
- **Open setup AP:** the provisioning AP is intentionally open (no password) so
  the QR auto-join works. The window is small (only while provisioning) and the
  portal only accepts a network selection. Still, provision in a trusted
  location, since during that window anyone nearby could connect to the AP and
  submit a network. If you prefer, change the soft-AP to use a WPA2 password and
  encode it in the QR (`WIFI:T:WPA;S:<ssid>;P:<pass>;;`).
- **Portal has no auth:** the captive portal does not authenticate the user.
  Because the AP is short-lived and local-only, this is an accepted trade-off
  for ease of setup. Do not expose the portal beyond the local AP.

---

## 6. Notes & limitations

- Compile-verified only; **not hardware-tested**. Bench-test before field use.
- The captive-portal auto-popup behaviour varies by phone OS; if it does not
  pop automatically, browse to `http://192.168.4.1` manually.
- Uptime is intentionally not shown as a live-ticking value to keep Setup-tab
  redraws minimal (the display is sensitive to excessive repaints). System Info
  is populated once at boot.
- STM32 remains the single source of truth for welding; none of this touches
  STM32 firmware.
