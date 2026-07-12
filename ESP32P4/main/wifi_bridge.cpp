// wifi_bridge.cpp  — see wifi_bridge.h for the behaviour overview.

#include "wifi_bridge.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mdns.h"

#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "ui.h"   // ui_set_wifi_info / ui_show_wifi_setup / ui_hide_wifi_setup
#include "ota.h"  // ota_register_handler (OTA firmware update over WiFi)
#include "stm32_flash.h"  // stm32_flash_register_handler (wireless STM32 update)

static const char *TAG = "WIFI_BRIDGE";

// ============================================================
//  CONFIG
// ============================================================
#define NVS_WIFI_NS              "wificfg"
#define NVS_KEY_SSID             "ssid"
#define NVS_KEY_PASS             "pass"

#define TCP_BRIDGE_PORT          8888       // Flask <-> P4 bridge (matches old S3 build)
#define MDNS_HOSTNAME            "spotwelder"
#define AP_IP_ADDR               "192.168.4.1"
#define DNS_PORT                 53

#define STA_CONNECT_TIMEOUT_MS   20000      // fall back to AP portal after this
#define STA_MAX_FAST_RETRIES     5          // immediate retries inside the window

#define WIFI_CONNECTED_BIT       BIT0
#define WIFI_FAIL_BIT            BIT1

// ============================================================
//  STATE
// ============================================================
typedef enum {
    PROV_STA_CONNECTING,   // trying saved creds
    PROV_STA_CONNECTED,    // associated, bridge running
    PROV_AP_PORTAL         // soft-AP + captive portal up (setup mode)
} prov_state_t;

static prov_state_t      s_state          = PROV_STA_CONNECTING;
static char              s_ssid[33]       = {0};   // loaded from NVS
static char              s_pass[65]       = {0};
static char              s_ap_ssid[24]    = {0};   // "SpotWelder-XXXX"
static esp_netif_t      *s_sta_netif      = NULL;
static esp_netif_t      *s_ap_netif       = NULL;
static httpd_handle_t    s_portal_httpd   = NULL;
static httpd_handle_t    s_lan_httpd      = NULL;   // STA-mode server (OTA on LAN)
static EventGroupHandle_t s_wifi_events    = NULL;
static volatile bool     s_creds_just_saved = false;
static volatile bool     s_reconfigure_req  = false;
static uint32_t          s_sta_attempt_ms = 0;
static int               s_fast_retries   = 0;
static bool              s_started        = false;
static bool              s_mdns_up        = false;

static char              s_scan_options[2048] = {0};  // cached <option> list

static wifi_bridge_cmd_cb_t s_cmd_cb = NULL;

// TCP bridge: multi-client support (up to MAX_BRIDGE_CLIENTS simultaneous connections)
#define MAX_BRIDGE_CLIENTS  5
static int               s_clients[MAX_BRIDGE_CLIENTS];  // client sockets; -1 = unused slot
static SemaphoreHandle_t s_client_mtx     = NULL;

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

// ============================================================
//  NVS CREDENTIALS
// ============================================================
static void nvs_load_creds(void)
{
    nvs_handle_t h;
    s_ssid[0] = s_pass[0] = '\0';
    if (nvs_open(NVS_WIFI_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t n = sizeof(s_ssid);
        nvs_get_str(h, NVS_KEY_SSID, s_ssid, &n);
        n = sizeof(s_pass);
        nvs_get_str(h, NVS_KEY_PASS, s_pass, &n);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Creds from NVS: ssid='%s' (%s)", s_ssid,
             s_ssid[0] ? "set" : "EMPTY");
}

static void nvs_save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, NVS_KEY_SSID, ssid);
        nvs_set_str(h, NVS_KEY_PASS, pass);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Credentials saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for write");
    }
}

void wifi_bridge_factory_reset(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGW(TAG, "WiFi credentials erased from NVS");
    }
}

// ============================================================
//  UI STATUS PUSH
// ============================================================
static void push_wifi_info_to_ui(void)
{
    bool connected = false, ap_mode = false;
    char ip[16] = {0};
    const char *ssid = "";
    int rssi = 0;

    if (s_state == PROV_AP_PORTAL) {
        connected = true;
        ap_mode   = true;
        ssid      = s_ap_ssid;
        strncpy(ip, AP_IP_ADDR, sizeof(ip) - 1);
    } else if (s_state == PROV_STA_CONNECTED) {
        connected = true;
        ssid      = s_ssid;
        esp_netif_ip_info_t ipi;
        if (s_sta_netif && esp_netif_get_ip_info(s_sta_netif, &ipi) == ESP_OK) {
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ipi.ip));
        }
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    } else {
        ssid = s_ssid;  // the network we are trying to join
    }
    ui_set_wifi_info(connected, ap_mode, ssid, ip, rssi);
}

// ============================================================
//  TCP BRIDGE (Flask dashboard) — port 8888
// ============================================================
void wifi_bridge_broadcast(const char *line)
{
    if (!line || !line[0] || !s_client_mtx) return;
    xSemaphoreTake(s_client_mtx, portMAX_DELAY);
    // Broadcast to ALL connected clients (Flask, MobaXterm, etc.)
    for (int i = 0; i < MAX_BRIDGE_CLIENTS; i++) {
        int sock = s_clients[i];
        if (sock >= 0) {
            // Append newline so the Flask side can split on lines.
            // send() is non-blocking with MSG_DONTWAIT — if a client is slow/dead,
            // we don't stall the broadcast for the others.
            send(sock, line, strlen(line), MSG_DONTWAIT);
            send(sock, "\n", 1, MSG_DONTWAIT);
        }
    }
    xSemaphoreGive(s_client_mtx);
}

bool wifi_bridge_get_info(bool *out_connected, bool *out_ap_mode,
                          char *out_ssid, char *out_ip, int *out_rssi)
{
    bool connected = false, ap_mode = false;
    char ip[16] = {0};
    const char *ssid = "";
    int rssi = 0;

    if (s_state == PROV_AP_PORTAL) {
        connected = true;
        ap_mode   = true;
        ssid      = s_ap_ssid;
        strncpy(ip, AP_IP_ADDR, sizeof(ip) - 1);
    } else if (s_state == PROV_STA_CONNECTED) {
        connected = true;
        ssid      = s_ssid;
        esp_netif_ip_info_t ipi;
        if (s_sta_netif && esp_netif_get_ip_info(s_sta_netif, &ipi) == ESP_OK) {
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ipi.ip));
        }
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    } else {
        ssid = s_ssid;  // the network we are trying to join
    }

    if (out_connected) *out_connected = connected;
    if (out_ap_mode)   *out_ap_mode   = ap_mode;
    if (out_ssid)      strncpy(out_ssid, ssid, 32), out_ssid[32] = '\0';
    if (out_ip)        strncpy(out_ip, ip, 15), out_ip[15] = '\0';
    if (out_rssi)      *out_rssi = rssi;

    return connected;
}

// Per-client RX handler: receives commands from one TCP client, forwards to STM32.
// Runs as a separate task so multiple clients can send commands concurrently.
typedef struct {
    int sock;
    int slot;
} client_rx_ctx_t;

static void client_rx_task(void *arg)
{
    client_rx_ctx_t *ctx = (client_rx_ctx_t *)arg;
    int sock = ctx->sock;
    int slot = ctx->slot;
    free(ctx);

    char rx[1024];
    char linebuf[1024];
    size_t linelen = 0;

    ESP_LOGI(TAG, "Client RX task started for slot %d (sock %d)", slot, sock);

    while (1) {
        int len = recv(sock, rx, sizeof(rx), 0);
        if (len <= 0) {
            if (len < 0) ESP_LOGE(TAG, "Client slot %d: recv errno %d", slot, errno);
            else         ESP_LOGI(TAG, "Client slot %d disconnected", slot);
            break;
        }
        // Split received data into newline-delimited commands.
        for (int i = 0; i < len; i++) {
            char c = rx[i];
            if (c == '\n' || c == '\r') {
                if (linelen > 0) {
                    linebuf[linelen] = '\0';
                    if (s_cmd_cb) s_cmd_cb(linebuf);  // forward to STM32
                    linelen = 0;
                }
            } else if (linelen < sizeof(linebuf) - 1) {
                linebuf[linelen++] = c;
            }
        }
    }

    // Client disconnected: remove from the client array.
    xSemaphoreTake(s_client_mtx, portMAX_DELAY);
    if (s_clients[slot] == sock) {
        s_clients[slot] = -1;
    }
    xSemaphoreGive(s_client_mtx);

    shutdown(sock, 0);
    close(sock);
    ESP_LOGI(TAG, "Client slot %d closed", slot);
    vTaskDelete(NULL);
}

static void tcp_bridge_task(void *arg)
{
    struct sockaddr_in dest = {};
    dest.sin_addr.s_addr = htonl(INADDR_ANY);
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(TCP_BRIDGE_PORT);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "bridge: socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(listen_sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGE(TAG, "bridge: bind() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_sock, MAX_BRIDGE_CLIENTS) != 0) {
        ESP_LOGE(TAG, "bridge: listen() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP bridge listening on port %d (multi-client, max %d)", 
             TCP_BRIDGE_PORT, MAX_BRIDGE_CLIENTS);

    while (1) {
        struct sockaddr_storage src;
        socklen_t slen = sizeof(src);
        int sock = accept(listen_sock, (struct sockaddr *)&src, &slen);
        if (sock < 0) {
            ESP_LOGE(TAG, "bridge: accept() failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char addr_str[40] = {0};
        if (src.ss_family == AF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&src)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        // TCP keepalive so dead clients are detected and cleaned up.
        int ka = 1, idle = 5, intvl = 5, cnt = 3;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

        // Find a free slot for this client.
        int slot = -1;
        xSemaphoreTake(s_client_mtx, portMAX_DELAY);
        for (int i = 0; i < MAX_BRIDGE_CLIENTS; i++) {
            if (s_clients[i] < 0) {
                slot = i;
                s_clients[i] = sock;
                break;
            }
        }
        xSemaphoreGive(s_client_mtx);

        if (slot < 0) {
            ESP_LOGW(TAG, "Client from %s rejected: all %d slots full", 
                     addr_str, MAX_BRIDGE_CLIENTS);
            shutdown(sock, 0);
            close(sock);
            continue;
        }

        ESP_LOGI(TAG, "Client connected from %s (slot %d)", addr_str, slot);

        // Spawn a lightweight RX task for this client.
        client_rx_ctx_t *ctx = (client_rx_ctx_t *)malloc(sizeof(client_rx_ctx_t));
        if (!ctx) {
            ESP_LOGE(TAG, "Failed to allocate client context");
            xSemaphoreTake(s_client_mtx, portMAX_DELAY);
            s_clients[slot] = -1;
            xSemaphoreGive(s_client_mtx);
            shutdown(sock, 0);
            close(sock);
            continue;
        }
        ctx->sock = sock;
        ctx->slot = slot;

        char task_name[24];
        snprintf(task_name, sizeof(task_name), "bridge_rx_%d", slot);
        BaseType_t ret = xTaskCreate(client_rx_task, task_name, 3072, ctx, 5, NULL);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to spawn RX task for slot %d", slot);
            free(ctx);
            xSemaphoreTake(s_client_mtx, portMAX_DELAY);
            s_clients[slot] = -1;
            xSemaphoreGive(s_client_mtx);
            shutdown(sock, 0);
            close(sock);
        }
    }
}

// ============================================================
//  CAPTIVE-PORTAL DNS RESPONDER
//  Answers every A query with the soft-AP IP so any URL the phone probes
//  lands on our setup page.
// ============================================================
static volatile bool s_dns_run = false;

static void dns_server_task(void *arg)
{
    uint8_t buf[512];
    struct sockaddr_in server = {};
    server.sin_family      = AF_INET;
    server.sin_port        = htons(DNS_PORT);
    server.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "dns: socket() failed");
        vTaskDelete(NULL);
        return;
    }
    if (bind(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
        ESP_LOGE(TAG, "dns: bind() failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint32_t ap_ip = ipaddr_addr(AP_IP_ADDR);  // network byte order
    ESP_LOGI(TAG, "Captive DNS responder up");

    while (s_dns_run) {
        struct sockaddr_in client;
        socklen_t clen = sizeof(client);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, &clen);
        if (n < (int)sizeof(uint16_t) * 6) continue;  // too short for a DNS query

        // Build a minimal response: copy the query, set QR=1, one answer that
        // points the queried name (compression ptr 0xC00C) at ap_ip.
        // Header flags: standard query response, no error.
        buf[2] = 0x81;  // QR=1, Opcode=0, AA=0, TC=0, RD (mirror) -> set RD from req? keep simple
        buf[3] = 0x80;  // RA=1, RCODE=0
        buf[6] = 0x00; buf[7] = 0x01;  // ANCOUNT = 1
        buf[8] = 0x00; buf[9] = 0x00;  // NSCOUNT = 0
        buf[10] = 0x00; buf[11] = 0x00; // ARCOUNT = 0

        int len = n;
        // Answer record appended after the original question section.
        uint8_t ans[16];
        int a = 0;
        ans[a++] = 0xC0; ans[a++] = 0x0C;            // name -> ptr to question
        ans[a++] = 0x00; ans[a++] = 0x01;            // TYPE A
        ans[a++] = 0x00; ans[a++] = 0x01;            // CLASS IN
        ans[a++] = 0x00; ans[a++] = 0x00;            // TTL (high)
        ans[a++] = 0x00; ans[a++] = 0x3C;            // TTL = 60s
        ans[a++] = 0x00; ans[a++] = 0x04;            // RDLENGTH = 4
        memcpy(&ans[a], &ap_ip, 4); a += 4;          // RDATA = AP IP

        if (len + a <= (int)sizeof(buf)) {
            memcpy(&buf[len], ans, a);
            len += a;
            sendto(sock, buf, len, 0, (struct sockaddr *)&client, clen);
        }
    }
    close(sock);
    vTaskDelete(NULL);
}

// ============================================================
//  CAPTIVE-PORTAL HTTP HANDLERS
// ============================================================
static void url_decode(char *s)
{
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '+') {
            *o++ = ' ';
        } else if (*p == '%' && p[1] && p[2]) {
            char hex[3] = { p[1], p[2], 0 };
            *o++ = (char)strtol(hex, NULL, 16);
            p += 2;
        } else {
            *o++ = *p;
        }
    }
    *o = '\0';
}

static void build_portal_html(char *out, size_t outsz)
{
    snprintf(out, outsz,
        "<!DOCTYPE html><html><head><meta name='viewport' "
        "content='width=device-width,initial-scale=1'>"
        "<title>SpotWelder WiFi Setup</title><style>"
        "body{font-family:sans-serif;background:#0b1623;color:#eee;margin:0;padding:20px}"
        "h2{color:#39c}label{display:block;margin:14px 0 4px}"
        "input,select{width:100%%;padding:10px;font-size:16px;border-radius:8px;"
        "border:1px solid #345;background:#13202f;color:#fff;box-sizing:border-box}"
        "button{margin-top:18px;width:100%%;padding:14px;font-size:18px;"
        "background:#2a8;color:#fff;border:0;border-radius:8px}"
        ".card{max-width:420px;margin:auto;background:#13202f;padding:20px;border-radius:12px}"
        "</style></head><body><div class='card'>"
        "<h2>SpotWelder WiFi Setup</h2>"
        "<form action='/save' method='POST'>"
        "<label>Network</label><select name='ssid'>%s</select>"
        "<label>or type SSID</label>"
        "<input name='ssid_manual' placeholder='(optional, overrides above)'>"
        "<label>Password</label>"
        "<input name='pass' type='password' placeholder='WiFi password'>"
        "<button type='submit'>Save &amp; Connect</button></form>"
        "<p style='text-align:center;margin-top:14px'>"
        "<a style='color:#39c' href='/?rescan=1'>&#x21bb; Rescan networks</a>"
        " &nbsp;|&nbsp; "
        "<a style='color:#f90' href='/update'>🔧 Update Firmware</a></p>"
        "</div></body></html>",
        s_scan_options);
}

static void rescan_networks(void);  // fwd

static esp_err_t root_get_handler(httpd_req_t *req)
{
    // ?rescan=1 -> refresh the cached network list before rendering.
    char q[32];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        strstr(q, "rescan=1")) {
        rescan_networks();
    }
    static char html[4096];
    build_portal_html(html, sizeof(html));
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Firmware update page (simple HTML form for uploading .bin files to /ota).
static esp_err_t update_get_handler(httpd_req_t *req)
{
    const char *html = 
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:Arial,sans-serif;max-width:600px;margin:50px auto;padding:20px;background:#f5f5f5}"
        ".card{background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
        "h1{color:#333;margin-top:0;border-bottom:3px solid #ff6b35;padding-bottom:10px}"
        ".info{background:#fff3cd;border-left:4px solid #ffc107;padding:15px;margin:20px 0;border-radius:4px}"
        ".warning{background:#f8d7da;border-left:4px solid #dc3545;padding:15px;margin:20px 0;border-radius:4px}"
        "input[type=file]{margin:20px 0;padding:10px;border:2px dashed #ccc;border-radius:5px;width:100%;cursor:pointer}"
        "input[type=file]:hover{border-color:#ff6b35}"
        "button{background:#ff6b35;color:white;border:none;padding:15px 30px;font-size:16px;border-radius:5px;cursor:pointer;width:100%}"
        "button:hover{background:#ff5722}"
        "button:disabled{background:#ccc;cursor:not-allowed}"
        ".progress{display:none;margin:20px 0}"
        ".progress-bar{width:100%;height:30px;background:#e0e0e0;border-radius:5px;overflow:hidden}"
        ".progress-fill{height:100%;background:#ff6b35;transition:width 0.3s;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold}"
        ".status{text-align:center;margin:10px 0;font-weight:bold}"
        "a{color:#ff6b35;text-decoration:none}"
        "a:hover{text-decoration:underline}"
        "</style></head><body><div class='card'>"
        "<h1>🔧 Spot Welder Firmware Update</h1>"
        "<div class='info'><strong>ℹ️ Instructions:</strong><br>"
        "1. Build your firmware: <code>idf.py build</code><br>"
        "2. Select the .bin file from <code>build/</code> directory<br>"
        "3. Click Upload and wait ~30-60 seconds<br>"
        "4. Device will reboot automatically when complete</div>"
        "<div class='warning'><strong>⚠️ Warning:</strong> Do not power off the device during update!</div>"
        "<form id='form' method='POST' action='/ota'>"
        "<input type='file' name='file' id='file' accept='.bin' required>"
        "<button type='submit' id='btn'>Upload Firmware</button>"
        "</form>"
        "<div class='progress' id='progress'>"
        "<div class='progress-bar'><div class='progress-fill' id='bar'>0%</div></div>"
        "<div class='status' id='status'>Uploading...</div>"
        "</div>"
        "<hr style='margin:30px 0;border:none;border-top:1px solid #ddd'>"
        "<h1>⚙️ STM32 Weld Controller Update</h1>"
        "<div class='info'><strong>ℹ️ Instructions:</strong><br>"
        "1. Build the STM32 project in PlatformIO<br>"
        "2. Select <code>firmware.bin</code> from <code>STM32G474CE/.pio/build/&lt;env&gt;/</code><br>"
        "3. Click Upload — the ESP32 receives it, then flashes the STM32 over UART<br>"
        "4. Watch the welder screen for the STM32 progress bar; the device reboots when done</div>"
        "<div class='warning'><strong>⚠️ Warning:</strong> Do not power off during the STM32 update! "
        "Keep an ST-LINK on the SWD header handy as a recovery fallback.</div>"
        "<form id='sform' method='POST' action='/stm32'>"
        "<input type='file' name='file' id='sfile' accept='.bin' required>"
        "<button type='submit' id='sbtn'>Upload STM32 Firmware</button>"
        "</form>"
        "<div class='progress' id='sprogress'>"
        "<div class='progress-bar'><div class='progress-fill' id='sbar'>0%</div></div>"
        "<div class='status' id='sstatus'>Uploading...</div>"
        "</div>"
        "<div style='margin-top:30px;text-align:center'>"
        "<a href='/'>← Back to WiFi Setup</a>"
        "</div></div>"
        "<script>"
        "const form=document.getElementById('form');"
        "const btn=document.getElementById('btn');"
        "const fileInput=document.getElementById('file');"
        "const progress=document.getElementById('progress');"
        "const bar=document.getElementById('bar');"
        "const status=document.getElementById('status');"
        "form.onsubmit=async(e)=>{e.preventDefault();"
        "const file=fileInput.files[0];"
        "if(!file){alert('Please select a .bin file');return;}"
        "if(!file.name.endsWith('.bin')){alert('File must be a .bin firmware image');return;}"
        "btn.disabled=true;"
        "progress.style.display='block';"
        "status.textContent='Uploading...';"
        "try{"
        "const xhr=new XMLHttpRequest();"
        "xhr.upload.onprogress=(e)=>{"
        "if(e.lengthComputable){"
        "const pct=Math.round((e.loaded/e.total)*100);"
        "bar.style.width=pct+'%';"
        "bar.textContent=pct+'%';"
        "}};"
        "xhr.onload=()=>{"
        "if(xhr.status===200){"
        "bar.style.width='100%';bar.textContent='100%';"
        "status.textContent='✅ Update complete! Device is rebooting...';"
        "status.style.color='green';"
        "setTimeout(()=>{window.location.href='/';},5000);"
        "}else{"
        "status.textContent='❌ Upload failed: '+xhr.statusText;"
        "status.style.color='red';btn.disabled=false;"
        "}};"
        "xhr.onerror=()=>{"
        "status.textContent='❌ Network error';"
        "status.style.color='red';btn.disabled=false;"
        "};"
        "xhr.open('POST','/ota',true);"
        "xhr.setRequestHeader('Authorization','Basic '+btoa('admin:spotwelder2024'));"
        "xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "xhr.send(file);"
        "}catch(err){"
        "alert('Upload failed: '+err.message);btn.disabled=false;"
        "}};"
        // ---- STM32 firmware upload (POST /stm32) ----
        "const sform=document.getElementById('sform');"
        "const sbtn=document.getElementById('sbtn');"
        "const sfileInput=document.getElementById('sfile');"
        "const sprogress=document.getElementById('sprogress');"
        "const sbar=document.getElementById('sbar');"
        "const sstatus=document.getElementById('sstatus');"
        "sform.onsubmit=async(e)=>{e.preventDefault();"
        "const file=sfileInput.files[0];"
        "if(!file){alert('Please select a .bin file');return;}"
        "if(!file.name.endsWith('.bin')){alert('File must be a .bin firmware image');return;}"
        "sbtn.disabled=true;"
        "sprogress.style.display='block';"
        "sstatus.textContent='Uploading to ESP32...';"
        "try{"
        "const xhr=new XMLHttpRequest();"
        "xhr.upload.onprogress=(e)=>{"
        "if(e.lengthComputable){"
        "const pct=Math.round((e.loaded/e.total)*100);"
        "sbar.style.width=pct+'%';"
        "sbar.textContent=pct+'%';"
        "if(pct>=100)sstatus.textContent='Flashing STM32 — watch the welder screen...';"
        "}};"
        "xhr.onload=()=>{"
        "if(xhr.status===200){"
        "sbar.style.width='100%';sbar.textContent='100%';"
        "sstatus.textContent='✅ Received! Flashing STM32, device will reboot...';"
        "sstatus.style.color='green';"
        "setTimeout(()=>{window.location.href='/';},8000);"
        "}else{"
        "sstatus.textContent='❌ Upload failed: '+xhr.statusText;"
        "sstatus.style.color='red';sbtn.disabled=false;"
        "}};"
        "xhr.onerror=()=>{"
        "sstatus.textContent='❌ Network error';"
        "sstatus.style.color='red';sbtn.disabled=false;"
        "};"
        "xhr.open('POST','/stm32',true);"
        "xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "xhr.send(file);"
        "}catch(err){"
        "alert('Upload failed: '+err.message);sbtn.disabled=false;"
        "}};"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char body[300] = {0};
    int total = req->content_len;
    if (total >= (int)sizeof(body)) total = sizeof(body) - 1;
    int recvd = 0;
    while (recvd < total) {
        int r = httpd_req_recv(req, body + recvd, total - recvd);
        if (r <= 0) break;
        recvd += r;
    }
    body[recvd] = '\0';

    char ssid[33] = {0}, ssid_manual[33] = {0}, pass[65] = {0};
    httpd_query_key_value(body, "ssid", ssid, sizeof(ssid));
    httpd_query_key_value(body, "ssid_manual", ssid_manual, sizeof(ssid_manual));
    httpd_query_key_value(body, "pass", pass, sizeof(pass));
    url_decode(ssid);
    url_decode(ssid_manual);
    url_decode(pass);

    const char *use_ssid = ssid_manual[0] ? ssid_manual : ssid;
    if (!use_ssid[0]) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req,
            "<html><body style='font-family:sans-serif;background:#0b1623;color:#eee;padding:24px'>"
            "<h2>No network selected</h2><a style='color:#39c' href='/'>Go back</a></body></html>",
            HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    nvs_save_creds(use_ssid, pass);
    strncpy(s_ssid, use_ssid, sizeof(s_ssid) - 1);
    strncpy(s_pass, pass, sizeof(s_pass) - 1);

    char resp[512];  // 256 too small for max-SSID + HTML (compiler -Werror=format-truncation)
    snprintf(resp, sizeof(resp),
        "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#0b1623;color:#eee;padding:24px'>"
        "<h2>Saved!</h2><p>Connecting to <b>%s</b>...</p>"
        "<p>You can close this page. The welder display will show the status.</p></body></html>",
        use_ssid);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    s_creds_just_saved = true;  // prov loop will switch to STA
    return ESP_OK;
}

// Catch-all: redirect every other probe to the setup page (captive portal).
static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP_ADDR "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void start_portal_httpd(void)
{
    if (s_portal_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn     = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 8;
    if (httpd_start(&s_portal_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "portal httpd start failed");
        return;
    }
    httpd_uri_t root = { .uri = "/", .method = HTTP_GET,
                         .handler = root_get_handler, .user_ctx = NULL };
    httpd_uri_t update = { .uri = "/update", .method = HTTP_GET,
                           .handler = update_get_handler, .user_ctx = NULL };
    httpd_uri_t save = { .uri = "/save", .method = HTTP_POST,
                         .handler = save_post_handler, .user_ctx = NULL };
    httpd_uri_t any  = { .uri = "/*", .method = HTTP_GET,
                         .handler = captive_redirect_handler, .user_ctx = NULL };
    httpd_register_uri_handler(s_portal_httpd, &root);
    httpd_register_uri_handler(s_portal_httpd, &update);
    httpd_register_uri_handler(s_portal_httpd, &save);
    httpd_register_uri_handler(s_portal_httpd, &any);

    // Register OTA endpoint (POST /ota, password-protected). Allows firmware
    // updates over WiFi without needing a USB cable. Matches the OLD ESP32's
    // ArduinoOTA feature.
    ota_register_handler(s_portal_httpd);
    stm32_flash_register_handler(s_portal_httpd);  // POST /stm32 (wireless STM32 flash)
}

static void stop_portal_httpd(void)
{
    if (s_portal_httpd) {
        httpd_stop(s_portal_httpd);
        s_portal_httpd = NULL;
    }
}

// ------------------------------------------------------------------
// STA-mode HTTP server (firmware OTA over the LAN, e.g. 192.168.1.77).
// ------------------------------------------------------------------
// The captive portal server above only runs while the device is in AP setup
// mode (192.168.4.1). Once the device joins the home WiFi we still want the
// firmware-update endpoints reachable on the LAN so you can flash over WiFi
// from VS Code / curl (the ESP-IDF equivalent of PlatformIO's espota), exactly
// like the OLD board. This lightweight server exposes ONLY /update (GET, the
// upload page) and /ota (POST, the firmware receiver) — no captive redirect.
static void start_lan_httpd(void)
{
    if (s_lan_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_uri_handlers = 4;
    // OTA pushes the whole image in one POST; allow plenty of time per recv.
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;
    if (httpd_start(&s_lan_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "LAN httpd start failed");
        return;
    }
    httpd_uri_t update = { .uri = "/update", .method = HTTP_GET,
                           .handler = update_get_handler, .user_ctx = NULL };
    httpd_register_uri_handler(s_lan_httpd, &update);

    // POST /ota (password-protected firmware receiver).
    ota_register_handler(s_lan_httpd);
    stm32_flash_register_handler(s_lan_httpd);  // POST /stm32 (wireless STM32 flash)

    ESP_LOGI(TAG, "LAN OTA server up: GET /update, POST /ota");
}

static void stop_lan_httpd(void)
{
    if (s_lan_httpd) {
        httpd_stop(s_lan_httpd);
        s_lan_httpd = NULL;
    }
}

// ============================================================
//  NETWORK SCAN (cached <option> list for the portal dropdown)
// ============================================================
static void rescan_networks(void)
{
    wifi_scan_config_t scan = {};
    scan.show_hidden = false;
    if (esp_wifi_scan_start(&scan, true) != ESP_OK) {
        ESP_LOGW(TAG, "scan start failed");
        return;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 30) n = 30;
    wifi_ap_record_t *recs = (wifi_ap_record_t *)calloc(n, sizeof(wifi_ap_record_t));
    if (!recs) { return; }
    esp_wifi_scan_get_ap_records(&n, recs);

    s_scan_options[0] = '\0';
    size_t off = 0;
    for (int i = 0; i < n; i++) {
        if (recs[i].ssid[0] == '\0') continue;
        int w = snprintf(s_scan_options + off, sizeof(s_scan_options) - off,
                         "<option value=\"%s\">%s (%d dBm)</option>",
                         (char *)recs[i].ssid, (char *)recs[i].ssid, recs[i].rssi);
        if (w < 0 || off + w >= sizeof(s_scan_options)) break;
        off += w;
    }
    free(recs);
    ESP_LOGI(TAG, "Portal scan cached %d network(s)", n);
}

// ============================================================
//  mDNS
// ============================================================
static void start_mdns(void)
{
    if (s_mdns_up) return;
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mdns init failed");
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("SpotWelder");
    mdns_service_add(NULL, "_spotwelder", "_tcp", TCP_BRIDGE_PORT, NULL, 0);
    s_mdns_up = true;
    ESP_LOGI(TAG, "mDNS up: %s.local", MDNS_HOSTNAME);
}

// ============================================================
//  AP NAME
// ============================================================
static void make_ap_ssid(void)
{
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "SpotWelder-%02X%02X", mac[4], mac[5]);
}

// ============================================================
//  MODE TRANSITIONS
// ============================================================
static void start_sta_connect(void)
{
    ESP_LOGI(TAG, "Connecting (STA) to '%s'", s_ssid);

    wifi_config_t wc = {};
    strncpy((char *)wc.sta.ssid, s_ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, s_pass, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = s_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    esp_wifi_connect();

    s_state          = PROV_STA_CONNECTING;
    s_sta_attempt_ms = now_ms();
    s_fast_retries   = 0;
    push_wifi_info_to_ui();
}

static void start_ap_portal(void)
{
    ESP_LOGI(TAG, "Starting AP + captive portal (setup mode)");

    // Free port 80 if the LAN OTA server was running (we're leaving STA mode).
    stop_lan_httpd();

    // Drop all bridge clients.
    xSemaphoreTake(s_client_mtx, portMAX_DELAY);
    for (int i = 0; i < MAX_BRIDGE_CLIENTS; i++) {
        if (s_clients[i] >= 0) {
            close(s_clients[i]);
            s_clients[i] = -1;
        }
    }
    xSemaphoreGive(s_client_mtx);

    // APSTA so we can scan for networks while the AP is up.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    make_ap_ssid();

    wifi_config_t ap = {};
    strncpy((char *)ap.ap.ssid, s_ap_ssid, sizeof(ap.ap.ssid) - 1);
    ap.ap.ssid_len       = strlen(s_ap_ssid);
    ap.ap.channel        = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode       = WIFI_AUTH_OPEN;   // open AP for easy phone join
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    // Scan once now so the portal dropdown is instant.
    rescan_networks();

    // Captive DNS + HTTP portal.
    if (!s_dns_run) {
        s_dns_run = true;
        xTaskCreate(dns_server_task, "captive_dns", 4096, NULL, 5, NULL);
    }
    start_portal_httpd();

    s_state            = PROV_AP_PORTAL;
    s_creds_just_saved = false;

    ESP_LOGI(TAG, "AP '%s' up at %s", s_ap_ssid, AP_IP_ADDR);

    // QR encodes the setup-page URL (most reliable path; auto-join WIFI: QR
    // proved flaky on the old build).
    char qr[64];
    snprintf(qr, sizeof(qr), "http://%s", AP_IP_ADDR);
    ui_show_wifi_setup(qr, s_ap_ssid, AP_IP_ADDR);
    push_wifi_info_to_ui();
}

static void stop_ap_portal(void)
{
    stop_portal_httpd();
    s_dns_run = false;  // dns task exits on next timeout
}

// ============================================================
//  WIFI / IP EVENT HANDLER
// ============================================================
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_state == PROV_STA_CONNECTING && s_fast_retries < STA_MAX_FAST_RETRIES) {
            s_fast_retries++;
            ESP_LOGI(TAG, "STA retry %d/%d", s_fast_retries, STA_MAX_FAST_RETRIES);
            esp_wifi_connect();
        } else if (s_state == PROV_STA_CONNECTED) {
            // Lost an established link -> try to reconnect; prov loop will
            // fall back to the portal if it stays down.
            ESP_LOGW(TAG, "STA link lost; reconnecting...");
            s_state          = PROV_STA_CONNECTING;
            s_sta_attempt_ms = now_ms();
            s_fast_retries   = 0;
            esp_wifi_connect();
            push_wifi_info_to_ui();
        }
        // else: connecting & out of fast retries -> prov loop timeout handles it
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_state = PROV_STA_CONNECTED;
        // If we came from the portal, tear it down now.
        stop_ap_portal();
        ui_hide_wifi_setup();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // drop the AP
        start_mdns();
        start_lan_httpd();   // expose /update + /ota on the LAN (WiFi OTA)
        push_wifi_info_to_ui();
    }
}

// ============================================================
//  WIFI STACK BRING-UP  (runs on prov_task's stack — see note below)
//
//  IMPORTANT: every call in here that can touch SPI flash (nvs_flash_init,
//  esp_wifi_init / esp_wifi_start -> PHY calibration reads NVS, esp_wifi_set_*
//  -> WiFi NVS) disables the flash cache while it runs. While the cache is off
//  the PSRAM is unreachable, so the *calling task's stack must live in internal
//  RAM*. On this board the main task (app_main) stack is NOT in internal DRAM,
//  so doing this init directly from app_main asserted in
//  esp_task_stack_is_sane_cache_disabled(). Tasks made with xTaskCreate() get
//  an internal-RAM stack on ESP-IDF v6.0, so we run all of this from prov_task
//  instead of from wifi_bridge_start()/app_main.
// ============================================================
// Returns true if the WiFi stack came up, false if the radio (onboard ESP32-C6
// over SDIO/esp_hosted) is unavailable. A false return is NOT fatal: the welder
// UI + STM32 link must keep running with WiFi simply disabled for this boot.
//
// IMPORTANT: do NOT ESP_ERROR_CHECK() the radio-dependent calls. esp_wifi_init/
// set_mode/start all talk to the C6 across the SDIO link, and if that link is
// not up (sdmmc_card_init failed / "ESP-Hosted link not yet up") they return an
// error. Aborting here bricks the whole device into a boot loop.
static bool wifi_stack_init(void)
{
    esp_err_t r;

    // --- Local-to-P4 init (does not touch the C6; treat failure as fatal). ---
    r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif  = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // --- Radio bring-up over esp_hosted/SDIO to the C6 (non-fatal). ---
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    r = esp_wifi_init(&cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s — onboard ESP32-C6 radio link "
                      "(esp_hosted/SDIO) is down. WiFi disabled this boot; "
                      "welder UI continues. Check C6 SDIO reset GPIO/polarity "
                      "and C6 slave firmware.", esp_err_to_name(r));
        return false;
    }

    // Start in STA mode by default; start_ap_portal() will flip to APSTA.
    r = esp_wifi_set_mode(WIFI_MODE_STA);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s — WiFi disabled this boot.",
                 esp_err_to_name(r));
        return false;
    }
    r = esp_wifi_start();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s — WiFi disabled this boot.",
                 esp_err_to_name(r));
        return false;
    }

    // The TCP bridge calls socket()/bind()/listen(), which need the lwIP
    // TCP/IP stack — created by esp_netif_init() above. Starting it here (rather
    // than in wifi_bridge_start) avoids a race where it called socket() before
    // the stack existed ("Invalid mbox" assert in tcpip_send_msg_wait_sem).
    xTaskCreate(tcp_bridge_task, "tcp_bridge", 6144, NULL, 5, NULL);

    // Boot: saved creds -> STA, else straight to the portal.
    nvs_load_creds();
    if (s_ssid[0]) {
        start_sta_connect();
    } else {
        start_ap_portal();
    }
    return true;
}

// ============================================================
//  PROVISIONING SUPERVISOR TASK
// ============================================================
static void prov_task(void *arg)
{
    // Bring the whole WiFi stack up here (internal-RAM stack — see note above).
    // If the C6 radio link is down, disable WiFi for this boot and end the task
    // so the welder UI keeps running (the supervisor loop only manages WiFi).
    if (!wifi_stack_init()) {
        ESP_LOGW(TAG, "WiFi unavailable — provisioning supervisor exiting; "
                      "welder runs without networking this boot.");
        vTaskDelete(NULL);
        return;
    }

    uint32_t last_ui_refresh = 0;
    while (1) {
        uint32_t now = now_ms();

        // Periodic UI refresh (RSSI/IP drift).
        if (now - last_ui_refresh > 3000) {
            last_ui_refresh = now;
            push_wifi_info_to_ui();
        }

        // Reconfigure requested from the Setup tab.
        if (s_reconfigure_req) {
            s_reconfigure_req = false;
            start_ap_portal();
        }

        switch (s_state) {
        case PROV_AP_PORTAL:
            if (s_creds_just_saved) {
                ESP_LOGI(TAG, "New creds saved -> switching to STA");
                stop_ap_portal();
                ui_hide_wifi_setup();
                start_sta_connect();
            }
            break;

        case PROV_STA_CONNECTING:
            // GOT_IP transition is handled in the event handler. Here we only
            // enforce the overall timeout -> AP portal fallback.
            if (now - s_sta_attempt_ms > STA_CONNECT_TIMEOUT_MS) {
                ESP_LOGW(TAG, "STA connect timed out -> AP portal fallback");
                start_ap_portal();
            }
            break;

        case PROV_STA_CONNECTED:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================
//  PUBLIC API
// ============================================================
void wifi_bridge_reconfigure(void)
{
    ESP_LOGI(TAG, "Reconfigure WiFi requested");
    s_reconfigure_req = true;
}

void wifi_bridge_start(wifi_bridge_cmd_cb_t cmd_cb)
{
    if (s_started) return;
    s_started = true;
    s_cmd_cb  = cmd_cb;

    s_client_mtx  = xSemaphoreCreateMutex();
    s_wifi_events = xEventGroupCreate();

    // Initialize all client slots to -1 (unused).
    for (int i = 0; i < MAX_BRIDGE_CLIENTS; i++) {
        s_clients[i] = -1;
    }

    // NOTE: we deliberately do NOT bring up NVS / WiFi here. app_main() runs on
    // the main task, whose stack is not in internal DRAM on this board, and any
    // flash-cache-disabling call (nvs_flash_init, esp_wifi_*) from such a stack
    // trips the esp_task_stack_is_sane_cache_disabled() assert. Instead prov_task
    // (created below with xTaskCreate, which gives an internal-RAM stack on
    // ESP-IDF v6.0) calls wifi_stack_init() as its first action. The TCP bridge
    // task is likewise started from inside wifi_stack_init(), only after the
    // lwIP stack exists (socket() on an uninitialised stack asserts).

    // Provisioning supervisor: brings up the WiFi stack, then runs the state
    // machine. 8 KB stack — esp_wifi_init/start need significantly more than the
    // old 4 KB once the bring-up runs inside this task.
    xTaskCreate(prov_task, "wifi_prov", 8192, NULL, 5, NULL);
}
