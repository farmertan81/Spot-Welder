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
static EventGroupHandle_t s_wifi_events    = NULL;
static volatile bool     s_creds_just_saved = false;
static volatile bool     s_reconfigure_req  = false;
static uint32_t          s_sta_attempt_ms = 0;
static int               s_fast_retries   = 0;
static bool              s_started        = false;
static bool              s_mdns_up        = false;

static char              s_scan_options[2048] = {0};  // cached <option> list

static wifi_bridge_cmd_cb_t s_cmd_cb = NULL;

// TCP bridge: current client socket (guarded). -1 if none.
static int               s_client_sock    = -1;
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
    int sock = s_client_sock;
    if (sock >= 0) {
        // Append newline so the Flask side can split on lines.
        send(sock, line, strlen(line), 0);
        send(sock, "\n", 1, 0);
    }
    xSemaphoreGive(s_client_mtx);
}

static void tcp_bridge_task(void *arg)
{
    char rx[1024];
    char linebuf[1024];
    size_t linelen = 0;

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
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "bridge: listen() failed: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP bridge listening on port %d", TCP_BRIDGE_PORT);

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
        ESP_LOGI(TAG, "Flask client connected from %s", addr_str);

        // TCP keepalive so a dead client is detected.
        int ka = 1, idle = 5, intvl = 5, cnt = 3;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

        // Only one client at a time: replace any previous.
        xSemaphoreTake(s_client_mtx, portMAX_DELAY);
        if (s_client_sock >= 0) { close(s_client_sock); }
        s_client_sock = sock;
        xSemaphoreGive(s_client_mtx);
        linelen = 0;

        // Receive loop: split into newline-delimited commands -> cmd_cb.
        while (1) {
            int len = recv(sock, rx, sizeof(rx), 0);
            if (len <= 0) {
                if (len < 0) ESP_LOGE(TAG, "bridge: recv errno %d", errno);
                else         ESP_LOGI(TAG, "Flask client disconnected");
                break;
            }
            for (int i = 0; i < len; i++) {
                char c = rx[i];
                if (c == '\n' || c == '\r') {
                    if (linelen > 0) {
                        linebuf[linelen] = '\0';
                        if (s_cmd_cb) s_cmd_cb(linebuf);
                        linelen = 0;
                    }
                } else if (linelen < sizeof(linebuf) - 1) {
                    linebuf[linelen++] = c;
                }
            }
        }

        xSemaphoreTake(s_client_mtx, portMAX_DELAY);
        if (s_client_sock == sock) s_client_sock = -1;
        xSemaphoreGive(s_client_mtx);
        shutdown(sock, 0);
        close(sock);
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
        "<a style='color:#39c' href='/?rescan=1'>&#x21bb; Rescan networks</a></p>"
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

    char resp[256];
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
    httpd_uri_t save = { .uri = "/save", .method = HTTP_POST,
                         .handler = save_post_handler, .user_ctx = NULL };
    httpd_uri_t any  = { .uri = "/*", .method = HTTP_GET,
                         .handler = captive_redirect_handler, .user_ctx = NULL };
    httpd_register_uri_handler(s_portal_httpd, &root);
    httpd_register_uri_handler(s_portal_httpd, &save);
    httpd_register_uri_handler(s_portal_httpd, &any);
}

static void stop_portal_httpd(void)
{
    if (s_portal_httpd) {
        httpd_stop(s_portal_httpd);
        s_portal_httpd = NULL;
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

    // Drop any bridge client.
    xSemaphoreTake(s_client_mtx, portMAX_DELAY);
    if (s_client_sock >= 0) { close(s_client_sock); s_client_sock = -1; }
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
        push_wifi_info_to_ui();
    }
}

// ============================================================
//  PROVISIONING SUPERVISOR TASK
// ============================================================
static void prov_task(void *arg)
{
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

    // NVS (required by WiFi).
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    // Netif + event loop + default STA/AP interfaces.
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif  = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Start in STA mode by default; start_ap_portal() will flip to APSTA.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // The TCP bridge listens regardless of link state.
    xTaskCreate(tcp_bridge_task, "tcp_bridge", 6144, NULL, 5, NULL);

    // Provisioning supervisor.
    xTaskCreate(prov_task, "wifi_prov", 4096, NULL, 5, NULL);

    // Boot: saved creds -> STA, else straight to the portal.
    nvs_load_creds();
    if (s_ssid[0]) {
        start_sta_connect();
    } else {
        start_ap_portal();
    }
}
