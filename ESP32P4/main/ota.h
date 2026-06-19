#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register the OTA HTTP endpoint (/ota POST) with the given HTTP server.
// Call this after the HTTP server is started (e.g., in wifi_bridge after
// the captive portal httpd is up). The endpoint is password-protected with
// HTTP basic auth (admin:spotwelder2024).
void ota_register_handler(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
