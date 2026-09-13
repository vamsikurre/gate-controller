/*
 * app_diag.h — Always-on diagnostic Wi-Fi AP + web page
 *
 * The nodes live inside a sealed gate-controller box. When one drops off
 * RainMaker there is no serial port and no way in. This module keeps a
 * SoftAP running alongside the normal STA connection so you can walk up to
 * the gate, join the AP from a phone, and see why it is offline (and fix it).
 *
 * See docs/DIAGNOSTIC_AP.md
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* The AP's SSID is simply the node name passed to diag_start() - "Front Gate",
 * "Back Gate". The page is the gate's control panel, not just a debug tool, so
 * it is named for what it controls. */
#define DIAG_AP_IP       "192.168.4.1"

/* Bootstrap AP password, used only until a per-node one is set from the
 * diagnostic page (stored in NVS, never in this repo). This value is public —
 * anyone who reads the repo knows it — so set a real one on each node the
 * first time you connect. See docs/DIAGNOSTIC_AP.md. */
#define DIAG_AP_PASS_BOOTSTRAP  "gate-setup"
#define DIAG_AP_PASS_MIN  8
#define DIAG_AP_PASS_MAX  63

/**
 * Install the log hook so recent ESP_LOG output is kept in RAM and served
 * at /log. Call this as the FIRST thing in app_main() so boot logs are caught.
 */
void diag_log_init(void);

/**
 * Bring up the node's own AP and its HTTP server.
 * Call AFTER app_network_start(). Safe to call once.
 *
 * @param name  Human name for this node ("Front Gate"). Used as the AP SSID
 *              and as the heading on every page.
 */
void diag_start(const char *name);

#ifdef __cplusplus
}
#endif
