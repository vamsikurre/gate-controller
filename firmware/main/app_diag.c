/*
 * app_diag.c — Always-on diagnostic Wi-Fi AP + web page
 *
 * Why this exists
 * ---------------
 * The two gate nodes are sealed inside the gate-controller box. When one
 * stops showing up in RainMaker there is no USB port to plug into and no
 * way to see what the ESP32 is doing. This module keeps a SoftAP alive in
 * parallel with the normal station connection (WIFI_MODE_APSTA), so you can
 * stand at the gate, join "GateDiag-<name>", open http://192.168.4.1/ and:
 *
 *   - see Wi-Fi state, RSSI, IP, last disconnect reason, MQTT state
 *   - read the last few KB of ESP_LOG output
 *   - scan for nearby APs (is the house Wi-Fi even reachable from the box?)
 *   - re-enter Wi-Fi credentials and force a reconnect
 *   - open / close / stop the gate while the cloud is down
 *   - reboot the node
 *
 * Security
 * --------
 * Joining the AP is what authorises you, so the AP password is the only
 * secret that matters:
 *   - it is per-node and lives in NVS, never in this repo. The compile-time
 *     DIAG_AP_PASS_BOOTSTRAP only gets you in the first time.
 *   - state-changing requests are POST and carry a per-boot CSRF token, so a
 *     web page you happen to be browsing cannot actuate the gate behind your
 *     back while your phone is joined to the AP.
 *   - every non-literal string rendered into the page is HTML-escaped; SSIDs
 *     of nearby APs are attacker-chosen text.
 *
 * The provisioning manager forces WIFI_MODE_STA at several points in its
 * lifecycle, which would silently kill the AP. Rather than chase every one
 * of those code paths, a 30s timer just re-applies APSTA if the mode has
 * drifted. Self-healing, and it survives SDK updates.
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_mac.h>
#include <esp_http_server.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rmaker_common_events.h>

#include "app_diag.h"
#include "app_gate.h"

static const char *TAG = "app_diag";

#define DIAG_NVS_NS        "diag"
#define DIAG_NVS_PASS_KEY  "ap_pass"

static void diag_apply_ap_config(void);

/* ---------------------------------------------------------------
 * Log ring buffer
 * ---------------------------------------------------------------
 * esp_log_set_vprintf() lets us tee every log line into RAM. We keep a
 * fixed circular buffer; when it wraps, the oldest bytes are overwritten
 * and /log serves the two halves in order.
 * --------------------------------------------------------------- */
#define LOG_BUF_SIZE 6144

static char s_log_buf[LOG_BUF_SIZE];
static size_t s_log_head;          /* next write position */
static bool   s_log_wrapped;
static portMUX_TYPE s_log_lock = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev_vprintf;

static int diag_vprintf(const char *fmt, va_list ap)
{
    char line[192];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(line, sizeof(line), fmt, ap2);
    va_end(ap2);

    if (n > 0) {
        if (n > (int)sizeof(line) - 1) {
            n = sizeof(line) - 1;   /* vsnprintf returns the untruncated length */
        }
        portENTER_CRITICAL(&s_log_lock);
        for (int i = 0; i < n; i++) {
            s_log_buf[s_log_head++] = line[i];
            if (s_log_head >= LOG_BUF_SIZE) {
                s_log_head = 0;
                s_log_wrapped = true;
            }
        }
        portEXIT_CRITICAL(&s_log_lock);
    }

    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : n;
}

void diag_log_init(void)
{
    s_prev_vprintf = esp_log_set_vprintf(diag_vprintf);
}

/* ---------------------------------------------------------------
 * Connectivity bookkeeping
 * --------------------------------------------------------------- */
static bool     s_mqtt_connected;
static uint32_t s_disconnect_count;
static uint8_t  s_last_disconnect_reason;
static int64_t  s_last_disconnect_us;
static int64_t  s_last_got_ip_us;

static void diag_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        s_last_disconnect_reason = e ? e->reason : 0;
        s_last_disconnect_us = esp_timer_get_time();
        s_disconnect_count++;
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_last_got_ip_us = esp_timer_get_time();
    } else if (base == RMAKER_COMMON_EVENT) {
        if (id == RMAKER_MQTT_EVENT_CONNECTED) {
            s_mqtt_connected = true;
        } else if (id == RMAKER_MQTT_EVENT_DISCONNECTED) {
            s_mqtt_connected = false;
        }
    }
}

/* ---------------------------------------------------------------
 * Small helpers
 * --------------------------------------------------------------- */

/* HTML-escape into a caller buffer. Nearby-AP SSIDs and log lines are not
 * ours to trust, and they land straight in the page. Truncates rather than
 * emitting a half-written entity. */
static void html_escape(const char *in, char *out, size_t n)
{
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)in; *p && o + 6 < n; p++) {
        const char *ent = NULL;
        switch (*p) {
        case '<':  ent = "&lt;";   break;
        case '>':  ent = "&gt;";   break;
        case '&':  ent = "&amp;";  break;
        case '"':  ent = "&quot;"; break;
        case '\'': ent = "&#39;";  break;
        default: break;
        }
        if (ent) {
            size_t l = strlen(ent);
            memcpy(out + o, ent, l);
            o += l;
        } else {
            out[o++] = (char)*p;
        }
    }
    out[o] = '\0';
}

static void fmt_ago(char *buf, size_t len, int64_t stamp_us)
{
    if (stamp_us == 0) {
        snprintf(buf, len, "never");
        return;
    }
    long long s = (esp_timer_get_time() - stamp_us) / 1000000;
    snprintf(buf, len, "%lld:%02lld:%02lld ago", s / 3600, (s / 60) % 60, s % 60);
}

/* Decode application/x-www-form-urlencoded in place. */
static void urldecode(char *s)
{
    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '+') {
            *w++ = ' ';
        } else if (*r == '%' && r[1] && r[2]) {
            char hex[3] = { r[1], r[2], 0 };
            *w++ = (char)strtol(hex, NULL, 16);
            r += 2;
        } else {
            *w++ = *r;
        }
    }
    *w = '\0';
}

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "PANIC / exception";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (check power supply)";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    default:                return "unknown";
    }
}

/* ---------------------------------------------------------------
 * AP password — per node, stored in NVS, set from the page
 * --------------------------------------------------------------- */
static char s_ap_pass[DIAG_AP_PASS_MAX + 1];
static bool s_ap_pass_is_bootstrap = true;

static void ap_pass_load(void)
{
    strlcpy(s_ap_pass, DIAG_AP_PASS_BOOTSTRAP, sizeof(s_ap_pass));
    s_ap_pass_is_bootstrap = true;

    nvs_handle_t h;
    if (nvs_open(DIAG_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    char stored[sizeof(s_ap_pass)];
    size_t len = sizeof(stored);
    if (nvs_get_str(h, DIAG_NVS_PASS_KEY, stored, &len) == ESP_OK &&
        strlen(stored) >= DIAG_AP_PASS_MIN) {
        strlcpy(s_ap_pass, stored, sizeof(s_ap_pass));
        s_ap_pass_is_bootstrap = false;
    }
    nvs_close(h);
}

static esp_err_t ap_pass_save(const char *pass)
{
    size_t len = strlen(pass);
    if (len < DIAG_AP_PASS_MIN || len > DIAG_AP_PASS_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(DIAG_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, DIAG_NVS_PASS_KEY, pass);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        strlcpy(s_ap_pass, pass, sizeof(s_ap_pass));
        s_ap_pass_is_bootstrap = false;
    }
    return err;
}

/* ---------------------------------------------------------------
 * CSRF token — regenerated every boot, never persisted
 * --------------------------------------------------------------- */
static char s_token[33];

static void token_init(void)
{
    uint8_t raw[16];
    esp_fill_random(raw, sizeof(raw));
    for (size_t i = 0; i < sizeof(raw); i++) {
        snprintf(s_token + i * 2, 3, "%02x", raw[i]);
    }
}

/* Read a form body, then confirm it carries our token. */
static bool body_ok(httpd_req_t *req, char *buf, size_t n)
{
    int len = req->content_len < (int)n - 1 ? req->content_len : (int)n - 1;
    int off = 0;
    while (off < len) {
        int r = httpd_req_recv(req, buf + off, len - off);
        if (r <= 0) {
            return false;
        }
        off += r;
    }
    buf[off] = '\0';

    char t[sizeof(s_token)] = {0};
    if (httpd_query_key_value(buf, "t", t, sizeof(t)) != ESP_OK ||
        strcmp(t, s_token) != 0) {
        ESP_LOGW(TAG, "Rejected %s: bad or missing CSRF token", req->uri);
        return false;
    }
    return true;
}

/* ---------------------------------------------------------------
 * HTTP handlers
 * --------------------------------------------------------------- */
#define CHUNK(req, ...) do { \
        char _b[320]; \
        snprintf(_b, sizeof(_b), __VA_ARGS__); \
        httpd_resp_sendstr_chunk(req, _b); \
    } while (0)

/* An inline POST form rendered as a button, carrying the CSRF token.
 * `extra` holds any additional hidden inputs (or ""). */
#define POST_BTN(req, action, extra, label) \
    CHUNK(req, "<form method=post action='%s' style='display:inline'>" \
               "<input type=hidden name=t value='%s'>%s<button>%s</button></form>", \
          action, s_token, extra, label)

static const char *PAGE_HEAD =
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Gate node</title><style>"
    "body{font:15px/1.5 system-ui,sans-serif;margin:0;padding:16px;max-width:640px}"
    "table{border-collapse:collapse;width:100%}td{padding:3px 6px;border-bottom:1px solid #ddd}"
    "td:first-child{color:#666;width:45%}"
    "a.btn,button{display:inline-block;padding:9px 14px;margin:3px 3px 3px 0;border:1px solid #888;"
    "border-radius:6px;background:#f4f4f4;text-decoration:none;color:#000;font:inherit;cursor:pointer}"
    "input{padding:8px;width:100%;box-sizing:border-box;margin:3px 0}"
    "h3{margin:22px 0 6px}pre{background:#111;color:#0f0;padding:8px;overflow:auto;font-size:12px}"
    ".warn{background:#fee;border:1px solid #c00;padding:8px;border-radius:6px}"
    "</style>";

static esp_err_t root_get(httpd_req_t *req)
{
    wifi_ap_record_t ap;
    bool connected = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
    wifi_config_t cfg = {0};
    esp_wifi_get_config(WIFI_IF_STA, &cfg);

    esp_netif_ip_info_t ip = {0};
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) {
        esp_netif_get_ip_info(sta, &ip);
    }

    long long up = esp_timer_get_time() / 1000000;
    char ago_dis[32], ago_ip[32], esc[200];
    fmt_ago(ago_dis, sizeof(ago_dis), s_last_disconnect_us);
    fmt_ago(ago_ip, sizeof(ago_ip), s_last_got_ip_us);

    httpd_resp_sendstr_chunk(req, PAGE_HEAD);

    if (s_ap_pass_is_bootstrap) {
        httpd_resp_sendstr_chunk(req,
            "<p class=warn><b>This node still uses the bootstrap AP password.</b> "
            "It is published in the source repo, so anyone nearby who has read it "
            "can open the gate. Set a real one at the bottom of this page.</p>");
    }

    httpd_resp_sendstr_chunk(req, "<h3>Connection</h3><table>");
    CHUNK(req, "<tr><td>Wi-Fi</td><td><b>%s</b></td></tr>", connected ? "connected" : "NOT CONNECTED");
    html_escape((char *)cfg.sta.ssid, esc, sizeof(esc));
    CHUNK(req, "<tr><td>Configured SSID</td><td>%s</td></tr>", esc);
    if (connected) {
        CHUNK(req, "<tr><td>RSSI</td><td>%d dBm (%s)</td></tr>", ap.rssi,
              ap.rssi > -60 ? "good" : ap.rssi > -75 ? "weak" : "very weak");
        CHUNK(req, "<tr><td>Channel</td><td>%d</td></tr>", ap.primary);
    }
    CHUNK(req, "<tr><td>IP</td><td>" IPSTR "</td></tr>", IP2STR(&ip.ip));
    CHUNK(req, "<tr><td>RainMaker MQTT</td><td><b>%s</b></td></tr>",
          s_mqtt_connected ? "connected" : "NOT CONNECTED");
    CHUNK(req, "<tr><td>Got IP</td><td>%s</td></tr>", ago_ip);
    CHUNK(req, "<tr><td>Last disconnect</td><td>reason %d, %s</td></tr>",
          s_last_disconnect_reason, ago_dis);
    CHUNK(req, "<tr><td>Disconnect count</td><td>%lu</td></tr>", (unsigned long)s_disconnect_count);
    httpd_resp_sendstr_chunk(req, "</table>");

    httpd_resp_sendstr_chunk(req, "<h3>Node</h3><table>");
    CHUNK(req, "<tr><td>Uptime</td><td>%lldh %lldm %llds</td></tr>", up / 3600, (up / 60) % 60, up % 60);
    CHUNK(req, "<tr><td>Last reset</td><td>%s</td></tr>", reset_reason_str());
    CHUNK(req, "<tr><td>Free heap</td><td>%lu B (min %lu B)</td></tr>",
          (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size());
    httpd_resp_sendstr_chunk(req, "</table>");

    httpd_resp_sendstr_chunk(req, "<h3>Gate</h3><table>");
    html_escape(gate_get_status_string(), esc, sizeof(esc));
    CHUNK(req, "<tr><td>Status</td><td>%s</td></tr>", esc);
    CHUNK(req, "<tr><td>Position</td><td>%s</td></tr>", gate_get_position_string());
    CHUNK(req, "<tr><td>Obstruction</td><td>%s</td></tr>", gate_is_obstructed() ? "ACTIVE" : "clear");
    httpd_resp_sendstr_chunk(req, "</table><p>");
    POST_BTN(req, "/cmd", "<input type=hidden name=c value=open>", "Open");
    POST_BTN(req, "/cmd", "<input type=hidden name=c value=close>", "Close");
    POST_BTN(req, "/cmd", "<input type=hidden name=c value=stop>", "Stop");
    POST_BTN(req, "/cmd", "<input type=hidden name=c value=partial>", "Partial");
    httpd_resp_sendstr_chunk(req, "</p>");

    httpd_resp_sendstr_chunk(req, "<h3>Force Wi-Fi</h3><form method=post action=/wifi>");
    CHUNK(req, "<input type=hidden name=t value='%s'>", s_token);
    httpd_resp_sendstr_chunk(req,
        "<input name=ssid placeholder='SSID' required>"
        "<input name=pass type=password placeholder='password'>"
        "<button type=submit>Save &amp; reconnect</button></form>");

    httpd_resp_sendstr_chunk(req, "<h3>AP password</h3>"
        "<p>Per node, stored on the device. Write it inside the gate box lid.</p>"
        "<form method=post action=/appass>");
    CHUNK(req, "<input type=hidden name=t value='%s'>", s_token);
    CHUNK(req, "<input name=pass type=password placeholder='new password (%d-%d chars)' required>"
               "<button type=submit>Change</button></form>",
          DIAG_AP_PASS_MIN, DIAG_AP_PASS_MAX);

    httpd_resp_sendstr_chunk(req,
        "<p><a class=btn href='/'>Refresh</a><a class=btn href='/scan'>Scan APs</a>"
        "<a class=btn href='/log'>Log</a>");
    POST_BTN(req, "/reboot", "", "Reboot");
    httpd_resp_sendstr_chunk(req, "</p>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t cmd_post(httpd_req_t *req)
{
    char body[128], c[16] = {0};
    if (!body_ok(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "bad token");
        return ESP_FAIL;
    }
    httpd_query_key_value(body, "c", c, sizeof(c));

    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (!strcmp(c, "open"))         err = gate_command(GATE_CMD_OPEN);
    else if (!strcmp(c, "close"))   err = gate_command(GATE_CMD_CLOSE);
    else if (!strcmp(c, "stop"))    err = gate_command(GATE_CMD_STOP);
    else if (!strcmp(c, "partial")) err = gate_command(GATE_CMD_PARTIAL_OPEN);

    ESP_LOGI(TAG, "Local AP command '%s' -> %s", c, esp_err_to_name(err));
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, err == ESP_OK ? "ok" : "rejected");
    return ESP_OK;
}

static esp_err_t wifi_post(httpd_req_t *req)
{
    char body[320];
    if (!body_ok(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "bad token");
        return ESP_FAIL;
    }

    char ssid[64] = {0}, pass[80] = {0};
    if (httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required");
        return ESP_FAIL;
    }
    httpd_query_key_value(body, "pass", pass, sizeof(pass));
    urldecode(ssid);
    urldecode(pass);

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));

    ESP_LOGW(TAG, "Local AP set new STA credentials, SSID '%s'", ssid);
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err == ESP_OK) {
        esp_wifi_disconnect();
        esp_wifi_connect();
    }

    char esc_ssid[160], esc_err[64];
    html_escape(ssid, esc_ssid, sizeof(esc_ssid));
    html_escape(err == ESP_OK ? "Saved." : esp_err_to_name(err), esc_err, sizeof(esc_err));

    httpd_resp_sendstr_chunk(req, PAGE_HEAD);
    CHUNK(req, "<p>%s</p><p>Reconnecting to <b>%s</b>.</p>", esc_err, esc_ssid);
    httpd_resp_sendstr_chunk(req,
        "<p>Give it 15s, then check the status page. If it sticks, reboot the node.</p>"
        "<p><a class=btn href='/'>Status</a></p>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static void reapply_ap_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(700));
    diag_apply_ap_config();
    vTaskDelete(NULL);
}

static esp_err_t appass_post(httpd_req_t *req)
{
    char body[160], pass[DIAG_AP_PASS_MAX + 2] = {0};
    if (!body_ok(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "bad token");
        return ESP_FAIL;
    }
    httpd_query_key_value(body, "pass", pass, sizeof(pass));
    urldecode(pass);

    esp_err_t err = ap_pass_save(pass);
    /* Don't leave the new secret sitting in the log ring buffer. */
    memset(body, 0, sizeof(body));

    httpd_resp_sendstr_chunk(req, PAGE_HEAD);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "Diagnostic AP password changed from the local page");
        httpd_resp_sendstr_chunk(req,
            "<p>Changed. The AP restarts with the new password — rejoin with it.</p>"
            "<p><b>Write it inside the box lid.</b> If it is lost, the only way back "
            "in is a USB cable or a firmware reflash.</p>");
        /* Deferred: re-arming the AP drops every client, this one included,
         * so let the response flush first. */
        xTaskCreate(reapply_ap_task, "diag_reap", 2560, NULL, 5, NULL);
    } else if (err == ESP_ERR_INVALID_ARG) {
        CHUNK(req, "<p>Rejected: password must be %d-%d characters.</p>"
                   "<p><a class=btn href='/'>Back</a></p>", DIAG_AP_PASS_MIN, DIAG_AP_PASS_MAX);
    } else {
        CHUNK(req, "<p>Failed: %s</p><p><a class=btn href='/'>Back</a></p>", esp_err_to_name(err));
    }
    memset(pass, 0, sizeof(pass));
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t scan_get(httpd_req_t *req)
{
    uint16_t n = 0;
    static wifi_ap_record_t recs[20];
    char esc[200];

    httpd_resp_sendstr_chunk(req, PAGE_HEAD);
    /* Blocking scan. Briefly interrupts the STA link, which is fine here. */
    if (esp_wifi_scan_start(NULL, true) == ESP_OK) {
        n = sizeof(recs) / sizeof(recs[0]);
        esp_wifi_scan_get_ap_records(&n, recs);
    }
    CHUNK(req, "<h3>%u APs visible from the gate box</h3><table>", n);
    for (uint16_t i = 0; i < n; i++) {
        /* SSIDs are whatever the neighbours decided to broadcast. */
        html_escape((char *)recs[i].ssid, esc, sizeof(esc));
        CHUNK(req, "<tr><td>%s</td><td>%d dBm, ch %d</td></tr>", esc, recs[i].rssi, recs[i].primary);
    }
    httpd_resp_sendstr_chunk(req, "</table><p><a class=btn href='/'>Back</a></p>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t log_get(httpd_req_t *req)
{
    /* Snapshot under lock so a concurrent log write can't tear the read. */
    static char snap[LOG_BUF_SIZE + 1];
    size_t head, total;

    portENTER_CRITICAL(&s_log_lock);
    head = s_log_head;
    total = s_log_wrapped ? LOG_BUF_SIZE : head;
    if (s_log_wrapped) {
        memcpy(snap, s_log_buf + head, LOG_BUF_SIZE - head);
        memcpy(snap + LOG_BUF_SIZE - head, s_log_buf, head);
    } else {
        memcpy(snap, s_log_buf, head);
    }
    portEXIT_CRITICAL(&s_log_lock);
    snap[total] = '\0';

    httpd_resp_sendstr_chunk(req, PAGE_HEAD);
    httpd_resp_sendstr_chunk(req, "<p><a class=btn href='/'>Back</a>"
                                  "<a class=btn href='/log'>Refresh</a></p><pre>");
    /* Log lines quote SSIDs and other outside text, so escape on the way out.
     * Escaped in slices to keep the stack buffer small; bytes >= 0x80 pass
     * through untouched, so UTF-8 split across slices still reassembles. */
    char slice[128], esc[sizeof(slice) * 6 + 1];
    for (size_t off = 0; off < total; off += sizeof(slice) - 1) {
        size_t n = total - off < sizeof(slice) - 1 ? total - off : sizeof(slice) - 1;
        memcpy(slice, snap + off, n);
        slice[n] = '\0';
        html_escape(slice, esc, sizeof(esc));
        httpd_resp_sendstr_chunk(req, esc);
    }
    httpd_resp_sendstr_chunk(req, "</pre>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    char body[64];
    if (!body_ok(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "bad token");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Rebooting. Rejoin the AP in ~20s.");
    xTaskCreate(reboot_task, "diag_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ---------------------------------------------------------------
 * AP + server bring-up, and the APSTA reconciler
 * --------------------------------------------------------------- */
static wifi_config_t s_ap_cfg;
static httpd_handle_t s_httpd;

static void diag_apply_ap_config(void)
{
    strlcpy((char *)s_ap_cfg.ap.password, s_ap_pass, sizeof(s_ap_cfg.ap.password));
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg);
}

static void ensure_ap(void *arg)
{
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode == WIFI_MODE_APSTA) {
        return;
    }
    ESP_LOGW(TAG, "Wi-Fi mode drifted to %d, restoring diagnostic AP", mode);
    diag_apply_ap_config();
}

void diag_start(const char *name)
{
    if (s_httpd) {
        return;
    }

    /* The AP netif only exists if the SDK created it during provisioning. */
    if (!esp_netif_get_handle_from_ifkey("WIFI_AP_DEF")) {
        esp_netif_create_default_wifi_ap();
    }

    ap_pass_load();
    token_init();

    snprintf((char *)s_ap_cfg.ap.ssid, sizeof(s_ap_cfg.ap.ssid), DIAG_AP_PREFIX "%s",
             name ? name : "Node");
    s_ap_cfg.ap.ssid_len = strlen((char *)s_ap_cfg.ap.ssid);
    s_ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    s_ap_cfg.ap.max_connection = 2;
    s_ap_cfg.ap.channel = 1;    /* follows the STA channel once connected */
    diag_apply_ap_config();

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, diag_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, diag_event_handler, NULL);
    esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, diag_event_handler, NULL);

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.lru_purge_enable = true;
    hcfg.max_uri_handlers = 10;
    if (httpd_start(&s_httpd, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start diagnostic HTTP server");
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",       .method = HTTP_GET,  .handler = root_get },
        { .uri = "/scan",   .method = HTTP_GET,  .handler = scan_get },
        { .uri = "/log",    .method = HTTP_GET,  .handler = log_get },
        { .uri = "/cmd",    .method = HTTP_POST, .handler = cmd_post },
        { .uri = "/wifi",   .method = HTTP_POST, .handler = wifi_post },
        { .uri = "/appass", .method = HTTP_POST, .handler = appass_post },
        { .uri = "/reboot", .method = HTTP_POST, .handler = reboot_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_httpd, &uris[i]);
    }

    /* ponytail: 30s poll instead of hooking every SDK path that resets the
     * Wi-Fi mode. Upgrade to event hooks only if AP downtime ever matters. */
    const esp_timer_create_args_t targs = {
        .callback = ensure_ap,
        .name = "diag_ap_keepalive",
    };
    esp_timer_handle_t t;
    if (esp_timer_create(&targs, &t) == ESP_OK) {
        esp_timer_start_periodic(t, 30 * 1000000ULL);
    }

    ESP_LOGI(TAG, "Diagnostic AP up: SSID '%s' -> http://%s/ (password: %s)",
             (char *)s_ap_cfg.ap.ssid, DIAG_AP_IP,
             s_ap_pass_is_bootstrap ? "BOOTSTRAP DEFAULT - change it" : "per-node, set on this device");
}
