/*
 * app_main.c — ESP RainMaker Gate Controller
 *
 * ===================================================================
 *  HOW ESP RAINMAKER WORKS (A Guide for the Developer)
 * ===================================================================
 *
 *  ESP RainMaker is Espressif's cloud platform for IoT devices.
 *  Think of it as a 3-layer system:
 *
 *  ┌─────────────────────────────────────────────────────┐
 *  │  LAYER 1: YOUR PHONE (ESP RainMaker App)            │
 *  │  - Discovers device via Bluetooth (BLE)             │
 *  │  - Sends your Wi-Fi credentials to the ESP32        │
 *  │  - Shows device controls (buttons, sliders, etc.)   │
 *  │  - Sends commands: "Open=true", "Close=true", etc.  │
 *  └───────────────────────┬─────────────────────────────┘
 *                          │  MQTT (via AWS IoT Core)
 *  ┌───────────────────────┴─────────────────────────────┐
 *  │  LAYER 2: RAINMAKER CLOUD                           │
 *  │  - Manages device identity (X.509 certificates)     │
 *  │  - Routes commands between phone and device          │
 *  │  - Stores device state (so app shows current state) │
 *  │  - Handles OTA firmware updates                     │
 *  │  - Manages user accounts and device sharing         │
 *  └───────────────────────┬─────────────────────────────┘
 *                          │  MQTT over TLS
 *  ┌───────────────────────┴─────────────────────────────┐
 *  │  LAYER 3: YOUR ESP32 (This firmware)                │
 *  │  - Connects to Wi-Fi                                │
 *  │  - Authenticates with RainMaker cloud via certs     │
 *  │  - Defines a "node" with "devices" and "params"     │
 *  │  - Receives commands via write_cb callback          │
 *  │  - Controls relays based on commands                │
 *  └─────────────────────────────────────────────────────┘
 *
 *  THE SETUP FLOW (First Boot):
 *  ============================
 *  1. You flash this firmware to the ESP32
 *  2. ESP32 boots and enters BLE provisioning mode
 *  3. Open "ESP RainMaker" app on your phone
 *  4. Tap "Add Device" → scan the QR code from serial monitor
 *  5. App connects to ESP32 via Bluetooth
 *  6. App sends your Wi-Fi SSID + password to ESP32
 *  7. ESP32 connects to Wi-Fi
 *  8. App performs "Assisted Claiming":
 *     - App talks to RainMaker cloud
 *     - Cloud generates an X.509 certificate for your ESP32
 *     - App sends the certificate to ESP32 via BLE
 *     - ESP32 stores the cert in flash (never needs to claim again)
 *  9. ESP32 connects to RainMaker cloud via MQTT
 *  10. Your device appears in the app with Open/Close/Stop buttons!
 *
 *  SUBSEQUENT BOOTS:
 *  =================
 *  - ESP32 already has Wi-Fi creds + certificate in NVS flash
 *  - Connects to Wi-Fi automatically
 *  - Connects to RainMaker cloud automatically
 *  - Ready in ~5 seconds
 *
 *  HOW COMMANDS WORK:
 *  ==================
 *  1. You tap "Open" in the phone app
 *  2. App sends { "Open": true } to RainMaker cloud via HTTPS
 *  3. Cloud forwards it to your ESP32 via MQTT
 *  4. ESP32's write_cb() is called with param="Open", val=true
 *  5. write_cb() calls gate_command(GATE_CMD_OPEN)
 *  6. Gate task pulses Relay 1 for 500ms
 *  7. Gate controller receives the dry-contact short → gate opens
 *  8. Firmware updates status to "Opening" and reports back to cloud
 *  9. App shows "Opening" status
 *
 *  RESET BUTTON:
 *  =============
 *  The BOOT button (GPIO 0) on the ESP32 dev board is used for resets:
 *  - Hold 3 seconds  → Wi-Fi reset (forgets network, re-enters provisioning)
 *  - Hold 10 seconds → Factory reset (erases everything, like a new device)
 *
 * ===================================================================
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_mac.h>
#include "nvs.h"
#include <esp_timer.h>
#include <esp_rmaker_utils.h>
#if CONFIG_ESP_INSIGHTS_ENABLED
#include <esp_insights.h>
#include <esp_diagnostics.h>
#endif


/* ESP RainMaker headers */
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_ota.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_common_events.h>
#include <esp_rmaker_mqtt.h>

/* Network provisioning helper (from espressif/rmaker_app_network component) */
#include <network_provisioning/manager.h>
#include <app_network.h>

/* Our gate control module */
#include "app_priv.h"
#include "app_gate.h"

static const char *TAG = "app_main";

/* Global handle to the device — needed to update params from gate callbacks */
static esp_rmaker_device_t *s_gate_device = NULL;

#define BOOT_COUNT_NAMESPACE "storage"
#define BOOT_COUNT_KEY       "boot_count"
#define BOOT_COUNT_RESET_S   5

static esp_timer_handle_t s_boot_count_timer = NULL;

static void reset_boot_count_cb(void *arg)
{
    nvs_handle_t handle;
    if (nvs_open(BOOT_COUNT_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, BOOT_COUNT_KEY, 0);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Stable running state reached. Boot count reset to 0.");
    }
}

static void check_rapid_power_cycle(void)
{
    nvs_handle_t handle;
    uint8_t boot_count = 0;

    esp_err_t err = nvs_open(BOOT_COUNT_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS storage namespace for boot count");
        return;
    }

    err = nvs_get_u8(handle, BOOT_COUNT_KEY, &boot_count);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        boot_count = 0;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading boot count from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    boot_count++;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    if (boot_count >= 3) {
        ESP_LOGW(TAG, "Rapid power-cycle detected (3x)! Triggering Wi-Fi credentials reset...");
        nvs_set_u8(handle, BOOT_COUNT_KEY, 0);
        nvs_commit(handle);
        nvs_close(handle);

        /* Reset Wi-Fi credentials (clears SSID/pass and reboots the chip) */
        esp_rmaker_wifi_reset(0, 0);
        return;
    }

    nvs_set_u8(handle, BOOT_COUNT_KEY, boot_count);
    nvs_commit(handle);
    nvs_close(handle);

    /* Start a 5-second timer to reset the boot count to 0 if we run stably */
    esp_timer_create_args_t timer_args = {
        .callback = reset_boot_count_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "boot_count_reset_tm"
    };

    if (esp_timer_create(&timer_args, &s_boot_count_timer) == ESP_OK) {
        esp_timer_start_once(s_boot_count_timer, BOOT_COUNT_RESET_S * 1000000ULL);
        ESP_LOGI(TAG, "Started %d-second timer to reset boot count.", BOOT_COUNT_RESET_S);
    } else {
        ESP_LOGE(TAG, "Failed to create boot count reset timer.");
    }
}
/* ---------------------------------------------------------------
 * write_cb — RainMaker Write Callback
 * ---------------------------------------------------------------
 * This function is called by the RainMaker framework whenever the
 * user taps a button in the phone app (or a schedule triggers).
 *
 * Java Note on Parameters:
 * - `const esp_rmaker_device_t *device`: A pointer to a device struct (like a read-only object reference).
 * - `const esp_rmaker_param_t *param`: A pointer to the specific parameter changing.
 * - `const esp_rmaker_param_val_t val`: A struct passed by value containing the new value.
 * - `void *priv_data`: A generic pointer (like Java's `Object`) for custom user data.
 * - `esp_rmaker_write_ctx_t *ctx`: Pointer to context data containing the command source (Cloud, Local, at).
 *
 * IMPORTANT: This runs in the RainMaker MQTT task context. We must NOT block
 * here (no vTaskDelay!). We post the command to the gate queue and return immediately.
 * --------------------------------------------------------------- */
static esp_err_t write_cb(const esp_rmaker_device_t *device,
                           const esp_rmaker_param_t *param,
                           const esp_rmaker_param_val_t val,
                           void *priv_data,
                           esp_rmaker_write_ctx_t *ctx)
{
    const char *param_name = esp_rmaker_param_get_name(param);

    if (ctx) {
        /* Java equivalent: ctx.src. In C, if we have a pointer to a struct, 
         * we access its fields using the arrow `->` operator (dereference + field access). */
        ESP_LOGI(TAG, "Received write request via: %s",
                 esp_rmaker_device_cb_src_to_str(ctx->src));
    }

    ESP_LOGI(TAG, "Parameter: %s", param_name);

    /* esp_err_t is an integer error code. ESP_OK is 0 (success).
     * Java equivalent: return values instead of throwing exceptions. */
    esp_err_t err = ESP_OK;

    /* ---- Action Buttons (trigger type: bool) ---- 
     * Java note: In Java, we do: paramName.equals(PARAM_OPEN).
     * In C, we use `strcmp(str1, str2)`. It returns:
     * - 0 if the strings are identical.
     * - non-zero if they differ.
     * 
     * `val.val.b` accesses a Union field. In C, a Union allows multiple types 
     * (bool, int, float, string) to share the same memory location to save RAM.
     * Here, `.b` reads the boolean representation of the parameter value.
     */
    if (strcmp(param_name, PARAM_OPEN) == 0) {
        if (val.val.b) {
            err = gate_command(GATE_CMD_OPEN);
        } else {
            /* Alexa "turn off" sends Open/Power=false → close the gate */
            err = gate_command(GATE_CMD_CLOSE);
        }
    }
    else if (strcmp(param_name, PARAM_CLOSE) == 0 && val.val.b) {
        err = gate_command(GATE_CMD_CLOSE);
    }
    else if (strcmp(param_name, PARAM_STOP) == 0 && val.val.b) {
        err = gate_command(GATE_CMD_STOP);
    }
    else if (strcmp(param_name, PARAM_PARTIAL_OPEN) == 0 && val.val.b) {
        err = gate_command(GATE_CMD_PARTIAL_OPEN);
    }
    /* ---- Configuration params removed from UI ---- */
    /* Pulse duration and partial delay use hardcoded defaults */

    /* Update status and report button state back to cloud */
    if (err == ESP_OK) {
        /* For the Open/Power param, keep the value reflecting gate direction so
         * Alexa shows "On" when opening and "Off" when closing.
         * For all other trigger params (Close/Stop/Partial Open), reset to false. */
        if (strcmp(param_name, PARAM_OPEN) == 0) {
            esp_rmaker_param_update_and_report(param, esp_rmaker_bool(val.val.b));
        } else {
            esp_rmaker_param_update_and_report(param, esp_rmaker_bool(false));

            /* When Close or Stop is triggered directly (not via Open=false),
             * also set the power param to false so Alexa shows "Off". */
            if (strcmp(param_name, PARAM_CLOSE) == 0 ||
                strcmp(param_name, PARAM_STOP) == 0) {
                esp_rmaker_param_t *open_param = esp_rmaker_device_get_param_by_name(
                    s_gate_device, PARAM_OPEN);
                if (open_param) {
                    esp_rmaker_param_update_and_report(open_param,
                        esp_rmaker_bool(false));
                }
            }
        }

        /* Update the status text */
        const char *status = gate_get_status_string();
        esp_rmaker_param_t *status_param = esp_rmaker_device_get_param_by_name(
            s_gate_device, PARAM_STATUS);
        if (status_param) {
            esp_rmaker_param_update_and_report(status_param, esp_rmaker_str(status));
        }
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Command rejected — gate is busy");
        /* Report "Busy" status to the app */
        esp_rmaker_param_t *status_param = esp_rmaker_device_get_param_by_name(
            s_gate_device, PARAM_STATUS);
        if (status_param) {
            esp_rmaker_param_update_and_report(status_param,
                esp_rmaker_str("Busy — wait for cooldown"));
        }
        /* For the Open/Power param, report actual gate position so Alexa stays accurate.
         * For other trigger params, just reset to false. */
        if (strcmp(param_name, PARAM_OPEN) == 0) {
            gate_position_t pos = gate_get_position();
            esp_rmaker_param_update_and_report(param,
                esp_rmaker_bool(pos == GATE_POS_OPEN));
        } else {
            esp_rmaker_param_update_and_report(param, esp_rmaker_bool(false));
        }
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------
 * create_gate_device — Build the RainMaker device definition
 * ---------------------------------------------------------------
 * This function creates a custom device with all its parameters.
 * The structure defined here is what the phone app displays.
 *
 * DEVICE HIERARCHY:
 *   Node "Gate Controller"
 *   └── Device "Sliding Gate"
 *       ├── Status        (read-only text — shown at top)
 *       ├── Open          (push button)
 *       ├── Close         (push button)
 *       ├── Stop          (push button)
 *       └── Partial Open  (push button)
 * --------------------------------------------------------------- */
static esp_rmaker_device_t *create_gate_device(const char *device_name)
{
    /* Create a custom device (NULL type = generic/custom) */
    /* Use standard Switch type so Alexa can discover this device.
     * Custom/NULL types are invisible to Alexa's device discovery. */
    esp_rmaker_device_t *device = esp_rmaker_device_create(device_name, ESP_RMAKER_DEVICE_SWITCH, NULL);
    if (device == NULL) {
        ESP_LOGE(TAG, "Failed to create device");
        return NULL;
    }

    /* Register the write callback — this is how we receive commands */
    esp_rmaker_device_add_cb(device, write_cb, NULL);

    /* ---- Status Text (read-only, non-editable) ---- */
    esp_rmaker_param_t *status_param = esp_rmaker_param_create(
        PARAM_STATUS, ESP_RMAKER_PARAM_OTA_STATUS, esp_rmaker_str("Idle"),
        PROP_FLAG_READ);
    esp_rmaker_device_add_param(device, status_param);

    /* ---- Open Button (Primary Parameter) ----
     * We assign the Open button as the primary parameter of the device.
     * This registers Open as the main control (showing a shortcut on the app's home dashboard)
     * and prevents the app from defaulting to the Status string parameter as the primary,
     * which would display a text-edit button next to it. */
    esp_rmaker_param_t *open_param = esp_rmaker_param_create(
        PARAM_OPEN, ESP_RMAKER_PARAM_POWER, esp_rmaker_bool(false),
        PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(open_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(device, open_param);
    esp_rmaker_device_assign_primary_param(device, open_param);

    /* ---- Close Button ---- */
    esp_rmaker_param_t *close_param = esp_rmaker_param_create(
        PARAM_CLOSE, ESP_RMAKER_PARAM_TOGGLE, esp_rmaker_bool(false),
        PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(close_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(device, close_param);

    /* ---- Stop Button ---- */
    esp_rmaker_param_t *stop_param = esp_rmaker_param_create(
        PARAM_STOP, ESP_RMAKER_PARAM_TOGGLE, esp_rmaker_bool(false),
        PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(stop_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(device, stop_param);

    /* ---- Partial Open Button ---- */
    esp_rmaker_param_t *partial_param = esp_rmaker_param_create(
        PARAM_PARTIAL_OPEN, ESP_RMAKER_PARAM_TOGGLE, esp_rmaker_bool(false),
        PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(partial_param, ESP_RMAKER_UI_TRIGGER);
    esp_rmaker_device_add_param(device, partial_param);

    return device;
}

/* ---------------------------------------------------------------
 * build_status_string — Create a combined status string
 * ---------------------------------------------------------------
 * Combines movement status + gate position into one string.
 * Examples: "Idle · Open", "Opening · Partial", "Obstructed!"
 * --------------------------------------------------------------- */
static void build_status_string(char *buf, size_t len)
{
    const char *status = gate_get_status_string();
    const char *pos = gate_get_position_string();
    bool obstructed = gate_is_obstructed();

    if (obstructed) {
        if (strcmp(pos, "Partial") == 0) {
            snprintf(buf, len, "⚠ Obstructed!");
        } else {
            snprintf(buf, len, "⚠ Obstructed! · %s", pos);
        }
    } else {
        /* Print the gate status string directly. The gate status string already
         * includes ' · Partial' suffix during the partial open sequence, but remains
         * clean 'Opening' or 'Closing' for standard operations. */
        snprintf(buf, len, "%s", status);
    }
}

/* ---------------------------------------------------------------
 * update_status_in_app — Push combined status to RainMaker
 * --------------------------------------------------------------- */
static void update_status_in_app(void)
{
    if (s_gate_device) {
        char status_buf[64];
        build_status_string(status_buf, sizeof(status_buf));

        esp_rmaker_param_t *status_param = esp_rmaker_device_get_param_by_name(
            s_gate_device, PARAM_STATUS);
        if (status_param) {
            esp_rmaker_param_update_and_report(status_param,
                esp_rmaker_str(status_buf));
        }

        /* Sync Open/Power param with actual gate position for Alexa.
         * Only update when the gate reaches a definitive position (limit switch).
         * For intermediate positions (PARTIAL/UNKNOWN), keep the last command's
         * value to avoid flickering during movement. */
        esp_rmaker_param_t *open_param = esp_rmaker_device_get_param_by_name(
            s_gate_device, PARAM_OPEN);
        if (open_param) {
            gate_position_t pos = gate_get_position();
            if (pos == GATE_POS_OPEN) {
                esp_rmaker_param_update_and_report(open_param,
                    esp_rmaker_bool(true));
            } else if (pos == GATE_POS_CLOSED) {
                esp_rmaker_param_update_and_report(open_param,
                    esp_rmaker_bool(false));
            }
            /* PARTIAL/UNKNOWN: don't update — avoids flickering during movement */
        }
    }
}

/* ---------------------------------------------------------------
 * on_gate_position_changed — Limit Switch Callback
 * --------------------------------------------------------------- */
static void on_gate_position_changed(gate_position_t new_pos)
{
    ESP_LOGI(TAG, "Gate position changed to: %s", gate_get_position_string());
    update_status_in_app();
}

/* ---------------------------------------------------------------
 * on_gate_status_changed — Movement Tracking Callback
 * --------------------------------------------------------------- */
static void on_gate_status_changed(const char *status)
{
    ESP_LOGI(TAG, "Gate status update: %s", status);
    update_status_in_app();
}

/* ---------------------------------------------------------------
 * app_main — Application Entry Point
 * ---------------------------------------------------------------
 * This is the ESP-IDF equivalent of main(). It runs once at boot.
 *
 * The startup sequence is:
 *   1. Initialise NVS flash (stores Wi-Fi creds + RainMaker certs)
 *   2. Initialise gate control (GPIOs + relay task)
 *   3. Initialise Wi-Fi (via app_wifi helper from RainMaker SDK)
 *   4. Create RainMaker node and device
 *   5. Enable OTA + scheduling
 *   6. Start RainMaker agent
 *   7. Start Wi-Fi (enters provisioning if not yet configured)
 *
 * After this function returns, FreeRTOS takes over and runs the
 * tasks we created (gate_task, RainMaker MQTT task, Wi-Fi task, etc.)
 * --------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, "  Gate Controller — Starting Up");
    ESP_LOGI(TAG, "===================================");

    /* ---- Step 1: NVS Flash ----
     * Non-Volatile Storage is used by Wi-Fi, RainMaker, and our config
     * to persist data across reboots. Must be initialised first. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition full or outdated — erasing and re-init");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ---- Step 2: Gate Control ----
     * Initialise GPIOs and create the relay control task.
     * All relays are forced OFF at this point. */
    ESP_ERROR_CHECK(gate_init());

    /* ---- Step 3: Network Init ----
     * app_network_init() sets up the Wi-Fi driver and provisioning manager.
     * It does NOT connect yet — that happens in app_network_start(). */
    app_network_init();

    /* Check for rapid power-cycle to reset Wi-Fi credentials */
    check_rapid_power_cycle();

    /* ---- Step 4: RainMaker Node ----
     * A "node" is the top-level container for your ESP32 in the RainMaker
     * cloud. Each node can have multiple "devices" (we have one: the gate). */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true,   /* Sync time from cloud for timestamps */
    };

    /* Get unique MAC address to generate unique node and device names */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char unique_device_name[32];
    char unique_node_name[32];
    snprintf(unique_device_name, sizeof(unique_device_name), "Sliding Gate - %02X%02X", mac[4], mac[5]);
    snprintf(unique_node_name, sizeof(unique_node_name), "Gate Controller - %02X%02X", mac[4], mac[5]);

    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, unique_node_name, NODE_TYPE);
    if (node == NULL) {
        ESP_LOGE(TAG, "Failed to init RainMaker node — aborting");
        abort();
    }

    /* Create our gate device with all parameters */
    s_gate_device = create_gate_device(unique_device_name);
    if (s_gate_device == NULL) {
        ESP_LOGE(TAG, "Failed to create gate device — aborting");
        abort();
    }

    /* Add the device to the node */
    esp_rmaker_node_add_device(node, s_gate_device);

    /* Register callbacks for live app updates */
    gate_set_position_callback(on_gate_position_changed);
    gate_set_status_callback(on_gate_status_changed);

    /* Set initial status in RainMaker */
    update_status_in_app();

    /* ---- Step 5: Enable Services ---- */

    /* OTA: allows you to push firmware updates from the RainMaker dashboard
     * without physical access to the ESP32.
     * Uses the two OTA partitions we defined in partitions.csv.
     * The device downloads new firmware, writes it to the inactive partition,
     * verifies the checksum, and reboots into it. */
    esp_rmaker_ota_enable_default();

    /* Scheduling: allows users to create schedules in the app
     * e.g., "Close gate at 10:00 PM every day" */
    esp_rmaker_schedule_enable();

    /* Timezone Service: allows timezone updates from the cloud/mobile app to sync with the device */
    esp_rmaker_timezone_service_enable();

    /* Scenes: allows grouping multiple actions
     * e.g., "Night Mode" = Close gate + turn off lights */
    esp_rmaker_scenes_enable();

#if CONFIG_ESP_INSIGHTS_ENABLED
    /* Initialize ESP Insights remote diagnostics over HTTPS using the user's Auth Key */
    esp_insights_config_t insights_cfg = {
        .log_type = ESP_DIAG_LOG_TYPE_ERROR | ESP_DIAG_LOG_TYPE_WARNING,
        .auth_key = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyIjoiR29vZ2xlX1BhblBSR1A3ZzNYTDNjRUtROEpzcEQiLCJpc3MiOiJlMzIyYjU5Yy02M2NjLTRlNDAtOGVhMi00ZTc3NjY1NDVjY2EiLCJzdWIiOiJhNTQ2YWRhZC1lNzJiLTQwNTktYmQ2OS1jZDdlMjI0MjZiMmIiLCJleHAiOjIwOTcwNTU4MjgsImlhdCI6MTc4MTY5NTgyOH0.ov9MfxOvJKHnvudy5G3xg26gzAsqnH8-QoppswhEIr17hQScNa2zjqpSQjQA6C5-jXKysTXShyONDaUxXwMCr2P_1nmdS6YeNBkJUGAUU3VGSa8qGwzeZPQPfXPs-ISJ8OGIdVaP3_GQB4DIXXpKuanft45htByv09zNGUcJwLXWDw-LKevG2emfphyg2qIqcxqqdT0Nme3cwTaSkI0O2CTyF3BsMNTAdL3qbjybO67uUhiC3vSd5JAyorqprLX1NKJ5WnLaedTypmFoVr3wxgQ-c3zQ8V258lIyT47pvHGKBSku8mrAzKN33yQqqx_TY_WjA3dKiREdTsKuy6KkMA",
    };
    esp_err_t insights_err = esp_insights_init(&insights_cfg);
    if (insights_err == ESP_OK) {
        ESP_LOGI(TAG, "ESP Insights diagnostics initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize ESP Insights: %s", esp_err_to_name(insights_err));
    }
#endif


    /* ---- Step 6: Start RainMaker ----
     * This starts the MQTT connection to the RainMaker cloud.
     * If this is the first boot, it waits for provisioning first. */
    ESP_ERROR_CHECK(esp_rmaker_start());

    /* ---- Step 7: Start Wi-Fi ----
     * If Wi-Fi credentials are stored in NVS → connect immediately
     * If not (first boot) → enter BLE provisioning mode
     *
     * The provisioning flow:
     * 1. ESP32 starts advertising via BLE
     * 2. A QR code is printed in the serial monitor
     * 3. User opens RainMaker app → Add Device → Scan QR code
     * 4. App connects to ESP32 via BLE
     * 5. App sends Wi-Fi SSID + password
     * 6. App performs Assisted Claiming (gets certificate from cloud)
     * 7. ESP32 connects to Wi-Fi and RainMaker cloud
     *
     * POP (Proof of Possession) is a security code printed in the
     * serial log. Some boards use a fixed POP, others generate random.
     *
     * BOOT BUTTON RESETS:
     * - Hold 3s  → clears Wi-Fi creds, re-enters provisioning
     * - Hold 10s → full factory reset (clears everything) */
    /* Set custom static Proof of Possession (PoP) so we can pair manually
     * without needing to view the serial console log for a random PIN. */
    app_network_set_custom_pop("gate1234");
    err = app_network_start(POP_TYPE_CUSTOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "app_network_start failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, "  Startup complete — waiting for");
    ESP_LOGI(TAG, "  provisioning or cloud connection");
    ESP_LOGI(TAG, "===================================");
}
