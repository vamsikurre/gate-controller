/*
 * app_gate.c — Relay pulse logic, state machine, position & obstruction monitoring
 *
 * HOW IT WORKS
 * ============
 * The gate controller board has dry-contact inputs (OPEN, CLOSE, STOP).
 * Shorting COM↔OPEN starts the gate opening, etc. Our relays simulate
 * these button presses by briefly closing the NO (Normally Open) contacts.
 *
 * ARCHITECTURE
 * ============
 * Two FreeRTOS tasks work together:
 *
 * 1. gate_task (relay control):
 *    - Receives commands via a queue
 *    - Pulses the appropriate relay
 *    - Sets movement tracking target (e.g., "expect gate to reach OPEN")
 *
 * 2. position_monitor_task (feedback):
 *    - Polls limit switches + obstruction GPIO every 500ms
 *    - Tracks movement: "Opening" until limit switch confirms or 10s timeout
 *    - Fires callbacks to update the RainMaker app
 *
 * MOVEMENT TRACKING
 * =================
 * After pressing OPEN:
 *   Status = "Opening" for up to 10 seconds
 *   → If OP limit switch triggers: Status = "Idle", Position = "Open"
 *   → If 10s timeout: Status = "Timeout" (possible obstruction)
 *   → If obstruction GPIO triggers: Status = "Obstructed"
 *   → If STOP command received: tracking cancelled, Status = "Idle"
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "app_priv.h"
#include "app_gate.h"

static const char *TAG = "gate";

/* ---------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------- */
static gate_state_t     s_state       = GATE_STATE_IDLE;
static gate_cmd_t       s_last_cmd;
static gate_movement_t  s_movement    = GATE_MOVE_NONE;
static TickType_t       s_move_deadline = 0;
static bool             s_obstructed  = false;
static bool             s_timed_out   = false;
static bool             s_stopped     = false;

/* Configurable timing (can be changed at runtime via RainMaker) */
static uint32_t s_pulse_duration_ms  = DEFAULT_PULSE_OPEN_MS;
static uint32_t s_partial_delay_ms   = DEFAULT_PARTIAL_DELAY_MS;

/* FreeRTOS queue: holds pending commands (depth = 1, we reject if full) */
static QueueHandle_t s_cmd_queue = NULL;

/* Callbacks */
static gate_position_cb_t s_position_cb = NULL;
static gate_status_cb_t   s_status_cb   = NULL;

/* Forward declarations */
static void position_monitor_task(void *arg);
static void notify_status(void);

/* ---------------------------------------------------------------
 * GPIO helpers
 * --------------------------------------------------------------- */

/**
 * Map a command to its corresponding GPIO pin.
 */
static int cmd_to_gpio(gate_cmd_t cmd)
{
    switch (cmd) {
        case GATE_CMD_OPEN:         return GPIO_RELAY_OPEN;
        case GATE_CMD_CLOSE:        return GPIO_RELAY_CLOSE;
        case GATE_CMD_STOP:         return GPIO_RELAY_STOP;
        case GATE_CMD_PARTIAL_OPEN: return GPIO_RELAY_OPEN;  /* Uses OPEN relay */
        default:                    return -1;
    }
}

/**
 * Get the pulse duration for a specific command.
 */
static uint32_t cmd_to_pulse_ms(gate_cmd_t cmd)
{
    switch (cmd) {
        case GATE_CMD_OPEN:         return s_pulse_duration_ms;
        case GATE_CMD_CLOSE:        return s_pulse_duration_ms;
        case GATE_CMD_STOP:         return DEFAULT_PULSE_STOP_MS;
        case GATE_CMD_PARTIAL_OPEN: return s_pulse_duration_ms;
        default:                    return s_pulse_duration_ms;
    }
}

/**
 * Pulse a single relay: LOW for duration_ms, then HIGH.
 */
static void relay_pulse(int gpio, uint32_t duration_ms)
{
    ESP_LOGI(TAG, "Relay GPIO %d ON (pulse %lu ms)", gpio, (unsigned long)duration_ms);
    gpio_set_level(gpio, RELAY_ON);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(gpio, RELAY_OFF);
    ESP_LOGI(TAG, "Relay GPIO %d OFF", gpio);
}

/**
 * Start movement tracking — sets the target position and deadline.
 */
static void start_movement_tracking(gate_movement_t movement)
{
    s_movement = movement;
    s_move_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(GATE_MOVE_TIMEOUT_MS);
    s_obstructed = false;
    s_timed_out = false;
    ESP_LOGI(TAG, "Movement tracking: %s (timeout %d ms)",
             movement == GATE_MOVE_OPENING ? "OPENING" : "CLOSING",
             GATE_MOVE_TIMEOUT_MS);
}

/**
 * Cancel movement tracking (e.g., on STOP command).
 */
static void cancel_movement_tracking(void)
{
    if (s_movement != GATE_MOVE_NONE) {
        ESP_LOGI(TAG, "Movement tracking cancelled");
        s_movement = GATE_MOVE_NONE;
    }
}

/* ---------------------------------------------------------------
 * State machine task
 * ---------------------------------------------------------------
 * Processes commands from the queue:
 *   IDLE → PULSING → COOLDOWN → IDLE (+ start movement tracking)
 * --------------------------------------------------------------- */
static void gate_task(void *arg)
{
    gate_cmd_t cmd;

    for (;;) {
        /* Block here until a command arrives */
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_last_cmd = cmd;

        if (cmd == GATE_CMD_PARTIAL_OPEN) {
            /* Check if already in closed state */
            gate_position_t pos = gate_get_position();
            if (pos != GATE_POS_CLOSED) {
                /* Not closed: first run the Close sequence */
                ESP_LOGI(TAG, "Gate not closed. Running CLOSE sequence first before partial open.");

                /* Temporarily set last cmd to CLOSE so status reports "Closing" */
                s_last_cmd = GATE_CMD_CLOSE;

                /* Step A: Pulse CLOSE relay */
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_CLOSE, cmd_to_pulse_ms(GATE_CMD_CLOSE));

                /* Step B: Wait/track closing movement */
                start_movement_tracking(GATE_MOVE_CLOSING);
                notify_status();

                bool close_success = false;

                while (s_movement == GATE_MOVE_CLOSING) {
                    /* Check if a command is received in the queue (like STOP) */
                    gate_cmd_t next_cmd;
                    if (xQueueReceive(s_cmd_queue, &next_cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (next_cmd == GATE_CMD_STOP) {
                            ESP_LOGI(TAG, "Close sequence during Partial Open interrupted by STOP");
                            cancel_movement_tracking();
                            s_stopped = true;
                            s_state = GATE_STATE_PULSING;
                            relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
                            break;
                        } else {
                            ESP_LOGW(TAG, "Unexpected command %d during partial close tracking", next_cmd);
                        }
                    }
                }

                /* Check if we reached closed successfully */
                if (gate_get_position() == GATE_POS_CLOSED && !s_obstructed && !s_stopped) {
                    close_success = true;
                }

                if (!close_success) {
                    ESP_LOGW(TAG, "Close sequence before partial open failed or was interrupted. Aborting partial open.");
                    /* Transition to cooldown then ready */
                    s_state = GATE_STATE_COOLDOWN;
                    vTaskDelay(pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS));
                    s_state = GATE_STATE_IDLE;
                    notify_status();
                    continue; // Skip the rest of the partial open sequence
                }

                /* Close succeeded, wait for cooldown before starting the open pulse */
                ESP_LOGI(TAG, "Close sequence succeeded. Performing cooldown before partial open.");
                s_state = GATE_STATE_COOLDOWN;
                vTaskDelay(pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS));

                /* Restore last cmd to PARTIAL_OPEN */
                s_last_cmd = GATE_CMD_PARTIAL_OPEN;
            }

            /* --- Start the actual Partial Open sequence --- */
            ESP_LOGI(TAG, "=== PARTIAL OPEN sequence start ===");

            /* Step 1: Pulse OPEN relay */
            s_state = GATE_STATE_PULSING;
            relay_pulse(GPIO_RELAY_OPEN, cmd_to_pulse_ms(GATE_CMD_OPEN));
            notify_status();

            /* Step 2: Wait for the configured delay, but check if we get a STOP command or obstruction */
            s_state = GATE_STATE_PARTIAL_WAIT;
            ESP_LOGI(TAG, "Partial wait: %lu ms", (unsigned long)s_partial_delay_ms);
            notify_status();

            TickType_t start_tick = xTaskGetTickCount();
            TickType_t delay_ticks = pdMS_TO_TICKS(s_partial_delay_ms);
            bool interrupted = false;
            bool obstructed = false;

            while ((xTaskGetTickCount() - start_tick) < delay_ticks) {
                /* Poll for obstruction every 100ms */
                if (gate_is_obstructed()) {
                    ESP_LOGW(TAG, "Obstruction detected during Partial Open!");
                    obstructed = true;
                    break;
                }

                /* Check if a command is received (like STOP) */
                gate_cmd_t next_cmd;
                if (xQueueReceive(s_cmd_queue, &next_cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (next_cmd == GATE_CMD_STOP) {
                        ESP_LOGI(TAG, "Partial open interrupted by STOP command");
                        interrupted = true;
                        break;
                    } else {
                        ESP_LOGW(TAG, "Unexpected command %d during partial wait", next_cmd);
                    }
                }
            }
            if (obstructed) {
                s_obstructed = true;
                s_stopped = false;
            } else if (interrupted) {
                cancel_movement_tracking();
                s_stopped = true;
                s_obstructed = false;
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
            } else {
                /* Step 3: Pulse STOP relay automatically after delay completed */
                s_stopped = true;
                s_obstructed = false;
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
            }
            notify_status();

            ESP_LOGI(TAG, "=== PARTIAL OPEN sequence complete ===");

            /* No movement tracking for partial — it's self-contained */

        } else if (cmd == GATE_CMD_STOP) {
            /* --- Stop command --- */
            ESP_LOGI(TAG, "Command: STOP");

            /* Cancel any active movement tracking */
            cancel_movement_tracking();

            s_state = GATE_STATE_PULSING;
            relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);

        } else {
            /* --- Open or Close --- */
            int gpio = cmd_to_gpio(cmd);
            uint32_t pulse_ms = cmd_to_pulse_ms(cmd);

            ESP_LOGI(TAG, "Command: %s",
                     cmd == GATE_CMD_OPEN ? "OPEN" : "CLOSE");

            s_state = GATE_STATE_PULSING;
            relay_pulse(gpio, pulse_ms);

            /* Start movement tracking — status stays "Opening"/"Closing"
             * until limit switch confirms or 10s timeout */
            start_movement_tracking(
                cmd == GATE_CMD_OPEN ? GATE_MOVE_OPENING : GATE_MOVE_CLOSING);
        }

        /* --- Cooldown period --- */
        s_state = GATE_STATE_COOLDOWN;
        ESP_LOGI(TAG, "Cooldown: %d ms", DEFAULT_COOLDOWN_MS);

        TickType_t cooldown_start = xTaskGetTickCount();
        TickType_t cooldown_ticks = pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS);

        while ((xTaskGetTickCount() - cooldown_start) < cooldown_ticks) {
            TickType_t elapsed = xTaskGetTickCount() - cooldown_start;
            if (elapsed >= cooldown_ticks) {
                break;
            }
            TickType_t remaining = cooldown_ticks - elapsed;

            gate_cmd_t pending_cmd;
            if (xQueueReceive(s_cmd_queue, &pending_cmd, remaining) == pdTRUE) {
                if (pending_cmd == GATE_CMD_STOP) {
                    ESP_LOGI(TAG, "STOP command received during cooldown! Processing immediately.");
                    /* Execute STOP logic immediately */
                    cancel_movement_tracking();
                    s_stopped = true;
                    s_obstructed = false;
                    s_timed_out = false;
                    s_last_cmd = GATE_CMD_STOP;

                    s_state = GATE_STATE_PULSING;
                    relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
                    notify_status();

                    /* Reset cooldown for the STOP command */
                    cooldown_start = xTaskGetTickCount();
                    cooldown_ticks = pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS);
                    s_state = GATE_STATE_COOLDOWN;
                } else {
                    ESP_LOGW(TAG, "Command %d rejected during active cooldown", pending_cmd);
                }
            }
        }

        s_state = GATE_STATE_IDLE;
        ESP_LOGI(TAG, "Ready for next command");
    }
}

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */

esp_err_t gate_init(void)
{
    ESP_LOGI(TAG, "Initialising gate control GPIOs");

    /* Configure all relay GPIOs as outputs, default HIGH (OFF) */
    const int relay_gpios[] = {
        GPIO_RELAY_OPEN, GPIO_RELAY_CLOSE, GPIO_RELAY_STOP, GPIO_RELAY_SPARE
    };

    for (int i = 0; i < sizeof(relay_gpios) / sizeof(relay_gpios[0]); i++) {
        gpio_reset_pin(relay_gpios[i]);
        gpio_set_direction(relay_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(relay_gpios[i], RELAY_OFF);  /* Ensure OFF at boot */
    }

    /* Configure limit switch GPIOs as inputs with internal pull-ups */
    const int input_gpios[] = { GPIO_LIMIT_OPEN, GPIO_LIMIT_CLOSE, GPIO_OBSTRUCTION };
    for (int i = 0; i < sizeof(input_gpios) / sizeof(input_gpios[0]); i++) {
        gpio_reset_pin(input_gpios[i]);
        gpio_set_direction(input_gpios[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(input_gpios[i], GPIO_PULLUP_ONLY);
    }
    ESP_LOGI(TAG, "Inputs configured: OP=GPIO%d, CL=GPIO%d, OBSTRUCTION=GPIO%d",
             GPIO_LIMIT_OPEN, GPIO_LIMIT_CLOSE, GPIO_OBSTRUCTION);

    /* Create command queue (depth 1 = reject if already processing) */
    s_cmd_queue = xQueueCreate(1, sizeof(gate_cmd_t));
    if (s_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_FAIL;
    }

    /* Create the gate control task */
    BaseType_t ret = xTaskCreate(gate_task, "gate_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gate task");
        return ESP_FAIL;
    }

    /* Create the position & movement monitor task */
    ret = xTaskCreate(position_monitor_task, "pos_task", 2048, NULL, 4, NULL);
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create position monitor task");
    }

    s_state = GATE_STATE_IDLE;
    ESP_LOGI(TAG, "Gate control initialised — all relays OFF, position: %s",
             gate_get_position_string());
    return ESP_OK;
}

esp_err_t gate_command(gate_cmd_t cmd)
{
    /* Read current states based on diagram */
    gate_position_t current_pos = gate_get_position();
    bool is_opened = (current_pos == GATE_POS_OPEN);
    bool is_closed = (current_pos == GATE_POS_CLOSED);
    
    bool is_opening = (s_movement == GATE_MOVE_OPENING || 
                       s_state == GATE_STATE_PARTIAL_WAIT ||
                       (s_state != GATE_STATE_IDLE && 
                        (s_last_cmd == GATE_CMD_OPEN || s_last_cmd == GATE_CMD_PARTIAL_OPEN)));
                       
    bool is_closing = (s_movement == GATE_MOVE_CLOSING || 
                       (s_state != GATE_STATE_IDLE && s_last_cmd == GATE_CMD_CLOSE));

    bool is_stopped = s_stopped;
    bool is_obstructed = s_obstructed;

    /* Validate command transitions based on workflow diagram */
    if (cmd == GATE_CMD_OPEN) {
        if (is_opened || is_opening) {
            ESP_LOGW(TAG, "Open command rejected: already in opened/opening state");
            return ESP_ERR_INVALID_STATE;
        }
    }
    else if (cmd == GATE_CMD_CLOSE) {
        if (is_closed || is_closing) {
            ESP_LOGW(TAG, "Close command rejected: already in closed/closing state");
            return ESP_ERR_INVALID_STATE;
        }
    }
    else if (cmd == GATE_CMD_PARTIAL_OPEN) {
        if (is_opening) {
            ESP_LOGW(TAG, "Partial Open command rejected: already in opening state");
            return ESP_ERR_INVALID_STATE;
        }
    }
    else if (cmd == GATE_CMD_STOP) {
        if (is_stopped || is_obstructed || is_opened || is_closed) {
            ESP_LOGW(TAG, "Stop command rejected: already in stopped/obstructed/opened/closed state");
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* Allow STOP even during pulsing, cooldown, or partial wait to ensure immediate response */
    if (s_state != GATE_STATE_IDLE && 
        !(cmd == GATE_CMD_STOP && (s_state == GATE_STATE_PARTIAL_WAIT || s_state == GATE_STATE_COOLDOWN || s_state == GATE_STATE_PULSING))) {
        ESP_LOGW(TAG, "Command rejected — state machine busy (state=%d)", s_state);
        return ESP_ERR_INVALID_STATE;
    }

    /* Try to post to queue without blocking (xTicksToWait = 0) */
    if (xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command rejected — queue full");
        return ESP_ERR_INVALID_STATE;
    }

    if (cmd == GATE_CMD_STOP) {
        s_stopped = true;
        s_obstructed = false;
        s_timed_out = false;
        s_movement = GATE_MOVE_NONE;
    } else {
        s_stopped = false;
        s_obstructed = false;
        s_timed_out = false;
    }

    return ESP_OK;
}

gate_state_t gate_get_state(void)
{
    return s_state;
}
gate_movement_t gate_get_movement(void)
{
    return s_movement;
}

const char *gate_get_status_string(void)
{
    /* Priority 1: Obstruction detected */
    if (s_obstructed) {
        return "Obstructed";
    }

    /* Priority 2: Actively pulsing / cooldown / partial wait */
    if (s_state != GATE_STATE_IDLE) {
        switch (s_state) {
            case GATE_STATE_PULSING:
            case GATE_STATE_COOLDOWN:
                switch (s_last_cmd) {
                    case GATE_CMD_OPEN:         return "Opening";
                    case GATE_CMD_CLOSE:        return "Closing";
                    case GATE_CMD_STOP:         return "Stopped";
                    case GATE_CMD_PARTIAL_OPEN: return "Opening";
                    default:                    return "Stopped";
                }
            case GATE_STATE_PARTIAL_WAIT:
                return "Opening";
            default:
                return "Stopped";
        }
    }

    /* Priority 3: Movement tracking */
    if (s_movement == GATE_MOVE_OPENING) {
        return "Opening";
    }
    if (s_movement == GATE_MOVE_CLOSING) {
        return "Closing";
    }

    /* Priority 4: Stopped flag */
    if (s_stopped) {
        return "Stopped";
    }

    /* Priority 5: Limit switches */
    gate_position_t pos = gate_get_position();
    if (pos == GATE_POS_OPEN) {
        return "Opened";
    }
    if (pos == GATE_POS_CLOSED) {
        return "Closed";
    }

    /* Priority 6: Default fallback (Idle/Stopped) */
    return "Stopped";
}

void gate_set_pulse_duration(uint32_t ms)
{
    if (ms > 0 && ms <= 2000) {
        s_pulse_duration_ms = ms;
        ESP_LOGI(TAG, "Pulse duration set to %lu ms", (unsigned long)ms);
    }
}

void gate_set_partial_delay(uint32_t ms)
{
    if (ms > 0 && ms <= 30000) {
        s_partial_delay_ms = ms;
        ESP_LOGI(TAG, "Partial delay set to %lu ms", (unsigned long)ms);
    }
}

/* ---------------------------------------------------------------
 * Gate Position Reading
 * --------------------------------------------------------------- */

gate_position_t gate_get_position(void)
{
    int op = gpio_get_level(GPIO_LIMIT_OPEN);
    int cl = gpio_get_level(GPIO_LIMIT_CLOSE);

    if (op == 0 && cl == 1) {
        return GATE_POS_OPEN;       /* OP switch triggered */
    } else if (op == 1 && cl == 0) {
        return GATE_POS_CLOSED;     /* CL switch triggered */
    } else if (op == 1 && cl == 1) {
        return GATE_POS_PARTIAL;    /* Neither triggered — moving or partial */
    } else {
        return GATE_POS_UNKNOWN;    /* Both triggered — shouldn't happen */
    }
}

const char *gate_get_position_string(void)
{
    switch (gate_get_position()) {
        case GATE_POS_OPEN:     return "Open";
        case GATE_POS_CLOSED:   return "Closed";
        case GATE_POS_PARTIAL:  return "Partial";
        case GATE_POS_UNKNOWN:
        default:                return "Unknown";
    }
}

/* ---------------------------------------------------------------
 * Obstruction Detection
 * --------------------------------------------------------------- */

bool gate_is_obstructed(void)
{
    return gpio_get_level(GPIO_OBSTRUCTION) == 0;  /* Active LOW */
}

/* ---------------------------------------------------------------
 * Callbacks
 * --------------------------------------------------------------- */

void gate_set_position_callback(gate_position_cb_t cb)
{
    s_position_cb = cb;
}

void gate_set_status_callback(gate_status_cb_t cb)
{
    s_status_cb = cb;
}

/**
 * Helper: fire status callback with current status string.
 */
static void notify_status(void)
{
    if (s_status_cb) {
        s_status_cb(gate_get_status_string());
    }
}

/* ---------------------------------------------------------------
 * Position & Movement Monitor Task
 * ---------------------------------------------------------------
 * Runs every 500ms. Responsibilities:
 *
 * 1. Detect limit switch changes → report position to RainMaker
 * 2. Track movement after Open/Close command:
 *    - "Opening" until OP limit switch triggers or 10s timeout
 *    - "Closing" until CL limit switch triggers or 10s timeout
 * 3. Detect obstruction signal
 * --------------------------------------------------------------- */
static void position_monitor_task(void *arg)
{
    gate_position_t last_pos = gate_get_position();
    bool last_obstruction = gate_is_obstructed();

    ESP_LOGI(TAG, "Position monitor started — initial: %s, obstructed: %s",
             gate_get_position_string(), last_obstruction ? "YES" : "no");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POSITION_POLL_MS));

        gate_position_t current_pos = gate_get_position();
        bool current_obstruction = gate_is_obstructed();

        /* --- 1. Position change detection --- */
        if (current_pos != last_pos) {
            ESP_LOGI(TAG, "Position changed: %s → %s",
                     last_pos == GATE_POS_OPEN    ? "Open" :
                     last_pos == GATE_POS_CLOSED   ? "Closed" :
                     last_pos == GATE_POS_PARTIAL  ? "Partial" : "Unknown",
                     current_pos == GATE_POS_OPEN   ? "Open" :
                     current_pos == GATE_POS_CLOSED ? "Closed" :
                     current_pos == GATE_POS_PARTIAL ? "Partial" : "Unknown");

            last_pos = current_pos;

            if (s_position_cb) {
                s_position_cb(current_pos);
            }
        }

        /* --- 2. Movement tracking --- */
        if (s_movement != GATE_MOVE_NONE) {
            bool reached_target = false;

            /* Check if gate reached target position */
            if (s_movement == GATE_MOVE_OPENING && current_pos == GATE_POS_OPEN) {
                ESP_LOGI(TAG, "✓ Gate reached OPEN position");
                s_movement = GATE_MOVE_NONE;
                reached_target = true;
            }
            else if (s_movement == GATE_MOVE_CLOSING && current_pos == GATE_POS_CLOSED) {
                ESP_LOGI(TAG, "✓ Gate reached CLOSED position");
                s_movement = GATE_MOVE_NONE;
                reached_target = true;
            }

            /* Check for obstruction */
            if (!reached_target && current_obstruction) {
                ESP_LOGW(TAG, "⚠ Obstruction detected during movement!");
                s_obstructed = true;
                s_movement = GATE_MOVE_NONE;
                notify_status();
            }

            /* Check for timeout */
            if (!reached_target && s_movement != GATE_MOVE_NONE &&
                xTaskGetTickCount() >= s_move_deadline) {
                ESP_LOGW(TAG, "⚠ Movement timeout — gate did not reach target in %d ms",
                         GATE_MOVE_TIMEOUT_MS);
                s_stopped = true;
                s_movement = GATE_MOVE_NONE;
                notify_status();
            }

            /* If target reached, clear any previous error/stopped flags */
            if (reached_target) {
                s_obstructed = false;
                s_timed_out = false;
                s_stopped = false;
                notify_status();
            }
        }

        /* --- 3. Obstruction change detection (independent of movement) --- */
        if (current_obstruction != last_obstruction) {
            ESP_LOGI(TAG, "Obstruction signal changed: %s → %s",
                     last_obstruction ? "ACTIVE" : "clear",
                     current_obstruction ? "ACTIVE" : "clear");
            last_obstruction = current_obstruction;

            /* If obstruction just cleared, reset the flag */
            if (!current_obstruction && s_obstructed) {
                s_obstructed = false;
                notify_status();
            }
        }
    }
}
