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
 * Internal state (Global variables scoped to this file)
 *
 * Java note: 
 * In C, there are no classes. File-level global variables declared with 
 * `static` are private to this file (similar to `private static` fields 
 * in a singleton class). Only code inside this `app_gate.c` file can 
 * read or modify them.
 * --------------------------------------------------------------- */
static gate_state_t     s_state       = GATE_STATE_IDLE;
static gate_cmd_t       s_last_cmd;
static gate_movement_t  s_movement    = GATE_MOVE_NONE;

/* TickType_t represents scheduler clock ticks. 
 * Java Note: Think of it like System.currentTimeMillis(). FreeRTOS counts ticks 
 * (usually 1 tick = 1ms or 10ms depending on CONFIG_FREERTOS_HZ). */
static TickType_t       s_move_deadline = 0;

/* Standard C booleans. Since C99, bool is supported (historically C used integers 0/1). */
static bool             s_obstructed  = false;
static bool             s_timed_out   = false;
static bool             s_stopped     = false;

/* Configurable timing (can be changed at runtime via RainMaker) */
static uint32_t s_pulse_duration_ms  = DEFAULT_PULSE_OPEN_MS;
static uint32_t s_partial_delay_ms   = DEFAULT_PARTIAL_DELAY_MS;

/* FreeRTOS queue: A thread-safe message queue.
 * Java note: This behaves exactly like an ArrayBlockingQueue<GateCommand> of capacity 1. 
 * Any task (thread) can write to it, and another task can block waiting to read from it. */
static QueueHandle_t s_cmd_queue = NULL;

/* Callbacks: Function pointers.
 * Java note: Think of these like Interfaces or Listener objects (e.g. Consumer<Position>).
 * They store the address of a function to be executed when an event occurs. */
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
 * Java note: This functions like a blocking write to an I/O port.
 * We set the pin state, sleep the current thread (task), and then unset it.
 */
static void relay_pulse(int gpio, uint32_t duration_ms)
{
    ESP_LOGI(TAG, "Relay GPIO %d ON (pulse %lu ms)", gpio, (unsigned long)duration_ms);
    /* RELAY_ON is 0 (Active-LOW: shorting the pin to GND closes the physical relay contact) */
    gpio_set_level(gpio, RELAY_ON);
    
    /* pdMS_TO_TICKS converts milliseconds into RTOS scheduler ticks.
     * vTaskDelay is like Thread.sleep(duration_ms) — it yields CPU control to other tasks. */
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    
    /* RELAY_OFF is 1 (setting the pin HIGH turns off the relay coil) */
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
    /* gate_cmd_t is an integer type (enum). This variable will hold our current command.
     * Java equivalent: GateCommand cmd; */
    gate_cmd_t cmd;

    /* Java equivalent: public void run() { while (true) { ... } }
     * This is the core run-loop of the gate_task thread. It runs forever. 
     * In embedded systems, worker threads must never return. if a task function returns,
     * it will crash the system unless explicitly deleted via vTaskDelete(NULL). */
    for (;;) {
        /* Block the thread here until a command arrives.
         * Java equivalent: cmd = s_cmd_queue.take(); // blocks until an item is available
         *
         * Detailed C/RTOS explanation:
         * - s_cmd_queue: the thread-safe queue handle.
         * - &cmd: the address-of operator. C is pass-by-value. To write the dequeued command
         *         directly into our local variable, we pass its memory address (pointer).
         * - portMAX_DELAY: tells the FreeRTOS scheduler to put this task to sleep indefinitely
         *                  and consume 0% CPU until a message is posted to the queue.
         */
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_last_cmd = cmd;

        /* =================================================================
         * CASE 1: PARTIAL OPEN COMMAND
         * =================================================================
         * Rule: The gate should first close completely (pre-close) and then 
         * open partially. If the pre-close times out, it should STILL open
         * partially. It only aborts on physical obstruction or STOP command.
         * ================================================================= */
        if (cmd == GATE_CMD_PARTIAL_OPEN) {
            /* Read current physical position of the gate from the limit switches */
            gate_position_t pos = gate_get_position();
            if (pos != GATE_POS_CLOSED) {
                /* Gate is not closed: we must run a CLOSE sequence first */
                ESP_LOGI(TAG, "Gate not closed. Running CLOSE sequence first before partial open.");

                /* Temporarily update s_last_cmd to CLOSE so our status reporting reads "Closing" */
                s_last_cmd = GATE_CMD_CLOSE;

                /* Step A: Pulse the physical CLOSE relay (hold 500ms, then release) */
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_CLOSE, cmd_to_pulse_ms(GATE_CMD_CLOSE));

                /* Step B: Start background tracking for the close movement. 
                 * This sets s_movement to GATE_MOVE_CLOSING and sets a 20-second timeout deadline. */
                start_movement_tracking(GATE_MOVE_CLOSING);
                notify_status();

                bool close_success = false;
                bool interrupted_by_stop = false;

                /* Monitor movement while the gate is closing.
                 * Java Note: In Java, we might use a CountDownLatch, a Future, or a lock condition.
                 * In FreeRTOS, we loop and poll with a short timeout. */
                while (s_movement == GATE_MOVE_CLOSING) {
                    /* Check if a new command is received on the queue (e.g. an emergency STOP).
                     * We poll the queue with a 100ms timeout (converted to scheduler ticks).
                     * Java equivalent: GateCommand nextCmd = s_cmd_queue.poll(100, TimeUnit.MILLISECONDS); */
                    gate_cmd_t next_cmd;
                    if (xQueueReceive(s_cmd_queue, &next_cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (next_cmd == GATE_CMD_STOP) {
                            ESP_LOGI(TAG, "Close sequence during Partial Open interrupted by STOP");
                            /* Abort the movement tracking immediately */
                            cancel_movement_tracking();
                            s_stopped = true;
                            interrupted_by_stop = true;
                            
                            /* Pulse physical STOP relay immediately */
                            s_state = GATE_STATE_PULSING;
                            relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
                            break;
                        } else {
                            /* Any other commands (OPEN/CLOSE/PARTIAL) are ignored/rejected while busy */
                            ESP_LOGW(TAG, "Unexpected command %d during partial close tracking", next_cmd);
                        }
                    }
                }

                /* If we didn't hit an obstruction and the user didn't command a STOP,
                 * we consider this phase done (even if it timed out, as per instructions) */
                if (!s_obstructed && !interrupted_by_stop) {
                    close_success = true;
                }

                if (!close_success) {
                    ESP_LOGW(TAG, "Close sequence before partial open failed or was interrupted. Aborting partial open.");
                    /* Cool down the relays for 1 second, then return to IDLE */
                    s_state = GATE_STATE_COOLDOWN;
                    vTaskDelay(pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS));
                    s_state = GATE_STATE_IDLE;
                    notify_status();
                    continue; /* Jump back to the top of the event loop */
                }

                /* Close sequence finished (either reached limit switch or timed out).
                 * Let the gate motor rest/settle during a 1-second cooldown. */
                ESP_LOGI(TAG, "Close sequence completed. Performing cooldown before partial open.");
                s_state = GATE_STATE_COOLDOWN;
                vTaskDelay(pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS));

                /* Restore the command state to PARTIAL_OPEN for the next step */
                s_last_cmd = GATE_CMD_PARTIAL_OPEN;
            }

            /* --- Start the actual Partial Open sequence --- */
            ESP_LOGI(TAG, "=== PARTIAL OPEN sequence start ===");
            s_stopped = false;
            s_obstructed = false;

            /* Step 1: Pulse OPEN relay to start gate opening */
            s_state = GATE_STATE_PULSING;
            relay_pulse(GPIO_RELAY_OPEN, cmd_to_pulse_ms(GATE_CMD_OPEN));
            notify_status();

            /* Step 2: Wait for the configured partial delay (default 5 seconds).
             * We can't use simple sleep/delay because we must remain responsive to:
             * 1. Obstruction signals.
             * 2. Manual STOP commands. */
            s_state = GATE_STATE_PARTIAL_WAIT;
            ESP_LOGI(TAG, "Partial wait: %lu ms", (unsigned long)s_partial_delay_ms);
            notify_status();

            TickType_t start_tick = xTaskGetTickCount();
            TickType_t delay_ticks = pdMS_TO_TICKS(s_partial_delay_ms);
            bool interrupted = false;
            bool obstructed = false;

            /* Loop until the elapsed ticks exceed the target delay ticks.
             * Java note: In Java, we'd do: while (System.currentTimeMillis() - startTime < delayMs) { ... }
             * Note on C math: Unsigned subtraction `xTaskGetTickCount() - start_tick` handles tick overflow 
             * (wrap-around) automatically without any errors! */
            while ((xTaskGetTickCount() - start_tick) < delay_ticks) {
                /* Check safety obstruction input */
                if (gate_is_obstructed()) {
                    ESP_LOGW(TAG, "Obstruction detected during Partial Open!");
                    obstructed = true;
                    break;
                }

                /* Poll for incoming emergency STOP commands every 100ms */
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
                /* STOP command received during opening phase: pulse physical STOP relay */
                cancel_movement_tracking();
                s_stopped = true;
                s_obstructed = false;
                s_last_cmd = GATE_CMD_STOP;
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
            } else {
                /* Delay successfully completed! Pulse physical STOP relay to hold the gate midway */
                s_stopped = true;
                s_obstructed = false;
                s_last_cmd = GATE_CMD_STOP;
                s_state = GATE_STATE_PULSING;
                relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
            }
            notify_status();

            ESP_LOGI(TAG, "=== PARTIAL OPEN sequence complete ===");

        }
        /* =================================================================
         * CASE 2: STOP COMMAND
         * ================================================================= */
        else if (cmd == GATE_CMD_STOP) {
            ESP_LOGI(TAG, "Command: STOP");

            /* Cancel active movement tracking (opening/closing monitoring thread is stopped) */
            cancel_movement_tracking();

            s_state = GATE_STATE_PULSING;
            relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
        }
        /* =================================================================
         * CASE 3: OPEN OR CLOSE COMMANDS
         * ================================================================= */
        else {
            int gpio = cmd_to_gpio(cmd);
            uint32_t pulse_ms = cmd_to_pulse_ms(cmd);

            ESP_LOGI(TAG, "Command: %s",
                     cmd == GATE_CMD_OPEN ? "OPEN" : "CLOSE");

            /* Pulse the appropriate relay (OPEN or CLOSE) */
            s_state = GATE_STATE_PULSING;
            relay_pulse(gpio, pulse_ms);

            /* Start background tracking. The position_monitor_task will look for the 
             * respective limit switch or obstruction for up to 20 seconds. */
            start_movement_tracking(
                cmd == GATE_CMD_OPEN ? GATE_MOVE_OPENING : GATE_MOVE_CLOSING);
        }

        /* =================================================================
         * COOLDOWN PERIOD
         * =================================================================
         * Rule: Force a 1-second delay after any operation to protect the relays
         * and motors from back-to-back switching stress.
         * Exception: A manual STOP command must execute instantly, interrupting
         * the cooldown period.
         * ================================================================= */
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

            /* Block on the command queue for the REMAINING duration of the cooldown.
             * This allows us to wake up immediately if a command arrives. */
            gate_cmd_t pending_cmd;
            if (xQueueReceive(s_cmd_queue, &pending_cmd, remaining) == pdTRUE) {
                if (pending_cmd == GATE_CMD_STOP) {
                    /* Emergency STOP: abort cooldown, pulse STOP relay, and restart cooldown */
                    ESP_LOGI(TAG, "STOP command received during cooldown! Processing immediately.");
                    cancel_movement_tracking();
                    s_stopped = true;
                    s_obstructed = false;
                    s_timed_out = false;
                    s_last_cmd = GATE_CMD_STOP;

                    s_state = GATE_STATE_PULSING;
                    relay_pulse(GPIO_RELAY_STOP, DEFAULT_PULSE_STOP_MS);
                    notify_status();

                    /* Reset the 1-second cooldown specifically for the STOP pulse we just fired */
                    cooldown_start = xTaskGetTickCount();
                    cooldown_ticks = pdMS_TO_TICKS(DEFAULT_COOLDOWN_MS);
                    s_state = GATE_STATE_COOLDOWN;
                } else {
                    /* Reject any non-STOP commands during active cooldown */
                    ESP_LOGW(TAG, "Command %d rejected during active cooldown", pending_cmd);
                }
            }
        }

        /* Cooldown finished: transition to IDLE and wait for next command */
        s_state = GATE_STATE_IDLE;
        ESP_LOGI(TAG, "Ready for next command");
        notify_status();
    }
}

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */

/**
 * Initialise the gate hardware configuration and background tasks.
 * Java Note: Think of this as our application initializer/bootstrap method.
 * We configure the hardware pins and start the daemon threads (FreeRTOS Tasks).
 *
 * @return ESP_OK (0) on success, or ESP_FAIL (-1) if task/queue creation fails.
 */
esp_err_t gate_init(void)
{
    ESP_LOGI(TAG, "Initialising gate control GPIOs");

    /* Configure all relay GPIOs as outputs, default HIGH (OFF)
     * Java Note: In C, we declare arrays using `type name[] = { ... }`.
     * `sizeof(relay_gpios) / sizeof(relay_gpios[0])` calculates the array length
     * (total bytes of the array divided by bytes of one element). */
    const int relay_gpios[] = {
        GPIO_RELAY_OPEN, GPIO_RELAY_CLOSE, GPIO_RELAY_STOP, GPIO_RELAY_SPARE
    };

    for (int i = 0; i < sizeof(relay_gpios) / sizeof(relay_gpios[0]); i++) {
        /* Reset and initialize the GPIO pin */
        gpio_reset_pin(relay_gpios[i]);
        /* Set direction as OUTPUT (we write to relays) */
        gpio_set_direction(relay_gpios[i], GPIO_MODE_OUTPUT);
        /* Force pin to HIGH (RELAY_OFF = 1) at boot to prevent relays from pulsing on power-up */
        gpio_set_level(relay_gpios[i], RELAY_OFF);
    }

    /* Configure limit switch GPIOs as inputs with internal pull-ups
     *
     * Why PULL-UP?
     * The limit switches are dry contacts. When open, the line "floats" and would give random noise.
     * Enabling the internal pull-up resistor pulls the pin to 3.3V (HIGH / 1) by default.
     * When the gate reaches the limit and hits the switch, the contact shorts the pin to GND (LOW / 0).
     */
    const int input_gpios[] = { GPIO_LIMIT_OPEN, GPIO_LIMIT_CLOSE, GPIO_OBSTRUCTION };
    for (int i = 0; i < sizeof(input_gpios) / sizeof(input_gpios[0]); i++) {
        gpio_reset_pin(input_gpios[i]);
        gpio_set_direction(input_gpios[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(input_gpios[i], GPIO_PULLUP_ONLY);
    }
    ESP_LOGI(TAG, "Inputs configured: OP=GPIO%d, CL=GPIO%d, OBSTRUCTION=GPIO%d",
             GPIO_LIMIT_OPEN, GPIO_LIMIT_CLOSE, GPIO_OBSTRUCTION);

    /* Create command queue (depth 1 = reject if already processing a command)
     * Java equivalent: s_cmd_queue = new ArrayBlockingQueue<>(1); */
    s_cmd_queue = xQueueCreate(1, sizeof(gate_cmd_t));
    if (s_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_FAIL;
    }

    /* Create the main gate control task
     * Java equivalent: new Thread(gate_task).start();
     *
     * Task properties:
     * - gate_task: function pointer containing the run-loop code.
     * - "gate_task": thread label for debugging.
     * - 4096: Stack size in bytes.
     * - 5: Priority (higher priority runs first; 5 is relatively high).
     */
    BaseType_t ret = xTaskCreate(gate_task, "gate_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gate task");
        return ESP_FAIL;
    }

    /* Create the position & movement monitor task (polls limit switches every 500ms)
     * Java equivalent: ScheduledExecutorService or another background Thread. */
    ret = xTaskCreate(position_monitor_task, "pos_task", 2048, NULL, 4, NULL);
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create position monitor task");
    }

    s_state = GATE_STATE_IDLE;
    ESP_LOGI(TAG, "Gate control initialised — all relays OFF, position: %s",
             gate_get_position_string());
    return ESP_OK;
}

/**
 * Public API to send a command to the gate.
 * Java Note: This is a thread-safe producer method. It checks if the command 
 * is allowed based on the gate's state machine, and if valid, posts it to the
 * queue for execution by the background task thread.
 *
 * @param cmd The gate command (OPEN, CLOSE, STOP, PARTIAL_OPEN)
 * @return ESP_OK (0) if successfully queued, or ESP_ERR_INVALID_STATE (0x103) if rejected.
 */
esp_err_t gate_command(gate_cmd_t cmd)
{
    /* Read current positions and movement states to decide if the command is valid */
    gate_position_t current_pos = gate_get_position();
    bool is_opened = (current_pos == GATE_POS_OPEN);
    bool is_closed = (current_pos == GATE_POS_CLOSED);
    
    bool is_opening = (s_movement == GATE_MOVE_OPENING || 
                       s_state == GATE_STATE_PARTIAL_WAIT ||
                       (s_state != GATE_STATE_IDLE && 
                        (s_last_cmd == GATE_CMD_OPEN || s_last_cmd == GATE_CMD_PARTIAL_OPEN)));
                       
    bool is_closing = (s_movement == GATE_MOVE_CLOSING || 
                       (s_state != GATE_STATE_IDLE && s_last_cmd == GATE_CMD_CLOSE));

    /* Validate command transitions based on workflow rules */
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
    /* Store the previous states so we only trigger callbacks on state transitions.
     * Java equivalent: storing previous values in a state machine object to detect changes. */
    gate_position_t last_pos = gate_get_position();
    bool last_obstruction = gate_is_obstructed();

    ESP_LOGI(TAG, "Position monitor started — initial: %s, obstructed: %s",
             gate_get_position_string(), last_obstruction ? "YES" : "no");

    /* Infinite loop for the background position monitor thread */
    for (;;) {
        /* Sleep the thread for POSITION_POLL_MS (500 ms).
         * Java equivalent: Thread.sleep(500);
         * 
         * Why are we polling instead of using hardware interrupts?
         * Physical switches (magnetic reed/limit switches) suffer from "contact bounce" where they open 
         * and close rapidly for a few milliseconds when triggered. Polling them at 500ms intervals acts 
         * as a natural, software-based "debounce" filter and simplifies the circuit logic.
         */
        vTaskDelay(pdMS_TO_TICKS(POSITION_POLL_MS));

        /* Read the current electrical GPIO pin status */
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

            /* Fire the registered callback listener if it has been configured.
             * Java equivalent: if (positionListener != null) { positionListener.onChanged(currentPos); } */
            if (s_position_cb) {
                s_position_cb(current_pos);
            }
        }

        /* --- 2. Movement tracking ---
         * If the gate is supposed to be moving (Opening/Closing), track its progress. */
        if (s_movement != GATE_MOVE_NONE) {
            bool reached_target = false;

            /* Check if gate reached the respective target limit switch */
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

            /* Obstruction safety check: if movement is active and safety sensor triggers,
             * stop tracking immediately. The physical gate controller will reverse/stop the motor. */
            if (!reached_target && current_obstruction) {
                ESP_LOGW(TAG, "⚠ Obstruction detected during movement!");
                s_obstructed = true;
                s_movement = GATE_MOVE_NONE;
                notify_status();
            }

            /* Timeout check: if the gate has been moving longer than the safety limit (20s)
             * without hitting the limit switch, declare a movement timeout. */
            if (!reached_target && s_movement != GATE_MOVE_NONE &&
                xTaskGetTickCount() >= s_move_deadline) {
                ESP_LOGW(TAG, "⚠ Movement timeout — gate did not reach target in %d ms",
                         GATE_MOVE_TIMEOUT_MS);
                s_stopped = true;
                s_movement = GATE_MOVE_NONE;
                notify_status();
            }

            /* If the gate successfully reached its end switch, clear any warning/stopped states */
            if (reached_target) {
                s_obstructed = false;
                s_timed_out = false;
                s_stopped = false;
                notify_status();
            }
        }

        /* --- 3. Obstruction change detection (independent of movement) ---
         * This checks for the obstruction status changes to keep the phone app state up-to-date
         * even when the gate is not actively commanded to move. */
        if (current_obstruction != last_obstruction) {
            ESP_LOGI(TAG, "Obstruction signal changed: %s → %s",
                     last_obstruction ? "ACTIVE" : "clear",
                     current_obstruction ? "ACTIVE" : "clear");
            last_obstruction = current_obstruction;

            /* If obstruction was cleared, update state */
            if (!current_obstruction && s_obstructed) {
                s_obstructed = false;
                notify_status();
            }
        }
    }
}
