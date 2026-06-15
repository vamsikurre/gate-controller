/*
 * app_gate.h — Gate control API
 *
 * Public interface for the relay pulse state machine.
 * All functions are thread-safe (use a FreeRTOS queue internally).
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Gate commands — each maps to a specific relay */
typedef enum {
    GATE_CMD_OPEN,
    GATE_CMD_CLOSE,
    GATE_CMD_STOP,
    GATE_CMD_PARTIAL_OPEN,
} gate_cmd_t;

/* Current state of the gate control state machine */
typedef enum {
    GATE_STATE_IDLE,            /* Ready to accept commands               */
    GATE_STATE_PULSING,         /* Relay is currently energised           */
    GATE_STATE_COOLDOWN,        /* Mandatory wait after a pulse           */
    GATE_STATE_PARTIAL_WAIT,    /* Waiting before sending auto-stop      */
} gate_state_t;

/* Movement tracking status (post-command, until position confirmed) */
typedef enum {
    GATE_MOVE_NONE,             /* Not tracking any movement              */
    GATE_MOVE_OPENING,          /* Waiting for gate to reach fully open   */
    GATE_MOVE_CLOSING,          /* Waiting for gate to reach fully closed */
} gate_movement_t;

/**
 * Initialise the gate control subsystem.
 * - Configures relay GPIO pins as outputs
 * - Configures limit switch + obstruction GPIO pins as inputs
 * - Sets all relays to OFF (HIGH)
 * - Creates the command-processing task and position monitor task
 *
 * Call this ONCE from app_main(), before starting RainMaker.
 */
esp_err_t gate_init(void);

/**
 * Send a command to the gate.
 *
 * @param cmd  The command to execute (OPEN, CLOSE, STOP, PARTIAL_OPEN)
 * @return ESP_OK if the command was accepted and queued
 *         ESP_ERR_INVALID_STATE if the state machine is busy (pulsing/cooldown)
 *
 * Thread-safe: can be called from any task (e.g. the RainMaker callback task).
 * The actual relay pulsing happens asynchronously in a dedicated FreeRTOS task.
 */
esp_err_t gate_command(gate_cmd_t cmd);

/**
 * Get the current state of the gate control state machine.
 */
gate_state_t gate_get_state(void);

/**
 * Get the current movement tracking status.
 * After a command, this returns OPENING/CLOSING until either:
 * - The limit switch confirms the gate reached its target position
 * - The 10-second timeout expires
 * - An obstruction is detected
 * - A STOP command is issued
 */
gate_movement_t gate_get_movement(void);

/**
 * Get a human-readable string for the current status.
 * Considers both state machine state AND movement tracking:
 * - During pulsing/cooldown: "Opening", "Closing", "Stopping"
 * - During movement tracking: "Opening", "Closing"  
 * - After timeout: "Timeout"
 * - When obstructed: "Obstructed"
 * - Otherwise: "Idle"
 */
const char *gate_get_status_string(void);

/**
 * Update configurable timing parameters.
 * Values are in milliseconds. Pass 0 to keep the current value.
 */
void gate_set_pulse_duration(uint32_t ms);
void gate_set_partial_delay(uint32_t ms);

/* ---------------------------------------------------------------
 * Gate Position (from limit switches)
 * --------------------------------------------------------------- */

/* Physical position of the gate */
typedef enum {
    GATE_POS_UNKNOWN,       /* No limit switch triggered (startup)        */
    GATE_POS_OPEN,          /* OP limit switch triggered (fully open)     */
    GATE_POS_CLOSED,        /* CL limit switch triggered (fully closed)   */
    GATE_POS_PARTIAL,       /* Neither limit switch triggered (in-between)*/
} gate_position_t;

/**
 * Get the current physical position of the gate.
 * Reads the limit switch GPIO inputs.
 */
gate_position_t gate_get_position(void);

/**
 * Get a human-readable string for the current position.
 * Returns "Open", "Closed", "Partial", or "Unknown".
 */
const char *gate_get_position_string(void);

/* ---------------------------------------------------------------
 * Obstruction Detection
 * --------------------------------------------------------------- */

/**
 * Check if the gate obstruction safeguard is currently active.
 * Reads the interference GPIO input.
 */
bool gate_is_obstructed(void);

/* ---------------------------------------------------------------
 * Callbacks — used by app_main.c to push updates to RainMaker
 * --------------------------------------------------------------- */

/**
 * Callback type for status/position/obstruction change notifications.
 */
typedef void (*gate_position_cb_t)(gate_position_t new_pos);
typedef void (*gate_status_cb_t)(const char *status);

/**
 * Register callbacks for events.
 * - position_cb: called when gate position changes (limit switches)
 * - status_cb: called when movement tracking status changes
 *              (target reached, timeout, obstruction)
 */
void gate_set_position_callback(gate_position_cb_t cb);
void gate_set_status_callback(gate_status_cb_t cb);

#ifdef __cplusplus
}
#endif
