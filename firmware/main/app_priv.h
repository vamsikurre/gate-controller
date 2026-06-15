/*
 * app_priv.h — Shared constants, GPIO definitions, and types
 *
 * This file contains all the hardware-specific definitions for the
 * ESP32 4-channel relay board (easyelectronics.in) connected to the
 * JIELONG JL-SD800 sliding gate controller.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------
 * GPIO Pin Assignments
 * ---------------------------------------------------------------
 * These match the silk-screen on the relay board PCB.
 * The relays are ACTIVE-LOW:
 *   GPIO LOW  → relay coil energised → NO contact CLOSED (short)
 *   GPIO HIGH → relay coil off       → NO contact OPEN
 * --------------------------------------------------------------- */
#define GPIO_RELAY_OPEN     19   /* Relay 1 → gate controller OPEN terminal  */
#define GPIO_RELAY_CLOSE    18   /* Relay 2 → gate controller CLOSE terminal */
#define GPIO_RELAY_STOP      5   /* Relay 3 → gate controller STOP terminal  */
#define GPIO_RELAY_SPARE    17   /* Relay 4 → reserved for future use        */

/* ---------------------------------------------------------------
 * Limit Switch Input Pins
 * ---------------------------------------------------------------
 * The gate controller has CL / COM / OP terminals for limit switches.
 * These are magnetic reed switches at the end-of-travel positions.
 * We read them as GPIO inputs to know the gate's actual position.
 *
 * Wiring: Controller OP → GPIO_LIMIT_OPEN
 *         Controller CL → GPIO_LIMIT_CLOSE
 *         Controller COM → ESP32 GND
 *
 * When limit switch is triggered (gate at end position):
 *   GPIO reads LOW  (switch closed, pulled to GND via COM)
 * When limit switch is NOT triggered:
 *   GPIO reads HIGH (internal pull-up)
 *
 * NOTE: Verify voltage at CL/COM/OP with multimeter first!
 *       If controller puts >3.3V on these terminals, use an
 *       optocoupler for isolation instead of direct connection.
 * --------------------------------------------------------------- */
#define GPIO_LIMIT_OPEN     13   /* Input: OP terminal → HIGH=moving, LOW=fully open  */
#define GPIO_LIMIT_CLOSE    14   /* Input: CL terminal → HIGH=moving, LOW=fully closed */

/* ---------------------------------------------------------------
 * Obstruction / Interference Detection
 * ---------------------------------------------------------------
 * The gate controller has a built-in obstruction safeguard that
 * reverses the motor when it detects resistance. This terminal
 * signals when that safeguard activates.
 *
 * Wiring: Controller interference terminal → GPIO_OBSTRUCTION
 *         Controller COM/GND → ESP32 GND
 *
 * When obstruction is detected: GPIO reads LOW (signal active)
 * When no obstruction:          GPIO reads HIGH (internal pull-up)
 *
 * NOTE: GPIO 11 is SPI flash (unusable). GPIO 12 is a bootstrap
 *       pin (pull-up causes crash). We use GPIO 27 (D27) instead.
 * --------------------------------------------------------------- */
#define GPIO_OBSTRUCTION    27   /* Input: interference signal from controller */

/* Active-low relay logic */
#define RELAY_ON     0
#define RELAY_OFF    1

/* ---------------------------------------------------------------
 * Timing Defaults (milliseconds)
 * ---------------------------------------------------------------
 * PULSE = how long the relay stays closed (simulating a button press)
 * COOLDOWN = mandatory dead-time between any two commands
 * PARTIAL_DELAY = how long to wait between OPEN and STOP for partial open
 * --------------------------------------------------------------- */
#define DEFAULT_PULSE_OPEN_MS      500
#define DEFAULT_PULSE_CLOSE_MS     500
#define DEFAULT_PULSE_STOP_MS      500
#define DEFAULT_COOLDOWN_MS       1000
#define DEFAULT_PARTIAL_DELAY_MS  5000
#define GATE_MOVE_TIMEOUT_MS     20000  /* 20s: max time to wait for gate to reach position */

/* ---------------------------------------------------------------
 * RainMaker Parameter Names
 * ---------------------------------------------------------------
 * These strings are the "keys" that the RainMaker cloud and mobile
 * app use to identify each control. They must match exactly between
 * firmware and cloud. */
#define PARAM_OPEN              "Open"
#define PARAM_CLOSE             "Close"
#define PARAM_STOP              "Stop"
#define PARAM_PARTIAL_OPEN      "Partial Open"
#define PARAM_STATUS            "Status"
#define PARAM_GATE_POSITION     "Gate Position"
#define PARAM_OBSTRUCTION       "Obstruction"
#define PARAM_PULSE_DURATION    "Pulse Duration"
#define PARAM_PARTIAL_DELAY     "Partial Delay"

/* Gate position polling interval */
#define POSITION_POLL_MS        500   /* Check limit switches every 500ms */

/* ---------------------------------------------------------------
 * Gate Selection Config
 * ---------------------------------------------------------------
 * To flash multiple gate controllers under the same RainMaker account,
 * configure a unique ID for each target board before building:
 *   1 = Front Gate
 *   2 = Back Gate
 * --------------------------------------------------------------- */
#define TARGET_GATE_ID    1   /* Set to 1 for Front Gate, 2 for Back Gate */

#if TARGET_GATE_ID == 1
  #define DEVICE_NAME             "Front Gate"
  #define NODE_NAME               "Front Gate Controller"
#elif TARGET_GATE_ID == 2
  #define DEVICE_NAME             "Back Gate"
  #define NODE_NAME               "Back Gate Controller"
#else
  #define DEVICE_NAME             "Sliding Gate"
  #define NODE_NAME               "Gate Controller"
#endif

#define NODE_TYPE               "Gate Opener"

/* ---------------------------------------------------------------
 * Boot button (GPIO 0 on most ESP32 dev boards)
 * Hold for 3s → Wi-Fi reset (re-provision)
 * Hold for 10s → factory reset (erase all RainMaker data)
 * --------------------------------------------------------------- */
#define GPIO_BOOT_BUTTON         0
#define WIFI_RESET_HOLD_SEC      3
#define FACTORY_RESET_HOLD_SEC  10

#ifdef __cplusplus
}
#endif
