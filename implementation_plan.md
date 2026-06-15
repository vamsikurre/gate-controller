# ESP RainMaker Smart Gate Controller — Implementation Plan

Retrofit the JIELONG JL-SD800 sliding gate controller with Wi-Fi control using an ESP32 4-channel relay board and ESP RainMaker cloud platform.

---

## Hardware Summary (Confirmed)

| Component | Detail |
|---|---|
| **Gate Motor** | JIELONG JL-SD800 — 800kg, 110W, 220V→DC24V, 1600RPM |
| **Controller Board** | BH900-family with DIP switches (only SOFT STOP enabled) |
| **Terminal Strip** | `B-` `B5` `A+` `12V` `COM` `LOOP` `OPEN` `CLOSE` `ONE` `STOP` `R` `M` |
| **ESP32 Module** | ESP32-WROOM-32 (FCC ID: 2AC7Z-ESP-32) — supports RainMaker self-claim |
| **ESP32 Relay Board** | 4-channel, active-low relays, 5V barrel jack input (easyelectronics.in) |
| **Relay GPIOs** | Relay 1: GPIO 19 · Relay 2: GPIO 18 · Relay 3: GPIO 5 · Relay 4: GPIO 17 |
| **Verified Dry Contacts** | COM↔OPEN (opens) · COM↔CLOSE (closes) · COM↔STOP (stops) |

---

## Phase 1 — Power Solution

### Problem

The existing buck converter only supports **≤15V input**. The gate controller's battery terminal is nominally 24V but **exceeds 24V during float/bulk charging** (typically 27–29V). This will destroy the current regulator.

### Requirements

| Spec | Value |
|---|---|
| Input voltage range | **8V – 36V DC** (wide-input to cover charge voltages + margin) |
| Output voltage | **5V DC** (regulated) |
| Output current | **≥ 2A** (ESP32 Wi-Fi peaks + 4 relay coils) |
| Efficiency | > 85% (switching, not linear) |
| Protection | Over-voltage, over-current, thermal shutdown |
| Form factor | Small PCB module, screw terminals or solder pads |

### Recommended Parts (any one)

| Part | Input Range | Output | Current | Notes |
|---|---|---|---|---|
| **LM2596-based module** (generic) | 4.5V–40V | 5V fixed | 3A | Cheap, widely available, proven |
| **DROK LM2596 module** | 3.2V–40V | 1.25V–35V adj. | 3A | Brand-name, adjustable, well-documented |
| **XL4015-based module** | 8V–36V | adjustable | 5A | Good margin, set to 5V via pot |
| ~~Mini-360 (MP2307)~~ | 4.75V–23V | adjustable | 1.8A | ⚠️ Input max too low — **NOT recommended** |

> **Recommendation:** Use an **LM2596-based 5V fixed-output module** (e.g., the common blue PCB module available on Amazon/AliExpress for ~$2). It handles 4.5V–40V input which provides ample margin above the 24V charging range. The 5V fixed version eliminates the risk of the output drifting.

> **Power input confirmed:** The relay board has a **5V barrel jack** (labeled +5V / GND on the PCB). The buck converter output can be wired directly to this barrel jack with a matching DC plug, or soldered to the +5V/GND pads.

### Wiring (Power)

```
Gate Controller                    Buck Converter               ESP32 Relay Board
┌──────────┐                     ┌──────────────┐              ┌──────────────┐
│  B+ (24V)├────────────────────►│ IN+          │              │              │
│          │                     │         OUT+ ├─────────────►│ 5V Barrel    │
│  B- (GND)├────────────────────►│ IN-          │              │   Jack       │
│          │                     │         OUT- ├─────────────►│ (GND)        │
└──────────┘                     └──────────────┘              └──────────────┘
```

> ⚠️ Before connecting the ESP32, power on the buck converter alone and **verify the output is 5.0V ± 0.1V** with a multimeter. If using an adjustable module, tune the potentiometer first.

---

## Phase 2 — Firmware Architecture

### 2.1 Project Setup

```
gate-controller/
├── firmware/
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults          # ESP-IDF + RainMaker defaults
│   └── main/
│       ├── CMakeLists.txt
│       ├── app_main.c              # Entry point, RainMaker init
│       ├── app_gate.h              # Gate control API
│       ├── app_gate.c              # Relay pulse logic & state machine
│       ├── app_priv.h              # Shared constants, GPIO defs
│       └── app_rainmaker.c         # RainMaker node/device/param setup
├── images/                         # Hardware photos (existing)
└── README.md
```

### 2.2 ESP RainMaker Node Definition

The device will be registered as a **custom device** type in RainMaker.

```
Node: "Gate Controller"
└── Device: "Sliding Gate" (type: esp.device.other)
    ├── Param: "Open"          (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Close"         (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Stop"          (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Partial Open"  (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Status"        (type: esp.param.text,   UI: read-only label)
    ├── Param: "Pulse Duration (ms)" (type: esp.param.int, UI: slider, default: 500)
    └── Param: "Partial Delay (ms)"  (type: esp.param.int, UI: slider, default: 5000)
```

**In the RainMaker mobile app**, the user will see:
- Three action buttons: **Open**, **Close**, **Stop**
- One **Partial Open** button (opens, waits configurable delay, then auto-stops)
- A read-only **Status** text showing the last command + timestamp
- Configurable pulse durations (for tuning)

### 2.3 GPIO & Relay Mapping

| Relay | GPIO | Function | Terminal Connection |
|---|---|---|---|
| Relay 1 | GPIO 19 | **OPEN** | COM ↔ OPEN |
| Relay 2 | GPIO 18 | **CLOSE** | COM ↔ CLOSE |
| Relay 3 | GPIO 5 | **STOP** | COM ↔ STOP |
| Relay 4 | GPIO 17 | *Reserved* | Future use (limit switch / lamp) |

> Relays are **active-low** (GPIO LOW = relay ON = contact closed). The firmware will pulse the relay LOW for the configured duration, then return HIGH.

### 2.4 Control Logic — State Machine

```
                         ┌──────────────────────┐
                         │        IDLE           │
                         └──┬───┬───┬───┬────────┘
                 Open cmd   │   │   │   │  Partial Open cmd
                 ┌──────────┘   │   │   └──────────────┐
                 ▼              │   │                   ▼
          ┌─────────────┐      │   │          ┌────────────────┐
          │ PULSING_OPEN │      │   │          │ PARTIAL_OPEN   │
          │  (500ms)     │      │   │          │ (open 500ms)   │
          └──────┬───────┘      │   │          └───────┬────────┘
                 │         Close│   │Stop               │
                 │           cmd│   │cmd                ▼
                 │              ▼   ▼          ┌────────────────┐
                 │     PULSING_CLOSE/STOP      │ PARTIAL_WAIT   │
                 │        (500/300ms)           │  (5000ms)      │
                 │              │               └───────┬────────┘
                 │              │                       │
                 ▼              ▼                       ▼
          ┌──────────────────────────────────────────────┐
          │              COOLDOWN (1000ms)                │
          │    All commands rejected during this period   │
          └──────────────────┬───────────────────────────┘
                             │
                             ▼
                           IDLE
```

### 2.5 Relay Pulse Logic (Pseudocode)

```c
// Pulse durations
#define PULSE_OPEN_MS      500   // Configurable via RainMaker
#define PULSE_CLOSE_MS     500
#define PULSE_STOP_MS      300
#define COOLDOWN_MS       1000   // Minimum gap between commands
#define PARTIAL_DELAY_MS  5000   // Configurable via RainMaker (default 5s)

typedef enum {
    GATE_IDLE,
    GATE_PULSING,
    GATE_COOLDOWN,
    GATE_PARTIAL_WAIT,
} gate_state_t;

// Core pulse function (runs in FreeRTOS task)
esp_err_t gate_command(gate_cmd_t cmd) {
    if (state != GATE_IDLE) return ESP_ERR_INVALID_STATE;  // Reject if busy

    state = GATE_PULSING;
    gpio_set_level(relay_gpio, 0);      // Active LOW = relay ON
    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    gpio_set_level(relay_gpio, 1);      // Relay OFF

    state = GATE_COOLDOWN;
    vTaskDelay(pdMS_TO_TICKS(COOLDOWN_MS));
    state = GATE_IDLE;

    return ESP_OK;
}
```

### 2.6 Safety Rules

| Rule | Implementation |
|---|---|
| **No simultaneous relays** | State machine only allows one command at a time |
| **Cooldown period** | 1-second mandatory gap between any two commands |
| **Command rejection** | Commands received during PULSING or COOLDOWN are silently rejected with status update |
| **Momentary pulse only** | Relay is never latched; worst case = pulse duration (500ms) |
| **Boot state** | All relays forced HIGH (OFF) at boot before any logic runs |
| **Watchdog** | ESP-IDF task watchdog enabled; reboot if firmware hangs |

### 2.7 RainMaker Features

| Feature | Implementation |
|---|---|
| **BLE Provisioning** | Standard `wifi_prov_mgr` with BLE transport; user scans QR code in RainMaker app |
| **Assisted Claiming** | Original ESP32 uses Assisted Claiming via BLE (app obtains certificate from cloud) |
| **User Sharing** | Built-in RainMaker user sharing (family members can control the gate) |
| **OTA Updates** | RainMaker OTA service enabled; push firmware from RainMaker dashboard |
| **Scheduling** | RainMaker built-in scheduling (e.g., auto-close at 10 PM) |
| **Time Zone** | Configured via RainMaker; used for status timestamps |
| **Remote Control** | Full cloud-based control via RainMaker app (iOS/Android) |

### 2.8 Key `sdkconfig.defaults`

```ini
# RainMaker
CONFIG_ESP_RMAKER_ASSISTED_CLAIM=y
CONFIG_ESP_RMAKER_USE_NVS=y

# BLE Provisioning
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_WIFI_PROV_BLE_BONDING=n
CONFIG_WIFI_PROV_TRANSPORT_BLE=y

# OTA
CONFIG_ESP_RMAKER_OTA_AUTOFETCH=y
CONFIG_ESP_RMAKER_OTA_AUTOFETCH_PERIOD=3600

# Watchdog
CONFIG_ESP_TASK_WDT=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=30
```

### 2.9 Useful ESP RainMaker Resources

- **ESP RainMaker Getting Started:** https://rainmaker.espressif.com/docs/get-started/
- **ESP RainMaker GitHub (examples):** https://github.com/espressif/esp-rainmaker
- **RainMaker Switch Example:** `esp-rainmaker/examples/switch/` — closest reference for our relay control
- **ESP-IDF Programming Guide:** https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
- **RainMaker Mobile App:** Search "ESP RainMaker" on App Store / Google Play
- **Self-Claiming Docs:** https://rainmaker.espressif.com/docs/self-claiming/

---

## Phase 3 — Wiring & Physical Integration

### 3.1 Relay-to-Controller Wiring

```
ESP32 Relay Board                     Gate Controller Terminal Strip
┌──────────────────┐                  ┌─────────────────────────┐
│                  │                  │                         │
│  Relay 1 (COM) ──├──────────────────┤── COM                   │
│  Relay 1 (NO)  ──├──────────────────┤── OPEN                  │
│                  │                  │                         │
│  Relay 2 (COM) ──├──────────────────┤── COM                   │
│  Relay 2 (NO)  ──├──────────────────┤── CLOSE                 │
│                  │                  │                         │
│  Relay 3 (COM) ──├──────────────────┤── COM                   │
│  Relay 3 (NO)  ──├──────────────────┤── STOP                  │
│                  │                  │                         │
│  Relay 4         │  (Reserved)      │                         │
└──────────────────┘                  └─────────────────────────┘
```

> 💡 All three relay COM terminals can be **daisy-chained** to a single wire going to the controller's COM terminal, reducing the wire count from 6 to 4.

### 3.2 Complete Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    GATE CONTROLLER (JL-SD800)                   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────┐        │
│  │ B+   B-   A+   12V  COM  LOOP OPEN CLOSE ONE STOP  │        │
│  └──┬────┬─────────────┬──────────┬────┬─────────┬─────┘        │
│     │    │             │          │    │         │               │
│     │    │             │          │    │         │               │
│  ┌──┴────┴──┐    ┌─────┴──────────┴────┴─────────┴───────┐      │
│  │ BUCK     │    │    ESP32 4-CH RELAY BOARD              │      │
│  │ CONVERTER│    │                                       │      │
│  │          │    │  5V Barrel Jack ◄── from buck OUT+     │      │
│  │ IN+◄─B+  │    │  GND             ◄── from buck OUT-   │      │
│  │ IN-◄─B-  │    │                                       │      │
│  │ OUT+──►──┼────┤  R1(NO)──►OPEN   R1(COM)──►COM        │      │
│  │ OUT-──►──┼────┤  R2(NO)──►CLOSE  R2(COM)──►COM(chain) │      │
│  └──────────┘    │  R3(NO)──►STOP   R3(COM)──►COM(chain) │      │
│                  │  R4 ── reserved                        │      │
│                  └───────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### 3.3 Enclosure

- Mount ESP32 relay board inside the existing gate controller housing (there is space visible in the photos)
- Route wires neatly along existing cable paths
- Ensure the ESP32's antenna area is not blocked by metal
- Consider a small weatherproof junction box if external mounting is needed

---

## Resolved Questions

| # | Question | Answer |
|---|---|---|
| 1 | Partial Open delay | **5 seconds** default (configurable via app) |
| 2 | DIP switch config | Only **SOFT STOP** enabled. AUTO-CLOSE off, MIDWAY off. |
| 3 | Relay board power | **5V barrel jack** confirmed on PCB |
| 4 | ESP32 module type | **ESP32-WROOM-32** confirmed (FCC ID: 2AC7Z-ESP-32). Supports RainMaker **self-claim**. |

> **DIP Switch Implication:** Since AUTO-CLOSE is disabled on the controller, auto-close behavior can be implemented in firmware if desired in the future (e.g., via RainMaker scheduling). SOFT STOP being enabled means the gate decelerates before stopping, which is good for safety and mechanical longevity.

---

## Verification Plan

### Phase 1 — Power
1. Measure voltage at B+ / B- terminals with multimeter (AC mains on, battery charging)
2. Connect buck converter, verify 5V output under no-load and under load (ESP32 connected)
3. Confirm ESP32 boots and connects to Wi-Fi on buck converter power

### Phase 2 — Firmware
1. Flash firmware via USB, verify serial log shows RainMaker initialization
2. Provision via BLE using RainMaker app
3. Test each command button in the app — verify relay clicks and gate responds
4. Test safety: rapid-fire commands → verify cooldown rejection
5. Test partial open → verify open-wait-stop sequence
6. Test OTA: push a minor version update from RainMaker dashboard

### Phase 3 — Integration
1. Wire relays to controller with gate powered off
2. Power on and test full cycle: Open → Stop → Close → Stop
3. Test partial open with real gate
4. Test from outside Wi-Fi range (cloud/cellular) to verify remote control
5. Share device with a second user account and verify control

### Manual Verification
- User physically verifies gate movement for each command
- User tests the RainMaker app on their phone
- User verifies operation continues after power cycle (battery backup)
