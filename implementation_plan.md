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
| **Optocoupler Module** | PC817 4-Channel Isolation (PC817 C545), 3kΩ onboard input resistors, individual screw terminals per channel |

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

The device is registered as a **Switch** type in RainMaker to enable Alexa integration.

```
Node: "Gate Controller - [MAC]"
└── Device: "Sliding Gate - [MAC]" (type: esp.device.switch)
    ├── Param: "Open"          (type: esp.param.power,  UI: trigger button / power switch)
    ├── Param: "Close"         (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Stop"          (type: esp.param.toggle, UI: trigger button)
    ├── Param: "Partial Open"  (type: esp.param.toggle, UI: trigger button)
    └── Param: "Status"        (type: esp.param.text,   UI: read-only label)
```

**In the RainMaker mobile app**, the user will see:
- Three action buttons: **Open** (the primary switch/power button), **Close**, and **Stop**
- One **Partial Open** button (pre-closes if not closed, opens, waits 5s, then auto-stops)
- A read-only **Status** text showing combined status + position (e.g., "Idle", "Opening", "Closed")

> **Note on Discrepancy:** The "Pulse Duration" and "Partial Delay" parameters are not exposed to the RainMaker cloud and are hardcoded to their default values (500ms and 5000ms respectively) in the firmware (`app_priv.h`).

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

### 2.10 Security & Recovery Features

- **Static Proof of Possession (PoP):** To simplify manual pairing/provisioning without needing to read the dynamic random PIN from the serial logs, a static PoP of `gate1234` is configured in the code.
- **Rapid Power-Cycle Recovery:** The firmware monitors boot cycles using NVS storage. If the device detects **3 consecutive rapid power-cycles** (reboots/power cycles within 5 seconds of booting), it automatically triggers a Wi-Fi credentials reset and enters BLE provisioning mode. This acts as a physical hardware-based reset method.
- **ESP Insights Integration:** Remote diagnostics are configured using a static auth key to report logs, warning/error states, and system heap/Wi-Fi metrics to the ESP Insights dashboard.

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

## Phase 4 — Optocoupler Isolation for Input Signals

The gate controller outputs **5V signals** on its limit switch and obstruction terminals. Since the ESP32 GPIOs are **3.3V max**, a PC817 optocoupler module is used to electrically isolate the controller's signals from the ESP32.

> The relay outputs (OPEN/CLOSE/STOP) do NOT need isolation — relays are already galvanically isolated dry contacts.

### 4.1 Voltage Measurements (Confirmed with Multimeter)

All measurements taken between the signal terminal and COM:

| Terminal | Condition | Voltage | Meaning |
|---|---|---|---|
| **Hi** (Open limit switch) | Gate NOT at open position | **0V** | Inactive |
| **Hi** (Open limit switch) | Gate AT open position | **5V** | Active — gate is fully open |
| **CL** (Close limit switch) | Gate NOT at close position | **0V** | Inactive |
| **CL** (Close limit switch) | Gate AT close position | **5V** | Active — gate is fully closed |
| **Infr** (Obstruction/Interference) | No obstruction | **0V** | Inactive |
| **Infr** (Obstruction/Interference) | Obstruction detected | **5V** | Active — obstacle in gate path |

> **Key Finding:** The controller actively outputs 5V (not dry contacts). Signals are **active-HIGH**: 5V = condition true, 0V = condition false.

### 4.2 PC817 Module Pinout

The module has **individual 2-pin screw terminals** per channel (no shared VCC bus):

```
    INPUT SIDE                                    OUTPUT SIDE
    (Gate Controller)                             (ESP32)

    ┌─────────────┐                              ┌─────────────┐
    │  G    IN1   │──── Ch1 [R 3kΩ → LED] ──────│  G    U1    │
    ├─────────────┤                              ├─────────────┤
    │  G    IN2   │──── Ch2 [R 3kΩ → LED] ──────│  G    U2    │
    ├─────────────┤                              ├─────────────┤
    │  G    IN3   │──── Ch3 [R 3kΩ → LED] ──────│  G    U3    │
    ├─────────────┤                              ├─────────────┤
    │  G    IN4   │──── Ch4 [R 3kΩ → LED] ──────│  G    U4    │
    └─────────────┘                              └─────────────┘
```

**Input side (per channel):** `INx` = signal positive, `G` = signal ground  
**Output side (per channel):** `Ux` = open-collector output, `G` = emitter ground  
**Onboard resistors:** 3kΩ (SMD marked "302") per input channel for LED current limiting

### 4.3 Complete Wiring Diagram

```
  GATE CONTROLLER                  PC817 4-CH OPTOCOUPLER               ESP32 RELAY BOARD
  ┌──────────────┐                                                     ┌─────────────────┐
  │              │           INPUT SIDE         OUTPUT SIDE             │                 │
  │              │          ┌──────────┐       ┌──────────┐            │                 │
  │  Hi (OP) ────┼─────────→│ IN1      │       │ U1  ─────┼───────────→│ GPIO 13         │
  │              │          │ G ◄──┐   │       │ G ◄──┐   │            │ (Limit Open)    │
  │              │          ├──────┼───┤       ├──────┼───┤            │                 │
  │  CL ─────────┼─────────→│ IN2  │   │       │ U2 ──┼───┼───────────→│ GPIO 14         │
  │              │          │ G ◄──┤   │       │ G ◄──┤   │            │ (Limit Close)   │
  │              │          ├──────┼───┤       ├──────┼───┤            │                 │
  │  Infr ───────┼─────────→│ IN3  │   │       │ U3 ──┼───┼───────────→│ GPIO 27         │
  │              │          │ G ◄──┤   │       │ G ◄──┤   │            │ (Obstruction)   │
  │              │          ├──────┼───┤       ├──────┼───┤            │                 │
  │              │          │ IN4  │   │       │ U4   │   │            │                 │
  │              │          │ G    │   │       │ G    │   │            │                 │
  │              │          └──────┼───┘       └──────┼───┘            │                 │
  │              │                 │                  │                │                 │
  │  COM ────────┼─────────────────┘                  └────────────────┤ GND             │
  │              │        (all input G pins)         (all output G)    │                 │
  └──────────────┘                                                     └─────────────────┘
```

**Wire list (7 wires total):**

| Wire | From | To |
|---|---|---|
| 1 | Controller **Hi** (OP) | Opto **IN1** |
| 2 | Controller **CL** | Opto **IN2** |
| 3 | Controller **Infr** | Opto **IN3** |
| 4 | Controller **COM** | Opto input **G** (all 3 channels, daisy-chain) |
| 5 | Opto **U1** | ESP32 **GPIO 13** |
| 6 | Opto **U2** | ESP32 **GPIO 14** |
| 7 | Opto **U3** | ESP32 **GPIO 27** |
| 8 | Opto output **G** (all 3 channels) | ESP32 **GND** |

> **IN4 / U4** are unused (spare channel).

### 4.4 Signal Logic Verification

The optocoupler **inverts** the input signal (5V active-HIGH → output active-LOW). This matches the firmware's existing GPIO expectations perfectly — **no firmware changes needed**.

| Gate Condition | Controller Output | LED | Opto Output (Ux) | GPIO Reads | Firmware Interprets |
|---|---|---|---|---|---|
| Gate at **open** position | Hi = **5V** | ON | **LOW** | **0** | `GATE_POS_OPEN` ✓ |
| Gate **not** at open position | Hi = **0V** | OFF | **HIGH** (pull-up) | **1** | Not open ✓ |
| Gate at **closed** position | CL = **5V** | ON | **LOW** | **0** | `GATE_POS_CLOSED` ✓ |
| Gate **not** at closed position | CL = **0V** | OFF | **HIGH** (pull-up) | **1** | Not closed ✓ |
| **Obstruction** detected | Infr = **5V** | ON | **LOW** | **0** | `gate_is_obstructed() = true` ✓ |
| **No** obstruction | Infr = **0V** | OFF | **HIGH** (pull-up) | **1** | `gate_is_obstructed() = false` ✓ |

> The ESP32 GPIO internal pull-ups (configured in firmware as `GPIO_PULLUP_ONLY`) serve as the output pull-up resistors. No external pull-ups needed.

### 4.5 Electrical Calculations

| Parameter | Value |
|---|---|
| Input voltage | 5V (from controller) |
| PC817 LED forward voltage | ~1.2V |
| Onboard resistor | 3kΩ (SMD "302") |
| LED current | (5V − 1.2V) / 3kΩ = **1.27 mA** |
| PC817 max forward current | 50 mA |
| Safety margin | ✅ Well within limits |

> **Note:** 1.27 mA is on the lower end for PC817 operation but sufficient for reliable switching since the output side uses the ESP32's high-impedance GPIO inputs with internal pull-ups (no significant current draw on the phototransistor side).

### 4.6 Important Notes

- **Power off the gate controller** before wiring the optocoupler.
- **Shared Grounds / Level Shifting:** Since the ESP32 is powered directly from the gate controller's 12V terminal (via a buck converter), both domains share a common ground reference. Connecting the grounds on both sides of the PC817 optocoupler module is **perfectly safe and required**. While this means we do not have true galvanic isolation, the PC817 remains **absolutely necessary** as a **voltage level shifter** to protect the ESP32 GPIOs (which are 3.3V max) from the controller's 5V active signals. Without it, the 5V signals would destroy the ESP32 GPIO pins.
- **Daisy-chaining Ground (G) pins:** On the input side of the optocoupler, connect all G pins to the controller's **COM** terminal. On the output side, connect all G pins to the ESP32's **GND** (which is electrically connected to COM anyway).
- **Quick smoke test:** After wiring, manually push the gate to the open limit switch. Check the serial monitor for `"Position changed: ... → Open"` log message. If you see the opposite behavior, verify the input wiring.

---

## Resolved Questions

| # | Question | Answer |
|---|---|---|
| 1 | Partial Open delay | **5 seconds** default (configurable via app) |
| 2 | DIP switch config | Only **SOFT STOP** enabled. AUTO-CLOSE off, MIDWAY off. |
| 3 | Relay board power | **5V barrel jack** confirmed on PCB |
| 4 | ESP32 module type | **ESP32-WROOM-32** confirmed (FCC ID: 2AC7Z-ESP-32). Supports RainMaker **self-claim**. |
| 5 | Limit switch voltage | **5V active-HIGH** — Hi/CL terminals output 5V when limit switch is triggered, 0V when not. Measured with multimeter (COM as reference). |
| 6 | Obstruction voltage | **5V active-HIGH** — Infr terminal outputs 5V when obstruction detected, 0V when clear. Measured with multimeter (COM as reference). |
| 7 | Input signal isolation | **PC817 4-channel optocoupler module** required — controller outputs 5V which exceeds ESP32's 3.3V GPIO max. |

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
7. Test Remote Diagnostics: verify serial log prints "ESP Insights diagnostics initialized successfully" and check for diagnostic graphs on the ESP RainMaker Dashboard (e.g. heap metrics, Wi-Fi metrics).

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
