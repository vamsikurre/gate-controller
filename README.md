# ESP RainMaker Smart Gate Controller — Implementation Plan

Retrofit the JIELONG JL-SD800 sliding gate controller with Wi-Fi control using an ESP32 4-channel relay board and the ESP RainMaker cloud platform.

---

## 1. System Components & Hardware State

| Component | Detail |
|---|---|
| **Gate Motor** | JIELONG JL-SD800 — 800kg, 110W, 220V→DC24V, 1600RPM |
| **Controller Board** | BH900-family with DIP switches (only SOFT STOP enabled) |
| **Terminal Strip** | `24V light-` `24V light+` `485 B-` `485 A+` `12V` `COM` `LOOP` `OPEN` `CLOSE` `ONE` `STOP` `INFR` `COM` |
| **ESP32 Module** | ESP32-WROOM-32 (FCC ID: 2AC7Z-ESP-32) — supports RainMaker self-claim |
| **ESP32 Relay Board** | 4-channel, active-low relays, 5V barrel jack input (easyelectronics.in) |
| **Relay GPIOs** | Relay 1: GPIO 19 · Relay 2: GPIO 18 · Relay 3: GPIO 5 · Relay 4: GPIO 17 |
| **Verified Dry Contacts** | COM↔OPEN (opens) · COM↔CLOSE (closes) · COM↔STOP (stops) |
| **Optocoupler Module** | PC817 4-Channel Isolation (PC817 C545) — used for 5V active-HIGH signal isolation |

---

## 2. Power Solution (Implemented)
The ESP32 relay board is powered from the BH900 motherboard's **12V** and **COM** terminals. 
- **Voltage Regulator**: An adjustable buck converter with its `5V` output pad soldered/bridged.
- **Output**: Regulated 5V output wired to the ESP32 relay board's 5V barrel jack.
- **Testing Status**: Power solution has been verified and successfully run continuously for over 5 days.

---

## 3. Wiring & Physical Integration (Single CAT5 Cable)
To simplify routing and ensure a waterproof seal, a single **half-foot (15 cm) CAT5 cable** is routed between the BH900 motherboard and the enclosure housing the ESP32 board and PC817 optocoupler.

Since all inputs and outputs on the BH900 share a common ground reference, a **single shared GND wire** is used in the CAT5.

### CAT5 Wire Allocation
| Pair Color | Wire Color | Signal / Connection | Connection Details |
|---|---|---|---|
| **Orange Pair** | Orange | **12V Power** | BH900 `12V` $\rightarrow$ Buck Converter `IN+` |
| | Orange-White | **Shared GND** | BH900 `COM` $\rightarrow$ Buck `IN-` / ESP32 `GND` / Opto `G-out` |
| **Green Pair** | Green | **Open Command** | Relay 1 NO $\rightarrow$ BH900 `OPEN` |
| | Green-White | **Close Command** | Relay 2 NO $\rightarrow$ BH900 `CLOSE` |
| **Blue Pair** | Blue | **Stop Command** | Relay 3 NO $\rightarrow$ BH900 `STOP` |
| | Blue-White | **Open Limit Sensor** | BH900 `Hi` $\rightarrow$ Opto `IN1` |
| **Brown Pair** | Brown | **Close Limit Sensor** | BH900 `CL` $\rightarrow$ Opto `IN2` |
| | Brown-White | **Obstruction Sensor** | BH900 `INFR` $\rightarrow$ Opto `IN3` |

*Note: The common contacts (COM) for Relays 1, 2, and 3 are daisy-chained directly on the relay board and connected to the Shared GND. The optocoupler input negative (`G`) terminals are also daisy-chained and connected to the Shared GND.*

---

## 4. Optocoupler Isolation (PC817 Module)
The gate controller outputs **active-HIGH 5V signals** on the limit switches and obstruction terminals (`Hi`, `CL`, and `INFR`). The PC817 module isolates and level-shifts these 5V signals to safe 3.3V levels for the ESP32.

### Input Signal Logic Mapping
The optocoupler inverts the logic level (5V active-HIGH input becomes 0V active-LOW output at the ESP32 GPIOs):

| Gate Condition | Controller Terminal | Opto LED | ESP32 GPIO (with Internal Pull-up) | GPIO Level | Firmware Interpretation |
|---|---|---|---|---|---|
| **Fully Open** | `Hi` = 5V | ON | GPIO 13 | **LOW (0)** | `GATE_POS_OPEN` ✓ |
| **Moving / Closed** | `Hi` = 0V | OFF | GPIO 13 | **HIGH (1)** | Not open ✓ |
| **Fully Closed** | `CL` = 5V | ON | GPIO 14 | **LOW (0)** | `GATE_POS_CLOSED` ✓ |
| **Moving / Open** | `CL` = 0V | OFF | GPIO 14 | **HIGH (1)** | Not closed ✓ |
| **Obstructed** | `INFR` = 5V | ON | GPIO 27 | **LOW (0)** | `gate_is_obstructed() = true` ✓ |
| **Clear** | `INFR` = 0V | OFF | GPIO 27 | **HIGH (1)** | `gate_is_obstructed() = false` ✓ |

---

## 5. Firmware Features & Safety Logic

### 5.1 ESP RainMaker Integration
- **BLE Provisioning**: Easy setup via the ESP RainMaker mobile app with a static Proof of Possession (PoP) pin: `gate1234`.
- **UI Elements**: Switch controls for **Open**, **Close**, **Stop**, and **Partial Open**, along with a unified **Status** text display.

### 5.2 Wi-Fi Credentials Recovery (Accidental Reset Safe)
- **3x Rapid Power Cycle**: If the device is power cycled 3 times quickly (within 5 seconds of boot), the current Wi-Fi credentials are backed up to NVS under `"wifi_backup"`, `"prov_triggered"` is set to `1`, and the active Wi-Fi config is cleared to reboot the chip into BLE provisioning mode.
- **4th Power Cycle / Bypassing Provisioning**: On the next manual boot (if provisioning is bypassed), the firmware detects that `"prov_triggered"` is `0` but a `"wifi_backup"` exists. It automatically restores the backed up configuration, deletes the backup from NVS, and connects to the old Wi-Fi network.
- **Clean Provisioning**: Once the device successfully connects to a new Wi-Fi network and gets an IP address (`IP_EVENT_STA_GOT_IP`), the backup is permanently erased.

### 5.3 Safety Watchdog
- **Stuck Relay Watchdog**: A high-priority background FreeRTOS task polls the relay GPIO levels every 100ms. If any relay stays in the active state (`RELAY_ON` / LOW) for $\ge$ 2 seconds, the watchdog triggers a critical error log and forces the relay pin back to the `RELAY_OFF` (HIGH) state.
- **Relay Cooldown**: A mandatory 1-second delay is enforced between operations to protect the motors and relays. A manual STOP command bypasses this cooldown immediately for safety.

---

## 6. Verification Checklist

### 6.1 Power & Hardware
1. Confirm buck converter output is stable at 5.0V with the 5V pad bridged.
2. Verify the ESP32 boots on buck converter power.
3. Verify CAT5 cable connections and common ground continuity.

### 6.2 Manual Functional Verification
1. **Pulsing & Cooldown**: Send commands from the app and verify the corresponding relay clicks, holds for 500ms, and enforces a 1s cooldown.
2. **Watchdog Verification**: Temporarily extend the open pulse duration in code to 2.5s and verify the watchdog cuts off the relay at exactly 2.0s with a `!!! SAFETY WATCHDOG TRIGGERED !!!` log.
3. **Power-Cycle Recovery**:
   - Power cycle the board 3 times to enter provisioning.
   - Power cycle it a 4th time without provisioning and verify it connects back to the old Wi-Fi.
   - Re-enter provisioning mode and connect to a new network; verify the backup is cleared.
4. **Limits & Sensors**: Physically trigger the limit switches and obstruction sensor; verify the status in the mobile app updates correctly.
