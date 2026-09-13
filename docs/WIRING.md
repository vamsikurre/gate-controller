# Wiring

Everything below is *as built and verified* on both gates (Front and Back).
Photos of the real installation are in [`../images/`](../images/).

---

## 1. Block diagram

```
      230 V mains
           │
   ┌───────┴────────┐
   │  JIELONG       │  JL-SD800 sliding gate motor
   │  JL-SD800      │  800 kg · 110 W · 220 V → DC 24 V · 1600 RPM
   │  + BH900 board │  (DIP: only SOFT STOP enabled)
   └───┬────────┬───┘
       │        │
  12V/COM     dry-contact + sensor terminals
       │        │
       │        └──────────── single 15 cm CAT5 ───────────┐
       │                                                   │
  ┌────┴──────────┐                                        │
  │ buck converter│  IN 12 V  →  OUT 5.0 V                  │
  │ (5V pad       │  (adjustable module, 5 V pad bridged)   │
  │  bridged)     │                                         │
  └────┬──────────┘                                         │
       │ 5 V barrel jack                                    │
  ┌────┴───────────────────────────┐        ┌───────────────┴────────┐
  │ ESP32 4-ch relay board         │        │ PC817 4-ch opto module │
  │ (easyelectronics.in)           │◄───────┤ 5 V in → 3.3 V out     │
  │ ESP32-WROOM-32                 │  OUT   │ (inverting)            │
  │ relays are ACTIVE-LOW          │        └────────────────────────┘
  └────────────────────────────────┘
       ▲ relay NO contacts → OPEN / CLOSE / STOP on the BH900
```

Relay board and optocoupler live in their own small enclosure; only the CAT5
crosses between it and the BH900 motherboard, which keeps the gate box sealed.

---

## 2. Power

| Item | Detail |
|---|---|
| Source | BH900 `12V` and `COM` terminals |
| Converter | Adjustable buck module with the `5V` output pad soldered/bridged |
| Output | Regulated 5.0 V into the relay board's 5 V barrel jack |
| Status | Verified, run continuously for 5+ days before first install |

The ESP32 is powered *from the gate controller itself* — there is no separate
supply. If the gate loses mains power the node goes offline with it.

> A `BROWNOUT` line under **Last reset** on the [diagnostic page](DIAGNOSTIC_AP.md)
> means the buck converter is sagging under Wi-Fi TX current. Check the 5 V rail
> and the barrel-jack connection first.

---

## 3. The CAT5 run

All BH900 inputs and outputs share one ground reference, so a single shared GND
wire carries the return for every signal.

| Pair | Wire | Signal | From → To |
|---|---|---|---|
| **Orange** | Orange | 12 V power | BH900 `12V` → buck `IN+` |
| | Orange-White | **Shared GND** | BH900 `COM` → buck `IN-` / ESP32 `GND` / opto `G-out` |
| **Green** | Green | Open command | Relay 1 NO → BH900 `OPEN` |
| | Green-White | Close command | Relay 2 NO → BH900 `CLOSE` |
| **Blue** | Blue | Stop command | Relay 3 NO → BH900 `STOP` |
| | Blue-White | Open limit sensor | BH900 `Hi` → opto `IN1` |
| **Brown** | Brown | Close limit sensor | BH900 `CL` → opto `IN2` |
| | Brown-White | Obstruction sensor | BH900 `INFR` → opto `IN3` |

Relay 1/2/3 **COM** contacts are daisy-chained on the relay board and tied to
the shared GND. The optocoupler input-negative (`G`) terminals are likewise
daisy-chained to the shared GND.

BH900 terminal strip, for reference:

```
24V light-  24V light+  485 B-  485 A+  12V  COM  LOOP  OPEN  CLOSE  ONE  STOP  INFR  COM
```

Verified dry contacts: `COM↔OPEN` opens · `COM↔CLOSE` closes · `COM↔STOP` stops.

---

## 4. ESP32 pinout

Defined in [`../firmware/main/app_priv.h`](../firmware/main/app_priv.h) — that file is
the source of truth; this table is a copy for wiring at the bench.

### Outputs — relays (ACTIVE-LOW)

| Relay | GPIO | Goes to | GPIO LOW | GPIO HIGH |
|---|---|---|---|---|
| Relay 1 | **19** | BH900 `OPEN` | coil on, contact closed | contact open |
| Relay 2 | **18** | BH900 `CLOSE` | coil on, contact closed | contact open |
| Relay 3 | **5** | BH900 `STOP` | coil on, contact closed | contact open |
| Relay 4 | **17** | *spare, unused* | — | — |

A command is a **500 ms pulse**, i.e. a simulated button press, not a held contact.

### Inputs — via the PC817 optocoupler (internal pull-ups enabled)

The BH900 drives `Hi`, `CL` and `INFR` as **active-HIGH 5 V**. The PC817 both
isolates and **inverts** them, so the ESP32 sees active-LOW at 3.3 V.

| Gate condition | BH900 terminal | Opto LED | GPIO | Level | Firmware reads |
|---|---|---|---|---|---|
| Fully **open** | `CL` = 5 V | on | **14** | LOW | `GATE_POS_OPEN` |
| not fully open | `CL` = 0 V | off | 14 | HIGH | — |
| Fully **closed** | `Hi` = 5 V | on | **13** | LOW | `GATE_POS_CLOSED` |
| not fully closed | `Hi` = 0 V | off | 13 | HIGH | — |
| **Obstructed** | `INFR` = 5 V | on | **27** | LOW | `gate_is_obstructed() == true` |
| clear | `INFR` = 0 V | off | 27 | HIGH | `false` |

Neither limit switch active → `GATE_POS_PARTIAL` (gate mid-travel).

> The `CL`/`Hi` naming is inverted relative to what you would guess: `CL`
> asserts when the gate is fully **open**. Confirmed on the hardware; don't
> "fix" it.

> GPIO 11 is SPI flash and GPIO 12 is a bootstrap pin (a pull-up on it hangs
> the boot). GPIO 27 is used for obstruction instead. Pick replacement pins
> with that in mind.

### Other pins

| Pin | Use |
|---|---|
| GPIO 0 (BOOT button) | hold 3 s → Wi-Fi reset · hold 10 s → factory reset |

---

## 5. Bench check before sealing the box

1. Buck converter reads **5.0 V** with the 5 V pad bridged and no load.
2. ESP32 boots and logs on buck power (not USB).
3. Continuity on the shared GND from BH900 `COM` to ESP32 `GND` and opto `G`.
4. Each command from the app makes the matching relay click once and release
   after ~500 ms. A second command inside 1 s is rejected (cooldown).
5. Drive the gate to each end and watch the position field on the
   [diagnostic page](DIAGNOSTIC_AP.md) flip between `Closed` → `Partial` → `Open`.
6. Break the IR beam and confirm **Obstruction: ACTIVE**.
7. Safety watchdog: temporarily raise the open pulse to 2500 ms, confirm the
   relay is forced off at 2.0 s with `!!! SAFETY WATCHDOG TRIGGERED !!!` in the
   log, then put the pulse back.
8. Join `GateDiag-<name>` from a phone and load `http://192.168.4.1/` *while
   the lid is closed* — that is the signal level you will actually have when
   something goes wrong.

---

## 6. Photos

| File | Shows |
|---|---|
| `images/whole_view.JPG`, `whole_view2.JPG` | complete installation |
| `images/center_view.JPG` | BH900 motherboard and terminal strip |
| `images/bl_view.JPG`, `bl_view2.JPG`, `br_view.JPG` | enclosure corners / cable entry |
| `images/pin_view.JPG` | terminal strip close-up |
| `images/controller.png` | BH900 controller board |
| `images/relayboard.png` | ESP32 4-channel relay board |
| `images/opto isolator.png` | PC817 4-channel module |
| `images/voltage_regulator.jpg` | buck converter |
| `images/motor_label.JPG` | JL-SD800 nameplate |
| `images/ESP32.jpeg` | ESP32-WROOM-32 module (FCC ID 2AC7Z-ESP-32) |
