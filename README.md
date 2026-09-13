# Smart Gate Controller

Wi-Fi control for two JIELONG JL-SD800 sliding gates (Front and Back), built by
retrofitting an ESP32 + 4-channel relay board into each gate's existing BH900
controller box and exposing it through [ESP RainMaker](https://rainmaker.espressif.com/)
(and from there, Alexa).

The ESP32 does not replace the gate controller. It presses its buttons —
each command is a 500 ms dry-contact pulse across the same terminals the wall
button and the remote use. Pull the ESP32 out and the gate works exactly as it
did before.

| | |
|---|---|
| **Wiring, pinout, bench checks** | [docs/WIRING.md](docs/WIRING.md) |
| **Node offline? Start here** | [docs/DIAGNOSTIC_AP.md](docs/DIAGNOSTIC_AP.md) |
| Firmware | [`firmware/`](firmware/) — ESP-IDF ≥ 5.1, built against 5.4.4 |
| Photos | [`images/`](images/) |
| State machine sketches | `state.drawio`, `virtual_sensor_states.drawio` |

---

## The two nodes

Both run the same binary. Identity comes from the Wi-Fi MAC, mapped in
`app_main.c`:

| MAC ends with | Device | Node | Diagnostic SSID |
|---|---|---|---|
| `3A:AC` | Front Gate | Front Gate Controller | `GateDiag-Front Gate` |
| `37:5C` | Back Gate | Back Gate Controller | `GateDiag-Back Gate` |
| anything else | `Sliding Gate - XXXX` | `Gate Controller - XXXX` | `GateDiag-Sliding Gate - XXXX` |

Add a third gate by adding a MAC case there — nothing else is per-node.

---

## Hardware

| Part | Detail |
|---|---|
| Gate motor | JIELONG JL-SD800 — 800 kg, 110 W, 220 V → DC 24 V, 1600 RPM |
| Controller board | BH900 family (DIP switches: only SOFT STOP enabled) |
| MCU | ESP32-WROOM-32 on a 4-channel relay board (easyelectronics.in), relays active-LOW |
| Isolation | PC817 4-channel optocoupler module, 5 V inputs → 3.3 V, inverting |
| Power | BH900 `12V` → adjustable buck converter (5 V pad bridged) → relay board |
| Interconnect | one 15 cm CAT5, shared ground |

Full detail and the pin table are in [docs/WIRING.md](docs/WIRING.md).

---

## What the firmware does

**Commands** — Open, Close, Stop, Partial Open. Each pulses its relay for
500 ms. A 1 s cooldown is enforced between commands, except STOP, which always
goes through immediately.

**Position** — read from the BH900 limit-switch terminals through the
optocoupler, polled every 10 ms: `Open`, `Closed`, `Partial`, `Unknown`.
Obstruction comes from the `INFR` terminal.

**Movement tracking** — after a command the node reports `Opening`/`Closing`
until the limit switch confirms arrival, a 25 s timeout expires, an obstruction
fires, or a STOP arrives.

**Partial open** — OPEN pulse, wait 4 s, STOP pulse.

**Safety watchdog** — a task polls the relay GPIOs every 100 ms. Any relay
energised for ≥ 2 s is forced off and logged as
`!!! SAFETY WATCHDOG TRIGGERED !!!`. A stuck relay would otherwise hold the
gate's input asserted indefinitely.

**Local control page** — served on the house network at
`http://front-gate.local/` / `http://back-gate.local/`, and on the node's own
always-on SoftAP when it cannot reach Wi-Fi at all: gate buttons and
full gate status, why the Wi-Fi dropped in plain words, a scan-and-pick network
chooser, gate timing, boot log, reboot. Its password is per node and lives on
the device; set one the first time you connect to each.
See [docs/DIAGNOSTIC_AP.md](docs/DIAGNOSTIC_AP.md).

**Wi-Fi credential recovery** — power-cycle the node 3× within 5 s of boot and
it backs up its current credentials to NVS and drops into BLE provisioning. If
you then power-cycle again without provisioning, it restores the backup and
rejoins the old network. Boot normally for 5 s and the counter resets. This
exists so that an accidental triple power cut doesn't strand a sealed node.

**Boot button (GPIO 0)** — hold 3 s for Wi-Fi reset, 10 s for factory reset.

**RainMaker** — BLE provisioning with a fixed PoP of `gate1234`, OTA,
schedules, scenes, timezone, plus a companion contact-sensor device so Alexa
can report gate state.

---

## Build and flash

```bash
# once per shell
. $IDF_PATH/export.sh          # Windows: . C:\esp\v5.4.4\esp-idf\export.ps1

cd firmware
idf.py build
idf.py -p COM5 flash monitor   # your port
```

Notes:

- Erasing flash (`idf.py erase-flash`) wipes the RainMaker certificate in the
  `fctry` partition. The node then has to be re-provisioned and re-claimed from
  the app. Don't do it casually — `idf.py flash` alone is enough.
- Ordinary updates are better pushed as **OTA** from the RainMaker dashboard;
  the nodes auto-fetch hourly. That avoids opening the gate boxes at all.
- The project targets ESP32 with the custom `partitions.csv` (two OTA slots +
  `fctry`). Don't switch to a stock partition table.

## First-time provisioning

1. Flash, power up.
2. RainMaker app → **Add Device** → scan the QR code from the serial log, or
   pair over BLE with PoP `gate1234`.
3. Send Wi-Fi credentials; the app performs assisted claiming.
   ESP32-WROOM-32 does **not** support self-claiming, hence
   `CONFIG_ESP_RMAKER_ASSISTED_CLAIM=y`.
4. The gate appears with Open / Close / Stop / Partial Open and a status line.

---

## Troubleshooting

| Symptom | Where to look |
|---|---|
| Node shows offline in the app | [docs/DIAGNOSTIC_AP.md](docs/DIAGNOSTIC_AP.md) — join the node's AP and read the status page |
| Gate doesn't move on a command | Diagnostic page: does the relay click? If yes, the problem is downstream on the BH900 |
| Status stuck on `Opening`/`Closing` | Limit switch / opto input. Check GPIO 13 and 14 wiring |
| Permanently `Obstructed` | `INFR` line or the IR beam alignment |
| `Timeout` in status | Gate didn't reach its limit within 25 s — mechanical, or a limit switch not wired |
| Node reboots repeatedly | Diagnostic page, **Last reset**. `BROWNOUT` = power; `PANIC` = read the log |
