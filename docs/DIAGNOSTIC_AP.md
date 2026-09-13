# Diagnostic AP

Each node runs its own Wi-Fi access point **at all times**, alongside the normal
connection to the house Wi-Fi (`WIFI_MODE_APSTA`). It exists for exactly one
situation: a node is sealed inside the gate box, it has dropped off RainMaker,
and there is no serial port to plug into.

Implementation: [`../firmware/main/app_diag.c`](../firmware/main/app_diag.c).

---

## Getting in

| | |
|---|---|
| SSID | `GateDiag-Front Gate` / `GateDiag-Back Gate` |
| Password | per node — see below |
| URL | **http://192.168.4.1/** |

The AP password is **per node and stored on the device**, not in this repo.
A node that has never had one set falls back to the bootstrap password
`gate-setup` (`DIAG_AP_PASS_BOOTSTRAP` in
[`app_diag.h`](../firmware/main/app_diag.h)) and says so in a red banner at the
top of the page.

**Set a real one on each node the first time you connect** — bottom of the page,
8–63 characters. It takes effect immediately, which kicks you off the AP; rejoin
with the new password. **Write it inside the box lid.** If it is lost the only
way back in is a USB cable or a reflash.

Your phone will warn that the network has no internet. Stay connected anyway —
Android in particular likes to bounce back to mobile data, so if the page won't
load, turn mobile data off for a minute.

The AP is up within a few seconds of boot and stays up whether or not the node
ever reaches the house Wi-Fi. That is the point: **a node that cannot connect to
anything can still be reached here.**

---

## What the page shows

**Gate** — the buttons come first: **Open · Close · Stop · Partial open**. They
go through the same `gate_command()` path as the RainMaker app, cooldown and
safety watchdog included, so the gate stays fully usable with no internet at
all. Below them:

| Field | Read it as |
|---|---|
| Status | same string the app shows: `Opening`, `Closed`, `Obstructed`… |
| Position | what the limit switches say right now |
| Obstruction | the `INFR` line |
| Controller | state machine: idle, relay energised, cooldown, partial-wait |
| Movement | what the node is waiting for after a command |
| Contact sensor reports | what Alexa is being told |
| Open / Close limit (GPIO 14 / 13) | raw pin levels, for when the position field looks wrong |

**Gate timing** — pulse duration (how long the relay is held, i.e. how long the
button is "pressed") and partial delay (how far the gate travels before the
partial-open sequence sends STOP). Saved on the device, so a tweak survives the
next power cut. Ranges are 100–2000 ms and 500–30000 ms.

**Connection**

| Field | Read it as |
|---|---|
| Wi-Fi | associated with the house AP? |
| Configured SSID | what the node is *trying* to join — check for a typo or a renamed network |
| RSSI | signal at the gate box. Below −75 dBm is the usual culprit |
| IP | `0.0.0.0` means associated but no DHCP lease |
| RainMaker MQTT | the actual "is it online in the app" answer |
| Last disconnect | how long ago |
| **Why** | the disconnect reason in words — "WRONG PASSWORD", "NETWORK NOT FOUND", "beacon lost — signal dropped out" — with the raw code after it |
| Disconnect count | climbing steadily = flapping link, not a one-off |

**Node** — uptime, last reset reason, free heap.

- Uptime resetting to near-zero every few minutes → the node is crash-looping.
- `Last reset: BROWNOUT` → power problem, see [WIRING.md](WIRING.md#2-power).
- `Last reset: PANIC` or a watchdog → firmware crash; the log should show it.
- Free heap trending toward zero across visits → leak.

**Auto-refresh 5s** reloads the page on a timer — useful while watching the gate
travel. It is off by default because it would wipe a half-typed password; hit
**Stop auto-refresh** before filling in a form.

---

## Fixing the Wi-Fi

**Scan APs** does a live scan from inside the box and gives you a dropdown of
everything it can hear, with signal levels. Pick the network, type the password,
**Connect**. That is the fix for the common case: router replaced, SSID renamed,
password changed, or the node latched onto credentials from an old provisioning.

A hidden network won't be in the list — type its SSID into the **Force Wi-Fi**
form on the status page instead.

Either way the credentials are persisted, so they survive a reboot. Wait ~15 s
and reload the status page. If Wi-Fi shows connected but MQTT does not, reboot
once — RainMaker's MQTT backoff can run to several minutes and a reboot
short-circuits it.

The scan briefly interrupts the station link, which is harmless here.

| Everything else | Effect |
|---|---|
| **AP password** form | Sets this node's diagnostic AP password (stored on the device). |
| **Log** | Last ~6 KB of `ESP_LOG` output, captured from the very first line of boot. |
| **Reboot** | Restarts the node. |

---

## Wi-Fi disconnect reason codes

The page already spells these out in words; this is the fuller table.

| Code | Meaning | Usual fix |
|---|---|---|
| 2 | AUTH_EXPIRE | weak signal, or AP dropped the session |
| 15 | 4WAY_HANDSHAKE_TIMEOUT | **wrong password** |
| 201 | NO_AP_FOUND | wrong SSID, AP off, or out of range — hit Scan |
| 202 | AUTH_FAIL | wrong password / auth mismatch |
| 203 | ASSOC_FAIL | AP refused, often a client limit or MAC filter |
| 205 | CONNECTION_FAIL | generic; usually marginal signal |

---

## Notes

- **The AP password is the only thing standing between a stranger and the gate
  button.** The page has no second login: joining the AP is what authorises you.
  So do not leave a node on the bootstrap password, and do not reuse the
  password across nodes or with anything else.
- Open, Close, Stop, Partial, Reboot, Force Wi-Fi and AP-password changes are
  POSTs carrying a token that is regenerated every boot. A web page you happen
  to be browsing while joined to the AP therefore cannot fire off a gate command
  behind your back. Stale browser tabs kept open across a node reboot will get
  `403 bad token` — reload the page.
- Nearby AP names and log lines are HTML-escaped before they reach the page;
  an SSID is whatever a neighbour decided to broadcast.
- A SoftAP has to share the radio with the station link. Expect a little extra
  latency while someone is connected to the AP; it does not affect relay timing.
- The provisioning manager resets the Wi-Fi mode to station-only at several
  points in its lifecycle. A 30 s timer re-applies `APSTA` if that happens, so
  the AP can be missing for up to half a minute after provisioning. Wait it out.
- The AP's channel follows the station connection once the node associates.
  That is normal ESP32 behaviour, not a bug.

## If even the AP is not there

Then the node is not running: no power, or it is stuck before
`diag_start()`. In order:

1. Check the 5 V rail at the relay board's barrel jack.
2. Check 12 V at the BH900 `12V`/`COM` terminals — no mains, no node.
3. Open the box and put a USB cable on it. `idf.py -p PORT monitor`.
