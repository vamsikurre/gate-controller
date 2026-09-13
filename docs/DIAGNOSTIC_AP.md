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

**Connection**

| Field | Read it as |
|---|---|
| Wi-Fi | associated with the house AP? |
| Configured SSID | what the node is *trying* to join — check for a typo or a renamed network |
| RSSI | signal at the gate box. Below −75 dBm is the usual culprit |
| IP | `0.0.0.0` means associated but no DHCP lease |
| RainMaker MQTT | the actual "is it online in the app" answer |
| Last disconnect | Wi-Fi reason code + how long ago (see table below) |
| Disconnect count | climbing steadily = flapping link, not a one-off |

**Node** — uptime, last reset reason, free heap.

- Uptime resetting to near-zero every few minutes → the node is crash-looping.
- `Last reset: BROWNOUT` → power problem, see [WIRING.md](WIRING.md#2-power).
- `Last reset: PANIC` or a watchdog → firmware crash; the log below should show it.
- Free heap trending toward zero across visits → leak.

**Gate** — live status, position from the limit switches, obstruction state,
plus **Open / Close / Stop / Partial** buttons. These go through the same
`gate_command()` path as the app, cooldown and safety watchdog included, so the
gate stays usable while the cloud is down.

---

## What you can do

| Button | Effect |
|---|---|
| **Force Wi-Fi** form | Writes a new SSID/password, drops the link and reconnects. Persisted, so it survives a reboot. |
| **AP password** form | Sets this node's diagnostic AP password (stored in NVS). |
| **Scan APs** | Blocking scan — what the node can actually hear from inside the box. Briefly interrupts the station link. |
| **Log** | Last ~6 KB of `ESP_LOG` output, captured from the very first line of boot. |
| **Reboot** | Restarts the node. |

**Force Wi-Fi is the fix for the common case**: router replaced, SSID renamed,
password changed, or the node latched onto credentials from an old provisioning.
Type the right ones in, wait ~15 s, reload the status page. If Wi-Fi shows
connected but MQTT does not, reboot once — RainMaker's MQTT backoff can run to
several minutes and a reboot short-circuits it.

---

## Common Wi-Fi disconnect reason codes

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
