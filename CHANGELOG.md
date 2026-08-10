# Changelog

## 0.5

### Calibration values moved out of H4's globals into a checksummed file

Calibration used to live in H4's persistent-globals map, at roughly 120 bytes an
entry — an `unordered_map` node, the name stored twice because `h4proxy` keeps its
own copy of the key, and two heap buffers whenever the name passed the 15-character
small-string limit. A fully calibrated two-sensor logger has **60** of them
(2 dataTypes × 5 quantities × 2 sensors × 3 calTypes), so about **7 KB** — most of
the heap the access point needs. And the `diff` triplet is created automatically by
`fix_all_calibrations()` once both sensors have absolute pairs, so it could not be
avoided by calibrating differently.

Values now live in a fixed `float` array — 240 bytes of static RAM — and are
persisted to `/cal.dat`. The cost is **constant**: measured `Free heap at setup:
14712` with nothing calibrated and `14712` with eight values stored.

The file is **name-keyed, not positional**:

```
sen_co2_0_zero=398.000000
ref_co2_0_zero=400.000000
crc=A31F
```

A positional record would silently shift every value into the wrong slot the day
someone adds a quantity or changes the sensor count. Here an unrecognised name is
ignored and a missing one stays NaN. Only values actually set are written — absent
and `nan` mean the same thing to `read_var()`.

**Power-loss safe.** Writes are staged, never in place: `/cal.new` is written with
a CRC-16, read back and verified, and only then replaces `/cal.dat`. On load the
order is `/cal.dat` → `/cal.new` (what survives a cut between write and rename) →
migrate from H4's globals → all-NaN. A cut at any point leaves either the previous
good file or a complete new one. This is better than before: H4 rewrites the whole
`/glob` in place with no checksum.

**Partial calibration is safe by construction.** `update_var()` writes one array
slot, and commit rewrites the file from the full in-RAM array that was loaded at
boot — so every value not being calibrated is carried through untouched. There is
no read-modify-write for a power cut to catch halfway.

Verified on hardware: calibrated co2 sensor 0, reset, values returned exactly with
everything else `nan`; then calibrated o2 sensor 1 and confirmed co2 sensor 0 was
untouched; `cal_reset` then reported "Reset 8 calibration value(s)", erasing only
what existed.

`purge_placeholder_calibrations()` is gone — nothing seeds placeholders now.
Migration from the old store happens once, in `storage_begin()`.

### The dashboard is reachable: connection leak, single page, no SSE

`H4AsyncWebServer` never closed a server-side connection — the only reclamation
was H4AsyncTCP's 20-second scavenger, against an lwIP built with
`MEMP_NUM_TCP_PCB 5`. Five sockets held 20 seconds each meant a couple of page
loads exhausted the pool, after which lwIP **silently dropped SYNs** (no RST) and
the browser simply timed out. Every response now schedules a deferred close.

The delay is load-bearing. `TX()` only hands lwIP what `altcp_sndbuf()` accepts
right now (`TCP_SND_BUF` = 2 × `TCP_MSS` = 2920) and queues the rest to drain as
ACKs return. The dashboard is ~8.6 KB, so closing a tick later discarded the
remainder and the browser reported `ERR_EMPTY_RESPONSE`.

Alongside that, to cut both connections and heap:

- **One request per page load.** `scripts/build_web.py` now inlines the CSS and JS
  into a single `app.htm.gz`, and the logo and favicon links are gone. Note the
  build must minify the HTML *before* inlining: `minify_html` strips every
  newline while `minify_js` deliberately keeps them, because joining JS lines
  without a parser breaks automatic semicolon insertion.
- **The SSE stream is gone.** `/api/events` held a connection open permanently —
  a fifth of the device's TCP capacity to receive something it could poll for.
  Server-initiated messages now ride on `/api/status` as `msg`/`msgseq`.
- **The stock `/ws` websocket dashboard is no longer built** (`H4P_USE_WS_DASHBOARD`,
  default 0). This project serves its own UI and never opens `/ws`, but the handler
  was still allocated, registered, and keeping the widget list alive.
- **Basic auth applies in station mode only.** In AP mode, making a request at all
  means already knowing the WPA2 password; what it cost was a 401 round-trip on a
  separate socket per page load, and Android opens captive portals in a webview
  that handles basic-auth prompts badly.

Measured idle heap with the AP up, across this work: **7008 → 10104** free,
**4808 → 9688** largest block, against the 7000/3000 that `checkMemory()` requires.

### Captive portal

Android's connectivity probe was reaching the device (DNS is a wildcard) but
getting **401** — the library's file catch-all matches every GET and authenticates,
so the probe never even reached the 404 handler. Android read that as "no internet
access" and never offered to sign in. Probe URLs for Android, iOS, Windows and
Firefox are now registered as unauthenticated 302 redirects, and the DNS pump runs
at 100 ms draining four queries a tick instead of one every 500 ms.

### Fixes

- `H4AW_BasicAuthenticator::authenticate()` built `std::string decoded{dec}` from a
  buffer `base64_decode_block` never NUL-terminates, reading stack garbage — so
  correct credentials were rejected non-deterministically. Now constructed with the
  known length.
- The 401 was sent with no body and therefore no `Content-Length`
  (`send()` only emits it when length is non-zero), delimited solely by a
  synchronous `close()` that made lwIP emit RST instead of FIN. It now has a body,
  and `_reset()` is called on the auth-failure path — without it the handler
  singleton kept `WWW-Authenticate`, `Connection: close` and `Content-Length`
  forever, and the next response went out with both `Content-Length` and
  `Transfer-Encoding: chunked`.
- `h4t::readFileChunks()` called `malloc` and wrote to the buffer **without a NULL
  check**, so a failed allocation wrote to address 0 and rebooted the device
  mid-response. `dl_send_static()`'s own per-chunk `malloc` is gone too — `TX()`
  copies by default, so it was allocating ~1.4 KB twice per chunk.
- `H4AT_DEBUG` and `H4T_DEBUG` were `#define`d unguarded, so `-D` on the command
  line was silently overridden and `-w` hid the warning. Both are now
  `#ifndef`-guarded. They are off by default: measured at +3.3 KB of static RAM,
  which took free heap below H4AsyncTCP's own accept threshold — enabling them
  broke the thing they were meant to diagnose.
- `H4P_WiFi::_ws` was uninitialised while `_sendWS()` tested `if(_ws && ...)`.
- H4Tools had no patch file at all; edits to it would have vanished on the next
  clean build. Added `h4tools.patch` and registered it in `scripts/patch_h4.py`.

### Only real calibrations are stored, which is what made the access point usable

Earlier firmware seeded every calibration variable with `nan` at boot — about 66
of them on a two-sensor logger — and reloaded the lot from LittleFS before
`setup()` even started. That bought nothing: `Cal::read_var()` already returns
NaN for a variable that does not exist, by the same path it returns NaN for one
holding `"nan"`. What it cost was heap, and on an ESP8266 running a Wi-Fi access
point there was none to spare. Each entry is a ~22-character key (past the
small-string limit, so a heap allocation of its own) plus its value and a map
node.

Measured on the bench, and this is why the access point could not hand out DHCP
leases:

| | before | after |
|---|---|---|
| free heap at setup | 7096 | **14400** |
| free heap after init | 6440 | **13744** |
| free heap with the AP up | 1760 (largest block 1064) | **9064** (largest block 8680) |

At 1760 bytes free with a 1064-byte largest block, the SDK could not accept a
station association at all — the AP heartbeat showed `stations=0` permanently,
so the phone never even got as far as DHCP. The reported symptom was "IP
configuration failed"; the actual cause was that there was no memory to associate
with.

`init_all_calibrations()` is replaced by `purge_placeholder_calibrations()`,
which deletes any calibration entry that decodes to NaN and reports how many went.
`reset_all_calibrations()` now deletes rather than writing `nan` back, for the
same reason. Both batch into a single flash write via the new `Cal::erase_var()`,
because H4's `gvErase()` rewrites the whole persistent file once per call.

**The read path is unchanged, and is safe against a missing entry by two
independent layers:** `Cal::read_var()` checks the in-session `_pending` map,
then does an explicit `if(!h4_gvExists(name)) return float(NAN);` *before* it ever
calls `h4_gvGetString`; and `decode_cal_value("")` returns NaN as well. Note
`h4_gvExists` is a real lookup, not `h4p[name]` — the latter creates an empty
variable just by reading it, which is exactly the hazard here, and is why
`h4_gvSetSave()` has been guarded that way all along.

Verified on hardware: all 60 entries read back `nan` (never `0`, never empty,
never stale); `cal_start` → `set/zero/co2/0/400/398` → `cal_stop` → reset →
`sen_co2_0_zero: 398.00` and `ref_co2_0_zero: 400.00` came back exactly, with
every other entry still `nan`; `cal_reset` then reported "Reset 2 calibration
value(s)", erasing only the two that existed.

### Access point crash-loop fixed: WiFi scanning is now on demand only

`_startAP()` armed `h4.every(H4P_AP_SCAN_RATE, scan)` — a full-band WiFi scan
every 5 seconds, for as long as the access point was up, with nothing to stop it.
On the ESP8266 that is fatal in two independent ways:

1. `WiFi.scanNetworks()` calls `WiFi.enableSTA(true)`, so the interface never
   really stayed in `WIFI_AP`. It flipped to `WIFI_AP_STA` and walked every
   channel, leaving the AP's own channel for roughly 1.5–2.5 s out of every 5 s.
   `_startScan()` also ran *before* `WiFi.mode(WIFI_AP)`, so the first scan was
   aborted by the mode change and guaranteed an immediate re-scan.
2. On completion the core's `ESP8266WiFiScanClass::_scanDone()` heap-allocates
   the result array (sized by the number of networks found). This firmware does
   not have that much contiguous heap spare, the allocation threw, and the
   unhandled OOM panicked the device.

Measured on the bench: the device rebooted every ~12 s, five times a minute,
with `last failed alloc call: 4022A9B1(1120..1840)`. The access point therefore
existed for about 7 seconds at a time, which is why a phone could see the
hotspot but never finish auth → assoc → 4-way handshake → DHCP. The reported
symptom was "can't get an IP address"; the actual cause was that the AP kept
vanishing mid-handshake.

Scanning is now one-shot and starts only on an explicit user request
(`GET /api/wifi/scan`, behind the WiFi tab's "Search for WiFi networks" button).
The result array is released with `WiFi.scanDelete()` as soon as it is copied,
and a scan is refused outright when the largest free block is under
`H4P_SCAN_HEAP_FLOOR`, so a low-memory moment reports "not enough memory to scan"
instead of panicking. `WiFi.softAP()`'s return value is now checked too — it
returns false without configuring anything if the PSK is outside 8–64
characters, and that was previously silent.

New patch hunks: `ap_scan_on_demand`, `ap_no_background_scan`, `ap_start_check`.

### I2C: a stuck bus is detected instead of stalling the boot

Every sensor failed to initialise, including the XA1110 GPS, which sits on the
raw bus and never touches the multiplexer — so the bus itself was dead, not the
mux. The boot timings showed what: the multiplexer failed in 36 ms, the GPS in
3.0 s, each BME280 in ~6.3 s. An absent device NACKs in microseconds, so
multi-second failures meant every bit-wait was running to the full clock-stretch
limit, i.e. **SCL was being held low**. Total boot time was ~53 s.

A soft reset resets the ESP but not its I2C slaves, so a slave left mid-transfer
by an earlier reset keeps holding the bus across every subsequent reboot. Only
removing power clears it, which is exactly what fixed it on the bench.

- `dl_i2c_bus_recover()` pulses SCL up to 9 times to clock a slave off the bus
  and issues a STOP. It reports failure when SCL itself is held low, since a
  master cannot recover that.
- `dl_i2c_bus_reset()` runs that, re-inits the peripheral and restores the
  clock-stretch limit. It is called once at boot before probing anything, and
  from `recoverI2C()`, which now also re-runs `mp.init()` — a glitched
  multiplexer was never being re-initialised.
- When the bus is stuck, sensor probing is **skipped** rather than attempted.
  Boot drops from ~53 s to under 1 s and says `STUCK (SCL held low —
  power-cycle the logger)` instead of printing eight identical `Failed` lines.

`I2C_CLOCK_STRETCH_US` is now 200 ms, not 2 ms. The old 2 ms value was never
actually in force — the GPS, SEN0465 and MLX90614 drivers each call
`Wire.begin()` from their own `init()`, and `twi_init()` resets the limit to the
150 ms core default every time — and enforcing it would have broken the SCD30,
which legitimately stretches SCL for tens of milliseconds and whose own driver
sets 200 ms. A stuck bus is now handled by detecting it, not by truncating
legitimate stretching. The limit is re-applied after the last sensor `init()`
and after every recovery, so it no longer depends on which driver ran last.

### Fixes

- `GPS::update_values()` returns early unless `init()` has run. `I2CGPS::begin()`
  assigns its `_i2cPort` before probing, so a *failed* GPS init still left a
  usable object, but *skipping* init (stuck bus) left it null and the 250 ms poll
  timer dereferenced it — Exception (28) every 3 s.
- `MULTI::init()` called `mp.begin(addr)`, but `TCA9548::begin()` takes a
  **channel mask**, not an address (the address comes from the constructor).
  Passing `0x70` enabled channels 4, 5 and 6 simultaneously, bridging three mux
  segments onto the bus until the disable loop ran. Now `mp.begin(0x00)`.
- `MULTI::_currently_active_bus` was a `uint8_t` initialised to `9999`, which
  truncates to 15. Every comparison against `9999` was therefore wrong:
  `disableCurrentBus()` could never fire. Replaced with `MULTI_NO_BUS` (`0xFF`).
  The narrowing warning that would have caught this is suppressed by `-w`.
- The boot banner printed `WiFi.localIP()` even in AP mode, where it is the
  unused station address and renders as `(IP unset)` — it reads like a failure.
  It now prints `softAPIP()` when in AP mode, plus free heap and largest free
  block at setup entry and after init.
- `H4P_WiFi::_lostIP()` called `_coreStart()` — which runs `_startAP()` and with
  it `_startWebserver()` — and then `_stopWebserver()` on the very next line, so
  the access point was left with no webserver at all. `_startAP()` early-returns
  on `if(_dns53) return`, so nothing ever started it again. `_stopWebserver()` is
  for losing a *station* connection; it is now skipped when `_coreStart()` has
  just brought the AP up (`ap_keep_webserver` hunk).
- The `softAP()` failure check and the scan-refusal message added with the
  scanner rework used `H4P_PRINTF`, which expands to nothing unless `H4P_DEBUG`
  is set — so an access point that failed to start would have said nothing at
  all. Both now use `_H4P_PRINTF`, and the AP start logs
  `AP UP: SSID=… IP=…` unconditionally.

### H4 dependency stack updated

All seven H4 libraries move together, since they are released and tested as a
set:

| Library | Was | Now |
| --- | --- | --- |
| H4 | 4.0.7 | 4.0.10 |
| H4Tools | 0.0.14 | 0.0.16 |
| H4AsyncTCP | 0.0.23 | 0.0.25 |
| H4AsyncMQTT | 1.0.0-rc11 | 1.0.0-rc12 |
| ArmadilloHTTP | 0.1.8 | 0.2.0 |
| H4AsyncWebServer | 0.0.10 | 0.0.13 |
| H4Plugins | 3.5.3 | 3.5.8 |

This is the combination h4plugins 3.5.8 pins in its own `platformio.ini`, which
is the set upstream CI builds every example against — including an ESP8266
target on the same platform and build flags used here. Only the versions were
taken from that file; none of its build settings were adopted. ArmadilloHTTP is
the single deviation, at 0.2.0 rather than the 0.1.9 upstream tests.

Two of the old pins were **below h4plugins' own declared minimums**
(`H4Tools>=0.0.16`, `H4AsyncWebServer>=0.0.12`), so this is a correctness fix
rather than housekeeping. H4AsyncWebServer 0.0.11 also fails to compile on a
case-sensitive filesystem — `H4AT_HTTPHandlerWS.cpp` included
`<H4ASyncWebServer.h>` with the wrong capitalisation, fixed in 0.0.13.

**Never pin h4plugins 3.5.6.** Its `H4P_Timekeeper.cpp` calls `LOCK_TCPIP_CORE()`
unconditionally, which does not exist on the ESP8266, so it does not build for
this board at all. 3.5.7 added the missing no-op fallback and it is intact at
3.5.8.

Beyond that, these releases are almost entirely the CC BY-NC-SA to MIT
relicensing, which rewrites the header comment of nearly every file and makes
the diffs look far larger than they are. The substantive code changes across the
whole set are: `#ifndef` guards around tuning macros in H4 and h4plugins (with
`SANITY` renamed to `H4P_SANITY` — unused here), a missing `#include <algorithm>`
and a member-initialiser order fix in H4Tools, one added method in ArmadilloHTTP,
and the include-case fix above. Flash and RAM usage came out byte-identical to
the previous build, which is the expected result.

All five patches were re-derived against the new revisions. Of the 30 hardening
changes, 28 carried over unchanged and two were retired:

- `tcp_scavenge_guard` — fixed upstream in H4AsyncTCP 0.0.25.
- `ap_dashboard` — it populated the old widget dashboard's system panel, and
  that dashboard no longer exists.

One hunk needed a real merge rather than a mechanical one. 3.5.6 added a guard
to `_gotIP()` that returns early when the address is `0.0.0.0` ("prevents fake
gotIP event"). Our access-point teardown now runs **after** that guard: running
it before would tear down the captive portal on a spurious event and strand
whoever was connected to it.

Also worth knowing: 3.5.6 changed the library's own default for
`H4P_USE_WIFI_AP` from 0 to 1, which is upstream independently noticing the
problem described in the next section. The fix below is still required — a user
choosing `WIFI_ACCESS_POINT_MODE 0` would otherwise still be ignored — and it
makes the value explicit rather than inherited from a default.

`scripts/patch_h4.py` now checks the version it finds on disk against the
version each patch was derived against, and stops with a clear message when they
disagree. Previously the version strings were used only in error text, so a
moved pin surfaced as a context mismatch naming the wrong revision. The check
reads the Git tag rather than `library.properties`, because upstream repeatedly
ships releases without bumping that file — H4AsyncWebServer 0.0.11 declares
0.0.10, and ArmadilloHTTP 0.2.0 declares 0.1.9.

**After updating, delete `.pio/libdeps/d1_mini_pro` once** so the new
dependencies are fetched and the re-derived patches apply to clean trees.

### Access-point mode was never actually compiled in

`H4P_USE_WIFI_AP` and `DATALOGGER_AP_PASSWORD` were defined in
`src/dl_config_map.h`, a project header. PlatformIO compiles H4Plugins' own
`.cpp` files as separate translation units that never include a project header,
so the library was built with its default of `H4P_USE_WIFI_AP 0`: the compiled
firmware contained no `_startAP`, no `_apViewers` and no network scanner, and
the AP-password hunk in `scripts/patches/h4plugins.patch` had nothing to apply
to. Had the access point started, it would have been open.

The macro also guards a data member of `class H4P_WiFi` (the captive-portal
`DNSServer`), so the project and the library disagreed about the layout of the
same class — undefined behaviour, not just a missing feature.

- New pre-build script `scripts/project_defines.py` reads those two settings
  out of `src/main.cpp` and passes them as global `-D` flags, so the project and
  its libraries are built from the same values. `src/main.cpp` stays the only
  file to edit.
- `src/dl_config_map.h` now defers to those flags instead of defining the
  macros itself.

### New web interface

The stock H4Plugins widget dashboard has been replaced by a single-page app
served from the logger's flash. The old one was locked to a 300 px column, used
paired JPEGs as buttons, and could only show the fixed set of widget types the
framework provides — which is why nothing but calibration was ever on it.

- Four tabs: **Status** (recording state, GPS quality, battery, heap, MicroSD,
  per-bus I2C health, supervisor counters), **Calibration**, **WiFi**, and a
  **Console** that runs any serial command.
- Responsive from a 320 px phone to a desktop, follows the system light/dark
  theme with a manual override, and uses real buttons instead of images.
- Live values arrive over server-sent events at `/api/events`, falling back to
  polling `/api/status` if the stream drops.
- New read-only endpoints `/api/status`, `/api/cal` and `/api/wifi/scan`, and
  `POST /api/wifi/connect` for provisioning.
- Sources live in `web/src/`; `scripts/build_web.py` minifies and gzips them
  into `data/h4/`. It fails the build if an asset exceeds the size the device's
  TCP send buffer can absorb without queueing it on the heap.
- The legacy assets are gone (`sta.htm`, `h4.css`, `h4.js` and 18 JPEGs),
  freeing about 54 KB of the filesystem. This also retires a latent bug: the
  dashboard's "discard" button referenced `discard0.jpg`/`discard1.jpg`, which
  never existed, so it always rendered as a broken image.

**After updating, upload the filesystem as well as the firmware**
(`pio run -t upload -t uploadfs`); otherwise the logger has no web interface to
serve.

### Calibration is now entirely command-driven

The dashboard used to drive calibration through framework global variables that
only its widgets could set, so several things were possible in the browser and
not over serial. Every action is now a command, and the web interface calls
those commands over `/rest`.

- New `cal_reset` command, the old "reset" button. Clears every stored
  coefficient in a single write to flash.
- New `cal_select` command, replacing the two dashboard dropdowns. It only
  chooses which reading the dashboard displays; no calibration depends on it.
- `cal/zero` and `cal/diff` accept `all` as the quantity, which the old
  dropdown offered but the command did not. `cal/span/all` is rejected with an
  explanation rather than silently doing nothing.
- Messages go to whichever channel asked for the work: the serial port always,
  the `/rest` reply when the request came over HTTP, and the browser as a
  notification.

### Framework fixes

Three hunks added to `scripts/patches/h4plugins.patch`:

- `H4P_WiFi::_rest` cleared its reply buffer only *after* building a response,
  and the websocket console never cleared it at all, so output from one command
  was replayed in later responses.
- `H4P_WiFi::change()` guarded its reboot with `H4P_WIFI_USE_AP`, a macro
  defined nowhere in the tree (the real one is `H4P_USE_WIFI_AP`), so changing
  the Wi-Fi credentials from a command never rebooted out of access-point mode.

`scripts/patches/README.md` gains the two workflows it was missing —
regenerating a patch, and upgrading a pinned upstream version — and no longer
renders every code block as literal backslashes.

Existing checkouts need `.pio/libdeps/d1_mini_pro/H4Plugins` deleted once so the
updated patch applies to a clean tree.

### Calibration sessions

Calibrating used to write to flash on every button press, and each write
rewrote the whole stored-settings file. Calibration now happens inside a
session: switch "Calibration mode" on in the dashboard, do as many zero, span
and differential calibrations as needed, then switch it off to save them all in
a single write. A "discard" button throws the session away without writing
anything, and the calibration log on the SD card gains one row per session
instead of one per span and per differential-high calibration. Seeding a fresh
logger dropped from one write per stored value to one in total.

- New setting in `main.cpp`: `CALIBRATION_TIMEOUT_MINUTES` (default 30). A
  calibration left switched on and untouched for this long saves itself so the
  work is not lost; 0 disables that.
- New serial commands `cal_start`, `cal_stop` and `cal_discard`, the equivalent
  of the dashboard switch. The existing `cal` and `set` commands now require a
  session, so it is always clear when values reach flash.
- Air pressure is a fully supported calibration quantity: it is initialised,
  reconstructed, reset and logged like the others, and appears in the zero/span
  selector rather than only in the differential one. The variable holding the
  list was renamed from `gases` to `quantities`, since it also holds temperature
  and pressure.

### Calibration correctness fixes

- Calibration values are stored as plain decimal numbers instead of the value
  multiplied by 100000 in a 32-bit integer. The old encoding could not represent
  anything above 21474, so calibrating air pressure (about 101325 Pa) silently
  stored a meaningless number. Values written by older firmware are still read
  correctly and are converted the next time they are written.
- A stored value of -0.09999 is no longer mistaken for "not calibrated".
- Zero-calibrating water vapour now calibrates against dry air. The reference
  value was previously read as a dew point for both calibration points, so a
  zero calibration with the default reference of 0 was interpreted as a dew point
  of 0 °C, about 6 mmol/mol, rather than as dry air. The span point still takes a
  dew point in °C, since that is what a dew-point generator is set to; the zero
  point now takes a mole fraction in mmol/mol, since it is produced with dry gas.
- The "set span" button read the reference value out of the *Zero gas* box, so
  whatever was typed into *Span gas* was ignored.
- The check for a calibration that ran across midnight never triggered, because
  it compared unsigned times and so could never see a negative interval. A span
  calibration just after midnight was rejected as being more than two hours
  after the zero calibration. The differential version of the same check added a
  day to the wrong variable and then let the interval bypass both stability
  checks entirely.
- The reference value typed into the dashboard is validated before use. An empty
  box previously crashed the logger, and a box showing "nan" stored a
  meaningless reference.
- Calibrations rejected for timing reasons now say so in the dashboard and on
  the serial port instead of failing silently while the display still showed the
  measured numbers.
- The once-a-minute pass that reconstructs missing calibration values no longer
  runs while a calibration is in progress, where it could overwrite values
  between the measuring and the storing step.
- Fixed builds with `ENABLE_GPS 0` and `ENABLE_MICROSD 1`, which failed to
  compile. Without a GPS there is no wall clock, so calibration timing checks
  now measure from power-on; they only ever compare two such times.

### Datalogger stability and offline operation

- GPS is initialized directly on its I2C bus rather than through multiplexer
  channel 2. This prevents GPS transactions from being disrupted by
  multiplexer selection or reset activity.
- GPS recovery no longer resets the global I2C or multiplexer state. GPS
  faults are handled locally, reducing the chance that recovery interrupts
  otherwise healthy sensors.
- GPS processing is time-sliced through H4 instead of allowing a long
  parse/read operation to monopolize the ESP8266 loop. Each slice is bounded
  to 20 ms or 64 characters and runs every 250 ms, keeping Wi-Fi, the web UI,
  MQTT, and sensor timers responsive during NMEA bursts.
- The ESP8266 I2C clock-stretch limit is configured for the GPS path, reducing
  failures from devices that temporarily hold SCL low and bounding the impact
  of a stuck I2C device.
- Enabled fallback Wi-Fi AP mode. The normal dashboard remains available
  offline, with Wi-Fi provisioning controls and the configured AP password.
- Web UI authentication is no longer coupled to TLS-only configuration. HTTP
  basic authentication protects both station and fallback-AP dashboard access.

Together, these changes isolate the most lockup-prone device, prevent GPS
recovery from causing collateral I2C failures, and make the H4 event loop more
predictable under bad GPS or network conditions.

### H4 dependency hardening

These patches harden the pinned H4 networking stack without changing normal
application-facing APIs.

#### H4Plugins

- Prevent crashes from blank serial commands, empty REST output, empty MQTT
  report lists, and empty MQTT topic strings.
- Improve AP fallback lifecycle: avoid duplicate AP/DNS startup; retain and
  cancel the DNS timer; stop stale scans; and clean DNS state on Wi-Fi
  reconnection.
- Keep the normal dashboard available in offline AP mode and add Wi-Fi
  provisioning controls.
- Support the configured AP password.

#### H4AsyncWebServer

- Reject malformed or empty HTTP request lines safely.
- Validate `Content-Length`, prevent oversized or truncated body handling, and
  check allocations before copying request data.
- Avoid indexing malformed query or form parameters.
- As a result, hostile or broken browser/TCP input returns a safe error rather
  than corrupting memory or crashing.

#### ArmadilloHTTP

- Use bounded header detection instead of unbounded string searching on raw
  TCP buffers.
- Safely reject incomplete or malformed HTTP responses and guard response
  reassembly.
- This reduces failures caused by partial network packets or bad upstream
  responses.

#### H4AsyncTCP

- Validate split URL components before accessing them.
- Malformed URLs therefore no longer risk vector-index crashes.

#### H4AsyncMQTT

- Validate receive-frame presence and length before accessing packet bytes.
- Malformed or empty MQTT packets are rejected safely.

### H4 dependency resource and safety fixes

A second round of patches to the pinned H4 stack, targeting correctness bugs and
ESP8266 resource waste found by a code audit. All edits are internal to the
libraries; no application-facing API changes, so the datalogger's calls are
unchanged. Edits are tagged `// H4HARDEN:` (bug/safety) or `// H4PERF:` (waste).

#### H4AsyncTCP

- Fix a shadowed local `_scavenging` that defeated the scavenger guard, so every
  socket shutdown re-queued a full connection scavenge (heap-allocating task plus
  a walk of all connection sets). The guard now works and scavenges are queued
  once per cycle. (`tcp_scavenge_guard`)
- Skip the per-accept/connect/shutdown connection walk in `checkPCBs` when debug
  is off — it only fed a compiled-out debug print. (`tcp_checkpcbs_debug_only`)

#### H4AsyncWebServer

- Validate WebSocket frame length before indexing the header, extended-length,
  mask and payload bytes; short or crafted frames are rejected instead of
  over-reading the receive buffer. (`ws_frame_bounds`)
- Guard the SSE `onChange` callback so an unset handler no longer calls an empty
  `std::function` (which would abort). (`sse_cbconnect_guard`)

#### H4AsyncMQTT

- Fix a stray semicolon that made the QoS2 once-only dispatch unconditional, so a
  re-sent (DUP) QoS2 PUBLISH was delivered to the user twice. (`mqtt_qos2_dedup`)
- Clamp the wire length prefix in `mqttTraits::_decodestring` to the packet
  buffer so malformed inbound strings degrade gracefully instead of over-reading.
  (`mqtt_decodestring_bounds`)

#### H4Plugins

- Guard an out-of-bounds `parts[1]` read in the `config` command parser when a
  token has no `=`. (`cfg_parts_bounds`)

### Patch installation

- Added portable per-library Git patches in `scripts/patches/` and the
  `scripts/patch_h4.py` pre-build installer.
- The installer applies all changes reproducibly from standard Git patch
  files, detects exact already-applied patches without reapplying them, and
  refuses unknown modified dependency trees.
- Recognized caches created by the retired inline patcher are reset and
  re-downloaded before the portable H4Plugins patch is applied. Windows
  file-lock retries make that cleanup resilient to temporary Git or indexer
  locks.
