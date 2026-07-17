# Changelog

## Unreleased

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
