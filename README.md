# Mission Planner (Rust)

A Rust port in progress of [ArduPilot Mission Planner](https://github.com/ArduPilot/MissionPlanner), targeting
Windows, macOS, Linux, Android and iOS. Full feature parity and native builds on every platform are still pending.
The UI is rebuilt from Mission Planner's own layout files
(`*.Designer.cs`, `*.resx`), theme (`BurntKermit.mpsystheme`, `ThemeManager.cs`) and icon assets, and
the HUD is a line-by-line port of `HUD.cs doPaint()`.

The original C# source is in `reference/MissionPlanner` (a clone of the fork, not built).

## Layout

| Crate | Ports | Contents |
|---|---|---|
| `crates/mp-core` | `ExtLibs/Comms`, `ExtLibs/ArduPilot`, `ExtLibs/Utilities` | transports (serial/TCP/UDP/UDPCl), `MAVLinkInterface` (heartbeat, streams, params, mission up/download, commands), `CurrentState`, `Locationwp`, `.waypoints` and `.param` files, `mavcmd.xml`, parameter metadata, built-in simulator |
| `crates/mp-app` | `MainV2`, `GCSViews/*`, `ExtLibs/Controls` | egui/eframe UI: menu, DATA, PLAN, SETUP, CONFIG, SIMULATION, HELP |

## Build and run

```sh
cargo run -p mp-app --release          # desktop: Windows / macOS / Linux
cargo test --workspace                 # unit tests + end-to-end link test against the simulator
```

Developer switches: `MP_AUTOSIM=1` starts the built-in simulator and connects, `MP_DEMO=1` uploads a square
mission and flies it, `MP_VIEW=data|plan|setup|config|sim|help` picks the opening screen.

Linux needs the usual X11/Wayland dev packages plus `libudev-dev` for serial ports.

### Android

```sh
rustup target add aarch64-linux-android
cargo install cargo-apk
cargo apk build -p mp-app --lib --release
```

Requires the Android SDK and NDK (`ANDROID_HOME`, `ANDROID_NDK_ROOT`). Connect over UDP/TCP (WiFi telemetry).

### iOS

```sh
rustup target add aarch64-apple-ios
cargo build -p mp-app --lib --target aarch64-apple-ios --release
```

The library exposes `start_mission_planner()` for an Xcode wrapper. The wrapper, signing, packaging and
device validation remain to be completed. This command alone does not produce an installable iOS app.
The iOS target selects wgpu for rendering.

## Status

Working end to end (tested against the simulator): connect, heartbeat, data streams, full parameter download
with retries, parameter writes, mission upload/download (MISSION_ITEM_INT with MISSION_ITEM fallback),
modes, arm/disarm, guided "Fly To Here", takeoff, servo/relay, calibration commands, `.waypoints` and
`.param` files, map tiles with disk cache.

Additional implementations include firmware upload/signing, log download/review and telemetry playback,
Python scripts and the MAVFtp Lua REPL, geofence/rally editing, survey grids, desktop joystick input,
MAVFtp, SiK radio settings, MJPEG video, desktop speech, georeferencing, OSD editing, and device pages.
Their presence in the source is not proof of physical-device compatibility or complete Mission Planner parity.

Joystick setup includes 16 button mapping rows, settings, detection and 14 of the reference button functions.
Actions trigger on edges, relay/custom-axis mappings handle release, and takeoff waits for Guided telemetry.
Manual Control selects MAVLink MANUAL_CONTROL; stopping RC control sends release values for all 16 channels.
The gilrs adapter currently exposes 19 standardized buttons; arbitrary DirectInput axes/buttons and gimbal
point tracking still need adapters. No physical joystick or flight-controller validation has been performed.

DATA Change Alt uses the altitude-only mission flag and reports the autopilot's acceptance, rejection or
timeout through Mission Planner's legacy MISSION_ITEM path. Vehicle selection cancels prior transactions;
commands retain separate queue/ACK/retry lifecycles. Controller or button-map changes require re-enabling,
and stopping controls releases held relays and cancels queued command retries.
Independent judging baselines and open gaps are recorded in `judging/round-05.json` and `round-06.json`.

Flight Plan has polygon and zoom icons using the original icon geometry and menu actions. Rotate Map
rotates tiles and overlays together, including pointer placement, panning, wheel zoom and mission fitting.
The remaining MainV2 keyboard tools include plugin discovery, tile cache management, device operations,
and connecting without an automatic parameter download. Editing shortcuts stay with focused text fields.

`cargo test --workspace` covers UDP simulator sessions, an independent TCP peer for command/altitude
transactions, the radio test double, core algorithms and egui input events for the map controls.
TCP lets the transaction tests intentionally omit ACKs without depending on host UDP packet delivery.

SiK setup can now use TELEM1 over the connected autopilot's `SERIAL_CONTROL` interface, or the
selected radio COM port while disconnected. The shared serial transport pins the source vehicle and
component, filters replies by device and destination, chunks writes into 70-byte packets, and releases
the device when closed. Independent TCP MAVLink tests exercise radio settings load/save, device
isolation, target changes, polling, malformed replies, timeouts, disconnects and release packets.

SETUP > Advanced > Terminal supports NSH over the active MAVLink link and raw serial connections.
It includes Show Settings, Setup Radio, Tests, the legacy Logs console menu, Log Browse, command
history, CR-only input and the unterminated SiK `+++` escape. Leaving Terminal releases its transport;
IO failures appear in the console and error dialog. SSH, the legacy board reboot/startup sequence,
and the dedicated serial CLI log downloader remain incomplete. The Logs button opens the console
menu; it does not implement the original Log Download dialog.

The MAVSerial pass tool uses the same transport for GPS1 on TCP port 500. Stop and closing its window
send a release packet while the MAVLink connection is active. Link disconnection also ends the bridge.
Its TCP bridge has a separate binary roundtrip/reconnect/stop test. These checks use local peers and
transport doubles, without hardware.

Fresh rendered screens can be exported with `MP_CAPTURE=<absolute.png>` and `MP_CAPTURE_AFTER=7`.
The application captures its actual eframe viewport and exits; these images exclude OS title bars and
other applications. `MP_DATA_DIR` isolates preferences, cache and logs from the normal installation.
On Windows, `scripts/capture_round.ps1 -Round 7` captures 50 scenes sequentially on desktop 1 with an
isolated profile, waits for simulator parameters, and records image receipts, process IDs and the binary
hash under `target/judging/round07`. It uses the built-in simulator and does not connect to real hardware.

Known gaps include:

- GDAL georeferenced raster loading and opacity control. The custom tile provider is a separate implementation.
- Mission Planner's place-name geocoder. The current Zoom To dialog takes latitude and longitude.
- Loading or executing existing .NET plugins. The plugin tool lists files only.
- Complete source-by-source behavior and visual parity across all pages and tools.
- Native macOS/Linux verification and Android/iOS packaging and device runs. CI jobs are configured,
  but local Windows tests do not establish that these platforms build or run.
- Mobile file picking and Android USB serial integration. Mobile serial and some desktop-only tools
  need platform adapters; several mobile file-open paths currently return no selection.
- Real flight-controller, firmware-flashing, camera, radio and tracker qualification for the implemented tools.

Independent parity reviews are recorded under `judging/`. Round 09 rates the sampled desktop clone
6.8/10 overall and lists the next confirmed gaps. Reviews are qualitative scores, not measured feature
completion percentages, and do not establish parity for unreviewed screens or platforms.

The next implementation run adds ConfigMount's RC/SERVO output choices, RC input choices, dynamic
PWM/reverse bindings and shutter routing on the original Camera Gimbal layout. Compass rows now map
priority IDs to the matching detected device, include unprioritized devices, and rewrite the first three
priorities after reordering or removing missing entries. The initial-parameter calculator selects 3.x
or 4.x filter names using the reported firmware major version, with parameter-based fallback if absent.
These changes have six new regression tests; the Windows workspace suite passes 87 tests. Fresh
Compass and Camera Gimbal renderer receipts are under `target/judging/round10`.

## License

GPL-3.0, like Mission Planner. Icons and marker images are Mission Planner / GMap.NET assets under the same license.
System fonts (Arial, Microsoft Sans Serif) are loaded from the OS at runtime and not redistributed.
