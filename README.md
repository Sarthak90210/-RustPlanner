# Mission Planner (Rust)

A Rust port of [ArduPilot Mission Planner](https://github.com/ArduPilot/MissionPlanner), one codebase for
Windows, macOS, Linux, Android and iOS. The UI is rebuilt from Mission Planner's own layout files
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

Builds `libmp_app` (static/cdylib). An Xcode wrapper app calls `start_mission_planner()`. Rendering uses wgpu/Metal.

## Status

Working end to end (tested against the simulator): connect, heartbeat, data streams, full parameter download
with retries, parameter writes, mission upload/download (MISSION_ITEM_INT with MISSION_ITEM fallback),
modes, arm/disarm, guided "Fly To Here", takeoff, servo/relay, calibration commands, `.waypoints` and
`.param` files, map tiles with disk cache.

Screens follow Mission Planner's layout. Not yet ported (buttons are present and report so): firmware install,
log download/review, telemetry log playback, scripts, geofence/rally editing, survey grid, joystick, MAVFtp,
SiK radio config, video/HUD overlay, speech, plugins. Serial ports are desktop only; Android USB serial
needs a JNI bridge.

## License

GPL-3.0, like Mission Planner. Icons and marker images are Mission Planner / GMap.NET assets under the same license.
System fonts (Arial, Microsoft Sans Serif) are loaded from the OS at runtime and not redistributed.
