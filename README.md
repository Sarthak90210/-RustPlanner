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
rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
python3 scripts/build_ios.py --target simulator --release
python3 scripts/smoke_ios_simulator.py
# Build for an iPhone/iPad instead:
python3 scripts/build_ios.py --target device --release
```

Requires macOS, Python 3.11+, Xcode and an installed iOS Simulator runtime. The builder packages the
Rust executable directly, with an iPhone/iPad manifest and a launch screen. It selects the simulator
architecture from the Mac's CPU and uses wgpu for rendering. The minimum deployment target is iOS 15.
The `.app`, ZIP and build receipt are under `target/ios/simulator` or `target/ios/device-unsigned`.
The device bundle is unsigned and requires Apple provisioning and signing before installation.
This is a development packaging path; App Store distribution is not configured.

The simulator script creates its own simulator, installs and launches the app, requires an image and
receipt from the eframe renderer, and saves evidence under `target/ios/smoke` before deleting that
simulator. CI runs this check and uploads its evidence. A passing simulator smoke establishes startup
and rendering only. Physical-device operation, touch workflows and feature parity require separate tests.
App settings and caches use persistent sandbox storage; iOS documents use the app's Documents directory,
which can be reached through Finder file sharing. Native document-provider selection is still pending.

## Status

Independent adversarial reviews are recorded in `judging/round-30.json` through
`judging/round-36.json`. These are source and targeted-test reviews, not full visual,
hardware or platform certification. Their ratings are provisional judgments, not
measured completion percentages. The exact-clone goal remains open.

The native DataFlash checks now distinguish unavailable evidence from healthy data,
avoid a fixed battery cell-count assumption, and distinguish ERR recovery records.
They are a partial analyzer, not a port of the reference's external LogAnalyzer runner.
WebSocket transport uses tungstenite with rustls for WSS handshakes and
incremental framing. BLE and WinUSB return Unsupported until real backends exist.
Reference Socket.IO negotiation and reconnect behavior remain open.

Proximity telemetry retains separate sensors and obstacle-array sectors with expiry,
vehicle filtering and selected-vehicle yaw. The radar uses reference range/size
controls, range rings, spokes and the original quad icon. Headless drawing and TCP
fixtures cover these paths; rendered parity and real sensor validation remain open.

`scripts/capture_proximity.ps1 -Round 34` builds an isolated native radar capture
example and renders three populated content fixtures through the production painter.
The C# comparison compiles the unchanged `Temp_Paint` method extracted from the
reference, with data/type adapters and matching content dimensions. Captures and
receipts go under `target/judging/round34/`, including the executable and source hashes.
This harness deliberately excludes form chrome: its reference Width/Height equal the
fixture size, whereas the full WinForms form uses outer dimensions for scale and
client dimensions for centering. It cannot establish full-form equivalence. Font
weight and stroke rasterization remain visible GDI/egui differences. Keyboard tests
now send actual egui pointer/text events, cover focus transfer to another text input,
and verify controls work after closing/reopening without another content click.

Proximity retention now follows the reference's id/orientation and id/raw-angle
replacement rules, preserves arrival order and short-lived angular history, and
separates each system/component subscription. Round 35 found that the outer C#
subscription dispatcher filters components despite the cache's contrary comment;
the implementation and multi-component TCP regression were corrected after that
review. Scalar CUSTOM orientation now uses the reference's yellow 45-degree sector
at zero degrees. Nine cache tests cover moving scan origins, changed increments,
unknown bins, expiry, source-key collisions and repeated yaw. Nonhorizontal sensor
retention and the reference closest/warning queries remain open.

Validation found an intermittent simulator startup timeout in the round-35 workspace
run, before the DroneCAN test received Connected. The isolated case, two subsequent
parallel link suites, and the final round-36 workspace run passed. Failure diagnostics
now include link status, received bytes and the last event. No runtime race fix or
root cause is claimed; this remains an open reliability issue despite the rerun.

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
- Native GUI verification outside Windows and Android/iOS device runs. Run 22 CI passed desktop
  tests/release builds on Windows/macOS/Linux, Android debug APK packaging and the iOS release library.
  iOS app packaging and simulator startup now have CI jobs; until those jobs pass, they remain unverified.
  Physical mobile runtime behavior remains unverified. See `judging/platform-build-22.json` for the older run.
- Mobile file picking and Android USB serial integration. Mobile serial and some desktop-only tools
  need platform adapters; several mobile file-open paths currently return no selection.
- Real flight-controller, firmware-flashing, camera, radio and tracker qualification for the implemented tools.

Independent parity reviews are recorded under `judging/`. Round 09 rates the sampled desktop clone
6.8/10 overall and lists the next confirmed gaps. Reviews are qualitative scores, not measured feature
completion percentages, and do not establish parity for unreviewed screens or platforms.

Run 10 adds ConfigMount's RC/SERVO output choices, RC input choices, dynamic
PWM/reverse bindings and shutter routing on the original Camera Gimbal layout. Compass rows now map
priority IDs to the matching detected device, include unprioritized devices, and rewrite the first three
priorities after reordering or removing missing entries. The initial-parameter calculator selects 3.x
or 4.x filter names using the reported firmware major version, with parameter-based fallback if absent.
These changes have six new regression tests; run 10 passed 87 Windows workspace tests. Its independent
review is `judging/round-10.json`, scoring 6.9/10 and recording the remaining gaps. Fresh Compass and
Camera Gimbal renderer receipts are under `target/judging/round10`.

Run 11 preserves the complete compass row list during page activity, adds the reboot prompt and
calibration gate, and performs the gimbal page's activation writes. Writes to the same parameter now
queue behind its pending readback, preserving clear/reassign operations. Verification includes a real
egui selector click through the production MAVLink link to simulator readback, an independent TCP peer
checking parameter ACK/rejection and target-switch cancellation, and five-device compass lifecycle tests.

Run 12 adds the original Plane and Rover Basic Tuning pages and the Tracker Extended Tuning page,
with 234 exported controls and 100 parameter bindings. Plane controls stage edits until Write
Params or Ctrl+S, retain pending values until readback, confirm doubled values, and provide full/partial
refresh. Rover and Tracker controls write directly. Tracker includes yaw/pitch PWM tests and the vertical pitch slider.
The layout exporter is `scripts/export_tuning_forms.py`; generated JSON is included in normal builds.
`MP_SIM_KIND=plane|rover|tracker` selects corresponding built-in simulator fixtures. Their metadata
parameter sets are development fixtures, not physical firmware qualification.

Parameter transactions now preserve ACK order across different names too. Compass reboot uses a
modal confirmation and waits for its result before clearing required state or starting calibration.
Run 12's validation includes 98 passing tests, numeric-edit/Ctrl+S simulator readback and separate TCP
ordering tests. `scripts/capture_vehicle_tuning.ps1 -Round 12` exports each actual firmware-specific
CONFIG page and records the reported firmware, complete parameters and binary hash in its receipts.

Run 13 corrects Rover's direct-edit workflow and its Avoidance visibility predicate, and adds metadata
tooltips to vehicle tuning combo boxes. Separate egui/simulator tests exercise Plane staged edits and
Rover direct edits through actual numeric clicks and parameter readback.

The web service from `Utilities/httpserver.cs` starts with the application on port 56781 on every
interface, with the original 200-connection and 20-per-address limits. It serves:

- Google Earth links: `network.kml` (opened by PLAN's View KML), `location.kml`, `wps.kml`, the aircraft
  model and `georefnetwork.kml` with its 640x480 photos from the georef tool.
- `/websocket/server` (CurrentState and waypoints as JSON at 5 Hz) and `/websocket/raw` (MAVLink frames
  in both directions), plus the original `hud.html` that uses them.
- `hud.jpg`, `map.jpg` and `both.jpg` MJPEG streams of the live DATA HUD and map, taken from renderer
  frames only while a client is connected.
- `/guided` and `POST /guide`, restricted to loopback and same-site requests as in the original.
- Mavelous `/mavlink/` JSON and `/mav/` files with path-traversal protection, the index page, and the
  original 404 stubs for `command_long`, `rcoverride` and `get_mission`.

Socket tests cover every route, the RFC 6455 handshake vector, masked client frames and CSRF rejection.
A run against the simulator returned real HUD/map JPEG frames and live packet JSON. Google Earth and a
Mavelous checkout have not been exercised.

On Windows a socket read timeout can surface as ERROR_IO_PENDING (997). All transports now treat it as
a timeout instead of dropping the link. This was the cause of the intermittent SiK test failure.

Run 14 adds PLAN's coordinate systems. The Coords selector switches the mouse readout between GEO,
UTM and MGRS (with SRTM altitude and source) and persists as `fpcoordmouse`. The waypoint grid swaps
Lat/Lon for editable Zone/Easting/Northing or MGRS columns. Edits convert back to latitude/longitude,
and unparsable entries are ignored, as in the original. `mp_core::coords` ports GeoUtility's
transforms, including its zone exceptions, rounded MGRS digits and trailing-zero trimming. Its UTM
values are checked against the independent PyPI `utm` package. An egui test switches systems and
edits UTM and MGRS cells. `MP_COORDS=UTM|MGRS` with capture cases 51/52 renders each mode.

Direct-write numeric form controls now follow MavlinkNumericUpDown. The first change starts a 300 ms timer,
the control shows its own value while the timer runs, and the value at that moment is written. Failed
direct writes show "Set X Failed" for numerics and check boxes and "Set X Failed!" for combos, as the
original controls do. The Rover simulator test confirms that nothing is written before the timer fires.

`DisplayView` is modeled in full: all of the original's flags, the Basic/Advanced presets, and a Custom
view read from `custom.displayview` (JSON or XML). The selected view is saved as JSON in "displayview",
and startup forces the parameter pages as MainV2 does. The flags control the SIMULATION and HELP
buttons, every DATA tab, the PLAN menus and Verify Height box, each SETUP and CONFIG page, and the
Planner Layout selector. Original page predicates apply: Frame Type (FRAME_CLASS or Copter), Initial
Tune (Copter or QuadPlane), CONFIG Loading while parameters arrive, and Full Parameter List only when
disconnected or complete. Pure page-list tests cover layouts, connection states and vehicles.

Guided targets (Fly To Here, Fly To Coords, `/guided`, scripts) follow `setGuidedModeWP`. Zero targets
are ignored, the vehicle is switched to GUIDED first, Plane gets MISSION_ITEM current=2, and other
vehicles get SET_POSITION_TARGET_GLOBAL_INT. The simulator now accepts guided targets only in GUIDED,
as ArduPilot does. A Copter test flies from Loiter to a target this way.

temp > Swarm now opens a port of FormationControl. It has the original button row, the vehicle selector
over every connection, Set Leader offset rebasing, Update Pos (offsets measured in the leader's UTM
frame) and Start/Stop. While running it sends Formation.SendCommand at 10 Hz: position+velocity targets
with CONDITION_YAW for multicopters, and the PID SET_ATTITUDE_TARGET path for ArduPlane. The offsets
Grid supports dragging, panning, wheel zoom, Change Alt and the Vertical view, and the Status panel
shows each vehicle. A three-Copter simulator test flies the followers into their slots.
`MP_TOOL=<button>` opens a temp tool once the startup simulators connect, and capture case 53 renders
the window. temp > Sequence Swarm opens the LayoutEditor. Sequences load and save in the original JSON
(Layouts with Id/DelayStart/DelayEnd/Offset, and Steps). The editor has the layout selector, New Layout
(copies the current offsets), the drone count rule, Add Step, a steps list (select, Delete, drag to
reorder), Takeoff (GUIDED, arm, 2 m), Run Step/Reset and a background image with its Move Image buttons.
A simulator test flies three Copters through a step. temp > Follow the leader opens FollowPathControl. It records the active vehicle's trail at 5 Hz and plans
2 m steps from the trail start. Every follower gets setGuidedModeWP to the first planned point, as in the
original. Deliberate deviation: targets use the relative-altitude frame, because the original's frame-0
Locationwp would send its relative altitude as AMSL. temp > Follow Swarm opens WPControl (WaypointLeader),
with ground/air master selection, collision checking, multi-mode drone group flight states, the original
ZedGraph distance-altitude elevation plot, and per-vehicle status cards.

Run 18 adds the remaining DisplayView consumers for Quick-view locking, checklist editing, simple
flight modes, parameter commit controls and forced menu hiding. The revealed menu stays open while
the pointer moves over its buttons. CONFIG selects its vehicle-specific starting page.

Guided commands carry Absolute, Relative or Terrain frames through both Plane's mission item and
Copter's position target. DATA prompts for altitude on the first Fly To Here, saves the altitude and
frame, rejects invalid integer altitude, converts display units to metres, and changes an existing
guided target's altitude only while in Guided. The HTTP guided endpoint uses the original frame 0.
Wire tests cover all three frames on both vehicle paths.

Status uses the original 190-pixel column steps and horizontal scrolling. Its registry is generated
from the 506 public CurrentState properties, with natural channel/cell ordering and original types.
The JSON stream preserves booleans and text. Additional telemetry includes secondary GPS/IMUs,
batteries, terrain, power, optical flow and PID tuning. Unsupported properties are blank/null;
the full CurrentState decoder and numeric selection lists still need further porting.

Run 19 removes DATA's global guided altitude/frame/marker cache. DATA reads the selected vehicle's
shared target, including targets sent by HTTP, scripts and raw position commands. Altitude edits
update that vehicle while keeping its coordinates. Switching MAVs or connections uses each one's
state. Production-link tests cover both selection and commands addressed to a nonselected vehicle.
Status reserves room for its solid scrollbar; an egui test drags it to the last column at two panel
sizes and confirms the bar remains inside the visible panel.

Run 20 completes the observed guided-cache message variants: legacy MISSION_ITEM and
MISSION_ITEM_INT require current=2 and mission_type=MISSION; position-target packets are observed
with the original mask-independent behavior. Peer tests verify that fence/rally items leave the
guided target intact. The final-column capture now uses a finite content offset.

CurrentState decodes hardware voltage/I2C errors, MCU values, free memory, RPM, AOA/SSA, fence state,
generator and EFI values, and voltage/current/RPM/temperature for all sixteen ESCs. Decoder tests
cover scales, ECU/MCU selection, unavailable fuel pressure, memory fallback, and all ESC channels.
This extends the decoder; unsupported properties and the wider parity gaps remain open.

Run 21 repairs the platform build paths found by CI: Android setup requests `platform-tools` rather
than the removed `tools` package, and iOS uses WGPU without pulling in the desktop OpenGL backend.
Sequence Swarm uses a platform file chooser. Desktop keeps OS dialogs; mobile can load/save sequences
and choose background images in the app's files directory. This browser does not implement Android
SAF or iOS document-provider access. CI caches Rust dependencies and cancels superseded branch builds.

Run 22 normalizes extensionless names in the mobile sequence Save browser to `.txt` and rejects
unsupported extensions, so saved files appear in Load. Cancelling background selection preserves
the existing image. Tests cover saving and loading the real sequence file and the cancel interaction.
Android CI produces a debug-signed APK for installation tests; distribution release signing is not
configured. The preceding iOS CI job built the aarch64 release library, which is not an iOS app bundle.

UI languages follow ConfigPlanner's CMB_language. It offers the original 17 cultures by display name,
saves the choice as Settings "language" and asks for a restart, as MainV2.changelanguage does.
`scripts/export_translations.py` exports each culture from Mission Planner's .resx files into
`crates/mp-app/assets/i18n/`:

- per-form control texts, applied to the Designer-exported forms the way
  ComponentResourceManager.ApplyResources does;
- an English-to-culture map for the hand-built views (menu bar, tabs, backstage, buttons, labels,
  checkboxes, menus, window titles and grid headers).

Lookups are per view, as ComponentResourceManager is per type. The calling source file chooses the
original view's .resx through `#[track_caller]`, and a shared map holds only strings every .resx
translates the same way. Labels, which also carry data, use only their view's resources. Mnemonic "&"
is stripped and numeric keys are dropped. Without a saved language the OS display language is used,
with .NET resource fallback. Toasts translate exact Strings.resx messages. PLAN's distance overlay
measures Prev/Home to the pointer as SetMouseDisplay does, with FormatDistance units.
Arabic follows Uniscribe in a left-to-right WinForms label: a simplified Unicode bidi algorithm (W2,
W4, N1/N2, L2) orders the runs. Latin and Arabic come from different font faces, so egui shapes each run
in its own direction. Grid headers that the original sets per control (e.g. PLAN's blank Up/Down) use
control-keyed resources.
Strings the original never translated stay English. A system CJK or Arabic font is added as a fallback,
standing in for GDI font linking. `MP_LANGUAGE=<culture>` overrides the setting for captures 58-60.
Quick view captions follow CurrentState.GetNameandUnit: [DisplayText] (exported with the property table)
passed through the original L10NU tables for zh-Hans and uk, with units filled in. A tile keeps the
designer's desc until Settings "quickView<n>" is chosen, so the sixth tile reads "DistToMAV" as it does
originally. The field chooser lists GetFieldDesc descriptions, sorted by description.

DATA video: Set GStreamer Source and HereLink Video follow FlightData.cs. They prompt for the pipeline or
HereLink IP (saved as "gstreamer_url" / "herelinkip"), look for a GStreamer runtime in the original
LookForGstreamer directories, and download the original 1.14.4 runtime zip into the data directory if
none is found. The pipeline then runs with its `outsink` appsink behind the HUD; GStreamer Stop stops
it. GStreamer is loaded dynamically, so the build does not depend on it. An ignored test
(`MP_GST_DATA=<dir> cargo test -p mp-app -- --ignored real_runtime`) downloads the runtime and checks
real 1280x720 videotestsrc frames and parse-error reporting. Capture 61 (`MP_GSTREAMER=1`,
`MP_GST_DIR=<runtime dir>`) renders the HUD over GStreamer video.

Plugins follow PluginLoader's lifecycle as native libraries, since .NET assemblies cannot load into this
program. A shared library in the `plugins` folder beside the program exports `mp_plugin_entry`; the ABI
is in the `mp-plugin` crate. The lifecycle runs Init, then Loaded once every plugin is initialised, Loop
on the plugin thread at the plugin's loopratehz, and Exit when the program closes. The host services
mirror PluginHost: CurrentState fields, config get/set, doCommand, AddWPtoList, and items in the DATA map,
HUD and PLAN map context menus with their click position. PluginManager (Ctrl+P) lists loaded plugins
and disabled files with the original colours. Save & Close stores "DisabledPlugins" and shows the
restart warning. Show Errors explains files that could not load, including .NET `.dll` and `.cs`
plugins. `crates/mp-plugin-example` is a working plugin, and a test runs its whole lifecycle.

Gimbal Video (DATA map menu, and the Payload tab's button) follows GimbalVideoControl and
FlightData.cs:
- Placement: Full Sized puts the video in the map panel with a 30% mini map. Mini puts a 30% video, sized
  to the picture's aspect, left of the zoom bar. Pop Out opens the 600x400 "Gimbal Control" window.
- Connection: the first time it opens it checks for GStreamer, offering the download. A second later it
  connects to the camera's VIDEO_STREAM_INFORMATION stream (the saved "gimbal_video_stream" URI, else the
  first) through CameraProtocol.GStreamerPipeline. Video Stream opens VideoStreamSelector.
- Commands: `mp-core::gimbal` ports GimbalManagerProtocol and CameraProtocol. Gimbal commands go
  through DO_GIMBAL_MANAGER_PITCHYAW and need GIMBAL_MANAGER_INFORMATION capabilities. Point Home and the
  POI click send DO_SET_ROI_LOCATION as COMMAND_INT.
- Camera: picture and recording are enabled from CAMERA_INFORMATION, and the camera reports tracking.
- Keys: held keys slew and zoom (WASD/E/Q, Shift fast, Ctrl slow). Pressed keys run the actions
  (Alt+F, Alt+R, L, N, H).
- Mouse: a click aims the camera; Ctrl+click sets the ROI; Alt+click or Alt+drag starts tracking. Moving
  the pointer puts a blue marker on the map where it meets the ground.
- Shortcuts: opens GimbalControlSettingsForm. It stores "GimbalControlPreferences" in Json.NET's format,
  with WinForms key codes, and asks before applying a clashing binding.
- Custom Settings opens a camera-definition editor with option lists, step sliders and
  text/Apply controls. Definitions load from HTTP(S) or a pinned MAVFTP camera target,
  including xz compression, localization, exclusions and dependent ranges. PARAM_EXT
  reads/writes use camera-source filtering, retries, IN_PROGRESS handling and readback
  after rejection/timeouts. ReadOnly disables edits and pending gestures; closing cancels
  loading and stops session retries. HTTP cancellation can wait for a current blocking
  network read, bounded by its 15-second per-read timeout. Wire and egui tests cover these
  software paths; full WinForms pixel equivalence, physical cameras and native mobile
  runtime qualification remain unverified.
- Captures: cases 64-66 (`MP_GIMBAL=full|mini|popout`).

PLAN Write and Write Fast follow BUT_write_Click and but_writewpfast_Click. Before uploading they:
1. Ask to confirm Absolute alt mode ("No" switches it to Relative).
2. Check the home location.
3. Check each row: Alt Warn ("Low alt on WP#n"), invalid numbers ("There are errors in your mission"),
   then checkZeroAlts' Show-me-again warning for Plane and Copter.

After that:
- Write Fast is mission-only and uses saveWPsFast's own protocol (`mp-core::wpfast`): MISSION_COUNT,
  items streamed ten at a time, partial re-upload on MAV_MISSION_ERROR, and the original error texts.
- Alt Warn is saved as "fpminaltwarning".
- Read asks "This will clear your existing points, Continue?" when the list is not empty.
- The progress dialog is titled "Sending WP's" or "Receiving WP's".
- The Plan screen keeps FlightPlanner's undo history (Ctrl+Z) and its other shortcuts: Ctrl+O, Ctrl+S,
  and Ctrl+Shift+F/W/R.

Params grid edits all follow Params_CellValueChanged: mXparser arithmetic, RC/HS `_REV` 0 becomes -1,
ReadOnly, and the "Out of range" Yes/No. This applies to the Options combo, the bitmask window and Set
to default as well. Escape cancels an edit. The write messages use the original texts and captions,
and Commit Params reports the autopilot's ACK.

## License

GPL-3.0, like Mission Planner. Icons and marker images are Mission Planner / GMap.NET assets under the same license.
System fonts (Arial, Microsoft Sans Serif) are loaded from the OS at runtime and not redistributed.
