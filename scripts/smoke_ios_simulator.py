#!/usr/bin/env python3
"""Install and render the app in a newly created, disposable iOS Simulator.

Passing requires a PNG and receipt from the app's own eframe renderer. This tests
startup and rendering only; it is not an iPhone hardware or feature-parity test.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import plistlib
import re
import shutil
import struct
import subprocess
import time
import zlib


def simctl(*args, **kwargs):
    return subprocess.run(["xcrun", "simctl", *map(str, args)], check=True, text=True,
                          capture_output=True, timeout=kwargs.pop("timeout", 60), **kwargs).stdout.strip()


def select_device(inventory):
    candidates = []
    for runtime, devices in inventory["devices"].items():
        if ".iOS-" not in runtime:
            continue
        for device in devices:
            if device.get("isAvailable") and device["name"].startswith("iPhone") and device.get("deviceTypeIdentifier"):
                version = tuple(int(n) for n in runtime.split(".iOS-", 1)[1].split("-"))
                candidates.append((version, runtime, device["deviceTypeIdentifier"]))
    if not candidates:
        raise RuntimeError("Xcode has no available iPhone simulator. Install an iOS runtime first.")
    _, runtime, device_type = max(candidates)
    return runtime, device_type


def png_dimensions(path):
    """Validate the complete non-interlaced RGBA8 PNG emitted by image::save_buffer."""
    if path.stat().st_size > 128 * 1024 * 1024:
        raise RuntimeError("Renderer PNG exceeds the capture limit")
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise RuntimeError("Renderer did not produce a PNG")
    position, width, height = 8, 0, 0
    compressed = bytearray()
    ended = False
    while position < len(data):
        if position + 12 > len(data):
            raise RuntimeError("Truncated PNG chunk")
        length = struct.unpack_from(">I", data, position)[0]
        kind = data[position + 4:position + 8]
        end = position + 8 + length
        if end + 4 > len(data):
            raise RuntimeError("Truncated PNG payload")
        payload = data[position + 8:end]
        if zlib.crc32(kind + payload) != struct.unpack_from(">I", data, end)[0]:
            raise RuntimeError("PNG checksum mismatch")
        if position == 8 and kind != b"IHDR":
            raise RuntimeError("PNG has no initial IHDR")
        if kind == b"IHDR":
            if position != 8 or length != 13:
                raise RuntimeError("Invalid PNG header")
            width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", payload)
            if not (width and height and width * height <= 32_000_000) or (depth, color, compression, filtering, interlace) != (8, 6, 0, 0, 0):
                raise RuntimeError("Capture must be a bounded RGBA8 PNG")
        elif kind == b"IDAT":
            compressed.extend(payload)
        elif kind == b"IEND":
            if length or end + 4 != len(data):
                raise RuntimeError("Invalid PNG end")
            ended = True
            break
        elif not (kind[0] & 32):
            raise RuntimeError("Unexpected critical PNG chunk")
        position = end + 4
    if not ended or not compressed:
        raise RuntimeError("PNG is missing image data or IEND")
    stride = 1 + width * 4
    decoder = zlib.decompressobj()
    try:
        pixels = decoder.decompress(compressed, stride * height + 1)
    except zlib.error as error:
        raise RuntimeError("Invalid PNG compressed pixels") from error
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail or len(pixels) != stride * height:
        raise RuntimeError("PNG compressed pixels are incomplete or exceed dimensions")
    if any(pixels[row * stride] > 4 for row in range(height)):
        raise RuntimeError("Invalid PNG row filter")
    return width, height


def verify_capture(directory):
    receipt = json.loads((directory / "startup.capture.json").read_text())
    width, height = png_dimensions(directory / "startup.png")
    if receipt.get("status") != "ok" or receipt.get("source") != "eframe renderer viewport":
        raise RuntimeError("Missing successful renderer receipt")
    if min(width, height) < 200 or [width, height] != [receipt.get("width"), receipt.get("height")]:
        raise RuntimeError("Renderer receipt and PNG dimensions disagree or are too small")
    if receipt.get("runtime", {}).get("view") != "FlightData":
        raise RuntimeError("Application did not render the requested Flight Data view")
    return receipt


def smoke(args):
    if platform.system() != "Darwin":
        raise RuntimeError("The iOS Simulator requires macOS with Xcode")
    app = args.app.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # A failed invocation must never leave a previous success in its evidence directory.
    for name in ["startup.png", "startup.capture.json", "display.png", "launch.txt",
                 "smoke.json", "processes.txt", "application.log", "failure.txt"]:
        (output / name).unlink(missing_ok=True)
    with (app / "Info.plist").open("rb") as handle:
        info = plistlib.load(handle)
    if info["CFBundleSupportedPlatforms"] != ["iPhoneSimulator"]:
        raise RuntimeError("Smoke test requires a simulator app, not a device app")
    runtime, device_type = select_device(json.loads(simctl("list", "--json")))
    device = simctl("create", f"MissionPlanner-smoke-{os.getpid()}", device_type, runtime)
    bundle_id = info["CFBundleIdentifier"]
    try:
        simctl("boot", device)
        simctl("bootstatus", device, "-b", timeout=240)
        simctl("install", device, app, timeout=120)
        container = Path(simctl("get_app_container", device, bundle_id, "data"))
        captures = container / "Documents" / "startup-smoke"
        captures.mkdir(parents=True, exist_ok=True)
        env = os.environ.copy()
        env.update({"SIMCTL_CHILD_MP_CAPTURE": str(captures / "startup.png"),
                    "SIMCTL_CHILD_MP_CAPTURE_AFTER": "3", "SIMCTL_CHILD_MP_VIEW": "data",
                    "SIMCTL_CHILD_MP_CAPTURE_KEEP_OPEN": "1"})
        launch = simctl("launch", device, bundle_id, env=env)
        pid_match = re.search(r": (\d+)$", launch)
        if not pid_match:
            raise RuntimeError(f"simctl did not report an application process: {launch}")
        pid = pid_match.group(1)
        (output / "launch.txt").write_text(launch + "\n")
        deadline = time.monotonic() + 90
        while True:
            try:
                # The receipt is written after the image, but may still be open.
                verify_capture(captures)
                break
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError("App did not deliver a renderer capture within 90 seconds")
            time.sleep(0.25)
        for name in ["startup.png", "startup.capture.json"]:
            shutil.copy2(captures / name, output / name)
        receipt = verify_capture(output)
        # Keep an independent image of the simulator's displayed surface as well.
        simctl("io", device, "screenshot", output / "display.png")
        processes = simctl("spawn", device, "launchctl", "list")
        (output / "processes.txt").write_text(processes + "\n")
        if not any(line.split() and line.split()[0] == pid and bundle_id in line for line in processes.splitlines()):
            raise RuntimeError("Application exited before the display capture completed")
        (output / "smoke.json").write_text(json.dumps({
            "status": "ok", "scope": "iOS Simulator startup and renderer",
            "runtime": runtime, "device_type": device_type, "launch": launch,
            "width": receipt["width"], "height": receipt["height"],
            "process_id": int(pid), "alive_after_display_capture": True,
            "bundle_id": bundle_id,
            "executable_sha256": hashlib.sha256((app / info["CFBundleExecutable"]).read_bytes()).hexdigest(),
            "physical_device_tested": False, "feature_parity_tested": False,
        }, indent=2) + "\n")
        print(f"Simulator rendered {receipt['width']} x {receipt['height']} pixels. Evidence: {output}")
    finally:
        # All operations target the simulator created above, never 'booted'.
        try:
            logs = simctl("spawn", device, "log", "show", "--last", "2m", "--style", "compact",
                          "--predicate", 'process == "missionplanner"', timeout=30)
            (output / "application.log").write_text(logs + "\n")
        except subprocess.SubprocessError:
            pass
        for command in [("terminate", device, bundle_id), ("shutdown", device), ("delete", device)]:
            try:
                simctl(*command)
            except subprocess.SubprocessError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", type=Path, default=Path("target/ios/simulator/MissionPlanner.app"))
    parser.add_argument("--output", type=Path, default=Path("target/ios/smoke"))
    args = parser.parse_args()
    try:
        smoke(args)
    except (RuntimeError, subprocess.SubprocessError) as error:
        details = f"iOS simulator smoke failed: {error}\n"
        if isinstance(error, subprocess.CalledProcessError):
            details += (error.stdout or "") + (error.stderr or "")
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "failure.txt").write_text(details)
        parser.exit(1, details)


if __name__ == "__main__":
    main()
