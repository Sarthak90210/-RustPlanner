#!/usr/bin/env python3
"""Build the Rust executable into an iOS .app. Requires macOS and Xcode.

Simulator bundles are ad-hoc signed. Device bundles are deliberately unsigned;
they must be provisioned and signed before installation on an iPhone or iPad.
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
import subprocess
import tempfile
import tomllib

ROOT = Path(__file__).resolve().parents[1]
BUNDLE_ID = "org.ardupilot.missionplanner.rs"
EXECUTABLE = "missionplanner"
TARGETS = {
    "aarch64-apple-ios": ("iphoneos", "iPhoneOS", "IOS"),
    "aarch64-apple-ios-sim": ("iphonesimulator", "iPhoneSimulator", "IOSSIMULATOR"),
    "x86_64-apple-ios": ("iphonesimulator", "iPhoneSimulator", "IOSSIMULATOR"),
}


def resolve_target(target):
    if target == "device":
        return "aarch64-apple-ios"
    if target == "simulator":
        return "aarch64-apple-ios-sim" if platform.machine() == "arm64" else "x86_64-apple-ios"
    if target not in TARGETS:
        raise ValueError(f"Unsupported iOS target: {target}")
    return target


def info_plist(version, build_number, minimum_os, target):
    # Keep the deployment target consistent with rustc and native dependencies.
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        raise ValueError("Application version must contain three numeric components")
    if not re.fullmatch(r"\d+(?:\.\d+){0,2}", build_number):
        raise ValueError("Build number must contain one to three numeric components")
    if not re.fullmatch(r"\d+\.\d+(?:\.\d+)?", minimum_os) or int(minimum_os.split(".")[0]) < 15:
        raise ValueError("Minimum iOS version must be 15.0 or newer")
    sdk, supported, _ = TARGETS[target]
    info = {
        "CFBundleDevelopmentRegion": "en",
        "CFBundleDisplayName": "Mission Planner",
        "CFBundleExecutable": EXECUTABLE,
        "CFBundleIdentifier": BUNDLE_ID,
        "CFBundleInfoDictionaryVersion": "6.0",
        "CFBundleName": "Mission Planner",
        "CFBundlePackageType": "APPL",
        "CFBundleShortVersionString": version,
        "CFBundleVersion": build_number,
        "CFBundleSupportedPlatforms": [supported],
        "MinimumOSVersion": minimum_os,
        "LSRequiresIPhoneOS": True,
        "UIDeviceFamily": [1, 2],
        "UILaunchScreen": {},
        "UISupportedInterfaceOrientations": [
            "UIInterfaceOrientationPortrait",
            "UIInterfaceOrientationPortraitUpsideDown",
            "UIInterfaceOrientationLandscapeLeft",
            "UIInterfaceOrientationLandscapeRight",
        ],
        "UIApplicationSupportsIndirectInputEvents": True,
        "NSLocalNetworkUsageDescription": "Connect to flight controllers and cameras using Wi-Fi telemetry.",
        "NSAppTransportSecurity": {"NSAllowsLocalNetworking": True},
        "UIFileSharingEnabled": True,
        "LSSupportsOpeningDocumentsInPlace": True,
    }
    if sdk == "iphoneos":
        info["UIRequiredDeviceCapabilities"] = ["arm64", "metal"]
    return info


def assemble_bundle(binary, directory, info):
    """Assemble in a fresh directory so a failed build cannot reuse an old app."""
    app = directory / "MissionPlanner.app"
    app.mkdir()
    shutil.copy2(binary, app / EXECUTABLE)
    (app / EXECUTABLE).chmod(0o755)
    with (app / "Info.plist").open("wb") as handle:
        plistlib.dump(info, handle)
    (app / "PkgInfo").write_bytes(b"APPL????")
    # Exposes the existing mobile file browser's persistent files via Finder.
    return app


def run(command, **kwargs):
    print("+ " + " ".join(map(str, command)), flush=True)
    return subprocess.run(command, check=True, text=True, **kwargs)


def invalidate_artifacts(destination, kind):
    destination.mkdir(parents=True, exist_ok=True)
    for name in ["build.json", f"missionplanner-ios-{kind}.zip", "MissionPlanner.app"]:
        artifact = destination / name
        if artifact.is_dir() and not artifact.is_symlink():
            shutil.rmtree(artifact)
        elif artifact.exists() or artifact.is_symlink():
            artifact.unlink()


def build(args):
    if platform.system() != "Darwin":
        raise RuntimeError("iOS application builds require macOS with Xcode selected by xcode-select")
    target = resolve_target(args.target)
    sdk, _, binary_platform = TARGETS[target]
    kind = "device-unsigned" if sdk == "iphoneos" else "simulator"
    destination = ROOT / "target" / "ios" / kind
    invalidate_artifacts(destination, kind)
    version = tomllib.loads((ROOT / "Cargo.toml").read_text())["workspace"]["package"]["version"]
    info = info_plist(version, args.build_number, args.minimum_os, target)
    sdk_path = run(["xcrun", "--sdk", sdk, "--show-sdk-path"], capture_output=True).stdout.strip()
    env = os.environ.copy()
    env["SDKROOT"] = sdk_path
    env["IPHONEOS_DEPLOYMENT_TARGET"] = args.minimum_os
    # Own the target directory so a caller's CARGO_TARGET_DIR cannot redirect the binary lookup.
    env["CARGO_TARGET_DIR"] = str(ROOT / "target")
    command = ["cargo", "build", "--locked", "-p", "mp-app", "--bin", EXECUTABLE, "--target", target]
    if args.release:
        command.append("--release")
    run(command, cwd=ROOT, env=env)
    binary = ROOT / "target" / target / ("release" if args.release else "debug") / EXECUTABLE
    load_commands = run(["xcrun", "vtool", "-show-build", str(binary)], capture_output=True).stdout
    if not re.search(r"platform\s+" + binary_platform + r"\b", load_commands, re.IGNORECASE):
        raise RuntimeError(f"Executable does not target {binary_platform}:\n{load_commands}")

    # Only replace artifacts created by this script inside this fixed directory.
    with tempfile.TemporaryDirectory(prefix="assemble-", dir=destination) as staging:
        app = assemble_bundle(binary, Path(staging), info)
        if sdk == "iphonesimulator":
            run(["codesign", "--force", "--sign", "-", "--timestamp=none", str(app)])
            run(["codesign", "--verify", "--strict", str(app)])
        archive = Path(staging) / f"missionplanner-ios-{kind}.zip"
        run(["ditto", "-c", "-k", "--keepParent", str(app), str(archive)])
        installed = destination / app.name
        shutil.move(str(app), installed)
        shutil.copy2(archive, destination / archive.name)
    receipt = {
        "status": "built",
        "source_commit": run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True).stdout.strip(),
        "source_dirty": bool(run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True).stdout.strip()),
        "target": target,
        "profile": "release" if args.release else "debug",
        "sdk": sdk,
        "minimum_os": args.minimum_os,
        "bundle_id": BUNDLE_ID,
        "executable_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "signing": "ad-hoc simulator" if sdk == "iphonesimulator" else "unsigned; provisioning and signing required",
        "load_commands": load_commands,
        "runtime_tested": False,
    }
    (destination / "build.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"Application: {installed}\nArchive: {destination / archive.name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=["simulator", "device", *TARGETS], default="simulator")
    parser.add_argument("--release", action="store_true")
    parser.add_argument("--build-number", default="1")
    parser.add_argument("--minimum-os", default="15.0")
    args = parser.parse_args()
    try:
        build(args)
    except (RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        details = f"iOS build failed: {error}\n"
        if isinstance(error, subprocess.CalledProcessError):
            details += (error.stdout or "") + (error.stderr or "")
        parser.exit(1, details)


if __name__ == "__main__":
    main()
