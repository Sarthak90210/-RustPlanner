#!/usr/bin/env python3
"""Build macOS .app bundle and DMG distribution layout for Mission Planner Rust port.
Handles Info.plist generation, document types, High-DPI support, and packaging.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import sys
from typing import Dict, List, Optional, Tuple
import zipfile

BUNDLE_ID = "org.ardupilot.missionplanner.rs"
EXECUTABLE = "missionplanner"
APP_NAME = "Mission Planner"

DOCUMENT_TYPES = [
    {
        "CFBundleTypeName": "ArduPilot Telemetry Log",
        "CFBundleTypeRole": "Viewer",
        "LSHandlerRank": "Owner",
        "LSItemContentTypes": ["org.ardupilot.tlog"],
    },
    {
        "CFBundleTypeName": "ArduPilot DataFlash Binary Log",
        "CFBundleTypeRole": "Viewer",
        "LSHandlerRank": "Owner",
        "LSItemContentTypes": ["org.ardupilot.bin"],
    },
    {
        "CFBundleTypeName": "ArduPilot Parameter File",
        "CFBundleTypeRole": "Editor",
        "LSHandlerRank": "Default",
        "LSItemContentTypes": ["public.plain-text", "org.ardupilot.param"],
    },
]

UT_EXPORTS = [
    {
        "UTTypeIdentifier": "org.ardupilot.tlog",
        "UTTypeDescription": "ArduPilot Telemetry Log",
        "UTTypeConformsTo": ["public.data"],
        "UTTypeTagSpecification": {"public.filename-extension": ["tlog", "rlog"]},
    },
    {
        "UTTypeIdentifier": "org.ardupilot.bin",
        "UTTypeDescription": "ArduPilot DataFlash Binary Log",
        "UTTypeConformsTo": ["public.data"],
        "UTTypeTagSpecification": {"public.filename-extension": ["bin"]},
    },
]


def generate_macos_info_plist(
    version: str,
    build_number: str = "1",
    minimum_os: str = "11.0",
) -> Dict:
    """Generates Info.plist dictionary for macOS desktop application."""
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        raise ValueError(f"Version must have 3 components: {version}")

    return {
        "CFBundleDevelopmentRegion": "en",
        "CFBundleDisplayName": APP_NAME,
        "CFBundleExecutable": EXECUTABLE,
        "CFBundleIconFile": "AppIcon",
        "CFBundleIdentifier": BUNDLE_ID,
        "CFBundleInfoDictionaryVersion": "6.0",
        "CFBundleName": APP_NAME,
        "CFBundlePackageType": "APPL",
        "CFBundleShortVersionString": version,
        "CFBundleVersion": build_number,
        "CFBundleSupportedPlatforms": ["MacOSX"],
        "LSMinimumSystemVersion": minimum_os,
        "NSHighResolutionCapable": True,
        "NSRequiresAquaSystemAppearance": False,
        "CFBundleDocumentTypes": DOCUMENT_TYPES,
        "UTExportedTypeDeclarations": UT_EXPORTS,
        "NSBluetoothAlwaysUsageDescription": "Mission Planner uses Bluetooth to connect to telemetry radios and flight controllers.",
        "NSLocalNetworkUsageDescription": "Mission Planner discovers and connects to MAVLink network endpoints (UDP/TCP).",
    }


def assemble_macos_app_bundle(
    binary_path: Path,
    output_dir: Path,
    version: str,
    build_number: str = "1",
) -> Path:
    """Assembles the .app directory structure."""
    if not binary_path.is_file():
        raise FileNotFoundError(f"Binary not found: {binary_path}")

    app_dir = output_dir / f"{APP_NAME}.app"
    if app_dir.exists():
        shutil.rmtree(app_dir)

    contents_dir = app_dir / "Contents"
    macos_dir = contents_dir / "MacOS"
    resources_dir = contents_dir / "Resources"

    macos_dir.mkdir(parents=True)
    resources_dir.mkdir(parents=True)

    # Copy binary
    shutil.copy2(binary_path, macos_dir / EXECUTABLE)

    # Write Info.plist
    info = generate_macos_info_plist(version, build_number)
    with open(contents_dir / "Info.plist", "wb") as f:
        plistlib.dump(info, f)

    # Write placeholder icon
    (resources_dir / "AppIcon.icns").write_bytes(b"icns_placeholder_data")

    return app_dir


def package_macos_zip(app_dir: Path, output_zip: Path) -> Tuple[Path, str]:
    """Zips the .app directory into a release archive and computes SHA-256."""
    output_zip.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_zip, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(app_dir):
            for file in files:
                p = Path(root) / file
                arc = p.relative_to(app_dir.parent)
                zf.write(p, arcname=str(arc))

    data = output_zip.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()
    return output_zip, sha256


def main():
    parser = argparse.ArgumentParser(description="Build macOS .app bundle and release archive")
    parser.add_argument("--version", default="0.1.0")
    parser.add_argument("--binary", type=Path, default=Path("target/release/missionplanner"))
    parser.add_argument("--out", type=Path, default=Path("dist/macos"))

    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.binary.is_file():
        app = assemble_macos_app_bundle(args.binary, args.out, args.version)
        print(f"Assembled macOS app: {app}")

        zip_path = args.out / f"MissionPlanner-{args.version}-macos.zip"
        package_macos_zip(app, zip_path)
        print(f"Packaged macOS archive: {zip_path}")


if __name__ == "__main__":
    main()
