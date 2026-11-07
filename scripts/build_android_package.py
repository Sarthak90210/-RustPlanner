#!/usr/bin/env python3
"""Build Android APK staging layout and manifest for Mission Planner Rust port.
Configures USB-OTG hardware intent filters, BLE telemetry permissions, and multi-ABI layout.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET
import zipfile

PACKAGE_NAME = "org.ardupilot.missionplanner.rs"
APP_NAME = "Mission Planner"
NATIVE_LIB_NAME = "mp_app"

PERMISSIONS = [
    "android.permission.INTERNET",
    "android.permission.ACCESS_NETWORK_STATE",
    "android.permission.ACCESS_FINE_LOCATION",
    "android.permission.ACCESS_COARSE_LOCATION",
    "android.permission.BLUETOOTH",
    "android.permission.BLUETOOTH_ADMIN",
    "android.permission.BLUETOOTH_SCAN",
    "android.permission.BLUETOOTH_CONNECT",
    "android.permission.READ_EXTERNAL_STORAGE",
    "android.permission.WRITE_EXTERNAL_STORAGE",
]

USB_DEVICE_FILTERS = [
    {"vendor_id": 0x26AC, "desc": "3DR / Hex Pixhawk"},
    {"vendor_id": 0x0483, "product_id": 0x5740, "desc": "STM32 Virtual COM"},
    {"vendor_id": 0x0403, "desc": "FTDI USB Serial"},
    {"vendor_id": 0x10C4, "product_id": 0xEA60, "desc": "SiK / CP2102"},
    {"vendor_id": 0x1209, "product_id": 0x5740, "desc": "Pid.codes ArduPilot"},
]


def generate_android_manifest(
    version_name: str,
    version_code: int = 1,
    min_sdk: int = 24,
    target_sdk: int = 34,
) -> str:
    """Generates AndroidManifest.xml string."""
    perm_lines = "\n    ".join(f'<uses-permission android:name="{p}" />' for p in PERMISSIONS)

    manifest = f"""<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="{PACKAGE_NAME}"
    android:versionCode="{version_code}"
    android:versionName="{version_name}">

    <uses-sdk android:minSdkVersion="{min_sdk}" android:targetSdkVersion="{target_sdk}" />

    <!-- Hardware Features -->
    <uses-feature android:name="android.hardware.usb.host" android:required="false" />
    <uses-feature android:name="android.hardware.bluetooth_le" android:required="false" />
    <uses-feature android:name="android.hardware.location.gps" android:required="false" />

    <!-- Runtime Permissions -->
    {perm_lines}

    <application
        android:label="{APP_NAME}"
        android:hasCode="false"
        android:allowBackup="true"
        android:supportsRtl="true">

        <activity
            android:name="android.app.NativeActivity"
            android:label="{APP_NAME}"
            android:configChanges="orientation|keyboardHidden|screenSize"
            android:exported="true">

            <meta-data android:name="android.app.lib_name" android:value="{NATIVE_LIB_NAME}" />

            <intent-filter>
                <action android:name="android.intent.action.MAIN" />
                <category android:name="android.intent.category.LAUNCHER" />
            </intent-filter>

            <!-- USB-OTG Auto-Connect Intent Filter -->
            <intent-filter>
                <action android:name="android.hardware.usb.action.USB_DEVICE_ATTACHED" />
            </intent-filter>
            <meta-data android:name="android.hardware.usb.action.USB_DEVICE_ATTACHED"
                       android:resource="@xml/device_filter" />
        </activity>
    </application>
</manifest>
"""
    return manifest


def generate_device_filter_xml() -> str:
    """Generates res/xml/device_filter.xml for USB-OTG attached device events."""
    lines = ['<?xml version="1.0" encoding="utf-8"?>', "<resources>"]
    for dev in USB_DEVICE_FILTERS:
        vid = dev["vendor_id"]
        pid = dev.get("product_id")
        if pid is not None:
            lines.append(f'    <usb-device vendor-id="{vid}" product-id="{pid}" />')
        else:
            lines.append(f'    <usb-device vendor-id="{vid}" />')
    lines.append("</resources>")
    return "\n".join(lines)


def assemble_apk_tree(
    binary_path: Path,
    output_dir: Path,
    version_name: str,
    version_code: int = 1,
    abi: str = "arm64-v8a",
) -> Path:
    """Assembles an unpacked APK staging directory."""
    if not binary_path.is_file():
        raise FileNotFoundError(f"Native shared library not found: {binary_path}")

    apk_root = output_dir / f"MissionPlanner-{version_name}-apk"
    if apk_root.exists():
        shutil.rmtree(apk_root)

    # 1. Manifest
    apk_root.mkdir(parents=True)
    (apk_root / "AndroidManifest.xml").write_text(generate_android_manifest(version_name, version_code))

    # 2. Res XML
    res_xml = apk_root / "res" / "xml"
    res_xml.mkdir(parents=True)
    (res_xml / "device_filter.xml").write_text(generate_device_filter_xml())

    # 3. Native Lib
    lib_dir = apk_root / "lib" / abi
    lib_dir.mkdir(parents=True)
    target_lib = lib_dir / f"lib{NATIVE_LIB_NAME}.so"
    shutil.copy2(binary_path, target_lib)

    # 4. Assets directory
    assets_dir = apk_root / "assets"
    assets_dir.mkdir(parents=True)
    (assets_dir / "placeholder.txt").write_text("Mission Planner Rust assets")

    return apk_root


def assemble_apk_zip(apk_root: Path, output_apk: Path) -> Tuple[Path, str]:
    """Zips the APK staging tree into an unaligned APK archive."""
    output_apk.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output_apk, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(apk_root):
            for file in files:
                p = Path(root) / file
                arc = p.relative_to(apk_root)
                zf.write(p, arcname=str(arc))

    data = output_apk.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()
    return output_apk, sha256


def main():
    parser = argparse.ArgumentParser(description="Build Android APK staging layout")
    parser.add_argument("--version", default="0.1.0")
    parser.add_argument("--version-code", type=int, default=1)
    parser.add_argument("--binary", type=Path, default=Path("target/aarch64-linux-android/release/libmp_app.so"))
    parser.add_argument("--out", type=Path, default=Path("dist/android"))
    parser.add_argument("--abi", default="arm64-v8a")

    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.binary.is_file():
        tree = assemble_apk_tree(args.binary, args.out, args.version, args.version_code, args.abi)
        print(f"Assembled Android staging directory: {tree}")

        apk_path = args.out / f"MissionPlanner-{args.version}-{args.abi}.apk"
        assemble_apk_zip(tree, apk_path)
        print(f"Packaged APK: {apk_path}")


if __name__ == "__main__":
    main()
