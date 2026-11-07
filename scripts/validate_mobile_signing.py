#!/usr/bin/env python3
"""Mobile application code signing, provisioning profile, and entitlements validator.
Validates iOS (.app / embedded.mobileprovision / CodeResources) and Android (APK / keystore)
delivery artifacts for production and device deployments.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import struct
import sys
from typing import Any, Dict, List, Optional, Tuple

BUNDLE_ID = "org.ardupilot.missionplanner.rs"

REQUIRED_IOS_ENTITLEMENTS = [
    "application-identifier",
    "com.apple.developer.networking.multicast",
    "keychain-access-groups",
]

REQUIRED_ANDROID_PERMISSIONS = [
    "android.permission.INTERNET",
    "android.permission.ACCESS_FINE_LOCATION",
    "android.permission.BLUETOOTH",
]


class SigningValidationError(Exception):
    pass


def extract_provisioning_plist(data: bytes) -> bytes:
    """Extracts XML plist embedded inside a PKCS#7 DER/CMS signedData envelope."""
    # Look for <?xml ... </plist>
    xml_start = data.find(b"<?xml")
    if xml_start == -1:
        xml_start = data.find(b"<plist")
    if xml_start == -1:
        raise SigningValidationError("No XML/plist found inside provisioning profile data")
    xml_end = data.find(b"</plist>", xml_start)
    if xml_end == -1:
        raise SigningValidationError("Truncated plist inside provisioning profile")
    xml_end += len(b"</plist>")
    return data[xml_start:xml_end]


def parse_provisioning_profile(profile_path: Path) -> Dict[str, Any]:
    if not profile_path.is_file():
        raise FileNotFoundError(f"Provisioning profile not found: {profile_path}")
    raw_bytes = profile_path.read_bytes()
    plist_bytes = extract_provisioning_plist(raw_bytes)
    try:
        profile = plistlib.loads(plist_bytes)
    except Exception as e:
        raise SigningValidationError(f"Failed to parse provisioning plist: {e}")
    return profile


def validate_ios_profile(
    profile: Dict[str, Any],
    expected_bundle_id: str = BUNDLE_ID,
    require_unexpired: bool = True,
) -> Dict[str, Any]:
    """Validates iOS provisioning profile fields, expiration date, and entitlements."""
    name = profile.get("Name")
    uuid = profile.get("UUID")
    team_ids = profile.get("TeamIdentifier", [])
    expiration = profile.get("ExpirationDate")
    entitlements = profile.get("Entitlements", {})

    if not name or not uuid or not team_ids:
        raise SigningValidationError("Missing critical header fields (Name, UUID, TeamIdentifier)")

    now = datetime.now(timezone.utc)
    if require_unexpired and expiration:
        if expiration.tzinfo is None:
            expiration = expiration.replace(tzinfo=timezone.utc)
        if expiration < now:
            raise SigningValidationError(f"Provisioning profile expired on {expiration.isoformat()}")

    app_id = entitlements.get("application-identifier", "")
    expected_suffix = expected_bundle_id
    if not app_id.endswith(expected_suffix) and not app_id.endswith("*"):
        raise SigningValidationError(
            f"Profile application-identifier '{app_id}' does not match bundle id '{expected_suffix}'"
        )

    for req in REQUIRED_IOS_ENTITLEMENTS:
        if req not in entitlements:
            raise SigningValidationError(f"Missing required entitlement '{req}' in provisioning profile")

    return {
        "status": "valid",
        "name": name,
        "uuid": uuid,
        "team_identifier": team_ids[0] if team_ids else "",
        "application_identifier": app_id,
        "expiration": expiration.isoformat() if expiration else None,
        "devices_count": len(profile.get("ProvisionedDevices", [])),
        "provisions_all_devices": profile.get("ProvisionsAllDevices", False),
    }


def validate_ios_app_bundle(app_dir: Path) -> Dict[str, Any]:
    """Validates an assembled iOS .app directory for code signing structures."""
    if not app_dir.is_dir():
        raise FileNotFoundError(f"App directory not found: {app_dir}")

    info_plist_path = app_dir / "Info.plist"
    if not info_plist_path.is_file():
        raise SigningValidationError("Missing Info.plist in app bundle")

    with open(info_plist_path, "rb") as f:
        info = plistlib.load(f)

    bundle_id = info.get("CFBundleIdentifier")
    executable_name = info.get("CFBundleExecutable")
    if not bundle_id or not executable_name:
        raise SigningValidationError("Info.plist missing CFBundleIdentifier or CFBundleExecutable")

    executable_path = app_dir / executable_name
    if not executable_path.is_file():
        raise SigningValidationError(f"Executable {executable_name} missing from bundle")

    # Check for mobileprovision if signed for device
    prov_path = app_dir / "embedded.mobileprovision"
    prov_info = None
    if prov_path.is_file():
        profile = parse_provisioning_profile(prov_path)
        prov_info = validate_ios_profile(profile, expected_bundle_id=bundle_id)

    # Check CodeResources if present
    code_resources_path = app_dir / "_CodeSignature" / "CodeResources"
    has_code_signature = code_resources_path.is_file()

    return {
        "status": "ok",
        "bundle_id": bundle_id,
        "executable": executable_name,
        "has_provisioning_profile": prov_path.is_file(),
        "has_code_signature": has_code_signature,
        "provisioning": prov_info,
    }


def validate_android_keystore_config(
    store_file: Path,
    alias: str,
    key_size_min: int = 2048,
) -> Dict[str, Any]:
    """Validates Android keystore configuration and existence."""
    if not store_file.is_file():
        raise FileNotFoundError(f"Android keystore file not found: {store_file}")
    if not alias:
        raise SigningValidationError("Android signing key alias cannot be empty")

    data = store_file.read_bytes()
    if len(data) < 32:
        raise SigningValidationError("Keystore file truncated")

    sha256 = hashlib.sha256(data).hexdigest()
    return {
        "status": "valid",
        "keystore": str(store_file),
        "alias": alias,
        "sha256": sha256,
        "size_bytes": len(data),
    }


def main():
    parser = argparse.ArgumentParser(description="Validate mobile signing and provisioning profiles")
    subparsers = parser.add_subparsers(dest="cmd")

    p_ios = subparsers.add_parser("check-ios-profile")
    p_ios.add_argument("profile_path", type=Path)
    p_ios.add_argument("--bundle-id", default=BUNDLE_ID)

    p_app = subparsers.add_parser("check-ios-app")
    p_app.add_argument("app_dir", type=Path)

    p_droid = subparsers.add_parser("check-android-keystore")
    p_droid.add_argument("keystore", type=Path)
    p_droid.add_argument("--alias", default="missionplanner")

    args = parser.parse_args()

    if args.cmd == "check-ios-profile":
        profile = parse_provisioning_profile(args.profile_path)
        res = validate_ios_profile(profile, expected_bundle_id=args.bundle_id)
        print(json.dumps(res, indent=2))
    elif args.cmd == "check-ios-app":
        res = validate_ios_app_bundle(args.app_dir)
        print(json.dumps(res, indent=2))
    elif args.cmd == "check-android-keystore":
        res = validate_android_keystore_config(args.keystore, alias=args.alias)
        print(json.dumps(res, indent=2))
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
