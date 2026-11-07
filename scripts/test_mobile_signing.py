#!/usr/bin/env python3
"""Unit tests for validate_mobile_signing.py."""

from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import plistlib
import tempfile
import unittest

import validate_mobile_signing as vms


class MobileSigningTests(unittest.TestCase):
    def _create_sample_profile_dict(self, expired=False, bad_bundle=False, missing_ent=False):
        exp = datetime.now(timezone.utc) + (timedelta(days=-1) if expired else timedelta(days=365))
        bundle = "com.other.app" if bad_bundle else vms.BUNDLE_ID
        entitlements = {
            "application-identifier": f"ABCDE12345.{bundle}",
            "keychain-access-groups": [f"ABCDE12345.{vms.BUNDLE_ID}"],
            "com.apple.developer.networking.multicast": True,
            "get-task-allow": False,
        }
        if missing_ent:
            entitlements.pop("com.apple.developer.networking.multicast")

        return {
            "Name": "Mission Planner iOS Dist",
            "UUID": "12345678-1234-1234-1234-123456789abc",
            "TeamIdentifier": ["ABCDE12345"],
            "CreationDate": datetime.now(timezone.utc),
            "ExpirationDate": exp,
            "Entitlements": entitlements,
            "ProvisionedDevices": ["udid1", "udid2"],
            "ProvisionsAllDevices": False,
        }

    def _wrap_in_cms(self, plist_bytes: bytes) -> bytes:
        # Mock CMS wrapper around XML plist
        prefix = b"\x30\x82\x04\x00\x06\x09\x2a\x86\x48\x86\xf7\x0d\x01\x07\x02"
        suffix = b"\x00\x00\x00\x00"
        return prefix + plist_bytes + suffix

    def test_valid_profile_passes_validation(self):
        prof = self._create_sample_profile_dict()
        res = vms.validate_ios_profile(prof)
        self.assertEqual(res["status"], "valid")
        self.assertEqual(res["team_identifier"], "ABCDE12345")
        self.assertEqual(res["devices_count"], 2)

    def test_expired_profile_raises_error(self):
        prof = self._create_sample_profile_dict(expired=True)
        with self.assertRaises(vms.SigningValidationError) as ctx:
            vms.validate_ios_profile(prof)
        self.assertIn("expired", str(ctx.exception).lower())

    def test_bundle_id_mismatch_raises_error(self):
        prof = self._create_sample_profile_dict(bad_bundle=True)
        with self.assertRaises(vms.SigningValidationError) as ctx:
            vms.validate_ios_profile(prof)
        self.assertIn("does not match", str(ctx.exception).lower())

    def test_missing_required_entitlement_raises_error(self):
        prof = self._create_sample_profile_dict(missing_ent=True)
        with self.assertRaises(vms.SigningValidationError) as ctx:
            vms.validate_ios_profile(prof)
        self.assertIn("multicast", str(ctx.exception).lower())

    def test_parse_provisioning_profile_file_with_envelope(self):
        prof_dict = self._create_sample_profile_dict()
        raw_plist = plistlib.dumps(prof_dict)
        wrapped = self._wrap_in_cms(raw_plist)

        with tempfile.TemporaryDirectory() as td:
            f = Path(td) / "embedded.mobileprovision"
            f.write_bytes(wrapped)
            parsed = vms.parse_provisioning_profile(f)
            self.assertEqual(parsed["Name"], prof_dict["Name"])
            self.assertEqual(parsed["UUID"], prof_dict["UUID"])

    def test_validate_ios_app_bundle_structure(self):
        with tempfile.TemporaryDirectory() as td:
            app_dir = Path(td) / "MissionPlanner.app"
            app_dir.mkdir()
            info = {
                "CFBundleIdentifier": vms.BUNDLE_ID,
                "CFBundleExecutable": "missionplanner",
            }
            with open(app_dir / "Info.plist", "wb") as f:
                plistlib.dump(info, f)
            (app_dir / "missionplanner").write_bytes(b"binary")

            # Add embedded.mobileprovision
            prof_dict = self._create_sample_profile_dict()
            (app_dir / "embedded.mobileprovision").write_bytes(self._wrap_in_cms(plistlib.dumps(prof_dict)))

            # Add _CodeSignature
            cs_dir = app_dir / "_CodeSignature"
            cs_dir.mkdir()
            (cs_dir / "CodeResources").write_bytes(b"codesig")

            res = vms.validate_ios_app_bundle(app_dir)
            self.assertEqual(res["status"], "ok")
            self.assertTrue(res["has_provisioning_profile"])
            self.assertTrue(res["has_code_signature"])
            self.assertEqual(res["provisioning"]["status"], "valid")

    def test_validate_android_keystore(self):
        with tempfile.TemporaryDirectory() as td:
            ks_path = Path(td) / "release.keystore"
            ks_path.write_bytes(b"dummy keystore data of sufficient length for test")
            res = vms.validate_android_keystore_config(ks_path, alias="release_alias")
            self.assertEqual(res["status"], "valid")
            self.assertEqual(res["alias"], "release_alias")
            self.assertTrue(res["sha256"])


if __name__ == "__main__":
    unittest.main()
