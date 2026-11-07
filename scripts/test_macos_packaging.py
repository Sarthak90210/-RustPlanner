#!/usr/bin/env python3
"""Unit tests for macOS bundle packaging script."""

from pathlib import Path
import plistlib
import tempfile
import unittest

import build_macos_bundle as bmb


class MacOSPackagingTests(unittest.TestCase):
    def test_macos_info_plist_keys_and_document_types(self):
        info = bmb.generate_macos_info_plist("0.1.0", "42", "11.0")
        self.assertEqual(info["CFBundleSupportedPlatforms"], ["MacOSX"])
        self.assertEqual(info["CFBundleIdentifier"], bmb.BUNDLE_ID)
        self.assertTrue(info["NSHighResolutionCapable"])
        self.assertEqual(len(info["CFBundleDocumentTypes"]), 3)

        type_names = [dt["CFBundleTypeName"] for dt in info["CFBundleDocumentTypes"]]
        self.assertIn("ArduPilot Telemetry Log", type_names)
        self.assertIn("ArduPilot DataFlash Binary Log", type_names)

    def test_invalid_version_rejected(self):
        with self.assertRaises(ValueError):
            bmb.generate_macos_info_plist("0.1", "1")

    def test_assemble_macos_app_bundle_layout(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            binary = root / "missionplanner"
            binary.write_bytes(b"Mach-O binary fixture")

            app_dir = bmb.assemble_macos_app_bundle(binary, root / "dist", "0.1.0")
            self.assertTrue(app_dir.is_dir())
            self.assertTrue((app_dir / "Contents" / "MacOS" / "missionplanner").is_file())
            self.assertTrue((app_dir / "Contents" / "Info.plist").is_file())
            self.assertTrue((app_dir / "Contents" / "Resources" / "AppIcon.icns").is_file())

            with open(app_dir / "Contents" / "Info.plist", "rb") as f:
                loaded = plistlib.load(f)
            self.assertEqual(loaded["CFBundleExecutable"], "missionplanner")

    def test_package_macos_zip(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            binary = root / "missionplanner"
            binary.write_bytes(b"Mach-O binary fixture")

            app_dir = bmb.assemble_macos_app_bundle(binary, root / "dist", "0.1.0")
            zip_path, sha256 = bmb.package_macos_zip(app_dir, root / "dist" / "output.zip")
            self.assertTrue(zip_path.is_file())
            self.assertEqual(len(sha256), 64)


if __name__ == "__main__":
    unittest.main()
