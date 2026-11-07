#!/usr/bin/env python3
"""Unit tests for Android APK staging and manifest packaging."""

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import build_android_package as bap


class AndroidPackagingTests(unittest.TestCase):
    def test_android_manifest_structure_and_permissions(self):
        manifest_xml = bap.generate_android_manifest("0.1.0", version_code=42)
        root = ET.fromstring(manifest_xml)

        self.assertEqual(root.attrib["package"], bap.PACKAGE_NAME)
        self.assertEqual(root.attrib["{http://schemas.android.com/apk/res/android}versionCode"], "42")
        self.assertEqual(root.attrib["{http://schemas.android.com/apk/res/android}versionName"], "0.1.0")

        # Verify permissions
        perms = [elem.attrib["{http://schemas.android.com/apk/res/android}name"] for elem in root.findall("uses-permission")]
        self.assertIn("android.permission.INTERNET", perms)
        self.assertIn("android.permission.ACCESS_FINE_LOCATION", perms)
        self.assertIn("android.permission.BLUETOOTH", perms)

    def test_usb_device_filter_xml_structure(self):
        filter_xml = bap.generate_device_filter_xml()
        root = ET.fromstring(filter_xml)
        self.assertEqual(root.tag, "resources")

        devices = root.findall("usb-device")
        self.assertTrue(len(devices) >= 5)
        vids = [dev.attrib["vendor-id"] for dev in devices]
        self.assertIn("9900", vids)  # 0x26AC in dec
        self.assertIn("1155", vids)  # 0x0483 in dec

    def test_assemble_apk_tree_layout(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            fake_so = root / "libmp_app.so"
            fake_so.write_bytes(b"ELF shared object fixture")

            apk_root = bap.assemble_apk_tree(fake_so, root / "dist", "0.1.0", version_code=1, abi="arm64-v8a")
            self.assertTrue((apk_root / "AndroidManifest.xml").is_file())
            self.assertTrue((apk_root / "res" / "xml" / "device_filter.xml").is_file())
            self.assertTrue((apk_root / "lib" / "arm64-v8a" / "libmp_app.so").is_file())
            self.assertTrue((apk_root / "assets").is_dir())

    def test_assemble_apk_zip(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            fake_so = root / "libmp_app.so"
            fake_so.write_bytes(b"ELF shared object fixture")

            apk_root = bap.assemble_apk_tree(fake_so, root / "dist", "0.1.0")
            apk_path, sha256 = bap.assemble_apk_zip(apk_root, root / "dist" / "output.apk")
            self.assertTrue(apk_path.is_file())
            self.assertEqual(len(sha256), 64)


if __name__ == "__main__":
    unittest.main()
