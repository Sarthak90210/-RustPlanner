#!/usr/bin/env python3
"""Unit tests for Windows WiX installer generation and portable packaging."""

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import build_windows_installer as bwi


class WindowsPackagingTests(unittest.TestCase):
    def test_wix_source_structure_and_guid_consistency(self):
        with tempfile.TemporaryDirectory() as td:
            fake_exe = Path(td) / "missionplanner.exe"
            fake_exe.write_bytes(b"fake PE binary")

            wxs = bwi.generate_wix_source("0.1.0", fake_exe)
            root = ET.fromstring(wxs)

            # Namespace check
            self.assertTrue(root.tag.endswith("Wix"))
            product = root.find("{http://schemas.microsoft.com/wix/2006/wi}Product")
            self.assertIsNotNone(product)
            self.assertEqual(product.attrib["UpgradeCode"], bwi.UPGRADE_CODE)
            self.assertEqual(product.attrib["Manufacturer"], bwi.MANUFACTURER)
            self.assertEqual(product.attrib["Version"], "0.1.0")

    def test_file_associations_included(self):
        with tempfile.TemporaryDirectory() as td:
            fake_exe = Path(td) / "missionplanner.exe"
            fake_exe.write_bytes(b"fake PE binary")

            wxs = bwi.generate_wix_source("0.1.0", fake_exe)
            for assoc in bwi.FILE_ASSOCIATIONS:
                ext = assoc["ext"]
                self.assertIn(f'Extension Id="{ext}"', wxs)
                self.assertIn(f'ProgId Id="MissionPlanner.{ext}"', wxs)

    def test_invalid_version_rejected(self):
        with self.assertRaises(ValueError):
            bwi.generate_wix_source("invalid_version", Path("fake.exe"))

    def test_assemble_portable_zip_creates_archive_and_manifest(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            fake_exe = root / "missionplanner.exe"
            fake_exe.write_bytes(b"executable payload bytes")

            out_dir = root / "dist"
            zip_path, sha256 = bwi.assemble_portable_zip(fake_exe, out_dir, "0.1.0")

            self.assertTrue(zip_path.is_file())
            self.assertTrue(len(sha256) == 64)

            manifest_path = out_dir / "MissionPlanner-0.1.0-windows-manifest.json"
            self.assertTrue(manifest_path.is_file())
            self.assertIn(sha256, manifest_path.read_text())

    def test_type_hints_resolve_cleanly(self):
        import typing
        hints = typing.get_type_hints(bwi.assemble_portable_zip)
        self.assertIn("return", hints)


if __name__ == "__main__":
    unittest.main()

