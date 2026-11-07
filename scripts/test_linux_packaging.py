#!/usr/bin/env python3
"""Unit tests for Linux packaging script."""

from pathlib import Path
import tempfile
import unittest

import build_linux_package as blp


class LinuxPackagingTests(unittest.TestCase):
    def test_debian_control_structure(self):
        ctrl = blp.generate_debian_control("0.1.0", "amd64")
        self.assertIn("Package: missionplanner-rs", ctrl)
        self.assertIn("Architecture: amd64", ctrl)
        self.assertIn("libudev1", ctrl)

    def test_invalid_debian_arch_or_version_rejected(self):
        with self.assertRaises(ValueError):
            blp.generate_debian_control("0.1.0", "riscv64")
        with self.assertRaises(ValueError):
            blp.generate_debian_control("bad.version", "amd64")

    def test_assemble_debian_tree_layout(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            binary = root / "missionplanner"
            binary.write_bytes(b"ELF executable fixture")

            deb_tree = blp.assemble_debian_tree(binary, root / "dist", "0.1.0", "amd64")
            self.assertTrue((deb_tree / "DEBIAN" / "control").is_file())
            self.assertTrue((deb_tree / "usr" / "bin" / "missionplanner").is_file())
            self.assertTrue((deb_tree / "usr" / "share" / "applications" / "missionplanner.desktop").is_file())
            self.assertTrue((deb_tree / "lib" / "udev" / "rules.d" / "99-ardupilot.rules").is_file())

            rules = (deb_tree / "lib" / "udev" / "rules.d" / "99-ardupilot.rules").read_text()
            self.assertIn("26ac", rules)  # Pixhawk
            self.assertIn("10c4", rules)  # SiK radio

    def test_assemble_appimage_appdir_layout(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            binary = root / "missionplanner"
            binary.write_bytes(b"ELF executable fixture")

            appdir = blp.assemble_appimage_appdir(binary, root / "dist")
            self.assertTrue((appdir / "AppRun").is_file())
            self.assertTrue((appdir / "missionplanner.desktop").is_file())
            self.assertTrue((appdir / "usr" / "bin" / "missionplanner").is_file())

    def test_assemble_tarball(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            binary = root / "missionplanner"
            binary.write_bytes(b"ELF executable fixture")

            deb_tree = blp.assemble_debian_tree(binary, root / "dist", "0.1.0", "amd64")
            tar_path, sha256 = blp.assemble_tarball(deb_tree, root / "dist" / "output.tar.gz")
            self.assertTrue(tar_path.is_file())
            self.assertEqual(len(sha256), 64)


if __name__ == "__main__":
    unittest.main()
