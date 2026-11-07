#!/usr/bin/env python3
"""Unit tests for the unified cross-platform release pipeline."""

from pathlib import Path
import tempfile
import unittest

import package_release as pr


class ReleasePackagingPipelineTests(unittest.TestCase):
    def test_package_all_targets_generates_manifest_and_all_platforms(self):
        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td) / "release"
            manifest = pr.package_all_targets("0.1.0", "abcdef1234567890", out_dir, mock_binaries=True)

            self.assertEqual(manifest["version"], "0.1.0")
            self.assertEqual(manifest["commit"], "abcdef1234567890")
            self.assertEqual(manifest["artifacts_count"], 9)

            manifest_file = out_dir / "release-manifest.json"
            self.assertTrue(manifest_file.is_file())

            platforms = {a["platform"] for a in manifest["artifacts"]}
            self.assertEqual(platforms, {"windows", "linux", "macos", "android", "ios"})

            # Check that files exist on disk
            for a in manifest["artifacts"]:
                p = Path(a["path"])
                self.assertTrue(p.exists(), f"Artifact missing on disk: {p}")

    def test_manifest_checksums_match_actual_files(self):
        with tempfile.TemporaryDirectory() as td:
            out_dir = Path(td) / "release"
            manifest = pr.package_all_targets("0.1.0", "HEAD", out_dir, mock_binaries=True)

            for a in manifest["artifacts"]:
                if "sha256" in a:
                    p = Path(a["path"])
                    import hashlib
                    actual_sha = hashlib.sha256(p.read_bytes()).hexdigest()
                    self.assertEqual(a["sha256"], actual_sha)


if __name__ == "__main__":
    unittest.main()
