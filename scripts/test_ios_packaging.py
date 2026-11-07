"""Host-independent checks. These do not substitute for the macOS build/smoke jobs."""

import json
from pathlib import Path
import plistlib
import struct
import tempfile
import unittest
import zlib

import build_ios
import smoke_ios_simulator


class PackagingTests(unittest.TestCase):
    def test_rebuild_invalidates_prior_success_but_preserves_unrelated_files(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ["build.json", "missionplanner-ios-simulator.zip", "unrelated.txt"]:
                (root / name).write_text("old")
            (root / "MissionPlanner.app").mkdir()
            (root / "MissionPlanner.app" / "missionplanner").write_text("old executable")
            build_ios.invalidate_artifacts(root, "simulator")
            self.assertEqual([p.name for p in root.iterdir()], ["unrelated.txt"])

    def test_device_and_simulator_bundles_keep_distinct_platforms(self):
        for target, supported in [("aarch64-apple-ios", "iPhoneOS"),
                                  ("aarch64-apple-ios-sim", "iPhoneSimulator"),
                                  ("x86_64-apple-ios", "iPhoneSimulator")]:
            with self.subTest(target=target), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                binary = root / "input-binary"
                binary.write_bytes(b"fixture, not a Mach-O executable")
                info = build_ios.info_plist("0.1.0", "42", "15.0", target)
                app = build_ios.assemble_bundle(binary, root, info)
                with (app / "Info.plist").open("rb") as handle:
                    manifest = plistlib.load(handle)
                self.assertEqual(manifest["CFBundleSupportedPlatforms"], [supported])
                self.assertEqual(manifest["CFBundlePackageType"], "APPL")
                self.assertEqual((app / manifest["CFBundleExecutable"]).read_bytes(), binary.read_bytes())
                self.assertTrue(manifest["NSLocalNetworkUsageDescription"])
                self.assertTrue(manifest["UIFileSharingEnabled"])
                self.assertEqual("UIRequiredDeviceCapabilities" in manifest, supported == "iPhoneOS")
                with self.assertRaises(FileExistsError):
                    build_ios.assemble_bundle(binary, root, info)

    def test_invalid_build_metadata_is_rejected(self):
        for args in [("0.1.0", "banana", "15.0"), ("0.1.0", "1", "14.0"),
                     ("bad", "1", "15.0"), ("0.1.0", "1", "15..0")]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                build_ios.info_plist(*args, "aarch64-apple-ios")
        with self.assertRaises(ValueError):
            build_ios.resolve_target("aarch64-apple-darwin")

    def test_simulator_selection_rejects_unavailable_and_non_ios_devices(self):
        def phone(available=True):
            return {"name": "iPhone 16", "isAvailable": available, "deviceTypeIdentifier": "phone"}
        inventory = {"devices": {
            "com.apple.CoreSimulator.SimRuntime.tvOS-99-0": [phone()],
            "com.apple.CoreSimulator.SimRuntime.iOS-99-0": [phone(False)],
            "com.apple.CoreSimulator.SimRuntime.iOS-18-2": [phone()],
            "com.apple.CoreSimulator.SimRuntime.iOS-18-4": [phone()],
        }}
        self.assertEqual(smoke_ios_simulator.select_device(inventory),
                         ("com.apple.CoreSimulator.SimRuntime.iOS-18-4", "phone"))
        with self.assertRaises(RuntimeError):
            smoke_ios_simulator.select_device({"devices": {}})

    def test_smoke_receipt_requires_matching_renderer_image_and_view(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            def chunk(kind, data):
                return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
            image = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 300, 400, 8, 6, 0, 0, 0))
            image += chunk(b"IDAT", zlib.compress((b"\x00" + b"\x55\x66\x77\xff" * 300) * 400))
            image += chunk(b"IEND", b"")
            (root / "startup.png").write_bytes(image)
            receipt = {"status": "ok", "source": "eframe renderer viewport", "width": 300,
                       "height": 400, "runtime": {"view": "FlightData"}}
            def write():
                (root / "startup.capture.json").write_text(json.dumps(receipt))
            write()
            smoke_ios_simulator.verify_capture(root)
            for key, value in [("width", 100), ("source", "simctl screenshot"), ("status", "failed"),
                               ("runtime", {"view": "Help"})]:
                original = receipt[key]
                receipt[key] = value
                write()
                with self.subTest(key=key), self.assertRaises(RuntimeError):
                    smoke_ios_simulator.verify_capture(root)
                receipt[key] = original
            write()
            corrupt = bytearray(image)
            corrupt[-5] ^= 1
            for invalid in [b"not a PNG", image[:24], image[:-12], bytes(corrupt),
                            image[:33] + chunk(b"IDAT", zlib.compress(b"truncated pixels")) + chunk(b"IEND", b"")]:
                (root / "startup.png").write_bytes(invalid)
                with self.assertRaises(RuntimeError):
                    smoke_ios_simulator.verify_capture(root)

    def test_committed_smoke_receipts_pass_verification(self):
        fixture_dir = Path(__file__).resolve().parent / "fixtures" / "ios_smoke"
        self.assertTrue(fixture_dir.exists(), "scripts/fixtures/ios_smoke directory must exist")
        receipt = smoke_ios_simulator.verify_capture(fixture_dir)
        self.assertEqual(receipt.get("status"), "ok")
        self.assertEqual(receipt.get("runtime", {}).get("view"), "FlightData")
        smoke_meta = json.loads((fixture_dir / "smoke.json").read_text())
        self.assertEqual(smoke_meta.get("status"), "ok")
        self.assertTrue(smoke_meta.get("alive_after_display_capture"))


if __name__ == "__main__":
    unittest.main()
