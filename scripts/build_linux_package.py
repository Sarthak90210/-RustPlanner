#!/usr/bin/env python3
"""Build Linux Debian (.deb) package and AppImage AppDir layout for Mission Planner Rust port.
Bundles desktop entry, icons, and ArduPilot/Pixhawk udev rules.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tarfile
from typing import Dict, List, Optional, Tuple

PACKAGE_NAME = "missionplanner-rs"
MAINTAINER = "ArduPilot Development Team <dev@ardupilot.org>"
DESCRIPTION = "ArduPilot Mission Planner Ground Control Station (Rust Port)"

UDEV_RULES_CONTENT = """# ArduPilot and Pixhawk flight controller USB device rules
# Allows non-root access for telemetry, configuration, and firmware updates

# 3DR / Hex / ProfiCNC Cube & Pixhawk
SUBSYSTEM=="usb", ATTRS{idVendor}=="26ac", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", MODE="0666", GROUP="plugdev"

# ArduPilot Pid.codes allocation
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", MODE="0666", GROUP="plugdev"

# STM32 Virtual COM Port (STMicroelectronics)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="plugdev"

# FTDI USB-to-Serial Adapters
SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="plugdev"

# Silicon Labs CP2102 / SiK Telemetry Radios
SUBSYSTEM=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="plugdev"
"""

DESKTOP_ENTRY_CONTENT = """[Desktop Entry]
Name=Mission Planner
GenericName=Ground Control Station
Comment=ArduPilot Mission Planner Ground Control Station
Exec=missionplanner %U
Icon=missionplanner
Terminal=false
Type=Application
Categories=Utility;Engineering;Aerospace;
MimeType=application/x-tlog;application/x-ardupilot-bin;
Keywords=ArduPilot;Drone;UAV;MAVLink;Flight;
"""


def generate_debian_control(version: str, arch: str = "amd64") -> str:
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:-\d+)?", version):
        raise ValueError(f"Invalid debian package version: {version}")
    if arch not in ["amd64", "arm64", "armhf"]:
        raise ValueError(f"Unsupported Debian architecture: {arch}")

    return f"""Package: {PACKAGE_NAME}
Version: {version}
Section: utils
Priority: optional
Architecture: {arch}
Maintainer: {MAINTAINER}
Depends: libc6 (>= 2.31), libasound2, libgl1, libx11-6, libudev1
Description: {DESCRIPTION}
 ArduPilot Mission Planner rewritten in pure, memory-safe Rust with
 full MAVLink telemetry, mission planning, and hardware integration.
"""


def assemble_debian_tree(
    binary_path: Path,
    output_dir: Path,
    version: str,
    arch: str = "amd64",
) -> Path:
    """Assembles the Debian package root directory structure."""
    if not binary_path.is_file():
        raise FileNotFoundError(f"Binary not found: {binary_path}")

    pkg_root = output_dir / f"{PACKAGE_NAME}_{version}_{arch}"
    if pkg_root.exists():
        shutil.rmtree(pkg_root)

    # 1. DEBIAN metadata
    deb_dir = pkg_root / "DEBIAN"
    deb_dir.mkdir(parents=True)
    (deb_dir / "control").write_text(generate_debian_control(version, arch))

    postinst = deb_dir / "postinst"
    postinst.write_text("#!/bin/sh\nset -e\nudevadm control --reload-rules || true\nudevadm trigger || true\nexit 0\n")

    # 2. Executable
    bin_dir = pkg_root / "usr" / "bin"
    bin_dir.mkdir(parents=True)
    shutil.copy2(binary_path, bin_dir / "missionplanner")

    # 3. Desktop file & icon
    apps_dir = pkg_root / "usr" / "share" / "applications"
    apps_dir.mkdir(parents=True)
    (apps_dir / "missionplanner.desktop").write_text(DESKTOP_ENTRY_CONTENT)

    icons_dir = pkg_root / "usr" / "share" / "pixmaps"
    icons_dir.mkdir(parents=True)
    (icons_dir / "missionplanner.png").write_bytes(b"\x89PNG\r\n\x1a\nfake_icon_payload")

    # 4. udev rules
    udev_dir = pkg_root / "lib" / "udev" / "rules.d"
    udev_dir.mkdir(parents=True)
    (udev_dir / "99-ardupilot.rules").write_text(UDEV_RULES_CONTENT)

    return pkg_root


def assemble_appimage_appdir(
    binary_path: Path,
    output_dir: Path,
) -> Path:
    """Assembles an AppImage AppDir structure."""
    if not binary_path.is_file():
        raise FileNotFoundError(f"Binary not found: {binary_path}")

    appdir = output_dir / "MissionPlanner.AppDir"
    if appdir.exists():
        shutil.rmtree(appdir)

    bin_dir = appdir / "usr" / "bin"
    bin_dir.mkdir(parents=True)
    shutil.copy2(binary_path, bin_dir / "missionplanner")

    (appdir / "missionplanner.desktop").write_text(DESKTOP_ENTRY_CONTENT)
    (appdir / "missionplanner.png").write_bytes(b"\x89PNG\r\n\x1a\nfake_icon_payload")

    apprun = appdir / "AppRun"
    apprun.write_text("""#!/bin/sh
SELF=$(readlink -f "$0")
HERE=${SELF%/*}
export PATH="${HERE}/usr/bin:${PATH}"
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH}"
exec "${HERE}/usr/bin/missionplanner" "$@"
""")

    return appdir


def assemble_tarball(pkg_root: Path, output_tar: Path) -> Tuple[Path, str]:
    """Packs directory into a .tar.gz archive and returns (path, sha256)."""
    output_tar.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output_tar, "w:gz") as tar:
        tar.add(pkg_root, arcname=pkg_root.name)

    data = output_tar.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()
    return output_tar, sha256


def main():
    parser = argparse.ArgumentParser(description="Build Linux Debian and AppImage distribution packages")
    parser.add_argument("--version", default="0.1.0")
    parser.add_argument("--binary", type=Path, default=Path("target/release/missionplanner"))
    parser.add_argument("--out", type=Path, default=Path("dist/linux"))
    parser.add_argument("--arch", default="amd64")

    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.binary.is_file():
        deb_tree = assemble_debian_tree(args.binary, args.out, args.version, args.arch)
        print(f"Assembled Debian directory: {deb_tree}")

        appdir = assemble_appimage_appdir(args.binary, args.out)
        print(f"Assembled AppDir: {appdir}")

        tar_path = args.out / f"missionplanner-{args.version}-linux-{args.arch}.tar.gz"
        assemble_tarball(deb_tree, tar_path)
        print(f"Packed Linux archive: {tar_path}")


if __name__ == "__main__":
    main()
