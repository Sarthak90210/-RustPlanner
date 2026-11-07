#!/usr/bin/env python3
"""Build Windows WiX installer (.msi) and portable ZIP distribution for Mission Planner Rust port.
Directly reproduces packaging structure from reference/MissionPlanner/wix/Program.cs and Drivers.cs.
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
import xml.etree.ElementTree as ET
import zipfile

UPGRADE_CODE = "{625389D7-EB3C-4d77-A5F6-A285CF99437D}"
MANUFACTURER = "Michael Oborne"
PRODUCT_NAME = "Mission Planner"
REG_ROOT_KEY = r"Software\MichaelOborne\MissionPlanner"

FILE_ASSOCIATIONS = [
    {
        "ext": "tlog",
        "id": "tlog_assoc",
        "description": "ArduPilot Telemetry Log",
        "icon_index": 0,
        "mime": "application/x-tlog",
    },
    {
        "ext": "bin",
        "id": "bin_assoc",
        "description": "ArduPilot DataFlash Binary Log",
        "icon_index": 0,
        "mime": "application/x-ardupilot-bin",
    },
    {
        "ext": "param",
        "id": "param_assoc",
        "description": "ArduPilot Parameter File",
        "icon_index": 0,
        "mime": "text/plain",
    },
    {
        "ext": "waypoints",
        "id": "wp_assoc",
        "description": "ArduPilot Waypoints File",
        "icon_index": 0,
        "mime": "text/plain",
    },
]


def generate_wix_source(
    version: str,
    executable_path: Path,
    include_drivers: bool = True,
    upgrade_code: str = UPGRADE_CODE,
) -> str:
    """Generates WiX .wxs XML source string matching reference/MissionPlanner/wix/Program.cs."""
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:\.\d+)?", version):
        raise ValueError(f"Invalid version format: {version}")

    exe_name = executable_path.name
    assoc_xml = []
    for a in FILE_ASSOCIATIONS:
        ext = a["ext"]
        desc = a["description"]
        mime = a["mime"]
        assoc_xml.append(f"""
        <ProgId Id="MissionPlanner.{ext}" Description="{desc}">
          <Extension Id="{ext}" ContentType="{mime}">
            <Verb Id="open" Command="Open with Mission Planner" TargetFile="MainExecutable" Argument='"%1"' />
          </Extension>
        </ProgId>""")

    assoc_str = "\n".join(assoc_xml)

    driver_actions = ""
    if include_drivers:
        driver_actions = """
    <!-- Device Driver Preinstallation actions matching C# wix/Drivers.cs -->
    <CustomAction Id="Install_signed_Driver86" Execute="deferred" Directory="INSTALLDIR"
                  ExeCommand='[INSTALLDIR]Drivers\\DPInstx86.exe /q /se' Return="ignore" Impersonate="no" />
    <CustomAction Id="Install_signed_Driver64" Execute="deferred" Directory="INSTALLDIR"
                  ExeCommand='[INSTALLDIR]Drivers\\DPInstx64.exe /q /se' Return="ignore" Impersonate="no" />
    <CustomAction Id="DriverCleanup" Execute="deferred" Directory="INSTALLDIR"
                  ExeCommand='[INSTALLDIR]Drivers\\DriverCleanup.exe' Return="ignore" Impersonate="no" />

    <InstallExecuteSequence>
      <Custom Action="Install_signed_Driver86" After="CreateShortcuts">NOT Installed AND NOT VersionNT64</Custom>
      <Custom Action="Install_signed_Driver64" After="CreateShortcuts">NOT Installed AND VersionNT64</Custom>
      <Custom Action="DriverCleanup" After="CreateShortcuts">NOT Installed</Custom>
    </InstallExecuteSequence>"""

    wxs = f"""<?xml version="1.0" encoding="utf-8"?>
<Wix xmlns="http://schemas.microsoft.com/wix/2006/wi">
  <Product Id="*" Name="{PRODUCT_NAME}" Language="1033" Version="{version}"
           Manufacturer="{MANUFACTURER}" UpgradeCode="{upgrade_code}">

    <Package Description="{PRODUCT_NAME} Installer" Comments="{PRODUCT_NAME} Installer"
             Manufacturer="{MANUFACTURER}" InstallerVersion="300" Compressed="yes" />

    <Upgrade Id="{upgrade_code}">
      <UpgradeVersion OnlyDetect="yes" Minimum="{version}" Property="NEWERVERSIONDETECTED" IncludeMinimum="no" />
      <UpgradeVersion OnlyDetect="no" Minimum="0.0.0" Maximum="{version}" Property="OLDERVERSIONBEINGUPGRADED" IncludeMinimum="yes" IncludeMaximum="yes" />
    </Upgrade>

    <InstallExecuteSequence>
      <RemoveExistingProducts After="InstallInitialize" />
    </InstallExecuteSequence>

    <Media Id="1" Cabinet="product.cab" EmbedCab="yes" />

    <Directory Id="TARGETDIR" Name="SourceDir">
      <Directory Id="ProgramFilesFolder" Name="PFiles">
        <Directory Id="INSTALLDIR" Name="{PRODUCT_NAME}">
          <Component Id="MainExecutableComponent" Guid="*">
            <File Id="MainExecutable" Source="{executable_path}" KeyPath="yes">
              <Shortcut Id="DesktopShortcut" Directory="DesktopFolder" Name="{PRODUCT_NAME}"
                        WorkingDirectory="INSTALLDIR" Advertise="yes" />
            </File>
            {assoc_str}
          </Component>
        </Directory>
      </Directory>

      <Directory Id="ProgramMenuFolder">
        <Directory Id="ApplicationProgramsFolder" Name="{PRODUCT_NAME}">
          <Component Id="ApplicationShortcutComponent" Guid="*">
            <Shortcut Id="StartMenuShortcut" Name="{PRODUCT_NAME}" Description="{PRODUCT_NAME}"
                      Target="[#MainExecutable]" WorkingDirectory="INSTALLDIR" />
            <Shortcut Id="UninstallShortcut" Name="Uninstall {PRODUCT_NAME}"
                      Target="[System64Folder]msiexec.exe" Arguments="/x [ProductCode]" />
            <RemoveFolder Id="RemoveApplicationProgramsFolder" Directory="ApplicationProgramsFolder" On="uninstall" />
            <RegistryValue Root="HKCU" Key="{REG_ROOT_KEY}" Name="installed" Type="integer" Value="1" KeyPath="yes" />
          </Component>
        </Directory>
      </Directory>

      <Directory Id="DesktopFolder" Name="Desktop" />
    </Directory>
{driver_actions}

    <Feature Id="Complete" Title="{PRODUCT_NAME}" Level="1">
      <ComponentRef Id="MainExecutableComponent" />
      <ComponentRef Id="ApplicationShortcutComponent" />
    </Feature>

    <Property Id="WIXUI_INSTALLDIR" Value="INSTALLDIR" />
  </Product>
</Wix>
"""
    return wxs


def assemble_portable_zip(
    executable_path: Path,
    output_dir: Path,
    version: str,
    extra_files: Optional[list] = None,
) -> Tuple[Path, str]:
    """Assembles a portable .zip distribution and computes its SHA-256 checksum."""
    if not executable_path.is_file():
        raise FileNotFoundError(f"Executable not found: {executable_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    zip_name = f"MissionPlanner-{version}-portable-windows.zip"
    zip_path = output_dir / zip_name

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.write(executable_path, arcname=executable_path.name)
        if extra_files:
            for ef in extra_files:
                p = Path(ef)
                if p.is_file():
                    zf.write(p, arcname=p.name)

    data = zip_path.read_bytes()
    sha256 = hashlib.sha256(data).hexdigest()

    manifest = {
        "package": zip_name,
        "version": version,
        "size_bytes": len(data),
        "sha256": sha256,
        "executable": executable_path.name,
    }
    manifest_path = output_dir / f"MissionPlanner-{version}-windows-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))

    return zip_path, sha256


def main():
    parser = argparse.ArgumentParser(description="Build Windows installer and portable packages")
    parser.add_argument("--version", default="0.1.0")
    parser.add_argument("--binary", type=Path, default=Path("target/release/missionplanner.exe"))
    parser.add_argument("--out", type=Path, default=Path("dist/windows"))
    parser.add_argument("--wix-only", action="store_true")

    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    wxs_content = generate_wix_source(args.version, args.binary)
    wxs_path = args.out / "installer.wxs"
    wxs_path.write_text(wxs_content)
    print(f"Generated WiX source: {wxs_path}")

    if not args.wix_only and args.binary.is_file():
        zip_path, sha = assemble_portable_zip(args.binary, args.out, args.version)
        print(f"Assembled portable zip: {zip_path} (SHA-256: {sha})")


if __name__ == "__main__":
    main()
