#!/usr/bin/env python3
"""File-by-file port manifest for the Mission Planner -> Rust rewrite.

Every C# file under reference/MissionPlanner gets exactly one row:

    <C# path> -> <Rust path> : <status>

The Rust path is derived mechanically from the C# path, so the Rust module tree mirrors
the C# project tree (and therefore its wiring):

  * MissionPlanner project files (MainV2.cs, GCSViews/, Controls/, Log/, ...)
        -> crates/mp-app/src/mp/<lowercased path>.rs
  * ExtLibs/<Lib>/...  -> crates/mp-core/src/extlibs/<lib>/...     (non-UI libraries)
                       -> crates/mp-app/src/extlibs/<lib>/...      (UI libraries)

Lowercasing: every path segment is lowercased and every non [a-z0-9] character becomes
'_'. `Foo.Designer.cs` therefore maps to `foo_designer.rs`, next to `foo.rs`.

A file counts as ported only when the mapped Rust file exists and its first line is

    //! Port of `<C# path relative to reference/MissionPlanner>`

Statuses:
  ported      mapped Rust file exists with the header
  todo        Mission Planner code still to be rewritten
  vendored    a third-party library bundled into the C# tree; the Rust build uses the
              listed crate instead (the row names it), so the file is not transliterated
  generated   code emitted by a generator from message definitions; the Rust side
              generates the same code from the same definitions (the row names how)
  n/a         build/packaging glue with no runtime behaviour (AssemblyInfo, csproj
              resources, installers, test fixtures for C#-only tooling)

Usage:
  python scripts/port_manifest.py            # writes port/MANIFEST.md and port/manifest.json
  python scripts/port_manifest.py --check    # exit 1 if any ported file lacks its header
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REF = ROOT / "reference" / "MissionPlanner"
OUT = ROOT / "port"

RUST_KEYWORDS = {
    "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn", "for",
    "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return",
    "self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use", "where",
    "while", "async", "await", "dyn", "abstract", "become", "box", "do", "final", "macro",
    "override", "priv", "typeof", "unsized", "virtual", "yield", "try", "lib", "main",
}

# Third-party libraries vendored into ExtLibs, and the Rust crate that replaces each one.
VENDORED = {
    "alglibnet": "nalgebra (only the solvers Mission Planner calls are ported, in utilities/)",
    "netDxf": "dxf",
    "ZedGraph": "egui_plot",
    "ObjectListView": "egui_extras::TableBuilder",
    "DirectShowLib": "nokhwa / gstreamer",
    "ICSharpCode.SharpZipLib": "zip + flate2",
    "GMap.NET.Core": "walkers-style tile engine in mp-app/src/map.rs",
    "GMap.NET.WindowsForms": "egui map widget in mp-app/src/map.rs",
    "GMap.NET.Drawing": "egui painter",
    "BaseClasses": "nokhwa / gstreamer",
    "MetaDataExtractorCSharp240d": "kamadak-exif",
    "SvgNet": "resvg",
    "SharpKml": "kml",
    "KMLib": "kml",
    "CsAssortedWidgets": "egui",
    "BSE.Windows.Forms": "egui",
    "ProjNet": "proj4rs",
    "GeoUtility": "utm / proj4rs",
    "SharpAdbClient": "adb_client",
    "zlib.net": "flate2",
    "LibVLC.NET": "gstreamer",
    "Xamarin.Forms.Platform.WinForms": "eframe",
    "SkiaSharp.Views.Forms.WinForms": "eframe",
    "7zip": "lzma-rs",
    "WinUSBNet": "rusb",
    "LibTessDotNet": "lyon_tessellation",
    "Exocortex.DSP": "rustfft",
    "Zeroconf": "mdns-sd",
    "GLControl": "eframe (wgpu/glow)",
    "AviFile": "gstreamer",
    "Transitions": "egui animation",
    "ManagedNativeWifi.Simple": "platform wifi APIs",
    "UsbSerialForAndroid": "android serial via JNI",
    "GeoidHeightsDotNet": "geoid tables in mp-core",
    "System.Drawing.android": "egui painter",
    "MissionPlanner.Drawing": "egui painter",
    "MissionPlanner.Drawing.Common": "egui painter",
    "uno": "eframe",
    "wasm": "eframe wasm32 target",
    "Onvif": "reqwest + quick-xml",
    "GDAL": "gdal",
    "Interfaces": "eframe",
}

# Directories that are build, packaging or C#-only test glue.
NOT_APPLICABLE_DIRS = {
    "wix", "Msi", "Installer", "Properties", "MissionPlannerTests", "resedit", "Updater",
    "pinvokegen", "Benchmark", "Mock", "MockDroneID", "SimpleExample", "TestPlugin",
    "DriverCleanup", "md5sum", "ParameterMetaDataGenerator", "tlogThumbnailHandler",
    "MissionPlanner.Stats", "WindowsStore", "mono", "Linux", "MAC",
}

# ExtLibs that need egui; everything else in ExtLibs lands in mp-core.
UI_EXTLIBS = {
    "Controls", "Maps", "Xamarin", "ImageVisualizer", "LEDBulb", "OSDConfigurator",
    "GStreamerHud", "TrackerHome", "SimpleGrid", "MissionPlanner.Gridv2", "Antenna",
    "WebCamService", "Flasher", "ExtGuided", "MavlinkMessagePlugin", "HIL", "solo", "Strings",
}


def seg(s: str) -> str:
    s = re.sub(r"[^a-z0-9]", "_", s.lower())
    s = re.sub(r"_+", "_", s).strip("_") or "_"
    if s[0].isdigit():
        s = "_" + s
    if s in RUST_KEYWORDS:
        s += "_"
    return s


def classify(rel: Path) -> tuple[str, str, str]:
    """Return (rust_path, status_if_not_ported, note)."""
    parts = list(rel.parts)
    stem = parts[-1][:-3]  # drop .cs
    rs_parts = [seg(p) for p in parts[:-1]] + [seg(stem) + ".rs"]
    note = ""
    if parts[0] == "ExtLibs":
        lib = parts[1]
        crate = "mp-app" if lib in UI_EXTLIBS else "mp-core"
        rust = Path("crates", crate, "src", *rs_parts)
        if lib == "Mavlink" and parts[2:4] == ["pymavlink", "generator"]:
            return str(rust).replace("\\", "/"), "n/a",                 "template copy inside the generator; ExtLibs/Mavlink/*.cs are the shipped versions"
        if lib == "Mavlink" and stem == "Mavlink":
            return str(rust).replace("\\", "/"), "generated",                 "emitted by crates/mp-core/build.rs, a port of pymavlink mavgen_cs.py, from the same message_definitions XML"
        if lib == "DroneCAN" and len(parts) > 2 and parts[2] == "out":
            return str(rust).replace("\\", "/"), "generated",                 "emitted by canard_dsdlc from DSDL; port the generator, not its output"
        if lib in VENDORED:
            return str(rust).replace("\\", "/"), "vendored", VENDORED[lib]
        if lib in NOT_APPLICABLE_DIRS or any(p in NOT_APPLICABLE_DIRS for p in parts[2:-1]):
            return str(rust).replace("\\", "/"), "n/a", "build/packaging glue"
    else:
        rust = Path("crates", "mp-app", "src", "mp", *rs_parts)
        if any(p in NOT_APPLICABLE_DIRS for p in parts[:-1]):
            return str(rust).replace("\\", "/"), "n/a", "build/packaging glue"
    if stem in ("AssemblyInfo", "GlobalSuppressions") or stem.endswith("Resources.Designer") \
            or stem.endswith("Settings.Designer"):
        return str(rust).replace("\\", "/"), "n/a", "assembly metadata"
    return str(rust).replace("\\", "/"), "todo", note


def header_ok(path: Path, rel: str) -> bool:
    try:
        with path.open(encoding="utf-8") as f:
            first = f.readline().strip()
    except OSError:
        return False
    return first == f"//! Port of `{rel}`"


def main() -> int:
    rows = []
    bad = []
    for cs in sorted(REF.rglob("*.cs")):
        rel = cs.relative_to(REF)
        rel_s = str(rel).replace("\\", "/")
        rust, status, note = classify(rel)
        rust_abs = ROOT / rust
        lines = sum(1 for _ in cs.open(encoding="utf-8", errors="replace"))
        if rust_abs.exists():
            if not header_ok(rust_abs, rel_s):
                bad.append(rust)
            elif status == "todo":
                status = "ported"
            else:
                # generated output reproduced by a ported generator: not owned code
                note = (note + "; " if note else "") + "Rust counterpart present"
        rows.append({"cs": rel_s, "rust": rust, "status": status, "lines": lines, "note": note})

    if "--check" in sys.argv:
        for b in bad:
            print(f"missing or wrong header: {b}")
        return 1 if bad else 0

    OUT.mkdir(exist_ok=True)
    (OUT / "manifest.json").write_text(json.dumps(rows, indent=1) + "\n", encoding="utf-8")

    def tally(key):
        out = {}
        for r in rows:
            k = key(r)
            c = out.setdefault(k, {"files": 0, "lines": 0})
            c["files"] += 1
            c["lines"] += r["lines"]
        return out

    total = tally(lambda r: r["status"])
    owned = [r for r in rows if r["status"] in ("ported", "todo")]
    done = [r for r in owned if r["status"] == "ported"]
    md = ["# Mission Planner -> Rust port manifest", "",
          "Generated by `scripts/port_manifest.py`. Do not edit by hand.", "",
          f"Mission Planner-owned files ported: **{len(done)} / {len(owned)}** "
          f"({sum(r['lines'] for r in done)} / {sum(r['lines'] for r in owned)} C# lines)", "",
          "| status | files | C# lines |", "|---|---:|---:|"]
    for k in ("ported", "todo", "generated", "vendored", "n/a"):
        c = total.get(k, {"files": 0, "lines": 0})
        md.append(f"| {k} | {c['files']} | {c['lines']} |")
    md += ["", "## By directory (Mission Planner-owned files only)", "",
           "| directory | ported | todo |", "|---|---:|---:|"]
    by_dir = {}
    for r in owned:
        p = r["cs"].split("/")
        d = "/".join(p[:2]) if p[0] == "ExtLibs" and len(p) > 2 else (p[0] if len(p) > 1 else "(root)")
        e = by_dir.setdefault(d, [0, 0])
        e[0 if r["status"] == "ported" else 1] += 1
    for d in sorted(by_dir):
        md.append(f"| {d} | {by_dir[d][0]} | {by_dir[d][1]} |")
    md += ["", "## Files", "", "| C# file | lines | Rust file | status | note |", "|---|---:|---|---|---|"]
    for r in rows:
        md.append(f"| `{r['cs']}` | {r['lines']} | `{r['rust']}` | {r['status']} | {r['note']} |")
    (OUT / "MANIFEST.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"ported {len(done)}/{len(owned)} owned files; vendored {total.get('vendored', {}).get('files', 0)}; "
          f"generated {total.get('generated', {}).get('files', 0)}; n/a {total.get('n/a', {}).get('files', 0)}")
    for b in bad:
        print(f"WARNING missing or wrong header: {b}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
