#!/usr/bin/env python3
"""Extract a digest of Mission Planner's generated ExtLibs/Mavlink/Mavlink.cs.

The reference tree is not committed, so the Rust tests compare the generated MAVLink layer
with this digest instead: every message_info row, every enum (underlying type, [Flags],
members with value, [Description], [Obsolete], [hasLocation()]) and every struct field
(C# type, array size, [Units], [Description]), in declaration order.

Usage: python scripts/mavlink_cs_digest.py  ->  crates/mp-core/tests/fixtures/mavlink_cs_digest.json.gz
"""
import gzip
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "reference" / "MissionPlanner" / "ExtLibs" / "Mavlink" / "Mavlink.cs"
OUT = ROOT / "crates" / "mp-core" / "tests" / "fixtures" / "mavlink_cs_digest.json.gz"


def main():
    cs = SRC.read_text(encoding="utf-8-sig")
    digest = {"build_date": re.search(r'MAVLINK_BUILD_DATE = "([^"]*)"', cs).group(1)}

    digest["message_infos"] = [
        [int(m[0]), m[1], int(m[2]), int(m[3]), int(m[4]), m[5]]
        for m in re.findall(r'new message_info\((\d+), "([^"]+)", (\d+), (\d+), (\d+), typeof\( (\w+) \)\)', cs)
    ]

    enums = []
    for m in re.finditer(r"(\[Flags\]\s*)?public enum (\w+)\s*:\s*([\w /*]+?)\s*\{(.*?)\n    \};", cs, re.S):
        flags, name, base, body = m.group(1) is not None, m.group(2), m.group(3), m.group(4)
        base = base.split("/*")[0].strip()
        members = []
        for e in re.finditer(r'\[Description\("(.*?)"\)\]\s*((?:\[\w+(?:\(\))?\]\s*)*)(\w+)=(-?\d+),', body, re.S):
            attrs = e.group(2)
            members.append([e.group(3), int(e.group(4)), e.group(1), "[Obsolete]" in attrs, "[hasLocation()]" in attrs])
        enums.append({"name": name, "base": base, "flags": flags, "members": members})
    digest["enums"] = enums

    structs = []
    for m in re.finditer(r"public struct (mavlink_\w+_t)\s*\{(.*?)\n    \};", cs, re.S):
        fields = []
        for f in re.finditer(
            r'\[Units\("(.*?)"\)\]\s*\[Description\("(.*?)"\)\]\s*//\[FieldOffset\((\d+)\)\]\s*'
            r'(?:\[MarshalAs\(UnmanagedType\.ByValArray,SizeConst=(\d+)\)\]\s*)?public\s+(?:/\*\w+\*/)?([\w\[\]]+) @?(\w+);',
            m.group(2), re.S,
        ):
            fields.append([f.group(6), f.group(5), int(f.group(4) or 0), int(f.group(3)), f.group(1), f.group(2)])
        prev = cs.rfind("};", 0, m.start())
        obsolete = "[Obsolete]" in cs[prev:m.start()]
        structs.append({"name": m.group(1), "obsolete": obsolete, "fields": fields})
    digest["structs"] = structs

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(OUT, "wt", encoding="utf-8", compresslevel=9) as f:
        json.dump(digest, f, ensure_ascii=False, separators=(",", ":"))
    print(f"{len(digest['message_infos'])} messages, {len(enums)} enums, {len(structs)} structs -> {OUT}")


if __name__ == "__main__":
    main()
