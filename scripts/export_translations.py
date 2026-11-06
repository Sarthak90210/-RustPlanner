"""Export Mission Planner's translated UI resources for the Rust port.

For each culture offered by ConfigPlanner (CMB_language) this writes
crates/mp-app/assets/i18n/<culture>.json with:

* "forms": per exported Designer form (crates/mp-app/assets/forms/*.json), the translated
  `<control>.Text` values, exactly what ComponentResourceManager.ApplyResources applies. Each form is
  matched to its WinForms .resx by the overlap of control names.
* "text": English -> translated text, from every neutral/culture .resx pair (`*.Text`, and the
  Strings.resx keys). Hand-built views look their labels up here.

Usage: python scripts/export_translations.py
"""
import collections
import glob
import json
import os
import re
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.path.join(ROOT, "reference", "MissionPlanner")
OUT = os.path.join(ROOT, "crates", "mp-app", "assets", "i18n")
FORMS = os.path.join(ROOT, "crates", "mp-app", "assets", "forms")
# ConfigPlanner cultureCodes (en-US needs no table)
CULTURES = ["zh-Hans", "zh-TW", "ru-RU", "fr", "pl", "it-IT", "es-ES", "de-DE", "ja-JP", "id-ID", "ko-KR", "ar", "pt", "tr", "ru-KZ", "uk"]
# .NET resource fallback: a culture uses its own file, else its parent's
FALLBACK = {"zh-TW": ["zh-TW", "zh-Hant", "zh-tw"], "zh-Hans": ["zh-Hans", "zh-CN", "zh-cn"], "ru-RU": ["ru-RU", "ru"],
            "fr": ["fr"], "pl": ["pl"], "it-IT": ["it-IT", "it"], "es-ES": ["es-ES", "es"], "de-DE": ["de-DE", "de"],
            "ja-JP": ["ja-JP", "ja"], "id-ID": ["id-ID"], "ko-KR": ["ko-KR"], "ar": ["ar"], "pt": ["pt"], "tr": ["tr"],
            "ru-KZ": ["ru-KZ", "ru"], "uk": ["uk"]}


# crates/mp-app/assets/forms/<name>.json -> the Designer class it came from
FORM_CLASSES = {
    "adsb": "ConfigADSB", "airspeed": "ConfigHWAirspeed", "bluetooth": "ConfigHWBT", "compassmot": "ConfigCompassMot",
    "cubeid": "ConfigCubeID", "dronecan": "ConfigDroneCAN", "esp8266": "ConfigHWesp8266", "firmwarelegacy": "ConfigFirmware",
    "fwdisabled": "ConfigFirmwareDisabled", "georef": "georefimage", "gpsorder": "ConfigGPSOrder", "hwosd": "ConfigHWOSD",
    "initialparams": "ConfigInitialParams", "mount": "ConfigMount", "optflow": "ConfigHWOptFlow", "osd": "ConfigOSD",
    "parachute": "ConfigHWParachute", "paramloading": "ConfigParamLoading", "plane_tuning": "ConfigArduplane",
    "rangefinder": "ConfigHWRangeFinder", "repl": "ConfigREPL", "rover_tuning": "ConfigArdurover",
    "rtkinject": "ConfigSerialInjectGPS", "secureap": "ConfigSecureAP", "serial": "ConfigSerial", "sikradio": "Sikradio",
    "tracker": "ConfigAntennaTracker", "tracker_tuning": "ConfigAntennaTracker", "trackerui": "TrackerUI",
    "tradheli": "ConfigTradHeli4",
}


def read_resx(path):
    """name -> string value for <data> entries without a non-string type."""
    out = {}
    try:
        tree = ET.parse(path)
    except ET.ParseError:
        return out
    for data in tree.getroot().iter("data"):
        if data.get("type") or data.get("mimetype"):
            continue
        value = data.find("value")
        if value is not None and value.text is not None:
            out[data.get("name")] = value.text
    return out


def culture_of(path):
    m = re.match(r".*\.([A-Za-z]{2,3}(?:-[A-Za-z]+)*)\.resx$", path)
    return m.group(1) if m else None


def main():
    neutral = {}
    by_culture = collections.defaultdict(dict)  # base path -> culture -> entries
    for path in glob.glob(os.path.join(REF, "**", "*.resx"), recursive=True):
        if "Xamarin" in path or "uno" in path.lower():
            continue
        c = culture_of(path)
        if c and c.lower() not in ("designer",):
            base = path[: -len(c) - 6] + ".resx"
            by_culture[base][c] = path
        else:
            neutral[path] = read_resx(path)

    # Designer forms of the port and the WinForms class each was exported from
    form_resx = {}
    for name, cls in FORM_CLASSES.items():
        hits = [p for p in neutral if os.path.basename(p) == cls + ".resx"]
        if hits and os.path.exists(os.path.join(FORMS, name + ".json")):
            form_resx[name] = hits[0]

    os.makedirs(OUT, exist_ok=True)
    total = 0
    for culture in CULTURES:
        files = {}
        for base, cultures in by_culture.items():
            for c in FALLBACK[culture]:
                if c in cultures:
                    files[base] = read_resx(cultures[c])
                    break
        forms = {}
        for form, base in form_resx.items():
            tr = files.get(base, {})
            texts = {k[: -len(".Text")]: v for k, v in tr.items() if k.endswith(".Text") and neutral[base].get(k) not in (None, v)}
            if texts:
                forms[form] = texts
        votes = collections.defaultdict(collections.Counter)
        for base, tr in files.items():
            src = neutral.get(base, {})
            strings = os.path.basename(base) == "Strings.resx"
            for k, v in tr.items():
                english = src.get(k)
                if english is None or english == v or not english.strip():
                    continue
                if strings or k.endswith(".Text") or k.endswith(".HeaderText") or k.endswith(".ToolTipText"):
                    votes[english.strip()][v.strip()] += 1
        text = {e: c.most_common(1)[0][0] for e, c in sorted(votes.items())}
        with open(os.path.join(OUT, f"{culture}.json"), "w", encoding="utf-8") as fh:
            json.dump({"culture": culture, "forms": forms, "text": text}, fh, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
        total += len(text)
        print(f"{culture}: {len(text)} texts, {sum(len(v) for v in forms.values())} form controls in {len(forms)} forms")
    print("forms matched:", json.dumps(form_resx and {k: os.path.relpath(v, REF) for k, v in sorted(form_resx.items())}, indent=0))


if __name__ == "__main__":
    main()
