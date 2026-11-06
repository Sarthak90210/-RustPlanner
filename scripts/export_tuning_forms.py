"""Export the three vehicle tuning layouts and bindings from the local C# reference.

Run from the workspace root. Generated JSON is committed, so builds do not need C#.
"""
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'reference/MissionPlanner/GCSViews/ConfigurationView'
DEST = ROOT / 'crates/mp-app/assets/forms'
PAGES = [('plane_tuning', 'ConfigArduplane'), ('rover_tuning', 'ConfigArdurover'), ('tracker_tuning', 'ConfigAntennaTracker')]


def arguments(text):
    result, start, depth, quoted = [], 0, 0, False
    for i, char in enumerate(text):
        if char == '"' and (i == 0 or text[i - 1] != '\\'):
            quoted = not quoted
        if quoted:
            continue
        if char in '({[':
            depth += 1
        elif char in ')}]':
            depth -= 1
        elif char == ',' and depth == 0:
            result.append(text[start:i].strip())
            start = i + 1
    result.append(text[start:].strip())
    return result


def export(name, source_name):
    res = {n.attrib['name']: n.findtext('value', '') for n in ET.parse(SOURCE / (source_name + '.resx')).getroot().findall('data')}
    designer = (SOURCE / (source_name + '.Designer.cs')).read_text(encoding='utf-8-sig')
    code = (SOURCE / (source_name + '.cs')).read_text(encoding='utf-8-sig')
    parents = {child: parent for parent, child in re.findall(r'this\.(\w+)\.Controls\.Add\(this\.(\w+)\)', designer)}
    bindings = {}
    for control, args in re.findall(r'(\w+)\.setup\((.*?)\);', code, re.S):
        args = arguments(args)
        aliases = []
        for arg in args:
            for param in re.findall(r'"([A-Z][A-Z0-9_]+)"', arg):
                if param not in aliases:
                    aliases.append(param)
        if not aliases:
            raise ValueError(f'{source_name}.{control}: setup without a literal parameter binding')
        nums = []
        for arg in args:
            try:
                nums.append(float(arg.rstrip('fFdDmM')))
            except ValueError:
                break
        bindings[control] = (aliases, nums)

    kinds = {'Label': 'label', 'LinkLabel': 'link', 'GroupBox': 'group', 'Panel': 'panel',
             'MyButton': 'button', 'Button': 'button', 'MavlinkNumericUpDown': 'mnum',
             'MavlinkComboBox': 'mcombo', 'MavlinkCheckBox': 'mcheck', 'CheckBox': 'check',
             'NumericUpDown': 'num', 'ComboBox': 'combo', 'TextBox': 'text',
             'HorizontalProgressBar2': 'progress', 'HorizontalProgressBar': 'progress', 'MyTrackBar': 'track'}
    controls = []
    for control, qualified in re.findall(r'this\.(\w+)\s*=\s*new ([\w.]+)\(', designer):
        ctype = qualified.rsplit('.', 1)[-1]
        if ctype in ['Container', 'ToolTip', 'Timer']:
            continue
        if ctype not in kinds:
            raise ValueError(f'Unsupported control: {source_name}.{control}: {ctype}')
        def pair(prop, default):
            return [float(x.strip()) for x in res.get(control + '.' + prop, default).split(',')]
        x, y = pair('Location', '0,0')
        w, h = pair('Size', '0,0')
        data = dict(name=control, kind=kinds[ctype], parent=res.get('>>' + control + '.Parent', parents.get(control, '$this')), x=x, y=y, w=w, h=h)
        if ctype == 'MyTrackBar' and (res.get(control + '.Orientation') == 'Vertical' or re.search(r'this\.' + re.escape(control) + r'\.Orientation\s*=\s*System\.Windows\.Forms\.Orientation\.Vertical', designer)):
            data['kind'] = 'vtrack'
        if control + '.Text' in res:
            data['text'] = res[control + '.Text']
        if res.get(control + '.Visible') == 'False':
            data['hidden'] = True
        if control in bindings:
            aliases, nums = bindings[control]
            data['param'] = aliases[0]
            data['aliases'] = aliases
            if nums:
                data['nums'] = nums
        font = res.get(control + '.Font', '')
        if font:
            match = re.search(r',\s*([\d.]+)pt', font)
            if match:
                data['font'] = float(match[1])
            if 'Bold' in font:
                data['bold'] = True
        controls.append(data)
    size = [float(x) for x in res['$this.Size'].split(',')]
    output = dict(name=name, w=size[0], h=size[1], controls=controls)
    (DEST / (name + '.json')).write_text(json.dumps(output, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'{name}: {len(controls)} controls, {len(bindings)} parameter bindings')


if __name__ == '__main__':
    for name, source in PAGES:
        export(name, source)
