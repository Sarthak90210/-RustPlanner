//! Generates the MAVLink message layer that Mission Planner keeps in `ExtLibs/Mavlink/Mavlink.cs`.
//!
//! Mission Planner produces that file with its forked pymavlink generator:
//!
//! ```text
//! python -m pymavlink.tools.mavgen --lang=CS --wire-protocol=2.0 \
//!     message_definitions\all.xml message_definitions\offspec.xml
//! ```
//!
//! This build script is a port of the parts of `pymavlink/generator/mavparse.py`, `mavgen.py`
//! and `mavgen_cs.py` that decide the output: XML parsing, include expansion, enum merging,
//! field ordering, wire offsets, CRC_EXTRA, the `MAVLINK_MESSAGE_INFOS` table and its order,
//! enum entry renaming (`NAV_` removal, enum prefix removal, leading digit underscore), enum
//! underlying types, `[Flags]`, `[Obsolete]`, `[hasLocation()]` and the `[Units]` /
//! `[Description]` attributes. The emitted Rust lives in `$OUT_DIR/mavlink_generated.rs` and is
//! included by `src/extlibs/mavlink/mavlink.rs`.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

const PROTOCOL_MARKER: u8 = 0xFD; // wire protocol 2.0

#[derive(Clone)]
struct MavField {
    name: String,
    ty: String, // normalised C type, e.g. "uint8_t"
    type_length: usize,
    array_length: usize,
    wire_length: usize,
    wire_offset: usize,
    enum_: String,
    display: String,
    units: String,
    description: String,
}

struct MavType {
    name: String,
    id: u32,
    description: String,
    fields: Vec<MavField>,
    extensions_start: Option<usize>,
    deprecated: bool,
    ordered_fields: Vec<MavField>,
    wire_length: usize,
    wire_min_length: usize,
    crc_extra: u8,
}

#[derive(Clone)]
struct MavEnumEntry {
    name: String,
    value: i64,
    description: String,
    end_marker: bool,
    autovalue: bool,
    deprecated: bool,
    has_location: bool,
}

struct MavEnum {
    name: String,
    description: String,
    entry: Vec<MavEnumEntry>,
    start_value: Option<i64>,
    highest_value: i64,
}

struct MavXml {
    filename: PathBuf,
    message: Vec<MavType>,
    enums: Vec<MavEnum>,
    version: u32,
    include: Vec<String>,
    message_crcs: HashMap<u32, u8>,
    message_lengths: HashMap<u32, usize>,
    message_min_lengths: HashMap<u32, usize>,
    message_names: HashMap<u32, String>,
    largest_payload: usize,
}

fn type_length(t: &str) -> Option<usize> {
    Some(match t {
        "float" => 4,
        "double" => 8,
        "char" => 1,
        "int8_t" => 1,
        "uint8_t" => 1,
        "uint8_t_mavlink_version" => 1,
        "int16_t" => 2,
        "uint16_t" => 2,
        "int32_t" => 4,
        "uint32_t" => 4,
        "int64_t" => 8,
        "uint64_t" => 8,
        _ => return None,
    })
}

/// mavgen_cs.py `map`: C type -> C# type.
fn cs_type(t: &str) -> &'static str {
    match t {
        "float" => "float",
        "double" => "double",
        "char" => "byte",
        "int8_t" => "sbyte",
        "uint8_t" => "byte",
        "int16_t" => "short",
        "uint16_t" => "ushort",
        "int32_t" => "int",
        "uint32_t" => "uint",
        "int64_t" => "long",
        "uint64_t" => "ulong",
        other => panic!("unknown type {other}"),
    }
}

/// C# type -> Rust type.
fn rs_type(cs: &str) -> &'static str {
    match cs {
        "float" => "f32",
        "double" => "f64",
        "byte" => "u8",
        "sbyte" => "i8",
        "short" => "i16",
        "ushort" => "u16",
        "int" => "i32",
        "uint" => "u32",
        "long" => "i64",
        "ulong" => "u64",
        other => panic!("unknown C# type {other}"),
    }
}

fn x25_accumulate(crc: u16, bytes: &[u8]) -> u16 {
    let mut accum = crc;
    for &b in bytes {
        let mut tmp = b ^ (accum & 0xff) as u8;
        tmp ^= tmp << 4;
        let t = tmp as u16;
        accum = (accum >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4);
    }
    accum
}

/// mavparse.message_checksum
fn message_checksum(m: &MavType) -> u8 {
    let mut crc = 0xffffu16;
    crc = x25_accumulate(crc, format!("{} ", m.name).as_bytes());
    let crc_end = m.extensions_start.unwrap_or(m.fields.len());
    for f in &m.ordered_fields[..crc_end] {
        crc = x25_accumulate(crc, format!("{} ", f.ty).as_bytes());
        crc = x25_accumulate(crc, format!("{} ", f.name).as_bytes());
        if f.array_length != 0 {
            crc = x25_accumulate(crc, &[f.array_length as u8]);
        }
    }
    ((crc & 0xff) ^ (crc >> 8)) as u8
}

/// Direct text children of a node, concatenated (expat CharacterDataHandler for that element).
fn own_text(n: roxmltree::Node) -> String {
    n.children().filter(|c| c.is_text()).map(|c| c.text().unwrap_or("")).collect()
}

/// mavparse evaluates `value` with Python `eval()`. The XML only ever uses integer literals
/// and integer arithmetic, so this evaluates Python integer expressions: decimal, hex, octal
/// and binary literals, unary + - ~, ** * // % + - << >> & ^ | and parentheses.
fn parse_value(s: &str) -> i64 {
    struct P<'a> {
        t: &'a [u8],
        i: usize,
    }
    impl P<'_> {
        fn ws(&mut self) {
            while self.i < self.t.len() && self.t[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
        }
        fn eat(&mut self, op: &str) -> bool {
            self.ws();
            if self.t[self.i..].starts_with(op.as_bytes()) {
                // don't take '*' out of '**', '/' out of '//', '<' out of '<<'
                self.i += op.len();
                true
            } else {
                false
            }
        }
        fn or(&mut self) -> i64 {
            let mut v = self.xor();
            while self.eat("|") {
                v |= self.xor();
            }
            v
        }
        fn xor(&mut self) -> i64 {
            let mut v = self.and();
            while self.eat("^") {
                v ^= self.and();
            }
            v
        }
        fn and(&mut self) -> i64 {
            let mut v = self.shift();
            while self.eat("&") {
                v &= self.shift();
            }
            v
        }
        fn shift(&mut self) -> i64 {
            let mut v = self.sum();
            loop {
                if self.eat("<<") {
                    v <<= self.sum();
                } else if self.eat(">>") {
                    v >>= self.sum();
                } else {
                    return v;
                }
            }
        }
        fn sum(&mut self) -> i64 {
            let mut v = self.term();
            loop {
                if self.eat("+") {
                    v += self.term();
                } else if self.eat("-") {
                    v -= self.term();
                } else {
                    return v;
                }
            }
        }
        fn term(&mut self) -> i64 {
            let mut v = self.unary();
            loop {
                self.ws();
                if self.t[self.i..].starts_with(b"**") {
                    return v;
                }
                if self.eat("//") {
                    v = v.div_euclid(self.unary());
                } else if self.eat("*") {
                    v *= self.unary();
                } else if self.eat("%") {
                    v = v.rem_euclid(self.unary());
                } else {
                    return v;
                }
            }
        }
        fn unary(&mut self) -> i64 {
            if self.eat("-") {
                -self.unary()
            } else if self.eat("+") {
                self.unary()
            } else if self.eat("~") {
                !self.unary()
            } else {
                self.power()
            }
        }
        fn power(&mut self) -> i64 {
            let b = self.atom();
            if self.eat("**") {
                let e = self.unary();
                b.pow(e as u32)
            } else {
                b
            }
        }
        fn atom(&mut self) -> i64 {
            if self.eat("(") {
                let v = self.or();
                assert!(self.eat(")"), "missing )");
                return v;
            }
            self.ws();
            let start = self.i;
            while self.i < self.t.len() && (self.t[self.i].is_ascii_alphanumeric() || self.t[self.i] == b'_') {
                self.i += 1;
            }
            let lit = std::str::from_utf8(&self.t[start..self.i]).unwrap().replace('_', "");
            let lower = lit.to_ascii_lowercase();
            let (digits, radix) = if let Some(h) = lower.strip_prefix("0x") {
                (h.to_string(), 16)
            } else if let Some(o) = lower.strip_prefix("0o") {
                (o.to_string(), 8)
            } else if let Some(b) = lower.strip_prefix("0b") {
                (b.to_string(), 2)
            } else {
                (lower, 10)
            };
            i64::from_str_radix(&digits, radix).unwrap_or_else(|_| panic!("enum value literal {lit:?}"))
        }
    }
    let mut p = P { t: s.as_bytes(), i: 0 };
    let v = p.or();
    p.ws();
    assert_eq!(p.i, p.t.len(), "unparsed enum value {s:?}");
    v
}

/// mavparse.MAVXML for wire protocol 2.0.
fn parse_xml(filename: &Path) -> MavXml {
    let text = std::fs::read_to_string(filename).unwrap_or_else(|e| panic!("{}: {e}", filename.display()));
    let doc = roxmltree::Document::parse(&text).unwrap_or_else(|e| panic!("{}: {e}", filename.display()));
    let root = doc.root_element();
    let mut x = MavXml {
        filename: filename.to_path_buf(),
        message: vec![],
        enums: vec![],
        version: 2,
        include: vec![],
        message_crcs: HashMap::new(),
        message_lengths: HashMap::new(),
        message_min_lengths: HashMap::new(),
        message_names: HashMap::new(),
        largest_payload: 0,
    };
    for top in root.children().filter(|n| n.is_element()) {
        match top.tag_name().name() {
            "version" => x.version = own_text(top).trim().parse().unwrap(),
            "include" => x.include.push(own_text(top)),
            "messages" => {
                for msg in top.children().filter(|n| n.is_element() && n.tag_name().name() == "message") {
                    let mut m = MavType {
                        name: msg.attribute("name").unwrap().to_string(),
                        id: msg.attribute("id").unwrap().parse().unwrap(),
                        description: String::new(),
                        fields: vec![],
                        extensions_start: None,
                        deprecated: false,
                        ordered_fields: vec![],
                        wire_length: 0,
                        wire_min_length: 0,
                        crc_extra: 0,
                    };
                    for c in msg.children().filter(|n| n.is_element()) {
                        match c.tag_name().name() {
                            "description" => m.description.push_str(&own_text(c)),
                            "extensions" => m.extensions_start = Some(m.fields.len()),
                            "deprecated" | "wip" => m.deprecated = true,
                            "field" => {
                                let mut ty = c.attribute("type").unwrap().to_string();
                                let name = c.attribute("name").unwrap().to_string();
                                if ty == "uint8_t_mavlink_version" {
                                    ty = "uint8_t".into();
                                }
                                let mut array_length = 0;
                                if let Some(aidx) = ty.find('[') {
                                    assert!(ty.ends_with(']'));
                                    array_length = ty[aidx + 1..ty.len() - 1].parse().unwrap();
                                    ty.truncate(aidx);
                                    if ty == "array" {
                                        ty = "int8_t".into();
                                    }
                                }
                                let tl = match type_length(&ty) {
                                    Some(l) => l,
                                    None => {
                                        ty.push_str("_t");
                                        type_length(&ty).unwrap_or_else(|| panic!("unknown type {ty}"))
                                    }
                                };
                                let units = c.attribute("units").filter(|u| !u.is_empty()).map(|u| format!("[{u}]")).unwrap_or_default();
                                m.fields.push(MavField {
                                    name,
                                    wire_length: if array_length != 0 { array_length * tl } else { tl },
                                    ty,
                                    type_length: tl,
                                    array_length,
                                    wire_offset: 0,
                                    enum_: c.attribute("enum").unwrap_or("").to_string(),
                                    display: c.attribute("display").unwrap_or("").to_string(),
                                    units,
                                    description: own_text(c),
                                });
                            }
                            _ => {}
                        }
                    }
                    x.message.push(m);
                }
            }
            "enums" => {
                for en in top.children().filter(|n| n.is_element() && n.tag_name().name() == "enum") {
                    let mut e = MavEnum {
                        name: en.attribute("name").unwrap().to_string(),
                        description: String::new(),
                        entry: vec![],
                        start_value: None,
                        highest_value: 0,
                    };
                    for c in en.children().filter(|n| n.is_element()) {
                        match c.tag_name().name() {
                            "description" => e.description.push_str(&own_text(c)),
                            "entry" => {
                                let (value, autovalue) = match c.attribute("value") {
                                    Some(v) => (parse_value(v), false),
                                    None => (e.highest_value + 1, true),
                                };
                                if e.start_value.is_none() || value < e.start_value.unwrap() {
                                    e.start_value = Some(value);
                                }
                                if value > e.highest_value {
                                    e.highest_value = value;
                                }
                                let has_location = match c.attribute("hasLocation") {
                                    None | Some("false") => false,
                                    Some("true") => true,
                                    Some(v) => panic!("invalid has_location value {v}"),
                                };
                                let mut entry = MavEnumEntry {
                                    name: c.attribute("name").unwrap().to_string(),
                                    value,
                                    description: String::new(),
                                    end_marker: false,
                                    autovalue,
                                    deprecated: false,
                                    has_location,
                                };
                                for d in c.children().filter(|n| n.is_element()) {
                                    match d.tag_name().name() {
                                        "description" => entry.description.push_str(&own_text(d)),
                                        "deprecated" | "wip" => entry.deprecated = true,
                                        _ => {}
                                    }
                                }
                                e.entry.push(entry);
                            }
                            _ => {}
                        }
                    }
                    x.enums.push(e);
                }
            }
            _ => {}
        }
    }

    for m in &mut x.message {
        // sort_fields: stable sort of the base fields by type length, descending.
        let sort_end = m.extensions_start.unwrap_or(m.fields.len());
        let mut ordered: Vec<MavField> = m.fields[..sort_end].to_vec();
        ordered.sort_by(|a, b| b.type_length.cmp(&a.type_length));
        ordered.extend(m.fields[sort_end..].iter().cloned());
        let mut wire_length = 0;
        for (i, f) in ordered.iter_mut().enumerate() {
            f.wire_offset = wire_length;
            wire_length += f.wire_length;
            if m.extensions_start.is_none() || i < m.extensions_start.unwrap() {
                m.wire_min_length = wire_length;
            }
        }
        m.wire_length = wire_length;
        m.ordered_fields = ordered;
        m.crc_extra = message_checksum(m);
        x.message_crcs.insert(m.id, m.crc_extra);
        x.message_lengths.insert(m.id, m.wire_length);
        x.message_min_lengths.insert(m.id, m.wire_min_length);
        x.message_names.insert(m.id, m.name.clone());
        x.largest_payload = x.largest_payload.max(m.wire_length);
    }
    x
}

/// mavparse.merge_enums
fn merge_enums(xml: &mut [MavXml]) {
    // emap: enum name -> (xml index, enum index) of the first definition.
    let mut emap: HashMap<String, (usize, usize)> = HashMap::new();
    for xi in 0..xml.len() {
        let enums = std::mem::take(&mut xml[xi].enums);
        let mut newenums = vec![];
        for mut en in enums {
            if let Some(&(ex, ei)) = emap.get(&en.name) {
                let item = if ex == xi { &mut newenums[ei] } else { &mut xml[ex].enums[ei] };
                let item: &mut MavEnum = item;
                if item.start_value.unwrap_or(0) <= en.highest_value && item.highest_value >= en.start_value.unwrap_or(0) {
                    for entry in &mut item.entry {
                        if entry.value <= en.highest_value && entry.autovalue {
                            entry.value = en.highest_value + 1;
                            en.highest_value = entry.value;
                        }
                    }
                }
                item.entry.extend(en.entry.drain(..));
                if item.description.is_empty() {
                    item.description = en.description.clone();
                }
            } else {
                emap.insert(en.name.clone(), (xi, newenums.len()));
                newenums.push(en);
            }
        }
        xml[xi].enums = newenums;
    }
    for x in xml.iter_mut() {
        for e in &mut x.enums {
            e.entry.sort_by_key(|en| en.value); // python sorted() is stable
            let last = e.entry.last().map(|l| l.value).unwrap_or(-1);
            e.entry.push(MavEnumEntry {
                name: format!("{}_ENUM_END", e.name),
                value: last + 1,
                description: String::new(),
                end_marker: true,
                autovalue: false,
                deprecated: false,
                has_location: false,
            });
        }
    }
}

/// mavgen_cs.cleanText
fn clean_text(t: &str) -> String {
    t.replace('\n', " ").replace('\r', " ").replace('"', "'")
}

fn rust_str(s: &str) -> String {
    format!("{s:?}")
}

const RUST_KEYWORDS: &[&str] = &[
    "as", "break", "const", "continue", "else", "enum", "extern", "false", "fn", "for", "if", "impl", "in",
    "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return", "static", "struct", "trait", "true",
    "type", "unsafe", "use", "where", "while", "async", "await", "dyn", "abstract", "become", "box", "do",
    "final", "macro", "override", "priv", "typeof", "unsized", "virtual", "yield", "try",
];

fn ident(name: &str) -> String {
    if RUST_KEYWORDS.contains(&name) {
        format!("r#{name}")
    } else {
        name.to_string()
    }
}

/// `MAVLINK_BUILD_DATE` in C# is the time mavgen ran (`time.strftime("%a %b %d %Y")`) and is
/// frozen into the committed Mavlink.cs. The Rust mirrors that file, so it carries the same
/// value instead of the time of each cargo build.
const MAVLINK_BUILD_DATE: &str = "Sun Oct 04 2026";

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let defs = manifest.join("message_definitions");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=message_definitions");

    // mavgen.mavgen: parse the arguments, then expand includes breadth-first.
    let args = [defs.join("all.xml"), defs.join("offspec.xml")];
    let mut all_files: HashSet<PathBuf> = HashSet::new();
    let mut xml: Vec<MavXml> = vec![];
    for a in &args {
        if all_files.insert(a.clone()) {
            println!("cargo:rerun-if-changed={}", a.display());
            xml.push(parse_xml(a));
        }
    }
    for _ in 0..5 {
        let before = all_files.len();
        let count = xml.len();
        for xi in 0..count {
            let includes = xml[xi].include.clone();
            for inc in includes {
                let fname = xml[xi].filename.parent().unwrap().join(&inc);
                if !all_files.insert(fname.clone()) {
                    continue;
                }
                let child = parse_xml(&fname);
                let x = &mut xml[xi];
                x.message_crcs.extend(child.message_crcs.iter().map(|(k, v)| (*k, *v)));
                x.message_lengths.extend(child.message_lengths.iter().map(|(k, v)| (*k, *v)));
                x.message_min_lengths.extend(child.message_min_lengths.iter().map(|(k, v)| (*k, *v)));
                x.message_names.extend(child.message_names.iter().map(|(k, v)| (*k, v.clone())));
                x.largest_payload = x.largest_payload.max(child.largest_payload);
                xml.push(child);
            }
        }
        if all_files.len() == before {
            break;
        }
    }
    let largest_payload = xml.iter().map(|x| x.largest_payload).max().unwrap_or(0);

    // check_duplicates -> merge_enums, plus the duplicate message checks.
    merge_enums(&mut xml);
    {
        let mut ids = HashSet::new();
        let mut names = HashSet::new();
        for x in &xml {
            for m in &x.message {
                assert!(ids.insert(m.id), "Duplicate message id {} for {}", m.id, m.name);
                assert!(names.insert(m.name.clone()), "Duplicate message name {}", m.name);
            }
        }
    }

    // generate_message_header: message_infos_array and message_names_enum, deduplicated by name.
    let mut dedup: HashSet<String> = HashSet::new();
    let mut infos: Vec<(u32, String, u8, usize, usize)> = vec![];
    for x in &xml {
        let mut ids: Vec<u32> = x.message_names.keys().copied().collect();
        ids.sort_unstable();
        for id in ids {
            let name = &x.message_names[&id];
            if dedup.insert(name.clone()) {
                infos.push((id, name.clone(), x.message_crcs[&id], x.message_min_lengths[&id], x.message_lengths[&id]));
            }
        }
    }
    // NAV_ removal happens in generate_message_header for every enum entry.
    for x in &mut xml {
        for e in &mut x.enums {
            for en in &mut e.entry {
                en.name = en.name.replace("NAV_", "");
            }
        }
    }

    // generate_message_enum_types: enum underlying type from the last non-array field using it.
    let mut enumtypes: HashMap<String, &'static str> = HashMap::new();
    for x in &xml {
        for m in &x.message {
            for f in &m.fields {
                if f.array_length == 0 && !f.enum_.is_empty() {
                    enumtypes.insert(f.enum_.clone(), cs_type(&f.ty));
                }
            }
        }
    }

    let mut out = String::with_capacity(4 << 20);
    let o = &mut out;
    writeln!(o, "// @generated by crates/mp-core/build.rs from message_definitions/all.xml and offspec.xml").unwrap();
    writeln!(o, "pub const MAVLINK_BUILD_DATE: &str = {};", rust_str(MAVLINK_BUILD_DATE)).unwrap();
    writeln!(o, "pub const MAVLINK_WIRE_PROTOCOL_VERSION: &str = \"2.0\";").unwrap();
    writeln!(o, "pub const MAVLINK_MAX_PAYLOAD_LEN: i32 = {largest_payload};").unwrap();
    writeln!(o, "pub const MAVLINK_STX: u8 = {PROTOCOL_MARKER};").unwrap();
    writeln!(o, "pub const MAVLINK_VERSION: u8 = {};", xml[0].version).unwrap();
    writeln!(o, "\n/// msgid, name, crc, minlength, length, type").unwrap();
    writeln!(o, "pub const MAVLINK_MESSAGE_INFOS_GENERATED: &[message_info] = &[").unwrap();
    for (id, name, crc, minlen, len) in &infos {
        let t = format!("mavlink_{}_t", name.to_lowercase());
        writeln!(
            o,
            "    message_info {{ msgid: {id}, name: {}, crc: {crc}, minlength: {minlen}, length: {len}, r#type: Some(&{t}::TYPE) }},",
            rust_str(name)
        )
        .unwrap();
    }
    writeln!(o, "];\n").unwrap();

    // MAVLINK_MSG_ID
    writeln!(o, "#[allow(non_camel_case_types)]\n#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]\npub struct MAVLINK_MSG_ID(pub i32);").unwrap();
    writeln!(o, "#[allow(non_upper_case_globals)]\nimpl MAVLINK_MSG_ID {{").unwrap();
    for (id, name, ..) in &infos {
        writeln!(o, "    pub const {}: MAVLINK_MSG_ID = MAVLINK_MSG_ID({id});", ident(name)).unwrap();
    }
    writeln!(o, "    pub const ENTRIES: &'static [EnumEntry] = &[").unwrap();
    for (id, name, ..) in &infos {
        writeln!(o, "        EnumEntry {{ name: {}, value: {id}, description: \"\", obsolete: false, has_location: false }},", rust_str(name)).unwrap();
    }
    writeln!(o, "    ];\n}}").unwrap();
    writeln!(o, "impl_cs_enum!(MAVLINK_MSG_ID, i32, false);\n").unwrap();

    // Enums.
    for x in &xml {
        for e in &x.enums {
            let desc = clean_text(&e.description);
            let flags = desc.to_lowercase().contains("bitmask") || e.name.to_lowercase().contains("_flags");
            let base = enumtypes.get(&e.name).copied().unwrap_or("int");
            let rt = rs_type(base);
            writeln!(o, "/// {}", desc.trim()).unwrap();
            writeln!(o, "#[allow(non_camel_case_types)]\n#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]\npub struct {}(pub {rt});", e.name).unwrap();
            writeln!(o, "#[allow(non_upper_case_globals)]\nimpl {} {{", e.name).unwrap();
            let mut entries = vec![];
            for fe in &e.entry {
                if fe.name.ends_with("ENUM_END") || fe.end_marker {
                    continue;
                }
                let mut name = fe.name.replace(&format!("{}_", e.name), "");
                if name.chars().next().is_some_and(|c| c.is_ascii_digit()) {
                    name = format!("_{name}");
                }
                let d = clean_text(&fe.description);
                writeln!(o, "    /// {}", d.trim()).unwrap();
                if fe.deprecated {
                    writeln!(o, "    #[deprecated]").unwrap();
                }
                writeln!(o, "    pub const {}: {} = {}({} as {rt});", ident(&name), e.name, e.name, fe.value).unwrap();
                entries.push((name, fe.value, d, fe.deprecated, fe.has_location));
            }
            writeln!(o, "    pub const ENTRIES: &'static [EnumEntry] = &[").unwrap();
            for (name, value, d, obs, loc) in &entries {
                writeln!(
                    o,
                    "        EnumEntry {{ name: {}, value: {value}, description: {}, obsolete: {obs}, has_location: {loc} }},",
                    rust_str(name),
                    rust_str(d)
                )
                .unwrap();
            }
            writeln!(o, "    ];\n}}").unwrap();
            writeln!(o, "impl_cs_enum!({}, {rt}, {flags});\n", e.name).unwrap();
        }
    }

    // Lookup by enum type name, standing in for `Type.GetType("MAVLink+" + name)` reflection.
    writeln!(o, "/// The generated enum called `name`: its members, C# underlying type and `[Flags]`.\npub fn enum_info(name: &str) -> Option<EnumInfo> {{\n    Some(match name {{").unwrap();
    for x in &xml {
        for e in &x.enums {
            let desc = clean_text(&e.description);
            let flags = desc.to_lowercase().contains("bitmask") || e.name.to_lowercase().contains("_flags");
            let base = enumtypes.get(&e.name).copied().unwrap_or("int");
            writeln!(
                o,
                "        {} => EnumInfo {{ name: {}, base_type: {}, flags: {flags}, entries: {}::ENTRIES }},",
                rust_str(&e.name),
                rust_str(&e.name),
                rust_str(base),
                e.name
            )
            .unwrap();
        }
    }
    writeln!(o, "        _ => return None,\n    }})\n}}\n").unwrap();

    // Message structs (generate_one / generate_message_h).
    for x in &xml {
        for m in &x.message {
            let t = format!("mavlink_{}_t", m.name.to_lowercase());
            let desc = clean_text(&m.description);
            writeln!(o, "/// {}\n/// extensions_start {}", desc.trim(), m.extensions_start.unwrap_or(0)).unwrap();
            if m.deprecated {
                writeln!(o, "#[deprecated]").unwrap();
            }
            writeln!(o, "#[allow(non_camel_case_types)]\n#[derive(Clone, Copy, Debug, PartialEq)]\npub struct {t} {{").unwrap();
            for f in &m.ordered_fields {
                let cs = cs_type(&f.ty);
                let rt = rs_type(cs);
                let fd = clean_text(&f.description);
                writeln!(o, "    /// {} {} {} {}", fd.trim(), f.enum_, f.units, f.display).unwrap();
                if f.array_length != 0 {
                    writeln!(o, "    pub {}: [{rt}; {}],", ident(&f.name), f.array_length).unwrap();
                } else {
                    writeln!(o, "    pub {}: {rt},", ident(&f.name)).unwrap();
                }
            }
            writeln!(o, "}}").unwrap();
            // Default: all zero, like `new mavlink_x_t()` once StructureToByteArray sizes the arrays.
            writeln!(o, "impl Default for {t} {{\n    fn default() -> Self {{\n        Self {{").unwrap();
            for f in &m.ordered_fields {
                let rt = rs_type(cs_type(&f.ty));
                if f.array_length != 0 {
                    writeln!(o, "            {}: [0 as {rt}; {}],", ident(&f.name), f.array_length).unwrap();
                } else {
                    writeln!(o, "            {}: 0 as {rt},", ident(&f.name)).unwrap();
                }
            }
            writeln!(o, "        }}\n    }}\n}}").unwrap();

            writeln!(o, "impl {t} {{").unwrap();
            writeln!(o, "    pub const MSG_ID: u32 = {};", m.id).unwrap();
            writeln!(o, "    pub const NAME: &'static str = {};", rust_str(&m.name)).unwrap();
            writeln!(o, "    pub const WIRE_LENGTH: usize = {};", m.wire_length).unwrap();
            writeln!(o, "    pub const EXTENSIONS_START: usize = {};", m.extensions_start.unwrap_or(0)).unwrap();
            writeln!(o, "    pub const FIELDS: &'static [FieldInfo] = &[").unwrap();
            for f in &m.ordered_fields {
                let cs = cs_type(&f.ty);
                let cs_full = if f.array_length != 0 { format!("{cs}[]") } else { cs.to_string() };
                writeln!(
                    o,
                    "        FieldInfo {{ name: {}, cs_type: {}, offset: {}, array_length: {}, units: {}, description: {} }},",
                    rust_str(&f.name),
                    rust_str(&cs_full),
                    f.wire_offset,
                    f.array_length,
                    rust_str(&f.units),
                    rust_str(&clean_text(&f.description))
                )
                .unwrap();
            }
            writeln!(o, "    ];").unwrap();
            writeln!(
                o,
                "    pub const TYPE: MessageType = MessageType {{ name: {}, size: {}, obsolete: {}, fields: Self::FIELDS, type_id: || std::any::TypeId::of::<{t}>(), deserialize: |b| Box::new({t}::from_bytes(b)) }};",
                rust_str(&t),
                m.wire_length,
                m.deprecated
            )
            .unwrap();

            // packet ordered constructor
            let args: Vec<String> = m
                .ordered_fields
                .iter()
                .map(|f| {
                    let rt = rs_type(cs_type(&f.ty));
                    if f.array_length != 0 {
                        format!("{}: [{rt}; {}]", ident(&f.name), f.array_length)
                    } else {
                        format!("{}: {rt}", ident(&f.name))
                    }
                })
                .collect();
            let names: Vec<String> = m.ordered_fields.iter().map(|f| ident(&f.name)).collect();
            writeln!(o, "    #[allow(clippy::too_many_arguments)]\n    pub fn new({}) -> Self {{ Self {{ {} }} }}", args.join(", "), names.join(", ")).unwrap();
            // packet xml order
            let xargs: Vec<String> = m
                .fields
                .iter()
                .map(|f| {
                    let rt = rs_type(cs_type(&f.ty));
                    if f.array_length != 0 {
                        format!("{}: [{rt}; {}]", ident(&f.name), f.array_length)
                    } else {
                        format!("{}: {rt}", ident(&f.name))
                    }
                })
                .collect();
            let xnames: Vec<String> = m.fields.iter().map(|f| ident(&f.name)).collect();
            writeln!(
                o,
                "    #[allow(clippy::too_many_arguments)]\n    pub fn populate_xml_order({}) -> Self {{ Self {{ {} }} }}",
                xargs.join(", "),
                xnames.join(", ")
            )
            .unwrap();

            // ByteArrayToStructureGC: zero-filled struct-sized buffer, payload copied in.
            writeln!(o, "    pub fn from_bytes(payload: &[u8]) -> Self {{").unwrap();
            writeln!(o, "        let mut b = [0u8; {}];", m.wire_length.max(1)).unwrap();
            writeln!(o, "        let n = payload.len().min({});\n        b[..n].copy_from_slice(&payload[..n]);", m.wire_length).unwrap();
            writeln!(o, "        Self {{").unwrap();
            for f in &m.ordered_fields {
                let rt = rs_type(cs_type(&f.ty));
                if f.array_length != 0 {
                    writeln!(o, "            {}: wire::get_arr::<{rt}, {}>(&b, {}),", ident(&f.name), f.array_length, f.wire_offset).unwrap();
                } else {
                    writeln!(o, "            {}: <{rt} as wire::Wire>::get(&b, {}),", ident(&f.name), f.wire_offset).unwrap();
                }
            }
            writeln!(o, "        }}\n    }}").unwrap();
            // StructureToByteArray
            writeln!(o, "    pub fn to_bytes(&self) -> Vec<u8> {{\n        let mut b = vec![0u8; {}];", m.wire_length).unwrap();
            for f in &m.ordered_fields {
                if f.array_length != 0 {
                    writeln!(o, "        wire::put_arr(&self.{}, &mut b, {});", ident(&f.name), f.wire_offset).unwrap();
                } else {
                    writeln!(o, "        wire::Wire::put(self.{}, &mut b, {});", ident(&f.name), f.wire_offset).unwrap();
                }
            }
            writeln!(o, "        b\n    }}").unwrap();
            writeln!(o, "}}").unwrap();

            writeln!(o, "impl crate::extlibs::mavlink::mavlinkutil::MarshalStruct for {t} {{").unwrap();
            writeln!(o, "    const SIZE: usize = {};", m.wire_length).unwrap();
            writeln!(o, "    fn fields() -> &'static [FieldInfo] {{ Self::FIELDS }}").unwrap();
            writeln!(o, "    fn from_bytes(b: &[u8]) -> Self {{ {t}::from_bytes(b) }}").unwrap();
            writeln!(o, "    fn to_bytes(&self) -> Vec<u8> {{ {t}::to_bytes(self) }}").unwrap();
            writeln!(o, "}}").unwrap();

            // Reflection-style access used by GetFields() callers (MAVLinkInspector, logging).
            writeln!(o, "impl MavlinkData for {t} {{").unwrap();
            writeln!(o, "    fn msgid(&self) -> u32 {{ {} }}", m.id).unwrap();
            writeln!(o, "    fn message_type(&self) -> &'static MessageType {{ &Self::TYPE }}").unwrap();
            writeln!(o, "    fn to_bytes(&self) -> Vec<u8> {{ {t}::to_bytes(self) }}").unwrap();
            writeln!(o, "    fn as_any(&self) -> &dyn std::any::Any {{ self }}").unwrap();
            writeln!(o, "    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {{ self }}").unwrap();
            writeln!(o, "    fn clone_box(&self) -> Box<dyn MavlinkData> {{ Box::new(*self) }}").unwrap();
            writeln!(o, "    fn get_field(&self, name: &str) -> Option<FieldValue> {{\n        match name {{").unwrap();
            for f in &m.ordered_fields {
                let rt = rs_type(cs_type(&f.ty));
                let variant = field_variant(rt, f.array_length != 0);
                if f.array_length != 0 {
                    writeln!(o, "            {} => Some(FieldValue::{variant}(self.{}.to_vec())),", rust_str(&f.name), ident(&f.name)).unwrap();
                } else {
                    writeln!(o, "            {} => Some(FieldValue::{variant}(self.{})),", rust_str(&f.name), ident(&f.name)).unwrap();
                }
            }
            writeln!(o, "            _ => None,\n        }}\n    }}").unwrap();
            writeln!(o, "    fn set_field(&mut self, name: &str, value: &FieldValue) -> bool {{\n        match (name, value) {{").unwrap();
            for f in &m.ordered_fields {
                let rt = rs_type(cs_type(&f.ty));
                let variant = field_variant(rt, f.array_length != 0);
                if f.array_length != 0 {
                    writeln!(
                        o,
                        "            ({}, FieldValue::{variant}(v)) => {{ let n = v.len().min({}); self.{}[..n].copy_from_slice(&v[..n]); for x in self.{}[n..].iter_mut() {{ *x = Default::default(); }} true }}",
                        rust_str(&f.name),
                        f.array_length,
                        ident(&f.name),
                        ident(&f.name)
                    )
                    .unwrap();
                } else {
                    writeln!(o, "            ({}, FieldValue::{variant}(v)) => {{ self.{} = *v; true }}", rust_str(&f.name), ident(&f.name)).unwrap();
                }
            }
            writeln!(o, "            _ => false,\n        }}\n    }}").unwrap();
            writeln!(o, "}}\n").unwrap();
        }
    }

    let _ = BTreeMap::<u8, u8>::new();
    let dest = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("mavlink_generated.rs");
    std::fs::write(&dest, out).unwrap();
}

fn field_variant(rt: &str, array: bool) -> String {
    let base = match rt {
        "f32" => "Float",
        "f64" => "Double",
        "u8" => "Byte",
        "i8" => "SByte",
        "i16" => "Short",
        "u16" => "UShort",
        "i32" => "Int",
        "u32" => "UInt",
        "i64" => "Long",
        "u64" => "ULong",
        _ => unreachable!(),
    };
    if array {
        format!("{base}Array")
    } else {
        base.to_string()
    }
}
