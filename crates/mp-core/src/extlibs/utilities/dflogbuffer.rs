//! Port of `ExtLibs/Utilities/DFLogBuffer.cs`
//!
//! A text or binary DataFlash log indexed by line: the start offset of every line and, per
//! message type, its offsets and line numbers, so lines are read on demand. Logs of 300 MB
//! or more keep the index in a gzip cache in the temp folder. `IEnumerable<String>` is
//! `lines()`; `GetEnumeratorType` returns a lazy enumerator that is advanced with the buffer.

#![allow(non_snake_case)]

use std::io::{Read, Seek, SeekFrom, Write};

use regex::Regex;

use super::binarylog::{self, stream_length, stream_position, stream_read, stream_read_byte, BinaryLog, LogStream};
use super::dflog::{DFItem, DFLog, InstanceSource};
use crate::dotnet::collections::Dictionary;
use crate::dotnet::io::console_write_line;
use crate::dotnet::{int_parse, int_try_parse, DateTime, Exception};

/// `List<long>[256]`
type Index = Vec<Vec<i64>>;

/// `public class DFLogBuffer : IEnumerable<String>, IDisposable`
pub struct DFLogBuffer {
    /// used for binary log line conversion
    binlog: BinaryLog,
    /// used for fmt messages
    pub dflog: DFLog,
    basestream: Box<dyn LogStream>,
    _count: i64,
    linestartoffset: Vec<i64>,
    /// Type and offsets
    messageindex: Index,
    /// Type and line numbers
    messageindexline: Index,
    binary: bool,
    indexcachelineno: i64,
    currentindexcache: Option<String>,
    pub UnitMultiList: Vec<(String, String, Option<String>, f64)>,
    pub InstanceType: Dictionary<i32, (i32, Vec<String>)>,
    _filename: String,
    pub FMT: Dictionary<i32, (i32, String, String, String)>,
    pub FMTU: Dictionary<i32, (String, String)>,
    pub Unit: Dictionary<char, String>,
    pub Mult: Dictionary<char, String>,
}

impl InstanceSource for DFLogBuffer {
    fn instance_type_contains(&self, typeno: i32) -> bool {
        self.InstanceType.contains_key(&typeno)
    }
    fn fmtu_units(&self, typeno: i32) -> Option<String> {
        self.FMTU.get(&typeno).map(|f| f.0.clone())
    }
}

/// The parts of `InstanceSource` a `TypeEnumerator` needs without borrowing the buffer.
struct Instances<'a> {
    instance_type: &'a Dictionary<i32, (i32, Vec<String>)>,
    fmtu: &'a Dictionary<i32, (String, String)>,
}

impl InstanceSource for Instances<'_> {
    fn instance_type_contains(&self, typeno: i32) -> bool {
        self.instance_type.contains_key(&typeno)
    }
    fn fmtu_units(&self, typeno: i32) -> Option<String> {
        self.fmtu.get(&typeno).map(|f| f.0.clone())
    }
}

/// `GetEnumeratorType(types)`: the lines of the requested types (and instances), read one
/// by one as it is advanced.
pub struct TypeEnumerator {
    instances: Vec<(String, Vec<String>)>,
    slist: Vec<i64>,
    pos: usize,
    progress: u32,
}

impl TypeEnumerator {
    /// `MoveNext()` / `Current`
    pub fn next(&mut self, buf: &mut DFLogBuffer) -> Option<Result<DFItem, Exception>> {
        while self.pos < self.slist.len() {
            let l = self.slist[self.pos];
            self.pos += 1;
            let sec = DateTime::now().parts().5;
            if sec != self.progress {
                console_write_line(&l.to_string());
                self.progress = sec;
            }
            let mut ans = match buf.item(l) {
                Ok(a) => a,
                Err(e) => return Some(Err(e)),
            };
            let Some((_, inst)) = self.instances.iter().find(|(k, _)| *k == ans.msgtype()) else { continue };
            // instance was requested, and its not a match
            if !inst.iter().any(|s| s.is_empty()) {
                let src = Instances { instance_type: &buf.InstanceType, fmtu: &buf.FMTU };
                let i = match ans.instance(&buf.dflog, Some(&src)) {
                    Ok(i) => i,
                    Err(e) => return Some(Err(e)),
                };
                if !inst.contains(&i) {
                    continue;
                }
            }
            return Some(Ok(ans));
        }
        None
    }
}

/// `GetEnumeratorTypeAll()` / `GetEnumerator()`: every line in order.
pub struct LineEnumerator {
    position: i32,
}

impl LineEnumerator {
    /// the next `DFItem` (`GetEnumeratorTypeAll`)
    pub fn next_item(&mut self, buf: &mut DFLogBuffer) -> Option<Result<DFItem, Exception>> {
        if self.position >= buf.Count() {
            return None;
        }
        let r = buf.item(self.position as i64);
        self.position += 1;
        Some(r)
    }

    /// the next line's text (`IEnumerable<String>`)
    pub fn next_line(&mut self, buf: &mut DFLogBuffer) -> Option<Result<String, Exception>> {
        if self.position >= buf.Count() {
            return None;
        }
        let r = buf.line(self.position);
        self.position += 1;
        Some(r)
    }
}

// any mismatch (old BinaryFormatter caches included) just rebuilds the index
const CACHE_VERSION: i32 = 1;
const CACHE_MAGIC: &[u8] = b"MPDFIDX";
// BinaryWriter/Reader move 8 bytes per call; unbuffered, every call is a separate deflate call
const CACHE_BUFFER_SIZE: usize = 1024 * 1024;

fn bad_cache() -> Exception {
    Exception::InvalidData("bad DFLogBuffer cache".into())
}

impl DFLogBuffer {
    /// `new DFLogBuffer(string filename)`
    pub fn new_file(filename: &str) -> Result<Self, Exception> {
        let file = std::fs::File::open(filename).map_err(|e| crate::dotnet::io::file_error(filename, &e))?;
        Self::new(Box::new(file), filename)
    }

    /// `new DFLogBuffer(Stream instream)` over a seekable stream; `filename` is the
    /// `FileStream.Name` ("" for any other stream). A stream that cannot seek is copied to a
    /// `MemoryStream` sized from its `Length` in C#, which such a stream does not support,
    /// so the C# throws there; seekable input is required here.
    pub fn new(instream: Box<dyn LogStream>, filename: &str) -> Result<Self, Exception> {
        let mut buf = DFLogBuffer {
            binlog: BinaryLog::new(),
            dflog: DFLog::new(),
            basestream: instream,
            _count: 0,
            linestartoffset: Vec::new(),
            messageindex: vec![Vec::new(); 256],
            messageindexline: vec![Vec::new(); 256],
            binary: false,
            indexcachelineno: -1,
            currentindexcache: None,
            UnitMultiList: Vec::new(),
            InstanceType: Dictionary::new(),
            _filename: filename.to_string(),
            FMT: Dictionary::new(),
            FMTU: Dictionary::new(),
            Unit: Dictionary::new(),
            Mult: Dictionary::new(),
        };

        if stream_read_byte(&mut *buf.basestream) == BinaryLog::HEAD_BYTE1 as i32 && stream_read_byte(&mut *buf.basestream) == BinaryLog::HEAD_BYTE2 as i32 {
            buf.binary = true;
        }

        console_write_line(&format!("Binary: {}", if buf.binary { "True" } else { "False" }));

        // back to start
        buf.seek(0);
        let start = DateTime::now();
        buf.setlinecount()?;
        console_write_line(&format!(
            "DFLogBuffer-linecount: {} time(ms): {}",
            buf.Count(),
            crate::dotnet::double_to_string((DateTime::now() - start).total_milliseconds())
        ));
        buf.seek(0);
        Ok(buf)
    }

    fn seek(&mut self, pos: i64) {
        let _ = self.basestream.seek(SeekFrom::Start(pos.max(0) as u64));
    }

    fn setlinecount(&mut self) -> Result<(), Exception> {
        if self._filename.is_empty() || !self.LoadCache()? {
            let mut buffer = vec![0u8; 1024 * 1024];
            let mut lineCount: i64 = 0;
            if self.binary {
                let length = stream_length(&mut *self.basestream);
                while stream_position(&mut *self.basestream) < length {
                    let ans = self.binlog.ReadMessageTypeOffset(&mut *self.basestream, length);
                    if ans.0 == 0 && ans.1 == 0 {
                        continue;
                    }
                    let type_ = ans.0 as usize;
                    self.messageindex[type_].push(ans.1);
                    self.messageindexline[type_].push(lineCount);
                    self.linestartoffset.push(ans.1);
                    lineCount += 1;
                    if lineCount % 1000000 == 0 {
                        let pct = stream_position(&mut *self.basestream) as f64 / length as f64 * 100.0;
                        console_write_line(&format!("reading lines {} {}", lineCount, crate::dotnet::double_to_string(pct)));
                    }
                }
                self._count = lineCount;

                // build fmt line database to pre seed the FMT message
                for a in self.messageindexline[128].clone() {
                    let line = self.line(a as i32)?;
                    self.dflog.FMTLine(&line);
                }

                for (_, v) in self.dflog.logformat.iter() {
                    let id = v.Id;
                    // messageindex[id] past 255 throws and the catch ends the loop
                    let Some(list) = usize::try_from(id).ok().and_then(|i| self.messageindex.get(i)) else { break };
                    if !list.is_empty() {
                        console_write_line(&format!("Seen {} count {}", v.Name, list.len()));
                    }
                }
            } else {
                // first line starts at 0
                self.linestartoffset.push(0);
                let length = stream_length(&mut *self.basestream);
                while stream_position(&mut *self.basestream) < length {
                    let startpos = stream_position(&mut *self.basestream);
                    let read = self.basestream.read(&mut buffer).unwrap_or(0);
                    if read == 0 {
                        break;
                    }
                    for (offset, b) in buffer[..read].iter().enumerate() {
                        if *b == b'\n' {
                            self.linestartoffset.push(startpos + 1 + offset as i64);
                            lineCount += 1;
                        }
                    }
                }
                self._count = lineCount;

                // create msg cache
                let mut b = 0usize;
                let mut it = self.lines();
                while let Some(item) = it.next_line(self) {
                    let item = item?;
                    let idx = item.find(',').map(|i| i as i64).unwrap_or(-1);
                    if idx <= 0 {
                        b += 1;
                        continue;
                    }
                    let msgtype = item[..idx as usize].to_string();
                    if msgtype == "FMT" {
                        self.dflog.FMTLine(&item);
                    }
                    if let Some(lbl) = self.dflog.logformat.get(&msgtype) {
                        let type_ = lbl.Id as u8 as usize;
                        let off = self.linestartoffset[b];
                        self.messageindex[type_].push(off);
                        self.messageindexline[type_].push(b as u32 as i64);
                    }
                    b += 1;
                }
            }
            self.SaveCache();
        }

        // build fmt line database using type
        let mut it = self.GetEnumeratorType(&["FMT"]);
        while let Some(item) = it.next(self) {
            let mut item = item?;
            let r = (|| -> Result<(), Exception> {
                if item.items().is_empty() {
                    return Ok(());
                }
                let d = &self.dflog;
                let req = |item: &mut DFItem, k: &str| -> Result<String, Exception> {
                    item.get(d, k)?.ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))
                };
                let key = int_parse(&req(&mut item, "Type")?)?;
                let length = int_parse(req(&mut item, "Length")?.trim())?;
                let name = req(&mut item, "Name")?.trim().to_string();
                let format = req(&mut item, "Format")?.trim().to_string();
                let skip = d.FindMessageOffset(Some("FMT"), Some("Columns"))?;
                let rest: Vec<String> = item.items().iter().skip(skip.max(0) as usize).map(|s| s.clone().unwrap_or_default()).collect();
                // Aggregate((s, s1) => s.Trim() + "," + s1.Trim()); an empty sequence throws
                let mut agg = rest.first().cloned().ok_or_else(|| Exception::InvalidOperation("Sequence contains no elements".into()))?;
                for s1 in &rest[1..] {
                    agg = format!("{},{}", agg.trim(), s1.trim());
                }
                let columns = agg.trim_start_matches(',').to_string();
                self.FMT.insert(key, (length, name, format, columns));
                let line = self.line(item.lineno)?;
                self.dflog.FMTLine(&line);
                Ok(())
            })();
            let _ = r;
        }

        let mut it = self.GetEnumeratorType(&["FMTU"]);
        while let Some(item) = it.next(self) {
            let mut item = item?;
            let r = (|| -> Result<(), Exception> {
                if item.items().is_empty() {
                    return Ok(());
                }
                let d = &self.dflog;
                let nre = || Exception::NullReference("Object reference not set to an instance of an object.".into());
                let fmttype = int_parse(&item.get(d, "FmtType")?.ok_or_else(nre)?)?;
                let unitids = item.get(d, "UnitIds")?.ok_or_else(nre)?.trim().to_string();
                let multids = item.get(d, "MultIds")?.ok_or_else(nre)?.trim().to_string();
                self.FMTU.insert(fmttype, (unitids.clone(), multids));
                if unitids.contains('#') {
                    let idx = unitids[..unitids.find('#').unwrap()].chars().count() as i32;
                    self.InstanceType.insert(fmttype, (idx, Vec::new()));
                }
                Ok(())
            })();
            let _ = r;
        }

        let keys: Vec<i32> = self.InstanceType.keys().copied().collect();
        for key in keys {
            let Some(name) = self.FMT.get(&key).map(|f| f.1.clone()) else { continue };
            let index = self.InstanceType.get(&key).map(|v| v.0).unwrap_or(0);
            let mut a = 0;
            let mut it = self.GetEnumeratorType(&[name.as_str()]);
            while let Some(item) = it.next(self) {
                let item = item?;
                let instancevalue = item
                    .raw()
                    .get((index + 1) as usize)
                    .ok_or_else(crate::dotnet::index_out_of_range)?
                    .ToString()
                    .ok_or_else(|| Exception::NullReference("Object reference not set to an instance of an object.".into()))?;
                if let Some(v) = self.InstanceType.get_mut(&key) {
                    if !v.1.contains(&instancevalue) {
                        v.1.push(instancevalue);
                    }
                }
                if a > 2000 {
                    break;
                }
                a += 1;
            }
        }

        // Unit and Mult start empty and nothing fills them first, so these never run
        if !self.Unit.is_empty() {
            let mut it = self.GetEnumeratorType(&["UNIT"]);
            while let Some(item) = it.next(self) {
                let mut item = item?;
                let d = &self.dflog;
                if let (Ok(Some(id)), Ok(Some(label))) = (item.get(d, "Id"), item.get(d, "Label")) {
                    if let Ok(id) = int_parse(&id) {
                        self.Unit.insert(char::from_u32(id as u32).unwrap_or('\0'), label.trim().to_string());
                    }
                }
            }
        }
        if !self.Mult.is_empty() {
            let mut it = self.GetEnumeratorType(&["MULT"]);
            while let Some(item) = it.next(self) {
                let mut item = item?;
                let d = &self.dflog;
                if let (Ok(Some(id)), Ok(Some(mult))) = (item.get(d, "Id"), item.get(d, "Mult")) {
                    if let Ok(id) = int_parse(&id) {
                        self.Mult.insert(char::from_u32(id as u32).unwrap_or('\0'), mult.trim().to_string());
                    }
                }
            }
        }

        self.BuildUnitMultiList();

        let mut limitcount = 0;
        // used to set the firmware type
        let mut it = self.GetEnumeratorType(&["MSG", "PARM"]);
        while let Some(item) = it.next(self) {
            let item = item?;
            // must be the string version to do the firmware type detection - binarylog
            let _line = self.line(item.lineno)?;
            limitcount += 1;
            if limitcount > 100000 {
                break;
            }
        }

        // try get gps time - when a dfitem is created and no valid gpstime has been establish
        // the messages are parsed to get a valid gpstime; here we just force the parsing of
        // gps messages to get the valid board time to gps time offset
        let mut gpsa = 0;
        let mut it = self.GetEnumeratorType(&["GPS", "GPS2", "GPSB"]);
        while let Some(item) = it.next(self) {
            let mut item = item?;
            gpsa += 1;
            let status = item.get(&self.dflog, "Status")?;
            if let Some(status) = status.as_deref().and_then(int_try_parse) {
                if status >= 3 {
                    break;
                }
            }
            // get first gps time
            if gpsa > 2000 {
                break;
            }
        }

        self.indexcachelineno = -1;
        Ok(())
    }

    /// `CachePath`: temp folder + the full path with separators replaced + name + size.
    fn CachePath(&self) -> String {
        let r = (|| -> Option<String> {
            let full = std::path::absolute(&self._filename).ok()?;
            let full = full.to_string_lossy().replace('/', "_").replace('\\', "_").replace(':', "_");
            let stem = std::path::Path::new(&self._filename).file_stem()?.to_string_lossy().to_string();
            let len = std::fs::metadata(&self._filename).ok()?.len();
            let mut tmp = std::env::temp_dir().to_string_lossy().to_string();
            if !tmp.ends_with(std::path::MAIN_SEPARATOR) {
                tmp.push(std::path::MAIN_SEPARATOR);
            }
            Some(format!("{tmp}{full}{stem}{len}"))
        })();
        // Path.GetTempFileName()
        r.unwrap_or_else(|| std::env::temp_dir().join(format!("tmp{:X}.tmp", std::process::id())).to_string_lossy().to_string())
    }

    fn SaveCache(&mut self) {
        if self._filename.is_empty() {
            return;
        }
        // save cache if file is over 300mb
        if stream_length(&mut *self.basestream) < 1024 * 1024 * 300 {
            return;
        }
        // save cache, it is optional so a failure must not stop the log from opening
        let r = (|| -> std::io::Result<()> {
            let file = std::fs::File::create(self.CachePath())?;
            let gs = flate2::write::GzEncoder::new(file, flate2::Compression::default());
            let mut bw = std::io::BufWriter::with_capacity(CACHE_BUFFER_SIZE, gs);
            bw.write_all(CACHE_MAGIC)?;
            bw.write_all(&CACHE_VERSION.to_le_bytes())?;
            bw.write_all(&self._count.to_le_bytes())?;
            write_cache_list(&mut bw, &self.linestartoffset)?;
            bw.write_all(&(self.messageindex.len() as i32).to_le_bytes())?;
            for a in 0..self.messageindex.len() {
                write_cache_list(&mut bw, &self.messageindex[a])?;
                write_cache_list(&mut bw, &self.messageindexline[a])?;
            }
            bw.into_inner().map_err(|e| e.into_error())?.finish()?;
            Ok(())
        })();
        if let Err(ex) = r {
            console_write_line(&format!("DFLogBuffer: failed to save cache {ex}"));
        }
    }

    fn LoadCache(&mut self) -> Result<bool, Exception> {
        let path = self.CachePath();
        if !std::path::Path::new(&path).exists() {
            return Ok(false);
        }
        let length = stream_length(&mut *self.basestream);
        let binary = self.binary;
        let n = self.messageindex.len();
        let r = (|| -> Result<Option<(i64, Vec<i64>, Index, Index)>, Exception> {
            let file = std::fs::File::open(&path).map_err(|e| Exception::Io(e.to_string()))?;
            let gs = flate2::read::GzDecoder::new(file);
            let mut br = std::io::BufReader::with_capacity(CACHE_BUFFER_SIZE, gs);
            let mut magic = vec![0u8; CACHE_MAGIC.len()];
            let got = read_up_to(&mut br, &mut magic)?;
            if got != magic.len() || magic != CACHE_MAGIC || read_i32(&mut br)? != CACHE_VERSION {
                return Ok(None);
            }
            // every line is at least one byte, so a valid count never exceeds the log size
            let lineCount = read_i64(&mut br)?;
            if lineCount < 0 || lineCount > length {
                return Ok(None);
            }
            // text logs also store the leading 0 offset. the indexers use these to seek and
            // size reads, so they must be in the log and in order
            let lineStarts = read_cache_list(&mut br, lineCount + 1, length)?;
            if lineStarts.len() as i64 != lineCount + if binary { 0 } else { 1 } {
                return Ok(None);
            }
            for i in 1..lineStarts.len() {
                if lineStarts[i] < lineStarts[i - 1] {
                    return Ok(None);
                }
            }
            if read_i32(&mut br)? != n as i32 {
                return Ok(None);
            }
            let mut index = Vec::with_capacity(n);
            let mut indexLine = Vec::with_capacity(n);
            for _ in 0..n {
                index.push(read_cache_list(&mut br, lineCount, length)?);
                indexLine.push(read_cache_list(&mut br, lineCount, lineCount - 1)?);
            }
            Ok(Some((lineCount, lineStarts, index, indexLine)))
        })();
        let Ok(Some((lineCount, lineStarts, index, indexLine))) = r else { return Ok(false) };
        self.messageindex = index;
        self.messageindexline = indexLine;
        self.linestartoffset = lineStarts;
        self._count = lineCount;

        // build fmt line database to pre seed the FMT message
        for a in self.messageindexline[128].clone() {
            let line = self.line(a as i32)?;
            self.dflog.FMTLine(&line);
        }
        Ok(true)
    }

    /// `SplitLog(pieces)`: writes `<file>_split<i>.bin` pieces, each led by every
    /// FMT/FMTU/UNIT/MULT message of the whole log.
    pub fn SplitLog(&mut self, pieces: i32) -> Result<(), Exception> {
        let length = stream_length(&mut *self.basestream);
        if pieces <= 0 {
            return Err(Exception::Custom("Exception", "Invalid pieces parameters".into()));
        }
        let sizeofpiece = length / pieces as i64;
        for i in 0..pieces as i64 {
            let start = i * sizeofpiece;
            let end = start + sizeofpiece;
            let path = format!("{}_split{}.bin", self._filename, i);
            // File.OpenWrite: opens or creates without truncating
            let mut file = std::fs::OpenOptions::new().write(true).create(true).truncate(false).open(&path).map_err(|e| crate::dotnet::io::file_error(&path, &e))?;
            let mut buffer = vec![0u8; 1024 * 256];
            for name in ["FMT", "FMTU", "UNIT", "MULT"] {
                let type_ = self.dflog.logformat.get(&name.to_string()).cloned().ok_or_else(|| Exception::KeyNotFound("The given key was not present in the dictionary.".into()))?;
                for a in self.messageindex[type_.Id as u8 as usize].clone() {
                    self.seek(a);
                    let n = (type_.Length.max(0) as usize).min(buffer.len());
                    let read = stream_read(&mut *self.basestream, &mut buffer[..n]);
                    file.write_all(&buffer[..read]).map_err(|e| Exception::Io(e.to_string()))?;
                }
            }
            let mut min = i64::MAX;
            let mut max = i64::MIN;
            // got min and max valid
            for &a in &self.linestartoffset {
                if a >= start && a < end {
                    min = min.min(a);
                    max = max.max(a);
                }
            }
            self.seek(min);
            while stream_position(&mut *self.basestream) < max {
                let readsize = (end - stream_position(&mut *self.basestream)).min(buffer.len() as i64) as usize;
                let read = stream_read(&mut *self.basestream, &mut buffer[..readsize]);
                file.write_all(&buffer[..read]).map_err(|e| Exception::Io(e.to_string()))?;
                if read == 0 {
                    break;
                }
            }
        }
        Ok(())
    }

    fn BuildUnitMultiList(&mut self) {
        for (key, msgtype) in self.FMT.iter() {
            // get unit and mult info
            let Some(fmtu) = self.FMTU.get(key) else { continue };
            let lookup = |d: &Dictionary<char, String>, c: char| d.iter().find(|(k, _)| **k == c).map(|(_, v)| v.clone());
            let units: Vec<Option<String>> = fmtu.0.chars().map(|a| lookup(&self.Unit, a)).collect();
            let multipliers: Vec<Option<String>> = fmtu.1.chars().map(|a| lookup(&self.Mult, a)).collect();
            let binfmts: Vec<char> = msgtype.2.chars().collect();
            let fields: Vec<&str> = msgtype.3.split(',').collect();
            let itemcount = fields.len();
            if binfmts.len() != itemcount {
                continue;
            }
            for i in 0..itemcount {
                let field = fields[i].trim().to_string();
                let unit = units.get(i).cloned().flatten();
                let binfmt = binfmts[i];
                // double.TryParse sets 0 on failure (a missing multiplier is null)
                let mut multi = multipliers
                    .get(i)
                    .cloned()
                    .flatten()
                    .and_then(|m| crate::dotnet::double_try_parse_culture(&m, &crate::dotnet::INVARIANT_CULTURE))
                    .unwrap_or(0.0);
                if matches!(binfmt, 'c' | 'C' | 'e' | 'E' | 'L') {
                    // these are scaled from the DF format * 100/1e7 etc; to ensure csv's
                    // continue to work we dont modify these values; 1 = no change
                    multi = 1.0;
                }
                self.UnitMultiList.push((msgtype.1.clone(), field, unit, multi));
            }
        }
    }

    /// `this[long indexin]`: the line as a `DFItem`.
    pub fn item(&mut self, indexin: i64) -> Result<DFItem, Exception> {
        if indexin > i32::MAX as i64 {
            return Err(Exception::Custom("Exception", "index too large".into()));
        }
        let index = indexin as i32;
        let (startoffset, endoffset) = self.bounds(index)?;
        let length = (endoffset - startoffset) as i32;
        // prevent multi io to file
        if startoffset != stream_position(&mut *self.basestream) {
            self.seek(startoffset);
        }
        if self.binary {
            let items = self.binlog.ReadMessageObjects(&mut *self.basestream, endoffset);
            DFItem::new(&mut self.dflog, items, indexin as i32)
        } else {
            let mut data = vec![0u8; length.max(0) as usize];
            stream_read(&mut *self.basestream, &mut data);
            self.dflog.GetDFItemFromLine(&binarylog::ascii_get_string(&data), indexin as i32)
        }
    }

    fn bounds(&mut self, index: i32) -> Result<(i64, i64), Exception> {
        let list_index = || Exception::ArgumentOutOfRange("Index was out of range. Must be non-negative and less than the size of the collection.\r\nParameter name: index".into());
        let startoffset = *usize::try_from(index).ok().and_then(|i| self.linestartoffset.get(i)).ok_or_else(list_index)?;
        let endoffset = if (index as i64 + 1) >= self.linestartoffset.len() as i64 {
            stream_length(&mut *self.basestream)
        } else {
            self.linestartoffset[index as usize + 1]
        };
        Ok((startoffset, endoffset))
    }

    /// `this[int index]`: the line's text (binary lines are converted), cached for the last
    /// index read.
    pub fn line(&mut self, index: i32) -> Result<String, Exception> {
        let (startoffset, endoffset) = self.bounds(index)?;
        let length = (endoffset - startoffset) as i32;
        // return cached value is same index
        if self.indexcachelineno == index as i64 {
            return Ok(self.currentindexcache.clone().unwrap_or_default());
        }
        if startoffset != stream_position(&mut *self.basestream) {
            self.seek(startoffset);
        }
        if self.binary {
            let len = stream_length(&mut *self.basestream);
            let answer = self.binlog.ReadMessage(&mut *self.basestream, len);
            self.currentindexcache = Some(answer);
        } else {
            let mut data = vec![0u8; length.max(0) as usize];
            stream_read(&mut *self.basestream, &mut data);
            self.currentindexcache = Some(binarylog::ascii_get_string(&data));
        }
        self.indexcachelineno = index as i64;
        Ok(self.currentindexcache.clone().unwrap_or_default())
    }

    /// `Clear()`
    pub fn Clear(&mut self) {
        self.basestream = Box::new(std::io::Cursor::new(Vec::new()));
        self._count = 0;
        self.linestartoffset.clear();
    }

    /// `Count`
    pub fn Count(&self) -> i32 {
        if self._count > i32::MAX as i64 {
            console_write_line("log line count is too large");
        }
        self._count as i32
    }

    /// `IsReadOnly`
    pub fn IsReadOnly(&self) -> bool {
        true
    }

    /// `GetEnumeratorTypeAll()` / `GetEnumerator()`
    pub fn lines(&self) -> LineEnumerator {
        LineEnumerator { position: 0 }
    }

    /// `GetEnumeratorType(types)`: "ACC[0]" asks for instance 0 of ACC, "ACC1" also asks for
    /// instance 0 of ACC; the line numbers are merged and sorted when more than one type is
    /// asked for.
    pub fn GetEnumeratorType(&self, types: &[&str]) -> TypeEnumerator {
        let mut instances: Vec<(String, Vec<String>)> = Vec::new();
        let mut add = |k: String, v: String| match instances.iter_mut().find(|(n, _)| *n == k) {
            Some((_, l)) => l.push(v),
            None => instances.push((k, vec![v])),
        };
        let re1 = Regex::new(r"(\w+)(\[([0-9]+)\])?").unwrap();
        let re2 = Regex::new(r"(\w+)([0-9]+)$").unwrap();
        for x in types {
            // match ACC[0] GPS[0] or ACC or GPS
            match re1.captures(x) {
                Some(m) => add(m[1].to_string(), m.get(3).map(|g| g.as_str().to_string()).unwrap_or_default()),
                None => add(x.to_string(), String::new()),
            }
            // match ACC1  GYR1
            if let Some(m) = re2.captures(x) {
                let n = m.get(2).map(|g| g.as_str().parse::<i32>().unwrap_or(0) - 1);
                add(m[1].to_string(), n.map(|v| v.to_string()).unwrap_or_default());
            }
        }
        // get the ids for the passed in types
        let mut slist: Vec<i64> = Vec::new();
        for (type_, _) in &instances {
            if let Some(lbl) = self.dflog.logformat.get(type_) {
                let typeid = lbl.Id as u8 as usize;
                slist.extend_from_slice(&self.messageindexline[typeid]);
            }
        }
        if types.len() > 1 {
            let n = slist.len();
            let cap = crate::dotnet::collections::list_capacity_for_count(n);
            let _ = crate::dotnet::collections::introspective_sort(&mut slist, cap, |a: &i64, b: &i64| Ok(a.cmp(b) as i32));
        }
        TypeEnumerator { instances, slist, pos: 0, progress: DateTime::now().parts().5 }
    }

    /// `Dispose()`
    pub fn Dispose(&mut self) {
        self.Clear();
        self.messageindex.clear();
    }

    /// `EndOfStream`
    pub fn EndOfStream(&self) -> bool {
        self.indexcachelineno >= self.linestartoffset.len() as i64 - 1
    }

    /// `SeenMessageTypes`
    pub fn SeenMessageTypes(&self) -> Vec<String> {
        let mut messagetypes = Vec::new();
        for (a, list) in self.messageindex.iter().enumerate() {
            if !list.is_empty() {
                if let Some(f) = self.FMT.get(&(a as i32)) {
                    messagetypes.push(f.1.clone());
                }
            }
        }
        messagetypes
    }

    /// `GetUnit(type, header)`: ("", 1) when unknown.
    pub fn GetUnit(&self, type_: &str, header: &str) -> (Option<String>, f64) {
        match self.UnitMultiList.iter().find(|t| t.0 == type_ && t.1 == header) {
            None => (Some(String::new()), 1.0),
            Some(t) => (t.2.clone(), t.3),
        }
    }

    /// `getInstanceIndex(type)`: the instance column (type at 0), -1 if unknown.
    pub fn getInstanceIndex(&self, type_: &str) -> i32 {
        let Some(lbl) = self.dflog.logformat.get(&type_.to_string()) else { return -1 };
        let Some(f) = self.FMTU.get(&lbl.Id) else { return -1 };
        f.0.find('#').map(|i| f.0[..i].chars().count() as i32).unwrap_or(-1) + 1
    }
}

fn write_cache_list(bw: &mut impl Write, list: &[i64]) -> std::io::Result<()> {
    bw.write_all(&(list.len() as i32).to_le_bytes())?;
    for item in list {
        bw.write_all(&item.to_le_bytes())?;
    }
    Ok(())
}

fn read_up_to(br: &mut impl Read, buf: &mut [u8]) -> Result<usize, Exception> {
    let mut n = 0;
    while n < buf.len() {
        match br.read(&mut buf[n..]) {
            Ok(0) => break,
            Ok(k) => n += k,
            Err(e) => return Err(Exception::Io(e.to_string())),
        }
    }
    Ok(n)
}

fn read_i32(br: &mut impl Read) -> Result<i32, Exception> {
    let mut b = [0u8; 4];
    if read_up_to(br, &mut b)? != 4 {
        return Err(Exception::EndOfStream("Unable to read beyond the end of the stream.".into()));
    }
    Ok(i32::from_le_bytes(b))
}

fn read_i64(br: &mut impl Read) -> Result<i64, Exception> {
    let mut b = [0u8; 8];
    if read_up_to(br, &mut b)? != 8 {
        return Err(Exception::EndOfStream("Unable to read beyond the end of the stream.".into()));
    }
    Ok(i64::from_le_bytes(b))
}

fn read_cache_list(br: &mut impl Read, maxCount: i64, maxValue: i64) -> Result<Vec<i64>, Exception> {
    let count = read_i32(br)?;
    if count < 0 || count as i64 > maxCount {
        return Err(bad_cache());
    }
    let mut list = Vec::with_capacity(count as usize);
    for _ in 0..count {
        let item = read_i64(br)?;
        if item < 0 || item > maxValue {
            return Err(bad_cache());
        }
        list.push(item);
    }
    Ok(list)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_log_index_and_types() {
        let data = super::super::binarylog::tests::sample();
        let mut b = DFLogBuffer::new(Box::new(std::io::Cursor::new(data)), "").unwrap();
        assert_eq!(b.Count(), 3);
        assert_eq!(b.line(2).unwrap(), "TST, 1000000, 7, hi\r\n");
        let mut it = b.GetEnumeratorType(&["TST"]);
        let mut item = it.next(&mut b).unwrap().unwrap();
        assert!(it.next(&mut b).is_none());
        assert_eq!(item.msgtype(), "TST");
        assert_eq!(item.timems(&b.dflog).unwrap(), 1000.0);
        assert_eq!(item.get(&b.dflog, "Msg").unwrap().as_deref(), Some("hi"));
        assert_eq!(b.FMT.get(&0x81).unwrap(), &(76, "TST".into(), "QBZ".into(), "TimeUS,V,Msg".into()));
        assert_eq!(b.SeenMessageTypes(), vec!["FMT".to_string(), "TST".to_string()]);
    }

    #[test]
    fn text_log_index() {
        let text = "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\r\nFMT, 129, 23, TST, QB, TimeUS,V\r\nTST, 5000, 1\r\nTST, 6000, 2\r\n";
        let mut b = DFLogBuffer::new(Box::new(std::io::Cursor::new(text.as_bytes().to_vec())), "").unwrap();
        assert_eq!(b.Count(), 4);
        let mut it = b.GetEnumeratorType(&["TST"]);
        let mut n = 0;
        while let Some(i) = it.next(&mut b) {
            let mut i = i.unwrap();
            n += 1;
            assert!(i.timems(&b.dflog).unwrap() >= 5.0);
        }
        assert_eq!(n, 2);
        assert_eq!(b.line(3).unwrap(), "TST, 6000, 2\r\n");
    }
}
