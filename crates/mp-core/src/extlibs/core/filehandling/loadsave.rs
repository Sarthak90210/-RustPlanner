//! Port of `ExtLibs/Core/FileHandling/LoadSave.cs`

use crate::dotnet::io::{read_all_text, StreamWriter};
use crate::dotnet::{to_upper, Exception};

/// `internal class FileFilter`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FileFilter {
    pub Name: Option<String>,
    pub Extension: String,
    pub Extensions: Option<Vec<String>>,
}

impl FileFilter {
    /// `FileFilter(string name, params string[] exts)`: the list has leading dots removed but
    /// `Extension` keeps `exts[0]` as given; an empty array throws.
    pub fn new_multi(name: Option<&str>, exts: &[&str]) -> Result<Self, Exception> {
        let extensions: Vec<String> = exts.iter().map(|e| e.strip_prefix('.').unwrap_or(e).to_string()).collect();
        let first = exts.first().ok_or_else(crate::dotnet::index_out_of_range)?;
        Ok(FileFilter { Name: name.map(|s| s.to_string()), Extension: first.to_string(), Extensions: Some(extensions) })
    }

    /// `FileFilter(string name, string ext)`
    pub fn new_named(name: Option<&str>, ext: &str) -> Self {
        let ext = ext.strip_prefix('.').unwrap_or(ext);
        FileFilter { Name: name.map(|s| s.to_string()), Extension: ext.to_string(), Extensions: None }
    }

    /// `FileFilter(string ext)`: the name is the upper-cased extension.
    pub fn new(ext: &str) -> Self {
        let ext = ext.strip_prefix('.').unwrap_or(ext);
        FileFilter { Name: Some(to_upper(ext)), Extension: ext.to_string(), Extensions: None }
    }

    /// `ExtensionString`: `*.ext`, or `*.a;*.b` for two or more extensions.
    pub fn extension_string(&self) -> String {
        match &self.Extensions {
            Some(e) if e.len() >= 2 => e.iter().map(|x| format!("*.{x}")).collect::<Vec<_>>().join(";"),
            _ => format!("*.{}", self.Extension),
        }
    }
}

/// `public class LoadSave`
#[allow(non_snake_case)]
#[derive(Debug, Clone, Default)]
pub struct LoadSave {
    file_type: String,
    filter: String,
    filter_list: Vec<FileFilter>,
    pub FilePath: Option<String>,
    pub Data: Option<String>,
    pub InitialDirectory: Option<String>,
    pub DefaultFilename: Option<String>,
    m_allow_all_files: bool,
}

impl LoadSave {
    /// `new LoadSave()`: `this("*")`.
    pub fn new_default() -> Self {
        Self::new("*")
    }

    /// `new LoadSave(string filterName, params string[] exts)`
    pub fn new_with_filter(filter_name: Option<&str>, exts: &[&str]) -> Result<Self, Exception> {
        let mut l = LoadSave::default();
        l.init(FileFilter::new_multi(filter_name, exts)?);
        Ok(l)
    }

    /// `new LoadSave(string ext)`
    pub fn new(ext: &str) -> Self {
        let mut l = LoadSave::default();
        l.init(FileFilter::new(ext));
        l
    }

    fn init(&mut self, d_filter: FileFilter) {
        self.file_type = d_filter.Extension.clone();
        self.filter_list.push(d_filter);
        self.regen_filter();
        self.FilePath = None;
        self.InitialDirectory = None;
        self.DefaultFilename = None;
    }

    /// The `fileType` field.
    pub fn file_type(&self) -> &str {
        &self.file_type
    }

    /// The `filter` field (`OpenFileDialog.Filter` syntax).
    pub fn filter(&self) -> &str {
        &self.filter
    }

    /// `AddFilter(string ext)`
    pub fn add_filter(&mut self, ext: &str) {
        self.filter_list.push(FileFilter::new(ext));
        self.regen_filter();
    }

    /// `AddFilter(string name, params string[] exts)`
    pub fn add_filter_named(&mut self, name: Option<&str>, exts: &[&str]) -> Result<(), Exception> {
        self.filter_list.push(FileFilter::new_multi(name, exts)?);
        self.regen_filter();
        Ok(())
    }

    /// `AllowAllFiles`
    pub fn allow_all_files(&self) -> bool {
        self.m_allow_all_files
    }

    /// `AllowAllFiles` setter
    pub fn set_allow_all_files(&mut self, value: bool) {
        self.m_allow_all_files = value;
        self.regen_filter();
    }

    fn regen_filter(&mut self) {
        self.filter = String::new();
        for f in &self.filter_list {
            if !self.filter.is_empty() {
                self.filter.push('|');
            }
            let es = f.extension_string();
            self.filter += &format!("{} ({})|{}", f.Name.as_deref().unwrap_or(""), es, es);
        }
        if self.m_allow_all_files {
            self.filter += "|All files (*.*)|*.*";
        }
    }

    /// `HasSavePath`
    pub fn has_save_path(&self) -> bool {
        self.FilePath.is_some()
    }

    /// `FileExists()`
    pub fn file_exists(&self) -> bool {
        self.FilePath.as_deref().is_some_and(|p| std::path::Path::new(p).is_file())
    }

    /// `Save()`: nothing happens without a path.
    pub fn save(&mut self) -> Result<(), Exception> {
        if self.Data.is_none() {
            return Err(Exception::Other("No data to save".into()));
        }
        if self.has_save_path() {
            self.save_data()?;
        }
        Ok(())
    }

    /// `SaveAs(string filePath)`
    pub fn save_as(&mut self, file_path: Option<&str>) -> Result<(), Exception> {
        self.FilePath = file_path.map(|s| s.to_string());
        self.save()
    }

    /// `SaveAs(string data, string filePath)`
    pub fn save_as_data(&mut self, data: Option<&str>, file_path: Option<&str>) -> Result<(), Exception> {
        self.FilePath = file_path.map(|s| s.to_string());
        self.Data = data.map(|s| s.to_string());
        self.save()
    }

    /// `Save(string data)`
    pub fn save_data_str(&mut self, data: Option<&str>) -> Result<(), Exception> {
        self.Data = data.map(|s| s.to_string());
        self.save()
    }

    fn save_data(&self) -> Result<(), Exception> {
        if let Some(path) = &self.FilePath {
            let mut w = StreamWriter::create(path)?;
            w.write(self.Data.as_deref().unwrap_or(""))?;
            w.close()?;
        }
        Ok(())
    }

    /// `Load(string path)`
    pub fn load(&mut self, path: Option<&str>) -> Result<bool, Exception> {
        self.Data = self.get_file_contents_from_path(path)?;
        if self.Data.is_some() {
            self.FilePath = path.map(|s| s.to_string());
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// `NewFile(string data)`
    pub fn new_file_data(&mut self, data: Option<&str>) {
        self.Data = data.map(|s| s.to_string());
        self.FilePath = None;
    }

    /// `NewFile()`
    pub fn new_file(&mut self) {
        self.Data = None;
        self.FilePath = None;
    }

    /// `GetFileContentsFromPath(string f_path)`: sets `FilePath` before opening the file.
    pub fn get_file_contents_from_path(&mut self, f_path: Option<&str>) -> Result<Option<String>, Exception> {
        match f_path {
            Some(p) => {
                self.FilePath = Some(p.to_string());
                Ok(Some(read_all_text(p)?))
            }
            None => Ok(None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filters() {
        let mut l = LoadSave::new(".xml");
        assert_eq!(l.filter(), "XML (*.xml)|*.xml");
        l.add_filter_named(Some("Images"), &["bmp", ".jpg"]).unwrap();
        l.set_allow_all_files(true);
        assert_eq!(l.filter(), "XML (*.xml)|*.xml|Images (*.bmp;*.jpg)|*.bmp;*.jpg|All files (*.*)|*.*");
        // a single extension keeps its dot in Extension
        let one = LoadSave::new_with_filter(Some("T"), &[".txt"]).unwrap();
        assert_eq!(one.filter(), "T (*..txt)|*..txt");
        assert!(LoadSave::new_with_filter(Some("T"), &[]).is_err());
    }

    #[test]
    fn save_load() {
        let dir = std::env::temp_dir().join(format!("mp_loadsave_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("a.txt");
        let ps = p.to_string_lossy().into_owned();
        let mut l = LoadSave::new_default();
        assert!(l.save().is_err());
        l.save_data_str(Some("x")).unwrap(); // no path: nothing written
        l.save_as(Some(&ps)).unwrap();
        let mut m = LoadSave::new_default();
        assert!(m.load(Some(&ps)).unwrap());
        assert_eq!(m.Data.as_deref(), Some("x"));
        assert!(!m.load(None).unwrap());
        let missing = dir.join("missing.txt").to_string_lossy().into_owned();
        assert!(m.load(Some(&missing)).is_err());
        assert_eq!(m.FilePath.as_deref(), Some(missing.as_str()));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
