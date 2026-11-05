//! SRTM terrain (`ExtLibs/Utilities/srtm.cs`): 1" / 3" .hgt tiles from
//! terrain.ardupilot.org, cached under the data directory, bilinearly interpolated.
//! Lookups never block: a missing tile is fetched in the background and `None` is returned
//! until it arrives.

use std::collections::{HashMap, HashSet};
use std::io::Read;
use std::path::PathBuf;
use std::sync::{Arc, Mutex, OnceLock};

struct Tile {
    size: usize,
    data: Vec<i16>,
}

#[derive(Default)]
struct Store {
    tiles: HashMap<String, Option<Arc<Tile>>>,
    pending: HashSet<String>,
}

fn store() -> &'static Mutex<Store> {
    static S: OnceLock<Mutex<Store>> = OnceLock::new();
    S.get_or_init(Default::default)
}

fn dir() -> PathBuf {
    let d = crate::platform::data_dir().join("srtm");
    let _ = std::fs::create_dir_all(&d);
    d
}

fn tile_name(lat: f64, lng: f64) -> String {
    let (y, x) = (lat.floor() as i32, lng.floor() as i32);
    format!("{}{:02}{}{:03}", if y >= 0 { 'N' } else { 'S' }, y.abs(), if x >= 0 { 'E' } else { 'W' }, x.abs())
}

fn load_file(name: &str) -> Option<Tile> {
    let bytes = std::fs::read(dir().join(format!("{name}.hgt"))).ok()?;
    let size = match bytes.len() {
        n if n == 3601 * 3601 * 2 => 3601,
        n if n == 1201 * 1201 * 2 => 1201,
        _ => return None,
    };
    let data = bytes.chunks_exact(2).map(|b| i16::from_be_bytes([b[0], b[1]])).collect();
    Some(Tile { size, data })
}

fn download(name: String) {
    std::thread::spawn(move || {
        let mut ok = false;
        for base in ["https://terrain.ardupilot.org/SRTM1/", "https://terrain.ardupilot.org/SRTM3/"] {
            if let Ok(zip) = crate::firmware_ui::get(&format!("{base}{name}.hgt.zip")) {
                if let Ok(mut ar) = zip::ZipArchive::new(std::io::Cursor::new(zip)) {
                    if let Ok(mut f) = ar.by_index(0) {
                        let mut buf = Vec::new();
                        if f.read_to_end(&mut buf).is_ok() && std::fs::write(dir().join(format!("{name}.hgt")), &buf).is_ok() {
                            ok = true;
                            break;
                        }
                    }
                }
            }
        }
        let mut s = store().lock().unwrap();
        s.pending.remove(&name);
        if ok {
            s.tiles.remove(&name);
        } else {
            // ocean / unavailable tile: remember so we stop asking
            s.tiles.insert(name, None);
        }
    });
}

/// Terrain altitude (m AMSL) at a point, if the tile is available.
pub fn altitude(lat: f64, lng: f64) -> Option<f64> {
    let name = tile_name(lat, lng);
    let tile = {
        let mut s = store().lock().unwrap();
        match s.tiles.get(&name) {
            Some(t) => t.clone(),
            None => match load_file(&name) {
                Some(t) => {
                    let t = Arc::new(t);
                    s.tiles.insert(name.clone(), Some(t.clone()));
                    Some(t)
                }
                None => {
                    if s.pending.insert(name.clone()) {
                        download(name);
                    }
                    None
                }
            },
        }
    }?;
    let n = (tile.size - 1) as f64;
    let fx = (lng - lng.floor()) * n;
    let fy = (lat.floor() + 1.0 - lat) * n;
    let (x0, y0) = (fx.floor() as usize, fy.floor() as usize);
    let (x1, y1) = ((x0 + 1).min(tile.size - 1), (y0 + 1).min(tile.size - 1));
    let at = |x: usize, y: usize| tile.data[y * tile.size + x] as f64;
    let (tx, ty) = (fx - x0 as f64, fy - y0 as f64);
    let v = at(x0, y0) * (1.0 - tx) * (1.0 - ty) + at(x1, y0) * tx * (1.0 - ty) + at(x0, y1) * (1.0 - tx) * ty + at(x1, y1) * tx * ty;
    (v > -1000.0).then_some(v)
}

#[cfg(test)]
mod tests {
    #[test]
    fn names() {
        assert_eq!(super::tile_name(-35.36, 149.16), "S36E149");
        assert_eq!(super::tile_name(51.5, -0.12), "N51W001");
    }
}
