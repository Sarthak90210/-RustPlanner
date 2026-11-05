//! Port of `GCSViews/SITL.cs` StartSITL / CheckandGetSITLImage: download the ArduPilot SITL
//! binary for this platform, launch it with `-M<model> -O<home> -s<speed> --serial0 tcp:0`,
//! then connect over TCP 5760.

use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;

use parking_lot::Mutex;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Vehicle {
    Plane,
    Rover,
    Copter,
    Heli,
}

impl Vehicle {
    fn elf(&self) -> &'static str {
        match self {
            Vehicle::Plane => "ArduPlane",
            Vehicle::Rover => "ArduRover",
            Vehicle::Copter => "ArduCopter",
            Vehicle::Heli => "ArduHeli",
        }
    }
    pub fn default_model(&self) -> &'static str {
        match self {
            Vehicle::Plane => "plane",
            Vehicle::Rover => "rover",
            Vehicle::Copter => "+",
            Vehicle::Heli => "heli",
        }
    }
    fn linux_path(&self) -> (&'static str, &'static str) {
        match self {
            Vehicle::Plane => ("Plane", "arduplane"),
            Vehicle::Rover => ("Rover", "ardurover"),
            Vehicle::Copter => ("Copter", "arducopter"),
            Vehicle::Heli => ("Copter", "arducopter-heli"),
        }
    }
}

/// cmb_version: APFirmware.RELEASE_TYPES
pub const VERSIONS: [&str; 3] = ["Latest Dev", "Beta", "Stable"];

#[derive(Debug, Clone, Default)]
pub struct SitlStatus {
    pub busy: bool,
    pub message: String,
    pub error: Option<String>,
    /// Set once the simulator is running and listening on tcp:5760.
    pub ready: bool,
}

pub struct SitlRunner {
    pub status: Arc<Mutex<SitlStatus>>,
    child: Arc<Mutex<Option<Child>>>,
}

impl Default for SitlRunner {
    fn default() -> Self {
        Self { status: Arc::new(Mutex::new(SitlStatus::default())), child: Arc::new(Mutex::new(None)) }
    }
}

impl Drop for SitlRunner {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.lock().take() {
            let _ = c.kill();
        }
    }
}

pub struct StartArgs {
    pub vehicle: Vehicle,
    pub version: usize,
    pub model: String,
    pub home: (f64, f64, f64, f64),
    pub speed: u32,
    pub extra: String,
    pub wipe: bool,
}

fn sitl_dir() -> PathBuf {
    crate::platform::data_dir().join("sitl")
}

fn download(agent: &ureq::Agent, url: &str, to: &std::path::Path) -> Result<(), String> {
    let mut resp = agent.get(url).call().map_err(|e| format!("{url}: {e}"))?;
    let body = resp.body_mut().with_config().limit(400 * 1024 * 1024).read_to_vec().map_err(|e| e.to_string())?;
    if let Some(p) = to.parent() {
        let _ = std::fs::create_dir_all(p);
    }
    std::fs::write(to, body).map_err(|e| e.to_string())
}

impl SitlRunner {
    pub fn stop(&self) {
        if let Some(mut c) = self.child.lock().take() {
            let _ = c.kill();
        }
    }

    pub fn start(&self, args: StartArgs) {
        self.stop();
        let status = self.status.clone();
        let child = self.child.clone();
        *status.lock() = SitlStatus { busy: true, message: "Downloading sitl software".into(), ..Default::default() };
        std::thread::spawn(move || {
            let r = run(&args, &status);
            let mut s = status.lock();
            s.busy = false;
            match r {
                Ok(c) => {
                    *child.lock() = Some(c);
                    s.ready = true;
                    s.message = "SITL running".into();
                }
                Err(e) => s.error = Some(e),
            }
        });
    }
}

fn run(a: &StartArgs, status: &Arc<Mutex<SitlStatus>>) -> Result<Child, String> {
    let agent: ureq::Agent = ureq::Agent::config_builder().timeout_global(Some(std::time::Duration::from_secs(600))).build().into();
    let dir = sitl_dir();
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let exe: PathBuf;
    if cfg!(target_os = "windows") {
        // Tools/MissionPlanner/sitl/{,Beta/,CopterStable/...}: cygwin builds + DLLs
        let base = match (a.version, a.vehicle) {
            (1, _) => "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/Beta/".to_string(),
            (2, Vehicle::Copter | Vehicle::Heli) => "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/".into(),
            (2, Vehicle::Plane) => "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/PlaneStable/".into(),
            (2, Vehicle::Rover) => "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/RoverStable/".into(),
            _ => "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/".into(),
        };
        exe = dir.join(format!("{}.exe", a.vehicle.elf()));
        download(&agent, &format!("{base}{}.elf", a.vehicle.elf()), &exe)?;
        for dll in ["cygatomic-1.dll", "cyggcc_s-1.dll", "cyggcc_s-seh-1.dll", "cyggomp-1.dll", "cygiconv-2.dll", "cygintl-8.dll", "cygquadmath-0.dll", "cygssp-0.dll", "cygstdc++-6.dll", "cygwin1.dll"] {
            status.lock().message = format!("Downloading {dll}");
            let _ = download(&agent, &format!("{base}{dll}"), &dir.join(dll));
        }
    } else if cfg!(target_os = "linux") {
        let plat = if cfg!(target_arch = "x86_64") { "SITL_x86_64_linux_gnu" } else { "SITL_arm_linux_gnueabihf" };
        let rel = ["latest", "beta", "stable"][a.version.min(2)];
        let (veh, bin) = a.vehicle.linux_path();
        exe = dir.join(bin);
        download(&agent, &format!("https://firmware.ardupilot.org/{veh}/{rel}/{plat}/{bin}"), &exe)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let _ = std::fs::set_permissions(&exe, std::fs::Permissions::from_mode(0o755));
        }
    } else {
        return Err("Failed to download the SITL image: no prebuilt SITL for this platform".into());
    }

    // GetDefaultConfig: default params for the model (sim_vehicle.py's default_params_filename)
    status.lock().message = "Getting default params".into();
    let model = if a.model.is_empty() { a.vehicle.default_model().to_string() } else { a.model.clone() };
    let parm = match model.as_str() {
        "plane" => Some("default_params/plane.parm"),
        "rover" => Some("default_params/rover.parm"),
        "heli" => Some("default_params/copter-heli.parm"),
        "+" | "quad" | "X" | "x" => Some("default_params/copter.parm"),
        _ => None,
    };
    let simdir = dir.join(model.replace(['+', '/', '\\'], "plus"));
    std::fs::create_dir_all(&simdir).map_err(|e| e.to_string())?;
    let mut cmd = Command::new(&exe);
    cmd.current_dir(&simdir);
    cmd.arg(format!("-M{model}"));
    cmd.arg(format!("-O{},{},{},{}", a.home.0, a.home.1, a.home.2, a.home.3));
    cmd.arg(format!("-s{}", a.speed.max(1)));
    cmd.args(["--serial0", "tcp:0"]);
    if let Some(p) = parm {
        let local = simdir.join(p.replace('/', "_"));
        if download(&agent, &format!("https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/{p}"), &local).is_ok() || local.exists() {
            cmd.arg("--defaults").arg(&local);
        }
    }
    for tok in a.extra.split_whitespace() {
        cmd.arg(tok);
    }
    if a.wipe {
        cmd.arg("--wipe");
    }
    if let Ok(path) = std::env::var("PATH") {
        let sep = if cfg!(windows) { ";" } else { ":" };
        cmd.env("PATH", format!("{}{sep}{}{sep}{path}", dir.display(), simdir.display()));
    }
    cmd.env("HOME", &simdir);
    cmd.stdout(Stdio::null()).stderr(Stdio::null());
    status.lock().message = "Starting simulator".into();
    let child = cmd.spawn().map_err(|e| format!("Failed to start the simulator\n{e}"))?;
    std::thread::sleep(std::time::Duration::from_secs(2));
    Ok(child)
}
