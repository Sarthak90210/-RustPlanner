//! Port of `ExtLibs/ArduPilot/Linux.cs`

use super::deviceinfo::DeviceInfo;

/// `MissionPlanner.Utilities.Linux`
pub struct Linux;

impl Linux {
    /// `GetAllCOMPorts()`: USB tty devices from sysfs, via the same bash/find/grep pipeline into
    /// `/tmp/usb.list`. Failures are printed and skipped.
    pub fn get_all_com_ports() -> Vec<DeviceInfo> {
        let mut ans = vec![];
        if !std::path::Path::new("/usr/bin/bash").exists() {
            return ans;
        }
        let run = || -> Result<(), String> {
            let mut proc = std::process::Command::new("/usr/bin/bash")
                .arg("-c")
                .arg("/usr/bin/find /sys/bus/usb/devices/usb*/ -name dev | /usr/bin/grep tty > /tmp/usb.list")
                .spawn()
                .map_err(|e| e.to_string())?;
            proc.wait().map_err(|e| e.to_string())?;
            Ok(())
        };
        use crate::dotnet::io::console_write_line;
        if let Err(e) = run() {
            console_write_line(&e);
            return ans;
        }
        let data = match std::fs::read_to_string("/tmp/usb.list") {
            Ok(d) => d,
            Err(e) => {
                console_write_line(&e.to_string());
                return ans;
            }
        };
        let lines: Vec<&str> = data.lines().collect();
        // Console.WriteLine(string[]) prints the type name
        console_write_line("System.String[]");
        for device in lines {
            let r = (|| -> Result<DeviceInfo, String> {
                let dir = std::path::Path::new(device).parent().map(|p| p.to_string_lossy().to_string()).unwrap_or_default();
                // Path.Combine(dir, "../../../")
                let pth = if dir.is_empty() { "../../../".to_string() } else { format!("{dir}/../../../") };
                let read = |f: &str| std::fs::read_to_string(format!("{pth}{f}")).map_err(|e| e.to_string());
                let product = read("product")?.trim_end().to_string();
                let vid = read("idVendor")?.trim_end().to_string();
                let pid = read("idProduct")?.trim_end().to_string();
                Ok(DeviceInfo {
                    board: product.clone(),
                    description: product.clone(),
                    name: product,
                    hardwareid: format!("USB\\VID_{vid}&PID_{pid}"),
                })
            })();
            match r {
                Ok(d) => ans.push(d),
                Err(e) => console_write_line(&e),
            }
        }
        ans
    }
}
