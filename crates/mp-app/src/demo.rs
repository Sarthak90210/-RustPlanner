//! "Demo flight" for the built-in simulator: upload a square mission, arm in Guided,
//! take off, then switch to Auto.

use std::time::{Duration, Instant};

use mp_core::link::{Link, LinkCommand, LinkEvent};
use mp_core::mavcmd;
use mp_core::mission::Locationwp;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    WaitParams,
    WaitUpload,
    WaitArm,
    Climb,
    Done,
}

pub struct Demo {
    stage: Stage,
    since: Instant,
}

impl Default for Demo {
    fn default() -> Self {
        Self {
            stage: Stage::WaitParams,
            since: Instant::now(),
        }
    }
}

const GUIDED: u32 = 4;
const AUTO: u32 = 3;

impl Demo {
    fn go(&mut self, s: Stage) {
        self.stage = s;
        self.since = Instant::now();
    }

    /// Returns `true` once finished (or failed).
    pub fn tick(&mut self, link: &Link, events: &[LinkEvent]) -> bool {
        let (params_done, alt, home) = {
            let s = link.shared.read();
            (s.params_complete, s.cs.alt, s.cs.home)
        };
        match self.stage {
            Stage::WaitParams => {
                if let (true, Some(h)) = (params_done, home) {
                    let mut items = vec![Locationwp {
                        id: mavcmd::WAYPOINT,
                        lat: h.lat,
                        lng: h.lng,
                        alt: h.alt as f32,
                        ..Default::default()
                    }];
                    items.push(Locationwp {
                        id: mavcmd::TAKEOFF,
                        alt: 30.0,
                        frame: 3,
                        ..Default::default()
                    });
                    // 200 m square starting 150 m north of home, climbing 10 m per leg
                    let mut corner = h.newpos(0.0, 150.0);
                    items.push(Locationwp::waypoint(corner.lat, corner.lng, 40.0, 3));
                    for (i, brg) in [90.0, 180.0, 270.0, 0.0].iter().enumerate() {
                        corner = corner.newpos(*brg, 200.0);
                        items.push(Locationwp::waypoint(
                            corner.lat,
                            corner.lng,
                            50.0 + i as f32 * 10.0,
                            3,
                        ));
                    }
                    items.push(Locationwp {
                        id: mavcmd::RETURN_TO_LAUNCH,
                        frame: 3,
                        ..Default::default()
                    });
                    link.send(LinkCommand::WriteMission {
                        mission_type: 0,
                        items,
                    });
                    self.go(Stage::WaitUpload);
                }
            }
            Stage::WaitUpload => {
                for e in events {
                    if let LinkEvent::MissionUploaded { result, .. } = e {
                        if result.is_err() {
                            return true;
                        }
                        link.send(LinkCommand::ReadMission { mission_type: 0 });
                        link.send(LinkCommand::SetMode(GUIDED));
                        link.send(LinkCommand::Arm {
                            arm: true,
                            force: false,
                        });
                        self.go(Stage::WaitArm);
                    }
                }
            }
            Stage::WaitArm => {
                if link.shared.read().cs.armed {
                    link.send(LinkCommand::CommandLong {
                        command: mavcmd::TAKEOFF,
                        params: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0],
                    });
                    self.go(Stage::Climb);
                } else if self.since.elapsed() > Duration::from_secs(3) {
                    link.send(LinkCommand::Arm {
                        arm: true,
                        force: false,
                    });
                    self.since = Instant::now();
                }
            }
            Stage::Climb => {
                if alt > 25.0 {
                    link.send(LinkCommand::SetMode(AUTO));
                    self.go(Stage::Done);
                }
            }
            Stage::Done => return true,
        }
        self.since.elapsed() > Duration::from_secs(60)
    }
}
