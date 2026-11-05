//! Port of `MAVLinkInterface`: owns the transport on a background thread and runs the
//! heartbeat, data-stream, parameter and mission protocols.
//!
//! The UI talks to it through [`Link`]: commands go in over a channel, state is shared behind
//! an `RwLock`, and one-shot results come back as [`LinkEvent`]s.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use mavlink::dialects::ardupilotmega::*;
use mavlink::error::MessageReadError;
use mavlink::{MavHeader, MavlinkVersion};
use num_traits::FromPrimitive;
use parking_lot::RwLock;

use crate::comms::{self, ConnectionSpec, Transport};
use crate::firmware::Firmware;
use crate::mission::Locationwp;
use crate::state::{CurrentState, StatusMessage};

pub const GCS_SYSID: u8 = 255;
pub const GCS_COMPID: u8 = 190; // MAV_COMP_ID_MISSIONPLANNER

const HEARTBEAT_TIMEOUT: Duration = Duration::from_secs(30);
const LINK_LOST: Duration = Duration::from_secs(5);
const MISSION_TIMEOUT: Duration = Duration::from_millis(1500);
const PARAM_IDLE: Duration = Duration::from_millis(1000);
const CMD_TIMEOUT: Duration = Duration::from_millis(1500);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinkStatus {
    #[default]
    Disconnected,
    Connecting,
    Connected,
}

#[derive(Debug, Clone)]
pub struct Param {
    pub value: f32,
    pub ptype: u8,
    pub index: u16,
}

#[derive(Debug, Clone)]
pub struct Progress {
    pub label: String,
    pub done: usize,
    pub total: usize,
}

/// Everything the UI may read about the link (MAVState + link stats).
#[derive(Debug, Default)]
pub struct LinkShared {
    pub status: LinkStatus,
    pub spec: Option<ConnectionSpec>,
    pub cs: CurrentState,
    pub params: BTreeMap<String, Param>,
    pub param_count: u16,
    pub params_complete: bool,
    pub messages: Vec<StatusMessage>,
    pub progress: Option<Progress>,
    pub last_packet: Option<Instant>,
    pub bytes_received: u64,
    pub link_lost: bool,
    pub autopilot_version: Option<(u32, u64)>,
}

impl LinkShared {
    pub fn firmware(&self) -> Firmware {
        self.cs.firmware
    }
}

#[derive(Debug, Clone)]
pub enum LinkEvent {
    Connected,
    Disconnected(Option<String>),
    ParamsLoaded(usize),
    ParamSet {
        name: String,
        ok: bool,
    },
    MissionDownloaded {
        mission_type: u8,
        items: Vec<Locationwp>,
    },
    MissionUploaded {
        mission_type: u8,
        result: Result<(), String>,
    },
    CommandAck {
        command: u16,
        result: String,
        ok: bool,
    },
}

#[derive(Debug, Clone)]
pub enum LinkCommand {
    Disconnect,
    Send(Box<MavMessage>),
    RequestParams,
    SetParam {
        name: String,
        value: f32,
    },
    ReadMission {
        mission_type: u8,
    },
    /// Items including home at index 0 (for MAV_MISSION_TYPE_MISSION).
    WriteMission {
        mission_type: u8,
        items: Vec<Locationwp>,
    },
    SetMode(u32),
    Arm {
        arm: bool,
        force: bool,
    },
    CommandLong {
        command: u16,
        params: [f32; 7],
    },
    SetCurrentWp(u16),
    /// Guided "Fly To Here".
    Guided {
        lat: f64,
        lng: f64,
        alt: f32,
    },
}

/// Handle to a running link (equivalent of `MainV2.comPort`).
pub struct Link {
    pub shared: Arc<RwLock<LinkShared>>,
    tx: Sender<LinkCommand>,
    pub events: Receiver<LinkEvent>,
    thread: Option<JoinHandle<()>>,
}

impl Link {
    pub fn connect(spec: ConnectionSpec) -> Self {
        let shared = Arc::new(RwLock::new(LinkShared {
            status: LinkStatus::Connecting,
            spec: Some(spec.clone()),
            progress: Some(Progress {
                label: "Connecting...".into(),
                done: 0,
                total: 0,
            }),
            ..Default::default()
        }));
        let (tx, rx) = crossbeam_channel::unbounded();
        let (etx, erx) = crossbeam_channel::unbounded();
        let sh = shared.clone();
        let thread = std::thread::Builder::new()
            .name("mavlink".into())
            .spawn(move || {
                let result = comms::open(&spec)
                    .and_then(|t| Worker::new(t, sh.clone(), rx, etx.clone()).run());
                let err = result.err().map(|e| e.to_string());
                {
                    let mut s = sh.write();
                    s.status = LinkStatus::Disconnected;
                    s.progress = None;
                }
                let _ = etx.send(LinkEvent::Disconnected(err));
            })
            .expect("spawn mavlink thread");
        Self {
            shared,
            tx,
            events: erx,
            thread: Some(thread),
        }
    }

    pub fn send(&self, cmd: LinkCommand) {
        let _ = self.tx.send(cmd);
    }

    pub fn status(&self) -> LinkStatus {
        self.shared.read().status
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        let _ = self.tx.send(LinkCommand::Disconnect);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct ParamFetch {
    received: BTreeSet<u16>,
    last_rx: Instant,
    retries: u32,
}

struct PendingSet {
    value: f32,
    sent: Instant,
    tries: u32,
}

enum MissionOp {
    Download {
        mission_type: MavMissionType,
        count: Option<u16>,
        items: Vec<Locationwp>,
        last_tx: Instant,
        retries: u32,
        use_int: bool,
    },
    Upload {
        mission_type: MavMissionType,
        items: Vec<Locationwp>,
        last_tx: Instant,
        retries: u32,
        max_requested: Option<u16>,
    },
}

struct PendingCmd {
    command: MavCmd,
    params: [f32; 7],
    sent: Instant,
    tries: u32,
}

struct Worker {
    reader: mavlink::MavlinkReader<Box<dyn Transport>>,
    shared: Arc<RwLock<LinkShared>>,
    rx: Receiver<LinkCommand>,
    events: Sender<LinkEvent>,
    seq: u8,
    target_sys: u8,
    target_comp: u8,
    started: Instant,
    last_heartbeat_tx: Instant,
    got_heartbeat: bool,
    param_fetch: Option<ParamFetch>,
    pending_sets: HashMap<String, PendingSet>,
    mission: Option<MissionOp>,
    pending_cmd: Option<PendingCmd>,
}

fn is_timeout(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
    )
}

fn param_name(id: &mavlink::types::CharArray<16>) -> String {
    let bytes: &[u8] = &id[..];
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).to_string()
}

fn now_hms() -> String {
    chrono::Local::now().format("%H:%M:%S").to_string()
}

impl Worker {
    fn new(
        transport: Box<dyn Transport>,
        shared: Arc<RwLock<LinkShared>>,
        rx: Receiver<LinkCommand>,
        events: Sender<LinkEvent>,
    ) -> Self {
        let now = Instant::now();
        Self {
            reader: mavlink::MavlinkReader::new(transport),
            shared,
            rx,
            events,
            seq: 0,
            target_sys: 1,
            target_comp: 1,
            started: now,
            last_heartbeat_tx: now - Duration::from_secs(10),
            got_heartbeat: false,
            param_fetch: None,
            pending_sets: HashMap::new(),
            mission: None,
            pending_cmd: None,
        }
    }

    fn run(mut self) -> io::Result<()> {
        loop {
            while let Ok(cmd) = self.rx.try_recv() {
                if matches!(cmd, LinkCommand::Disconnect) {
                    return Ok(());
                }
                self.handle_command(cmd)?;
            }

            match self.reader.read_any_message::<MavMessage>() {
                Ok((hdr, msg)) => self.handle_message(hdr, msg)?,
                Err(MessageReadError::Io(e)) if is_timeout(&e) => {}
                Err(MessageReadError::Io(e)) => return Err(e),
                Err(MessageReadError::Parse(e)) => log::trace!("mavlink parse: {e:?}"),
            }

            self.tick()?;
        }
    }

    fn send(&mut self, msg: MavMessage) -> io::Result<()> {
        let hdr = MavHeader {
            system_id: GCS_SYSID as u32,
            component_id: GCS_COMPID,
            sequence: self.seq,
        };
        self.seq = self.seq.wrapping_add(1);
        let w = self.reader.get_mut();
        mavlink::write_versioned_msg(w, MavlinkVersion::V2, hdr, &msg)
            .map_err(|e| io::Error::other(format!("{e:?}")))?;
        w.flush()
    }

    fn event(&self, e: LinkEvent) {
        let _ = self.events.send(e);
    }

    fn set_progress(&self, p: Option<Progress>) {
        self.shared.write().progress = p;
    }

    // ---------------------------------------------------------------- commands from the UI

    fn handle_command(&mut self, cmd: LinkCommand) -> io::Result<()> {
        match cmd {
            LinkCommand::Disconnect => {}
            LinkCommand::Send(m) => self.send(*m)?,
            LinkCommand::RequestParams => self.start_param_fetch()?,
            LinkCommand::SetParam { name, value } => {
                self.send_param_set(&name, value)?;
                self.pending_sets.insert(
                    name,
                    PendingSet {
                        value,
                        sent: Instant::now(),
                        tries: 1,
                    },
                );
            }
            LinkCommand::ReadMission { mission_type } => {
                let mt = MavMissionType::from_u8(mission_type)
                    .unwrap_or(MavMissionType::MAV_MISSION_TYPE_MISSION);
                self.send(MavMessage::MISSION_REQUEST_LIST(
                    MISSION_REQUEST_LIST_DATA {
                        target_system: self.target_sys,
                        target_component: self.target_comp,
                        mission_type: mt,
                    },
                ))?;
                self.mission = Some(MissionOp::Download {
                    mission_type: mt,
                    count: None,
                    items: Vec::new(),
                    last_tx: Instant::now(),
                    retries: 0,
                    use_int: true,
                });
                self.set_progress(Some(Progress {
                    label: "Getting WP count".into(),
                    done: 0,
                    total: 0,
                }));
            }
            LinkCommand::WriteMission {
                mission_type,
                items,
            } => {
                let mt = MavMissionType::from_u8(mission_type)
                    .unwrap_or(MavMissionType::MAV_MISSION_TYPE_MISSION);
                self.send_mission_count(mt, items.len() as u16)?;
                self.set_progress(Some(Progress {
                    label: "Set total wps".into(),
                    done: 0,
                    total: items.len(),
                }));
                self.mission = Some(MissionOp::Upload {
                    mission_type: mt,
                    items,
                    last_tx: Instant::now(),
                    retries: 0,
                    max_requested: None,
                });
            }
            LinkCommand::SetMode(mode) => self.command_long(
                MavCmd::MAV_CMD_DO_SET_MODE,
                [1.0, mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
            )?,
            LinkCommand::Arm { arm, force } => self.command_long(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                [
                    if arm { 1.0 } else { 0.0 },
                    if force { 21196.0 } else { 0.0 },
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )?,
            LinkCommand::CommandLong { command, params } => match MavCmd::from_u16(command) {
                Some(c) => self.command_long(c, params)?,
                None => self.event(LinkEvent::CommandAck {
                    command,
                    result: "Unknown command".into(),
                    ok: false,
                }),
            },
            LinkCommand::SetCurrentWp(seq) => {
                self.send(MavMessage::MISSION_SET_CURRENT(MISSION_SET_CURRENT_DATA {
                    seq,
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                }))?
            }
            LinkCommand::Guided { lat, lng, alt } => {
                // setGuidedModeWP: MISSION_ITEM_INT with current = 2
                let wp = Locationwp::waypoint(lat, lng, alt, 3);
                if let Some(mut item) = wp.to_mission_item_int(
                    0,
                    self.target_sys,
                    self.target_comp,
                    MavMissionType::MAV_MISSION_TYPE_MISSION,
                ) {
                    item.current = 2;
                    self.send(MavMessage::MISSION_ITEM_INT(item))?;
                }
            }
        }
        Ok(())
    }

    fn command_long(&mut self, command: MavCmd, params: [f32; 7]) -> io::Result<()> {
        self.send_command_long(command, params, 0)?;
        self.pending_cmd = Some(PendingCmd {
            command,
            params,
            sent: Instant::now(),
            tries: 1,
        });
        Ok(())
    }

    fn send_command_long(
        &mut self,
        command: MavCmd,
        p: [f32; 7],
        confirmation: u8,
    ) -> io::Result<()> {
        self.send(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: p[0],
            param2: p[1],
            param3: p[2],
            param4: p[3],
            param5: p[4],
            param6: p[5],
            param7: p[6],
            command,
            target_system: self.target_sys,
            target_component: self.target_comp,
            confirmation,
        }))
    }

    fn send_param_set(&mut self, name: &str, value: f32) -> io::Result<()> {
        let ptype = self
            .shared
            .read()
            .params
            .get(name)
            .and_then(|p| MavParamType::from_u8(p.ptype))
            .unwrap_or(MavParamType::MAV_PARAM_TYPE_REAL32);
        self.send(MavMessage::PARAM_SET(PARAM_SET_DATA {
            param_value: value,
            target_system: self.target_sys,
            target_component: self.target_comp,
            param_id: name.into(),
            param_type: ptype,
        }))
    }

    fn send_mission_count(&mut self, mt: MavMissionType, count: u16) -> io::Result<()> {
        self.send(MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count,
            target_system: self.target_sys,
            target_component: self.target_comp,
            mission_type: mt,
            opaque_id: 0,
        }))
    }

    fn start_param_fetch(&mut self) -> io::Result<()> {
        {
            let mut s = self.shared.write();
            s.params.clear();
            s.params_complete = false;
            s.param_count = 0;
            s.progress = Some(Progress {
                label: "Getting Params".into(),
                done: 0,
                total: 0,
            });
        }
        self.param_fetch = Some(ParamFetch {
            received: BTreeSet::new(),
            last_rx: Instant::now(),
            retries: 0,
        });
        self.send(MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
            target_system: self.target_sys,
            target_component: self.target_comp,
        }))
    }

    fn request_datastreams(&mut self) -> io::Result<()> {
        // Mission Planner's default rates (CurrentState.rate*), attitude bumped for a smooth HUD.
        let streams = [
            (MavDataStream::MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            (MavDataStream::MAV_DATA_STREAM_POSITION, 3),
            (MavDataStream::MAV_DATA_STREAM_EXTRA1, 10),
            (MavDataStream::MAV_DATA_STREAM_EXTRA2, 4),
            (MavDataStream::MAV_DATA_STREAM_EXTRA3, 2),
            (MavDataStream::MAV_DATA_STREAM_RAW_SENSORS, 2),
            (MavDataStream::MAV_DATA_STREAM_RC_CHANNELS, 2),
            (MavDataStream::MAV_DATA_STREAM_RAW_CONTROLLER, 2),
        ];
        for (stream, rate) in streams {
            self.send(MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
                req_message_rate: rate,
                target_system: self.target_sys,
                target_component: self.target_comp,
                req_stream_id: stream,
                start_stop: 1,
            }))?;
        }
        // ask for AUTOPILOT_VERSION and HOME_POSITION
        self.send_command_long(
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            [148.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            0,
        )?;
        self.send_command_long(
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            [242.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            0,
        )
    }

    // ---------------------------------------------------------------- incoming messages

    fn handle_message(&mut self, hdr: MavHeader, msg: MavMessage) -> io::Result<()> {
        // Pick the first autopilot that heartbeats as our target (MAVlist[0]).
        if !self.got_heartbeat {
            if let MavMessage::HEARTBEAT(hb) = &msg {
                if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID
                    && hb.mavtype != MavType::MAV_TYPE_GCS
                {
                    self.got_heartbeat = true;
                    self.target_sys = hdr.system_id as u8;
                    self.target_comp = hdr.component_id;
                    {
                        let mut s = self.shared.write();
                        s.status = LinkStatus::Connected;
                        s.cs.sysid = hdr.system_id as u8;
                        s.cs.compid = hdr.component_id;
                        s.progress = None;
                    }
                    self.event(LinkEvent::Connected);
                    self.request_datastreams()?;
                    self.start_param_fetch()?;
                }
            }
        }
        if hdr.system_id != self.target_sys as u32 || !self.got_heartbeat {
            return Ok(());
        }
        // Only telemetry from the autopilot component updates CurrentState.
        let from_ap = hdr.component_id == self.target_comp;
        {
            let mut s = self.shared.write();
            s.last_packet = Some(Instant::now());
            s.link_lost = false;
            if from_ap {
                s.cs.note_packet(hdr.sequence);
                s.cs.update(&msg);
            }
            if let MavMessage::STATUSTEXT(t) = &msg {
                let text = t
                    .text
                    .to_str()
                    .unwrap_or("")
                    .trim_end_matches('\0')
                    .to_string();
                s.messages.push(StatusMessage {
                    time: now_hms(),
                    severity: t.severity as u8,
                    text,
                });
                if s.messages.len() > 1000 {
                    s.messages.remove(0);
                }
            }
            if let MavMessage::AUTOPILOT_VERSION(v) = &msg {
                s.autopilot_version = Some((v.flight_sw_version, v.capabilities.bits()));
            }
        }

        match msg {
            MavMessage::PARAM_VALUE(p) => self.on_param_value(p),
            MavMessage::MISSION_COUNT(c) => self.on_mission_count(c)?,
            MavMessage::MISSION_ITEM_INT(m) => {
                let wp = Locationwp::from_mission_item_int(&m);
                self.on_mission_item(m.seq, m.mission_type, wp)?
            }
            MavMessage::MISSION_ITEM(m) => {
                let wp = Locationwp::from_mission_item(&m);
                self.on_mission_item(m.seq, m.mission_type, wp)?
            }
            MavMessage::MISSION_REQUEST_INT(r) => self.on_mission_request(r.seq, true)?,
            MavMessage::MISSION_REQUEST(r) => self.on_mission_request(r.seq, false)?,
            MavMessage::MISSION_ACK(a) => self.on_mission_ack(a),
            MavMessage::COMMAND_ACK(a) => self.on_command_ack(a),
            _ => {}
        }
        Ok(())
    }

    fn on_param_value(&mut self, p: PARAM_VALUE_DATA) {
        let name = param_name(&p.param_id);
        let mut value = p.param_value;
        let fw = self.shared.read().cs.firmware;
        if fw == Firmware::Px4 {
            // PX4 uses byte-wise encoding for integer params.
            use MavParamType::*;
            if matches!(
                p.param_type,
                MAV_PARAM_TYPE_INT32
                    | MAV_PARAM_TYPE_UINT32
                    | MAV_PARAM_TYPE_INT16
                    | MAV_PARAM_TYPE_UINT16
                    | MAV_PARAM_TYPE_INT8
                    | MAV_PARAM_TYPE_UINT8
            ) {
                value = i32::from_le_bytes(p.param_value.to_le_bytes()) as f32;
            }
        }
        let mut done = None;
        {
            let mut s = self.shared.write();
            s.params.insert(
                name.clone(),
                Param {
                    value,
                    ptype: p.param_type as u8,
                    index: p.param_index,
                },
            );
            if let Some(f) = self.param_fetch.as_mut() {
                s.param_count = p.param_count;
                if p.param_index != u16::MAX {
                    f.received.insert(p.param_index);
                }
                f.last_rx = Instant::now();
                s.progress = Some(Progress {
                    label: format!("Got param {name}"),
                    done: f.received.len(),
                    total: p.param_count as usize,
                });
                if f.received.len() >= p.param_count as usize && p.param_count > 0 {
                    s.params_complete = true;
                    s.progress = None;
                    done = Some(s.params.len());
                }
            }
        }
        if let Some(n) = done {
            self.param_fetch = None;
            self.event(LinkEvent::ParamsLoaded(n));
        }
        if let Some(ps) = self.pending_sets.remove(&name) {
            let ok = (ps.value - value).abs() <= f32::EPSILON.max(ps.value.abs() * 1e-6);
            self.event(LinkEvent::ParamSet { name, ok });
        }
    }

    fn on_mission_count(&mut self, c: MISSION_COUNT_DATA) -> io::Result<()> {
        let (ts, tc) = (self.target_sys, self.target_comp);
        let mut req = None;
        let mut finished = None;
        if let Some(MissionOp::Download {
            mission_type,
            count,
            items,
            last_tx,
            retries,
            ..
        }) = self.mission.as_mut()
        {
            if count.is_none() && c.mission_type == *mission_type {
                *count = Some(c.count);
                *retries = 0;
                *last_tx = Instant::now();
                items.clear();
                if c.count == 0 {
                    finished = Some((*mission_type, Vec::new()));
                } else {
                    req = Some((0u16, *mission_type));
                }
            }
        }
        if let Some((seq, mt)) = req {
            self.set_progress(Some(Progress {
                label: "Getting WP 0".into(),
                done: 0,
                total: c.count as usize,
            }));
            self.send(MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq,
                target_system: ts,
                target_component: tc,
                mission_type: mt,
            }))?;
        }
        if let Some((mt, items)) = finished {
            self.finish_download(mt, items)?;
        }
        Ok(())
    }

    fn on_mission_item(
        &mut self,
        seq: u16,
        mt_in: MavMissionType,
        wp: Locationwp,
    ) -> io::Result<()> {
        let mut next = None;
        let mut finished = None;
        if let Some(MissionOp::Download {
            mission_type,
            count: Some(count),
            items,
            last_tx,
            retries,
            use_int,
        }) = self.mission.as_mut()
        {
            if mt_in == *mission_type && seq as usize == items.len() {
                items.push(wp);
                *retries = 0;
                *last_tx = Instant::now();
                if items.len() >= *count as usize {
                    finished = Some((*mission_type, std::mem::take(items)));
                } else {
                    next = Some((items.len() as u16, *count, *mission_type, *use_int));
                }
            }
        }
        if let Some((seq, count, mt, use_int)) = next {
            self.set_progress(Some(Progress {
                label: format!("Getting WP {seq}"),
                done: seq as usize,
                total: count as usize,
            }));
            self.request_item(seq, mt, use_int)?;
        }
        if let Some((mt, items)) = finished {
            self.finish_download(mt, items)?;
        }
        Ok(())
    }

    fn request_item(&mut self, seq: u16, mt: MavMissionType, use_int: bool) -> io::Result<()> {
        let (ts, tc) = (self.target_sys, self.target_comp);
        if use_int {
            self.send(MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
                seq,
                target_system: ts,
                target_component: tc,
                mission_type: mt,
            }))
        } else {
            #[allow(deprecated)]
            self.send(MavMessage::MISSION_REQUEST(MISSION_REQUEST_DATA {
                seq,
                target_system: ts,
                target_component: tc,
                mission_type: mt,
            }))
        }
    }

    fn finish_download(&mut self, mt: MavMissionType, items: Vec<Locationwp>) -> io::Result<()> {
        self.mission = None;
        self.set_progress(None);
        self.send(MavMessage::MISSION_ACK(MISSION_ACK_DATA {
            target_system: self.target_sys,
            target_component: self.target_comp,
            mavtype: MavMissionResult::MAV_MISSION_ACCEPTED,
            mission_type: mt,
            opaque_id: 0,
        }))?;
        self.event(LinkEvent::MissionDownloaded {
            mission_type: mt as u8,
            items,
        });
        Ok(())
    }

    fn on_mission_request(&mut self, seq: u16, int: bool) -> io::Result<()> {
        let (ts, tc) = (self.target_sys, self.target_comp);
        let mut out = None;
        if let Some(MissionOp::Upload {
            mission_type,
            items,
            last_tx,
            retries,
            max_requested,
        }) = self.mission.as_mut()
        {
            if let Some(wp) = items.get(seq as usize) {
                *last_tx = Instant::now();
                *retries = 0;
                *max_requested = Some(max_requested.map_or(seq, |m| m.max(seq)));
                out = Some((
                    wp.to_mission_item_int(seq, ts, tc, *mission_type),
                    *wp,
                    *mission_type,
                    items.len(),
                ));
            }
        }
        if let Some((item, wp, mt, total)) = out {
            self.set_progress(Some(Progress {
                label: format!("Setting WP {seq}"),
                done: seq as usize + 1,
                total,
            }));
            match item {
                Some(item) if int => self.send(MavMessage::MISSION_ITEM_INT(item))?,
                _ => {
                    #[allow(deprecated)]
                    let m = MISSION_ITEM_DATA {
                        param1: wp.p1,
                        param2: wp.p2,
                        param3: wp.p3,
                        param4: wp.p4,
                        x: wp.lat as f32,
                        y: wp.lng as f32,
                        z: wp.alt,
                        seq,
                        command: MavCmd::from_u16(wp.id).unwrap_or_default(),
                        target_system: ts,
                        target_component: tc,
                        frame: MavFrame::from_u8(wp.frame)
                            .unwrap_or(MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT),
                        current: 0,
                        autocontinue: 1,
                        mission_type: mt,
                    };
                    #[allow(deprecated)]
                    self.send(MavMessage::MISSION_ITEM(m))?
                }
            }
        }
        Ok(())
    }

    fn on_mission_ack(&mut self, a: MISSION_ACK_DATA) {
        if let Some(MissionOp::Upload {
            mission_type,
            items,
            max_requested,
            ..
        }) = &self.mission
        {
            if a.mission_type != *mission_type {
                return;
            }
            // An ACK before any item was requested only counts for an empty mission.
            if max_requested.is_none()
                && !items.is_empty()
                && a.mavtype == MavMissionResult::MAV_MISSION_ACCEPTED
            {
                return;
            }
            let mt = *mission_type as u8;
            let result = if a.mavtype == MavMissionResult::MAV_MISSION_ACCEPTED {
                Ok(())
            } else {
                Err(format!("{:?}", a.mavtype))
            };
            self.mission = None;
            self.set_progress(None);
            self.event(LinkEvent::MissionUploaded {
                mission_type: mt,
                result,
            });
        }
    }

    fn on_command_ack(&mut self, a: COMMAND_ACK_DATA) {
        if let Some(p) = &self.pending_cmd {
            if p.command == a.command {
                let ok = a.result == MavResult::MAV_RESULT_ACCEPTED
                    || a.result == MavResult::MAV_RESULT_IN_PROGRESS;
                let result = format!("{:?}", a.result)
                    .trim_start_matches("MAV_RESULT_")
                    .to_string();
                self.event(LinkEvent::CommandAck {
                    command: a.command as u16,
                    result,
                    ok,
                });
                self.pending_cmd = None;
            }
        }
    }

    // ---------------------------------------------------------------- periodic work

    fn tick(&mut self) -> io::Result<()> {
        let now = Instant::now();

        if now.duration_since(self.last_heartbeat_tx) >= Duration::from_secs(1) {
            self.last_heartbeat_tx = now;
            self.send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GCS,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            }))?;
        }

        if !self.got_heartbeat {
            if now.duration_since(self.started) > HEARTBEAT_TIMEOUT {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "No Heartbeat Packets Received",
                ));
            }
            return Ok(());
        }

        {
            let mut s = self.shared.write();
            s.link_lost = s
                .last_packet
                .map(|t| now.duration_since(t) > LINK_LOST)
                .unwrap_or(false);
        }

        // Parameter fetch: re-request missing indices once the stream goes quiet.
        let mut missing = Vec::new();
        let mut give_up = false;
        if let Some(f) = self.param_fetch.as_mut() {
            if now.duration_since(f.last_rx) > PARAM_IDLE {
                let total = self.shared.read().param_count;
                if total == 0 {
                    if f.retries < 3 {
                        f.retries += 1;
                        f.last_rx = now;
                        missing.push(u16::MAX);
                    } else {
                        give_up = true;
                    }
                } else if f.retries < 30 {
                    f.retries += 1;
                    f.last_rx = now;
                    missing = (0..total)
                        .filter(|i| !f.received.contains(i))
                        .take(10)
                        .collect();
                } else {
                    give_up = true;
                }
            }
        }
        if give_up {
            self.param_fetch = None;
            let n = {
                let mut s = self.shared.write();
                s.progress = None;
                s.params.len()
            };
            self.event(LinkEvent::ParamsLoaded(n));
        }
        for idx in missing {
            if idx == u16::MAX {
                self.send(MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                }))?;
            } else {
                self.send(MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
                    param_index: idx as i16,
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                    param_id: "".into(),
                }))?;
            }
        }

        // Param set retries
        let retry: Vec<(String, f32)> = self
            .pending_sets
            .iter_mut()
            .filter(|(_, p)| now.duration_since(p.sent) > Duration::from_millis(700))
            .filter_map(|(n, p)| {
                if p.tries >= 3 {
                    None
                } else {
                    p.tries += 1;
                    p.sent = now;
                    Some((n.clone(), p.value))
                }
            })
            .collect();
        let failed: Vec<String> = self
            .pending_sets
            .iter()
            .filter(|(_, p)| {
                p.tries >= 3 && now.duration_since(p.sent) > Duration::from_millis(700)
            })
            .map(|(n, _)| n.clone())
            .collect();
        for n in failed {
            self.pending_sets.remove(&n);
            self.event(LinkEvent::ParamSet { name: n, ok: false });
        }
        for (n, v) in retry {
            self.send_param_set(&n, v)?;
        }

        // Command retries
        if let Some(p) = self.pending_cmd.as_mut() {
            if now.duration_since(p.sent) > CMD_TIMEOUT {
                if p.tries >= 3 {
                    let c = p.command as u16;
                    self.pending_cmd = None;
                    self.event(LinkEvent::CommandAck {
                        command: c,
                        result: "Timeout".into(),
                        ok: false,
                    });
                } else {
                    p.tries += 1;
                    p.sent = now;
                    let (c, params, conf) = (p.command, p.params, p.tries as u8 - 1);
                    self.send_command_long(c, params, conf)?;
                }
            }
        }

        // Mission retries
        enum Retry {
            List(MavMissionType),
            Item(u16, MavMissionType, bool),
            Count(MavMissionType, u16),
            FailDown(MavMissionType),
            FailUp(MavMissionType),
        }
        let mut action = None;
        match self.mission.as_mut() {
            Some(MissionOp::Download {
                mission_type,
                count,
                items,
                last_tx,
                retries,
                use_int,
            }) => {
                if now.duration_since(*last_tx) > MISSION_TIMEOUT {
                    *retries += 1;
                    *last_tx = now;
                    if *retries > 6 {
                        action = Some(Retry::FailDown(*mission_type));
                    } else if count.is_none() {
                        action = Some(Retry::List(*mission_type));
                    } else {
                        // Fall back to MISSION_REQUEST for old firmware that ignores _INT.
                        if *retries == 3 {
                            *use_int = !*use_int;
                        }
                        action = Some(Retry::Item(items.len() as u16, *mission_type, *use_int));
                    }
                }
            }
            Some(MissionOp::Upload {
                mission_type,
                items,
                last_tx,
                retries,
                max_requested,
            }) => {
                if now.duration_since(*last_tx) > MISSION_TIMEOUT {
                    *retries += 1;
                    *last_tx = now;
                    if *retries > 6 {
                        action = Some(Retry::FailUp(*mission_type));
                    } else if max_requested.is_none() {
                        action = Some(Retry::Count(*mission_type, items.len() as u16));
                    }
                }
            }
            None => {}
        }
        match action {
            Some(Retry::List(mt)) => self.send(MavMessage::MISSION_REQUEST_LIST(
                MISSION_REQUEST_LIST_DATA {
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                    mission_type: mt,
                },
            ))?,
            Some(Retry::Item(seq, mt, int)) => self.request_item(seq, mt, int)?,
            Some(Retry::Count(mt, n)) => self.send_mission_count(mt, n)?,
            Some(Retry::FailDown(mt)) => {
                self.mission = None;
                self.set_progress(None);
                let _ = mt;
                self.event(LinkEvent::MissionDownloaded {
                    mission_type: mt as u8,
                    items: Vec::new(),
                });
                self.push_local_message("Timeout getting waypoints");
            }
            Some(Retry::FailUp(mt)) => {
                self.mission = None;
                self.set_progress(None);
                self.event(LinkEvent::MissionUploaded {
                    mission_type: mt as u8,
                    result: Err("Timeout".into()),
                });
            }
            None => {}
        }
        Ok(())
    }

    fn push_local_message(&self, text: &str) {
        self.shared.write().messages.push(StatusMessage {
            time: now_hms(),
            severity: 4,
            text: text.into(),
        });
    }
}
