//! Port of `MAVLinkInterface`: owns the transport on a background thread and runs the
//! heartbeat, data-stream, parameter and mission protocols.
//!
//! The UI talks to it through [`Link`]: commands go in over a channel, state is shared behind
//! an `RwLock`, and one-shot results come back as [`LinkEvent`]s.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::io::{self, Write};
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
    /// Last ACCELCAL_VEHICLE_POS requested by the vehicle (COMMAND_LONG from the autopilot).
    pub accelcal_pos: Option<u32>,
    /// MAG_CAL_PROGRESS completion % per compass
    pub mag_progress: [u8; 3],
    /// MAG_CAL_REPORT per compass: (status, fitness, offsets)
    pub mag_report: [Option<(String, f32, [f32; 3])>; 3],
    /// Set while replaying a tlog.
    pub playback: Option<crate::tlog::PlaybackInfo>,
    /// The tlog being recorded for this connection.
    pub tlog_path: Option<std::path::PathBuf>,
    /// LOG_ENTRY list: (id, size bytes, time_utc)
    pub log_entries: Vec<(u16, u32, u32)>,
    /// MAVLink Inspector: last message per (sysid, compid, msgid) with rate.
    pub inspector: BTreeMap<(u8, u8, u32), MsgStat>,
    /// ADSB_VEHICLE reports by ICAO address: (callsign, lat, lng, alt m, heading deg, last seen)
    pub adsb: BTreeMap<u32, (String, f64, f64, f32, f32, Instant)>,
    /// MainV2.comPort.MAVlist: every vehicle heard on this link, by (sysid, compid)
    pub mavs: BTreeMap<(u8, u8), MavSummary>,
    /// MAVLinkInterface.Open(getparams: false): skip the parameter download on connect
    pub skip_params: bool,
    /// MainV2.comPort.MAV.wps: latest downloaded or uploaded mission
    pub mission: Vec<Locationwp>,
    /// Per-vehicle mission by (sysid, compid)
    pub missions: BTreeMap<(u8, u8), Vec<Locationwp>>,
    /// Last guided target for each MAV, shared by DATA, HTTP and scripting.
    pub guided_targets: BTreeMap<(u8, u8), Locationwp>,
}

/// What the map and the vehicle selector need about each vehicle on the link.
#[derive(Debug, Clone)]
pub struct MavSummary {
    pub firmware: Firmware,
    pub custom_mode: u32,
    pub armed: bool,
    pub lat: f64,
    pub lng: f64,
    pub alt: f32,
    pub heading: f32,
    pub last: Instant,
}

/// One row of the MAVLink Inspector.
#[derive(Debug, Clone)]
pub struct MsgStat {
    pub name: &'static str,
    pub count: u64,
    /// messages per second (smoothed)
    pub rate: f32,
    pub last: Instant,
    pub msg: MavMessage,
}

impl LinkShared {
    pub fn firmware(&self) -> Firmware {
        self.cs.firmware
    }

    pub fn set_mission(&mut self, sys: u8, comp: u8, items: Vec<Locationwp>) {
        self.mission = items.clone();
        self.missions.insert((sys, comp), items);
    }
}

#[derive(Debug, Clone)]
pub enum LinkEvent {
    Connected,
    Disconnected(Option<String>),
    ParamsLoaded(usize),
    AltitudeChanged(Result<(), String>),
    OperationError(String),
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
    LogList(usize),
    LogDownloaded { id: u16, path: std::path::PathBuf, result: Result<(), String> },
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
    /// Stop queued command retries before releasing interactive controls.
    CancelCommands,
    SetCurrentWp(u16),
    /// Guided "Fly To Here".
    Guided {
        lat: f64,
        lng: f64,
        alt: f32,
        frame: u8,
    },
    /// setNewWPAlt: MISSION_ITEM current=3, without a new horizontal target.
    ChangeAltitude { alt: f32 },
    /// LOG_REQUEST_LIST
    LogList,
    /// REQUEST_DATA_STREAM rates (CurrentState.rateattitude etc.)
    SetStreamRates { attitude: u16, position: u16, status: u16, rc: u16, sensors: u16 },
    /// LOG_REQUEST_DATA for a whole log, written to `path`.
    LogDownload { id: u16, size: u32, path: std::path::PathBuf },
    /// LOG_ERASE
    LogErase,
    /// Telemetry log playback control.
    Playback(crate::tlog::PlaybackCommand),
    /// SerialOutputPass / "Mavlink Mirror": forward every frame to `host:port` over UDP;
    /// with `write` set, frames coming back are passed on to the vehicle. `None` stops it.
    Mirror { target: Option<String>, write: bool },
    /// Send pre-built frame bytes unchanged (MAVLink signing setup, hex injection).
    SendRaw(Vec<u8>),
    /// Forward every received message with this id to `tx` (MAVFtp and other tools).
    Subscribe { msgid: u32, tx: Sender<MavMessage> },
    /// Subscribe to one source vehicle/component, independent of later target selection.
    SubscribeFrom { msgid: u32, sysid: u8, compid: u8, tx: Sender<MavMessage> },
    /// Forward the raw payload (sysid, compid, bytes) of a message id, for messages the
    /// dialect cannot decode (REMOTE_LOG_DATA_BLOCK carries plain u32 sequence numbers).
    SubscribeRaw { msgid: u32, tx: Sender<(u8, u8, Vec<u8>)> },
    /// Make another vehicle on the link the current one (sysid selector).
    SetTarget { sysid: u8, compid: u8 },
    /// OnPacketReceived: every received frame's bytes, until the receiver is dropped.
    TapFrames(Sender<Vec<u8>>),
}

/// Handle to a running link (equivalent of `MainV2.comPort`).
pub struct Link {
    pub shared: Arc<RwLock<LinkShared>>,
    tx: Sender<LinkCommand>,
    pub events: Receiver<LinkEvent>,
    thread: Option<JoinHandle<()>>,
    playback: Option<Sender<crate::tlog::PlaybackCommand>>,
}

impl Link {
    pub fn connect(spec: ConnectionSpec) -> Self {
        Self::connect_with(spec, None)
    }

    /// Replay a telemetry log as if it were a live connection.
    pub fn playback(path: &std::path::Path) -> io::Result<Self> {
        let shared = Arc::new(RwLock::new(LinkShared::default()));
        let ctl = crate::tlog::playback(path, shared.clone())?;
        let (tx, _rx) = crossbeam_channel::unbounded();
        let (_etx, erx) = crossbeam_channel::unbounded();
        Ok(Self { shared, tx, events: erx, thread: None, playback: Some(ctl) })
    }

    /// Connect, recording a `.tlog` into `tlog_dir` (Mission Planner logs every connection).
    pub fn connect_with(spec: ConnectionSpec, tlog_dir: Option<std::path::PathBuf>) -> Self {
        Self::connect_with_params(spec, tlog_dir, true)
    }

    /// MainV2 Ctrl+T opens the link without starting a parameter download.
    pub fn connect_with_params(spec: ConnectionSpec, tlog_dir: Option<std::path::PathBuf>, get_params: bool) -> Self {
        let shared = Arc::new(RwLock::new(LinkShared {
            status: LinkStatus::Connecting,
            skip_params: !get_params,
            spec: Some(spec.clone()),
            progress: Some(Progress {
                label: if spec == ConnectionSpec::Auto { "Waiting for ports".into() } else { "Connecting Mavlink".into() },
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
                let result = comms::open(&spec).and_then(|t| {
                    let mut w = Worker::new(t, sh.clone(), rx, etx.clone());
                    if let Some(dir) = &tlog_dir {
                        match crate::tlog::TlogWriter::create(dir) {
                            Ok(tl) => {
                                sh.write().tlog_path = Some(tl.path.clone());
                                w.tlog = Some(tl);
                            }
                            Err(e) => log::warn!("tlog: {e}"),
                        }
                    }
                    w.run()
                });
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
            playback: None,
        }
    }

    pub fn send(&self, cmd: LinkCommand) {
        if let (Some(p), LinkCommand::Playback(c)) = (&self.playback, &cmd) {
            let _ = p.send(*c);
            return;
        }
        let _ = self.tx.send(cmd);
    }

    /// A sender for background producers (e.g. RTK injection).
    pub fn sender(&self) -> Sender<LinkCommand> {
        self.tx.clone()
    }

    /// Store a mission copy for (sysid, compid) on this link.
    pub fn set_mission(&self, sys: u8, comp: u8, items: Vec<Locationwp>) {
        self.shared.write().set_mission(sys, comp, items);
    }

    pub fn is_playback(&self) -> bool {
        self.playback.is_some()
    }

    pub fn status(&self) -> LinkStatus {
        self.shared.read().status
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        if let Some(p) = &self.playback {
            let _ = p.send(crate::tlog::PlaybackCommand::Stop);
        }
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
    in_progress: bool,
}

struct LogDownload {
    id: u16,
    size: u32,
    path: std::path::PathBuf,
    data: Vec<u8>,
    have: Vec<bool>,
    last_rx: Instant,
    retries: u32,
}

struct Worker {
    tlog: Option<crate::tlog::TlogWriter>,
    log_dl: Option<LogDownload>,
    log_expect: Option<u16>,
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
    queued_sets: VecDeque<(String, f32)>,
    mission: Option<MissionOp>,
    pending_cmd: Option<PendingCmd>,
    command_queue: VecDeque<(MavCmd, [f32; 7])>,
    pending_alt: Option<(MISSION_ITEM_DATA, Instant, u8)>,
    mirror: Option<(std::net::UdpSocket, std::net::SocketAddr, bool)>,
    subscribers: Vec<(u32, Sender<MavMessage>)>,
    source_subscribers: Vec<(u32, u8, u8, Sender<MavMessage>)>,
    raw_subscribers: Vec<(u32, Sender<(u8, u8, Vec<u8>)>)>,
    frame_taps: Vec<Sender<Vec<u8>>>,
}

fn is_timeout(e: &io::Error) -> bool {
    crate::comms::is_timeout(e)
}

fn param_name(id: &mavlink::types::CharArray<16>) -> String {
    let bytes: &[u8] = &id[..];
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).to_string()
}

pub fn now_hms() -> String {
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
            tlog: None,
            log_dl: None,
            log_expect: None,
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
            queued_sets: VecDeque::new(),
            mission: None,
            pending_cmd: None,
            command_queue: VecDeque::new(),
            pending_alt: None,
            mirror: None,
            subscribers: Vec::new(),
            source_subscribers: Vec::new(),
            raw_subscribers: Vec::new(),
            frame_taps: Vec::new(),
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

            match self.reader.read_any_raw_message::<MavMessage>() {
                Ok(raw) => {
                    let bytes = match &raw {
                        mavlink::MAVLinkMessageRaw::V1(m) => m.raw_bytes(),
                        mavlink::MAVLinkMessageRaw::V2(m) => m.raw_bytes(),
                    };
                    if let Some(t) = self.tlog.as_mut() {
                        t.write_frame(bytes);
                    }
                    if let Some((sock, to, _)) = &self.mirror {
                        let _ = sock.send_to(bytes, to);
                    }
                    if !self.frame_taps.is_empty() {
                        self.frame_taps.retain(|tx| tx.send(bytes.to_vec()).is_ok());
                    }
                    if !self.raw_subscribers.is_empty() {
                        let id = raw.message_id();
                        let item = (raw.system_id() as u8, raw.component_id(), raw.payload().to_vec());
                        self.raw_subscribers.retain(|(m, tx)| *m != id || tx.send(item.clone()).is_ok());
                    }
                    let hdr = MavHeader { system_id: raw.system_id(), component_id: raw.component_id(), sequence: raw.sequence() };
                    match <MavMessage as mavlink::Message>::parse(raw.version(), raw.message_id(), raw.payload()) {
                        Ok(msg) => self.handle_message(hdr, msg)?,
                        Err(e) => log::trace!("mavlink parse: {e:?}"),
                    }
                }
                Err(MessageReadError::Io(e)) if is_timeout(&e) => {}
                Err(MessageReadError::Io(e)) => return Err(e),
                Err(MessageReadError::Parse(e)) => log::trace!("mavlink parse: {e:?}"),
            }

            // mirror write-back: frames from the mirror client go to the vehicle
            let mut back = Vec::new();
            if let Some((sock, _, true)) = &self.mirror {
                let mut b = [0u8; 2048];
                while let Ok((n, _)) = sock.recv_from(&mut b) {
                    back.extend_from_slice(&b[..n]);
                }
            }
            if !back.is_empty() {
                self.reader.get_mut().write_all(&back)?;
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
        let mut buf = Vec::with_capacity(280);
        mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, hdr, &msg).map_err(|e| io::Error::other(format!("{e:?}")))?;
        if let Some(t) = self.tlog.as_mut() {
            t.write_frame(&buf);
        }
        let w = self.reader.get_mut();
        w.write_all(&buf)?;
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
        if self.pending_alt.is_some() && matches!(cmd, LinkCommand::ReadMission { .. } | LinkCommand::WriteMission { .. } | LinkCommand::Guided { .. }) {
            self.event(LinkEvent::OperationError("Wait for the pending altitude change before another mission operation".into()));
            return Ok(());
        }
        match cmd {
            LinkCommand::Disconnect => {}
            LinkCommand::CancelCommands => {
                self.pending_cmd = None;
                self.command_queue.clear();
            }
            LinkCommand::Send(m) => self.send(*m)?,
            LinkCommand::RequestParams => self.start_param_fetch()?,
            LinkCommand::SetParam { name, value } => {
                if !self.pending_sets.is_empty() || !self.queued_sets.is_empty() {
                    self.queued_sets.push_back((name, value));
                    return Ok(());
                }
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
                if mt == MavMissionType::MAV_MISSION_TYPE_MISSION {
                    self.shared.write().set_mission(self.target_sys, self.target_comp, items.clone());
                }
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
            LinkCommand::Playback(_) => {}
            LinkCommand::SetStreamRates { attitude, position, status, rc, sensors } => {
                let streams = [
                    (MavDataStream::MAV_DATA_STREAM_EXTENDED_STATUS, status),
                    (MavDataStream::MAV_DATA_STREAM_POSITION, position),
                    (MavDataStream::MAV_DATA_STREAM_EXTRA1, attitude),
                    (MavDataStream::MAV_DATA_STREAM_EXTRA2, attitude),
                    (MavDataStream::MAV_DATA_STREAM_EXTRA3, status),
                    (MavDataStream::MAV_DATA_STREAM_RAW_SENSORS, sensors),
                    (MavDataStream::MAV_DATA_STREAM_RC_CHANNELS, rc),
                ];
                for (stream, r) in streams {
                    self.send(MavMessage::REQUEST_DATA_STREAM(REQUEST_DATA_STREAM_DATA {
                        req_message_rate: r,
                        target_system: self.target_sys,
                        target_component: self.target_comp,
                        req_stream_id: stream,
                        start_stop: if r > 0 { 1 } else { 0 },
                    }))?;
                }
            }
            LinkCommand::LogList => {
                self.shared.write().log_entries.clear();
                self.log_expect = None;
                self.send(MavMessage::LOG_REQUEST_LIST(LOG_REQUEST_LIST_DATA {
                    start: 0,
                    end: 0xffff,
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                }))?;
            }
            LinkCommand::Mirror { target, write } => {
                self.mirror = None;
                if let Some(t) = target {
                    use std::net::ToSocketAddrs;
                    let addr = t.to_socket_addrs().ok().and_then(|mut a| a.next()).ok_or_else(|| io::Error::other(format!("bad address {t}")));
                    match addr.and_then(|a| std::net::UdpSocket::bind("0.0.0.0:0").map(|s| (s, a))) {
                        Ok((sock, a)) => {
                            let _ = sock.set_nonblocking(true);
                            self.mirror = Some((sock, a, write));
                        }
                        Err(e) => log::warn!("mirror: {e}"),
                    }
                }
            }
            LinkCommand::Subscribe { msgid, tx } => self.subscribers.push((msgid, tx)),
            LinkCommand::SubscribeFrom { msgid, sysid, compid, tx } => self.source_subscribers.push((msgid, sysid, compid, tx)),
            LinkCommand::SubscribeRaw { msgid, tx } => self.raw_subscribers.push((msgid, tx)),
            LinkCommand::TapFrames(tx) => self.frame_taps.push(tx),
            LinkCommand::SetTarget { sysid, compid } => {
                if sysid != self.target_sys || compid != self.target_comp {
                    self.pending_cmd = None;
                    self.command_queue.clear();
                    self.param_fetch = None;
                    for (name, _) in std::mem::take(&mut self.pending_sets) {
                        self.event(LinkEvent::ParamSet { name, ok: false });
                    }
                    self.queued_sets.clear();
                    if let Some(operation) = self.mission.take() {
                        match operation {
                            MissionOp::Upload { mission_type, .. } => self.event(LinkEvent::MissionUploaded {
                                mission_type: mission_type as u8,
                                result: Err("Mission upload cancelled because the selected vehicle changed".into()),
                            }),
                            MissionOp::Download { .. } => self.event(LinkEvent::OperationError("Mission download cancelled because the selected vehicle changed".into())),
                        }
                    }
                    if let Some(dl) = self.log_dl.take() {
                        self.event(LinkEvent::LogDownloaded { id: dl.id, path: dl.path,
                            result: Err("Log download cancelled because the selected vehicle changed".into()) });
                    }
                    if self.pending_alt.take().is_some() {
                        self.event(LinkEvent::AltitudeChanged(Err("Altitude change cancelled because the selected vehicle changed".into())));
                    }
                    self.target_sys = sysid;
                    self.target_comp = compid;
                    {
                        let mut s = self.shared.write();
                        s.cs = CurrentState::default();
                        s.cs.sysid = sysid;
                        s.cs.compid = compid;
                        s.params.clear();
                        s.param_count = 0;
                        s.params_complete = false;
                        s.autopilot_version = None;
                        if let Some(m) = s.mavs.get(&(sysid, compid)) {
                            s.cs.firmware = m.firmware;
                        }
                    }
                    self.request_datastreams()?;
                    self.start_param_fetch()?;
                }
            }
            LinkCommand::SendRaw(bytes) => {
                if let Some(t) = self.tlog.as_mut() {
                    t.write_frame(&bytes);
                }
                self.reader.get_mut().write_all(&bytes)?;
            }
            LinkCommand::LogErase => self.send(MavMessage::LOG_ERASE(LOG_ERASE_DATA { target_system: self.target_sys, target_component: self.target_comp }))?,
            LinkCommand::LogDownload { id, size, path } => {
                let chunks = (size as usize).div_ceil(90);
                self.log_dl = Some(LogDownload { id, size, path, data: vec![0; size as usize], have: vec![false; chunks], last_rx: Instant::now(), retries: 0 });
                self.set_progress(Some(Progress { label: format!("Downloading log {id}"), done: 0, total: size as usize }));
                self.send(MavMessage::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA {
                    ofs: 0,
                    count: 0xffff_ffff,
                    id,
                    target_system: self.target_sys,
                    target_component: self.target_comp,
                }))?;
            }
            LinkCommand::ChangeAltitude { alt } => {
                if !alt.is_finite() || self.mission.is_some() || self.pending_alt.is_some() {
                    self.event(LinkEvent::AltitudeChanged(Err("Altitude invalid or mission transfer busy".into())));
                    return Ok(());
                }
                // setWP defaults to use_int=false in the reference, including on
                // older autopilots that do not implement MISSION_ITEM_INT.
                let item = MISSION_ITEM_DATA { target_system: self.target_sys,
                    target_component: self.target_comp, seq: 0,
                    frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command: MavCmd::MAV_CMD_NAV_WAYPOINT, current: 3, autocontinue: 1,
                    x: 0.0, y: 0.0, z: alt, param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0,
                    mission_type: MavMissionType::MAV_MISSION_TYPE_MISSION };
                self.send(MavMessage::MISSION_ITEM(item.clone()))?;
                self.pending_alt = Some((item, Instant::now(), 1));
            }
            LinkCommand::Guided { lat, lng, alt, frame } => {
                // setGuidedModeWP: a zero target is ignored
                if alt == 0.0 || lat == 0.0 || lng == 0.0 {
                    return Ok(());
                }
                self.shared.write().guided_targets.insert((self.target_sys, self.target_comp), Locationwp::waypoint(lat, lng, alt, frame));
                let (firmware, mode) = {
                    let s = self.shared.read();
                    (s.cs.firmware, s.cs.mode.clone())
                };
                // "fix for followme change": switch to GUIDED first
                if !mode.eq_ignore_ascii_case("GUIDED") {
                    if let Some((number, _)) = firmware.modes().into_iter().find(|(_, name)| name.eq_ignore_ascii_case("GUIDED")) {
                        self.command_long(MavCmd::MAV_CMD_DO_SET_MODE, [1.0, number as f32, 0.0, 0.0, 0.0, 0.0, 0.0])?;
                    }
                }
                let mav_frame = match frame {
                    0 => MavFrame::MAV_FRAME_GLOBAL,
                    10 => MavFrame::MAV_FRAME_GLOBAL_TERRAIN_ALT,
                    _ => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                };
                if firmware == Firmware::ArduPlane {
                    // setWP(..., current = 2): MISSION_ITEM_INT
                    let wp = Locationwp::waypoint(lat, lng, alt, frame);
                    if let Some(mut item) = wp.to_mission_item_int(0, self.target_sys, self.target_comp, MavMissionType::MAV_MISSION_TYPE_MISSION) {
                        item.current = 2;
                        self.send(MavMessage::MISSION_ITEM_INT(item))?;
                    }
                } else {
                    // setPositionTargetGlobalInt(pos only): every ignore bit but position and FORCE
                    self.send(MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
                        lat_int: (lat * 1e7) as i32,
                        lon_int: (lng * 1e7) as i32,
                        alt,
                        target_system: self.target_sys,
                        target_component: self.target_comp,
                        coordinate_frame: mav_frame,
                        type_mask: PositionTargetTypemask::from_bits_truncate(u16::MAX - (1 << 9) - 0b111),
                        ..Default::default()
                    }))?;
                }
            }
        }
        Ok(())
    }

    fn command_long(&mut self, command: MavCmd, params: [f32; 7]) -> io::Result<()> {
        // Relay release is idempotent and must take effect immediately, even if a
        // different command is waiting for an ACK. Keep a queued copy for tracking.
        let release = command == MavCmd::MAV_CMD_DO_SET_RELAY && params[1] == 0.0;
        if release {
            self.command_queue.retain(|(c, p)| *c != command || p[0] != params[0]);
            if self.pending_cmd.as_ref().is_some_and(|p| p.command == command && p.params[0] == params[0]) {
                self.pending_cmd = None;
            }
        }
        if self.pending_cmd.is_some() {
            if self.command_queue.len() >= 128 {
                self.event(LinkEvent::OperationError("Command queue is full".into()));
            } else if release {
                self.send_command_long(command, params, 0)?;
                self.command_queue.push_front((command, params));
            } else {
                self.command_queue.push_back((command, params));
            }
            return Ok(());
        }
        self.send_command_long(command, params, 0)?;
        self.pending_cmd = Some(PendingCmd {
            command,
            params,
            sent: Instant::now(),
            tries: 1,
            in_progress: false,
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
        if !self.source_subscribers.is_empty() {
            let id = <MavMessage as mavlink::Message>::message_id(&msg);
            self.source_subscribers.retain(|(m, sys, comp, tx)| {
                *m != id || *sys as u32 != hdr.system_id || *comp != hdr.component_id || tx.send(msg.clone()).is_ok()
            });
        }
        if !self.subscribers.is_empty() {
            let id = <MavMessage as mavlink::Message>::message_id(&msg);
            self.subscribers.retain(|(m, tx)| *m != id || tx.send(msg.clone()).is_ok());
        }
        {
            use mavlink::Message;
            let now = Instant::now();
            let key = (hdr.system_id as u8, hdr.component_id, msg.message_id());
            let mut s = self.shared.write();
            if hdr.system_id != 255 {
                match &msg {
                    MavMessage::HEARTBEAT(hb) if hb.mavtype != MavType::MAV_TYPE_GCS && hb.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID => {
                        let e = s.mavs.entry((hdr.system_id as u8, hdr.component_id)).or_insert(MavSummary { firmware: Firmware::default(), custom_mode: 0, armed: false, lat: 0.0, lng: 0.0, alt: 0.0, heading: 0.0, last: now });
                        e.firmware = Firmware::from_heartbeat(hb.autopilot, hb.mavtype);
                        e.custom_mode = hb.custom_mode;
                        e.armed = hb.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
                        e.last = now;
                    }
                    MavMessage::GLOBAL_POSITION_INT(g) => {
                        if let Some(e) = s.mavs.get_mut(&(hdr.system_id as u8, hdr.component_id)) {
                            e.lat = g.lat as f64 * 1e-7;
                            e.lng = g.lon as f64 * 1e-7;
                            e.alt = g.relative_alt as f32 / 1000.0;
                            if g.hdg != u16::MAX {
                                e.heading = g.hdg as f32 / 100.0;
                            }
                        }
                    }
                    _ => {}
                }
            }
            if let MavMessage::ADSB_VEHICLE(a) = &msg {
                let cs: String = a.callsign.to_str().unwrap_or("").trim_end_matches('\0').trim().to_string();
                s.adsb.insert(a.ICAO_address, (cs, a.lat as f64 * 1e-7, a.lon as f64 * 1e-7, a.altitude as f32 / 1000.0, a.heading as f32 / 100.0, now));
                s.adsb.retain(|_, v| now.duration_since(v.5).as_secs() < 60);
            }
            match s.inspector.get_mut(&key) {
                Some(st) => {
                    let dt = now.duration_since(st.last).as_secs_f32().max(1e-3);
                    // the first interval seeds the average so a 1 Hz message reads 1 Hz at once
                    st.rate = if st.count == 1 { 1.0 / dt } else { st.rate * 0.8 + (1.0 / dt) * 0.2 };
                    st.count += 1;
                    st.last = now;
                    st.msg = msg.clone();
                }
                None => {
                    s.inspector.insert(key, MsgStat { name: msg.message_name(), count: 1, rate: 0.0, last: now, msg: msg.clone() });
                }
            }
        }
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
                    if !self.shared.read().skip_params {
                        self.start_param_fetch()?;
                    }
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
            if let MavMessage::MAG_CAL_PROGRESS(m) = &msg {
                if let Some(p) = s.mag_progress.get_mut(m.compass_id as usize) {
                    *p = m.completion_pct;
                }
            }
            if let MavMessage::MAG_CAL_REPORT(m) = &msg {
                if let Some(r) = s.mag_report.get_mut(m.compass_id as usize) {
                    let st = format!("{:?}", m.cal_status).trim_start_matches("MAG_CAL_").to_string();
                    *r = Some((st, m.fitness, [m.ofs_x, m.ofs_y, m.ofs_z]));
                }
            }
            if let MavMessage::COMMAND_LONG(c) = &msg {
                if c.command == MavCmd::MAV_CMD_ACCELCAL_VEHICLE_POS {
                    s.accelcal_pos = Some(c.param1 as u32);
                }
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
            MavMessage::MISSION_ACK(a) if hdr.component_id == self.target_comp => self.on_mission_ack(a),
            MavMessage::COMMAND_ACK(a) if from_ap => self.on_command_ack(a),
            MavMessage::LOG_ENTRY(e) => {
                let n = {
                    let mut s = self.shared.write();
                    if e.num_logs > 0 && !s.log_entries.iter().any(|x| x.0 == e.id) {
                        s.log_entries.push((e.id, e.size, e.time_utc));
                    }
                    s.log_entries.len()
                };
                self.log_expect = Some(e.num_logs);
                if n >= e.num_logs as usize {
                    self.event(LinkEvent::LogList(n));
                }
            }
            MavMessage::LOG_DATA(d) => self.on_log_data(d)?,
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
        if mt == MavMissionType::MAV_MISSION_TYPE_MISSION {
            self.shared.write().set_mission(self.target_sys, self.target_comp, items.clone());
        }
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
        if self.pending_alt.is_some()
            && a.mission_type == MavMissionType::MAV_MISSION_TYPE_MISSION
            && (a.target_system == GCS_SYSID || a.target_system == 0)
            && (a.target_component == GCS_COMPID || a.target_component == 0) {
            self.pending_alt = None;
            self.event(LinkEvent::AltitudeChanged(if a.mavtype == MavMissionResult::MAV_MISSION_ACCEPTED {
                Ok(())
            } else { Err(format!("Altitude change failed: {:?}", a.mavtype)) }));
            return;
        }
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
        if (a.target_system != 0 && a.target_system != GCS_SYSID)
            || (a.target_component != 0 && a.target_component != GCS_COMPID) { return }
        if a.result == MavResult::MAV_RESULT_IN_PROGRESS {
            if let Some(p) = self.pending_cmd.as_mut().filter(|p| p.command == a.command) {
                p.sent = Instant::now();
                p.in_progress = true;
            }
            return;
        }
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

    fn on_log_data(&mut self, d: LOG_DATA_DATA) -> io::Result<()> {
        let mut finished = None;
        if let Some(dl) = self.log_dl.as_mut() {
            if d.id == dl.id {
                let ofs = d.ofs as usize;
                let n = (d.count as usize).min(90).min(dl.data.len().saturating_sub(ofs));
                dl.data[ofs..ofs + n].copy_from_slice(&d.data[..n]);
                if let Some(h) = dl.have.get_mut(ofs / 90) {
                    *h = true;
                }
                dl.last_rx = Instant::now();
                let got = dl.have.iter().filter(|h| **h).count();
                self.shared.write().progress = Some(Progress { label: format!("Downloading log {}", dl.id), done: (got * 90).min(dl.size as usize), total: dl.size as usize });
                if got == dl.have.len() || d.count == 0 {
                    finished = self.log_dl.take();
                }
            }
        }
        if let Some(dl) = finished {
            self.set_progress(None);
            let result = std::fs::create_dir_all(dl.path.parent().unwrap_or(std::path::Path::new(".")))
                .and_then(|_| std::fs::write(&dl.path, &dl.data))
                .map_err(|e| e.to_string());
            self.send(MavMessage::LOG_REQUEST_END(LOG_REQUEST_END_DATA { target_system: self.target_sys, target_component: self.target_comp }))?;
            self.event(LinkEvent::LogDownloaded { id: dl.id, path: dl.path, result });
        }
        Ok(())
    }

    /// Re-request the first missing log chunk once the stream goes quiet.
    fn log_retry(&mut self) -> io::Result<()> {
        let mut req = None;
        let mut fail = None;
        if let Some(dl) = self.log_dl.as_mut() {
            if dl.last_rx.elapsed() > Duration::from_millis(1000) {
                dl.retries += 1;
                dl.last_rx = Instant::now();
                if dl.retries > 20 {
                    fail = Some((dl.id, dl.path.clone()));
                } else if let Some(first) = dl.have.iter().position(|h| !h) {
                    let last = dl.have[first..].iter().position(|h| *h).map(|k| first + k).unwrap_or(dl.have.len());
                    req = Some((dl.id, (first * 90) as u32, ((last - first) * 90) as u32));
                }
            }
        }
        if let Some((id, path)) = fail {
            self.log_dl = None;
            self.set_progress(None);
            self.event(LinkEvent::LogDownloaded { id, path, result: Err("Timeout".into()) });
        }
        if let Some((id, ofs, count)) = req {
            self.send(MavMessage::LOG_REQUEST_DATA(LOG_REQUEST_DATA_DATA { ofs, count, id, target_system: self.target_sys, target_component: self.target_comp }))?;
        }
        Ok(())
    }

    // ---------------------------------------------------------------- periodic work

    fn tick(&mut self) -> io::Result<()> {
        let now = Instant::now();
        self.log_retry()?;

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

        // ConfigMount and other forms call synchronous setParam in a defined
        // order. Preserve that order across names as well as repeated writes.
        if self.pending_sets.is_empty() {
            if let Some((name, value)) = self.queued_sets.pop_front() {
                self.send_param_set(&name, value)?;
                self.pending_sets.insert(name.clone(), PendingSet { value, sent: now, tries: 1 });
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

        if let Some((item, sent, tries)) = self.pending_alt.as_mut() {
            if now.duration_since(*sent) > MISSION_TIMEOUT {
                if *tries >= 3 {
                    self.pending_alt = None;
                    self.event(LinkEvent::AltitudeChanged(Err("Altitude change timed out".into())));
                } else {
                    *tries += 1;
                    *sent = now;
                    let item = item.clone();
                    self.send(MavMessage::MISSION_ITEM(item))?;
                }
            }
        }

        // Command retries
        if let Some(p) = self.pending_cmd.as_mut() {
            let timeout = if p.in_progress { Duration::from_secs(60) } else { CMD_TIMEOUT };
            if now.duration_since(p.sent) > timeout {
                if p.tries >= 3 || p.in_progress {
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

        if self.pending_cmd.is_none() {
            if let Some((command, params)) = self.command_queue.pop_front() {
                self.command_long(command, params)?;
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
