//! CameraCustomSettingsForm: definition loading and PARAM_EXT editing on the UI thread.
use std::{collections::HashMap, io::Read, sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}}, time::{Duration, Instant}};
use crossbeam_channel::Receiver;
use mp_core::{camdef::{self, CameraDefinition, CameraSettingsSession}, link::{Link, LinkCommand, LinkStatus}, mavlink::dialects::ardupilotmega::*};

fn cancelled(cancel: &AtomicBool) -> Result<(), String> {
    if cancel.load(Ordering::Relaxed) { Err("Camera definition download cancelled".into()) } else { Ok(()) }
}

fn download_http(uri: &str, cancel: &AtomicBool, progress: &Mutex<String>) -> Result<Vec<u8>, String> {
    let mut address = url::Url::parse(uri).map_err(|e| e.to_string())?;
    let agent: ureq::Agent = ureq::Agent::config_builder().max_redirects(0).max_redirects_will_error(false).timeout_global(None)
        .timeout_connect(Some(Duration::from_secs(15))).timeout_recv_response(Some(Duration::from_secs(15)))
        .timeout_recv_body(None).timeout_per_read(Some(Duration::from_secs(15))).build().into();
    for redirects in 0..=5 {
        cancelled(cancel)?;
        if !matches!(address.scheme(), "http" | "https") { return Err("Definition URI must use MAVFTP, HTTP or HTTPS".into()); }
        let mut response = agent.get(address.as_str()).call().map_err(|e| e.to_string())?;
        if response.status().is_redirection() {
            if redirects == 5 { return Err("Too many definition redirects".into()); }
            let location = response.headers().get("location").and_then(|v| v.to_str().ok()).ok_or("Missing definition redirect address")?;
            let next = address.join(location).map_err(|e| e.to_string())?;
            if !matches!(next.scheme(), "http" | "https") || (address.scheme() == "https" && next.scheme() != "https") { return Err("Invalid camera definition redirect".into()); }
            address = next; continue;
        }
        if response.headers().get("content-length").and_then(|v| v.to_str().ok()).and_then(|v| v.parse::<u64>().ok()).is_some_and(|n| n > camdef::MAX_SIZE as u64) { return Err("Camera definition is too large".into()); }
        let mut data = Vec::new(); let mut chunk = [0u8; 8192];
        let mut reader = response.body_mut().as_reader();
        loop {
            cancelled(cancel)?;
            let n = reader.read(&mut chunk).map_err(|e| e.to_string())?;
            if n == 0 { break; }
            if data.len() + n > camdef::MAX_SIZE { return Err("Camera definition is too large".into()); }
            data.extend_from_slice(&chunk[..n]);
            *progress.lock().unwrap() = format!("Downloading camera definition… {} bytes", data.len());
        }
        cancelled(cancel)?; return Ok(data);
    }
    Err("Too many definition redirects".into())
}

#[derive(Default)]
struct Row { text: String, dirty: bool, slider: i32, sliding: bool }
type Job = (Receiver<Result<CameraDefinition, String>>, Arc<AtomicBool>, Arc<Mutex<String>>);

pub struct CameraSettingsForm {
    pub target: (u8, u8),
    shared: Arc<parking_lot::RwLock<mp_core::link::LinkShared>>,
    incoming: Receiver<MavMessage>,
    information: Option<CAMERA_INFORMATION_DATA>,
    waiting: Option<Instant>,
    advertised: Option<(String, u16)>,
    job: Option<Job>,
    session: Option<CameraSettingsSession>,
    rows: HashMap<String, Row>,
    status: String,
}

impl Drop for CameraSettingsForm { fn drop(&mut self) { self.cancel(); } }
impl CameraSettingsForm {
    pub fn open(link: &Link, target: (u8,u8), readonly: bool) -> Self {
        let (tx, incoming) = crossbeam_channel::unbounded();
        for msgid in [259, 322, 324] { link.send(LinkCommand::SubscribeFrom { msgid, sysid: target.0, compid: target.1, tx: tx.clone() }); }
        let information = link.shared.read().inspector.get(&(target.0,target.1,259)).and_then(|s| match &s.msg { MavMessage::CAMERA_INFORMATION(m) => Some(m.clone()), _ => None });
        let mut form = Self { target, shared: link.shared.clone(), incoming, information, waiting: None, advertised: None, job: None, session: None, rows: HashMap::new(), status: String::new() };
        form.reload(link, readonly); form
    }

    fn cancel(&mut self) { if let Some((_, cancel, _)) = self.job.take() { cancel.store(true, Ordering::Relaxed); } }
    fn reload(&mut self, link: &Link, readonly: bool) {
        self.cancel(); self.session = None; self.rows.clear();
        if readonly { self.waiting = None; self.status = "Link is read-only; disable ReadOnly link mode to request or change camera settings".into(); return; }
        self.status = "Requesting camera information…".into();
        self.waiting = Some(Instant::now() + Duration::from_secs(5));
        link.send(LinkCommand::Send(Box::new(MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: self.target.0, target_component: self.target.1, command: MavCmd::MAV_CMD_REQUEST_MESSAGE, param1: 259.0, ..Default::default()
        }))));
    }

    fn load_definition(&mut self, link: &Link) {
        self.waiting = None; self.cancel(); self.session = None; self.rows.clear();
        let Some(info) = &self.information else { self.status = "Could not load custom settings: Camera did not send CAMERA_INFORMATION. Use Refresh to retry.".into(); return; };
        let uri = String::from_utf8_lossy(&info.cam_definition_uri[..]).split('\0').next().unwrap_or("").trim().to_string();
        self.advertised = Some((uri.clone(), info.cam_definition_version));
        if uri.is_empty() { self.status = "Camera does not advertise a custom settings definition. Use Refresh to retry.".into(); return; }
        let cancel = Arc::new(AtomicBool::new(false)); let progress = Arc::new(Mutex::new("Loading camera definition…".to_string()));
        let (tx, rx) = crossbeam_channel::bounded(1);
        let ftp_target = if uri.to_ascii_lowercase().starts_with("mftp://") || uri.to_ascii_lowercase().starts_with("mavftp://") { Some(camdef::parse_ftp_address(&uri, self.target.0 as u32, self.target.1)) } else { None };
        let ftp = match ftp_target {
            Some(Ok((sys, comp, path))) if sys <= 255 => Some((mp_core::ftp::Ftp::for_target(link, sys as u8, comp), path)),
            Some(Ok(_)) => { self.status = "Invalid MAVFTP system ID".into(); return; }
            Some(Err(e)) => { self.status = e; return; }
            None => None,
        };
        let worker_cancel = cancel.clone(); let worker_progress = progress.clone(); let language = crate::i18n::culture().replace('-', "_");
        std::thread::spawn(move || {
            let result = (|| {
                let bytes = if let Some((mut ftp, path)) = ftp {
                    ftp.get_bounded(&path, camdef::MAX_SIZE, worker_cancel.clone(), |n,total| *worker_progress.lock().unwrap() = format!("Downloading camera definition… {n}/{total} bytes"))?
                } else { download_http(&uri, &worker_cancel, &worker_progress)? };
                cancelled(&worker_cancel)?;
                let definition = CameraDefinition::parse(&bytes, &language)?;
                cancelled(&worker_cancel)?; Ok(definition)
            })();
            let _ = tx.send(result);
        });
        self.job = Some((rx, cancel, progress));
    }

    fn poll(&mut self, link: &Link, readonly: bool) {
        let now = Instant::now();
        if readonly && (self.waiting.is_some() || self.job.is_some()) { self.cancel(); self.waiting = None; self.status = "Link is read-only; changing camera settings is disabled".into(); }
        while let Ok(message) = self.incoming.try_recv() {
            match message {
                MavMessage::CAMERA_INFORMATION(info) => {
                    let uri = String::from_utf8_lossy(&info.cam_definition_uri[..]).split('\0').next().unwrap_or("").trim().to_string();
                    let changed = self.advertised.as_ref().is_some_and(|v| *v != (uri, info.cam_definition_version));
                    self.information = Some(info);
                    if !readonly && (self.waiting.is_some() || changed) { self.load_definition(link); }
                }
                message => if let Some(session) = &mut self.session { session.receive(&message, now); },
            }
        }
        if self.waiting.is_some_and(|deadline| now >= deadline) && !readonly { self.load_definition(link); }
        let result = self.job.as_ref().and_then(|(rx,_,_)| match rx.try_recv() { Ok(result) => Some(result), Err(crossbeam_channel::TryRecvError::Empty) => None, Err(_) => Some(Err("Camera definition loader stopped".into())) });
        if let Some(result) = result {
            self.job = None;
            match result {
                Ok(definition) => { let mut session = CameraSettingsSession::new(definition,self.target.0,self.target.1); session.request_all(now); self.session = Some(session); }
                Err(e) => self.status = format!("Could not load custom settings: {e}. Use Refresh to retry."),
            }
        }
        if !readonly { if let Some(session) = &mut self.session {
            session.tick(now);
            for message in session.outbox.drain(..) { link.send(LinkCommand::Send(Box::new(message))); }
        }}
    }

    pub fn ui(&mut self, ctx: &egui::Context, link: Option<&Link>, readonly: bool) -> bool {
        let Some(link) = link.filter(|l| Arc::ptr_eq(&l.shared, &self.shared) && l.status() == LinkStatus::Connected) else { self.cancel(); return false; };
        self.poll(link, readonly);
        let mut open = true; let mut refresh = false; let mut close = false;
        let title = self.session.as_ref().map(|s| format!("Custom Settings - {} ({}:{})", s.definition.model,self.target.0,self.target.1)).unwrap_or_else(|| "Custom Settings".into());
        egui::Window::new(title).id(egui::Id::new("camera_custom_settings")).open(&mut open).default_size([780.0,650.0]).min_size([600.0,350.0]).show(ctx, |ui| {
            let loading = self.waiting.is_some() || self.job.is_some();
            egui::ScrollArea::vertical().max_height((ui.available_height()-85.0).max(100.0)).show(ui, |ui| {
                if let Some(session) = &mut self.session { Self::rows_ui(ui, session, &mut self.rows, readonly); }
            });
            let status = if let Some((_,_,progress)) = &self.job { progress.lock().unwrap().clone() }
                else if readonly { "Link is read-only; changing camera settings is disabled".into() }
                else if let Some(s) = &self.session { format!("{} — {}/{} settings read", s.status, s.definition.parameters.iter().filter(|p| !p.write_only && s.values.contains_key(&p.name)).count(), s.definition.parameters.iter().filter(|p| !p.write_only).count()) }
                else { self.status.clone() };
            ui.label(status);
            ui.horizontal(|ui| { refresh = ui.add_enabled(!loading, egui::Button::new("Refresh")).clicked(); close = ui.button("Close").clicked(); });
        });
        if refresh { self.reload(link, readonly); }
        if close || !open { self.cancel(); return false; }
        if !readonly { if let Some(session) = &mut self.session { for message in session.outbox.drain(..) { link.send(LinkCommand::Send(Box::new(message))); } } }
        ctx.request_repaint_after(Duration::from_millis(100)); true
    }

    fn rows_ui(ui: &mut egui::Ui, session: &mut CameraSettingsSession, rows: &mut HashMap<String,Row>, readonly: bool) {
        let controls = session.definition.controls(&session.values);
        for parameter in session.definition.parameters.clone() {
            let Some(options) = controls.get(&parameter.name) else { continue };
            let row = rows.entry(parameter.name.clone()).or_default();
            let value = session.values.get(&parameter.name);
            if !row.dirty && !session.is_pending(&parameter.name) { row.text = value.map(camdef::format).unwrap_or_default(); }
            let enabled = !readonly && session.can_edit(&parameter.name, Some(&controls));
            if !enabled { row.sliding = false; }
            let mut submit = None;
            ui.push_id(&parameter.name, |ui| {
                ui.horizontal(|ui| {
                    ui.add_sized([ui.available_width().mul_add(0.45,0.0).min(325.0),23.0], egui::Label::new(&parameter.description));
                    ui.add_enabled_ui(enabled, |ui| {
                        if !options.is_empty() || !parameter.options.is_empty() {
                            let label = options.iter().find(|o| Some(&o.value) == value).map(|o| o.label.clone()).unwrap_or_else(|| value.map(|v| format!("Current: {}", camdef::format(v))).unwrap_or_else(|| "Waiting for camera".into()));
                            egui::ComboBox::from_id_salt("choice").width(195.0).selected_text(label).show_ui(ui, |ui| { for o in options { if ui.selectable_label(Some(&o.value)==value, &o.label).clicked() { submit = Some(camdef::format(&o.value)); } } });
                        } else if let Some(steps) = parameter.slider_steps() {
                            if !row.sliding && !session.is_pending(&parameter.name) { if let Some(v) = value { row.slider=parameter.slider_position(v).unwrap_or(if v.to_f64()<0.0 { 0 } else { steps }); } }
                            let response = ui.add(egui::Slider::new(&mut row.slider,0..=steps).show_value(false));
                            if response.changed() { row.sliding = true; }
                            let held = ui.input(|i| [egui::Key::ArrowLeft,egui::Key::ArrowRight,egui::Key::ArrowUp,egui::Key::ArrowDown,egui::Key::Home,egui::Key::End,egui::Key::PageUp,egui::Key::PageDown].iter().any(|k| i.key_down(*k)));
                            if row.sliding && ((!response.is_pointer_button_down_on() && !held) || response.lost_focus()) {
                                let candidate=parameter.slider_text(row.slider).ok();
                                if candidate.as_deref().and_then(|s| parameter.convert(Some(s)).ok()).as_ref() != value { submit=candidate; }
                                row.sliding=false;
                            }
                            ui.label(if row.sliding || session.is_pending(&parameter.name) { parameter.slider_text(row.slider).unwrap_or_default() } else { row.text.clone() });
                        } else {
                            let response = ui.add(egui::TextEdit::singleline(&mut row.text).desired_width(195.0));
                            row.dirty |= response.changed();
                            if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter))) || (!parameter.read_only && ui.button("Apply").clicked()) { submit=Some(row.text.clone()); }
                        }
                    });
                });
                let detail = session.errors.get(&parameter.name).cloned().unwrap_or_else(|| if session.is_pending(&parameter.name) { "Applying…".into() } else if value.is_none() && !parameter.write_only { "Waiting for camera".into() } else { format!("{}{}{}{}",parameter.name,parameter.minimum.as_ref().map(|v| format!(" Min: {}",camdef::format(v))).unwrap_or_default(),parameter.maximum.as_ref().map(|v| format!(" Max: {}",camdef::format(v))).unwrap_or_default(),parameter.step.as_ref().map(|v|format!(" Step: {v}")).unwrap_or_default()) });
                ui.colored_label(if session.errors.contains_key(&parameter.name) { egui::Color32::RED } else { crate::theme::palette().text },detail);
                ui.add_space(8.0);
            });
            if let Some(text) = submit.filter(|_| enabled && !readonly) {
                match session.set_value(&parameter.name, &text, Instant::now()) { Ok(()) => row.dirty=false, Err(e) => { session.errors.insert(parameter.name.clone(),e); } }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{io::Write, net::TcpListener};
    use mp_core::mavlink::{self, MavHeader, MavlinkReader, MavlinkVersion};
    const XML: &[u8] = br#"<mavlinkcamera><definition><model>Test Camera</model></definition><parameters><parameter name="CAM_TEST" type="uint32"><description>Test setting</description></parameter></parameters></mavlinkcamera>"#;

    #[test]
    fn editor_apply_and_readonly_slider_transition_use_actual_egui_events() {
        use crate::platform::picker_test_support as input;
        let definition=CameraDefinition::parse(br#"<mavlinkcamera><definition/><parameters><parameter name="TEXT" type="uint32"><description>Text</description></parameter><parameter name="RANGE" type="uint32" min="0" max="10" step="1"><description>Range</description></parameter></parameters></mavlinkcamera>"#,"en").unwrap();
        let mut session=CameraSettingsSession::new(definition,42,100);
        session.values.insert("TEXT".into(),camdef::Value::U32(0)); session.values.insert("RANGE".into(),camdef::Value::U32(0));
        let ctx=egui::Context::default(); let mut rows=HashMap::new();
        {
            let mut draw=|ctx: &egui::Context| { egui::Window::new("Editor").default_size([780.0,650.0]).show(ctx,|ui| CameraSettingsForm::rows_ui(ui,&mut session,&mut rows,false)); };
            input::click(&ctx,&mut draw,"0");
            input::frame(&ctx,&mut draw,vec![egui::Event::Key { key:egui::Key::A,physical_key:None,pressed:true,repeat:false,modifiers:egui::Modifiers::CTRL }]);
            input::frame(&ctx,&mut draw,vec![egui::Event::Text("7".into())]);
            input::click(&ctx,&mut draw,"Apply");
        }
        assert!(matches!(session.outbox.last(),Some(MavMessage::PARAM_EXT_SET(m)) if &m.param_id[..4]==b"TEXT" && m.param_value[0]==7));
        session.outbox.clear();
        // Active gesture is cancelled if editability changes before release.
        rows.get_mut("RANGE").unwrap().slider=8; rows.get_mut("RANGE").unwrap().sliding=true;
        input::frame(&ctx,&mut |ctx| { egui::Window::new("Editor").show(ctx,|ui| CameraSettingsForm::rows_ui(ui,&mut session,&mut rows,true)); },Vec::new());
        assert!(!rows["RANGE"].sliding); assert!(session.outbox.is_empty());
        // Returning to the confirmed value must not create a redundant write.
        rows.get_mut("RANGE").unwrap().slider=0; rows.get_mut("RANGE").unwrap().sliding=true;
        input::frame(&ctx,&mut |ctx| { egui::Window::new("Editor").show(ctx,|ui| CameraSettingsForm::rows_ui(ui,&mut session,&mut rows,false)); },Vec::new());
        assert!(session.outbox.is_empty());
        // Pending preview is preserved until the camera acknowledges it.
        rows.get_mut("RANGE").unwrap().slider=6;
        session.set_value("RANGE","6",Instant::now()).unwrap(); session.outbox.clear();
        input::frame(&ctx,&mut |ctx| { egui::Window::new("Editor").show(ctx,|ui| CameraSettingsForm::rows_ui(ui,&mut session,&mut rows,false)); },Vec::new());
        assert_eq!(rows["RANGE"].slider,6); assert!(session.outbox.is_empty());
    }

    fn serve(body: &'static [u8], declared: usize) -> (String, std::thread::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap(); let addr=listener.local_addr().unwrap();
        let thread = std::thread::spawn(move || {
            let (mut stream,_) = listener.accept().unwrap(); stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
            let mut header=Vec::new(); let mut byte=[0];
            while !header.ends_with(b"\r\n\r\n") { stream.read_exact(&mut byte).unwrap(); header.push(byte[0]); }
            write!(stream,"HTTP/1.1 200 OK\r\nContent-Length: {declared}\r\nConnection: close\r\n\r\n").unwrap();
            let _=stream.write_all(body);
        });
        (format!("http://{addr}/definition.xml"),thread)
    }

    #[test]
    fn http_definition_limit_and_cancel_are_enforced() {
        let cancel=AtomicBool::new(false); let progress=Mutex::new(String::new());
        let (url,peer)=serve(XML,XML.len()); assert_eq!(download_http(&url,&cancel,&progress).unwrap(),XML); peer.join().unwrap();
        let (url,peer)=serve(b"",camdef::MAX_SIZE+1); assert!(download_http(&url,&cancel,&progress).unwrap_err().contains("too large")); peer.join().unwrap();
        cancel.store(true,Ordering::Relaxed); assert!(download_http("http://127.0.0.1:1/",&cancel,&progress).unwrap_err().contains("cancelled"));
    }

    #[test]
    fn camera_information_and_setting_write_use_the_camera_source_on_the_real_link() {
        let listener=TcpListener::bind("127.0.0.1:0").unwrap(); let addr=listener.local_addr().unwrap();
        let link=Link::connect_with_params(mp_core::comms::ConnectionSpec::Tcp { host:"127.0.0.1".into(),port:addr.port() },None,false);
        let (mut stream,_)=listener.accept().unwrap(); stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let mut reader=MavlinkReader::new(stream.try_clone().unwrap());
        let mut sequence=0;
        let mut send = |component, message: MavMessage| {
            let header=MavHeader { system_id:42,component_id:component,sequence }; sequence=sequence.wrapping_add(1);
            mavlink::write_versioned_msg(&mut stream,MavlinkVersion::V2,header,&message).unwrap();
        };
        for _ in 0..2 { send(1,MavMessage::HEARTBEAT(HEARTBEAT_DATA { mavtype:MavType::MAV_TYPE_QUADROTOR,autopilot:MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,..Default::default() })); }
        let deadline=Instant::now()+Duration::from_secs(5);
        while link.status()!=LinkStatus::Connected { assert!(Instant::now()<deadline); std::thread::sleep(Duration::from_millis(5)); }
        let mut form=CameraSettingsForm::open(&link,(42,100),false);
        loop { if let MavMessage::COMMAND_LONG(c)=reader.read_any_message::<MavMessage>().unwrap().1 { if c.param1==259.0 && c.target_component==100 { assert_eq!((c.target_system,c.target_component),(42,100)); break; } } }
        let (uri,http)=serve(XML,XML.len());
        let mut info=CAMERA_INFORMATION_DATA::default(); let mut raw=[0u8;140]; raw[..uri.len()].copy_from_slice(uri.as_bytes()); info.cam_definition_uri=raw.into();
        send(100,MavMessage::CAMERA_INFORMATION(info));
        let deadline=Instant::now()+Duration::from_secs(8);
        while form.session.is_none() { form.poll(&link,false); assert!(Instant::now()<deadline,"{}",form.status); std::thread::sleep(Duration::from_millis(5)); }
        http.join().unwrap();
        loop { if let MavMessage::PARAM_EXT_REQUEST_LIST(c)=reader.read_any_message::<MavMessage>().unwrap().1 { assert_eq!((c.target_system,c.target_component),(42,100)); break; } }
        let parameter=form.session.as_ref().unwrap().definition.get("CAM_TEST").unwrap().clone();
        let packet=PARAM_EXT_VALUE_DATA { param_id:parameter.id().into(),param_value:parameter.encode(&camdef::Value::U32(0)).unwrap().into(),param_type:MavParamExtType::MAV_PARAM_EXT_TYPE_UINT32,param_count:1,param_index:0 };
        send(99,MavMessage::PARAM_EXT_VALUE(packet.clone()));
        std::thread::sleep(Duration::from_millis(30)); form.poll(&link,false); assert!(form.session.as_ref().unwrap().values.is_empty());
        send(100,MavMessage::PARAM_EXT_VALUE(packet));
        let deadline=Instant::now()+Duration::from_secs(3);
        while form.session.as_ref().unwrap().values.is_empty() { form.poll(&link,false); assert!(Instant::now()<deadline); std::thread::sleep(Duration::from_millis(5)); }
        form.session.as_mut().unwrap().set_value("CAM_TEST","2",Instant::now()).unwrap(); form.poll(&link,false);
        loop { if let MavMessage::PARAM_EXT_SET(c)=reader.read_any_message::<MavMessage>().unwrap().1 { assert_eq!((c.target_system,c.target_component),(42,100)); assert_eq!(&c.param_value[..4],&2u32.to_le_bytes()); break; } }
        send(100,MavMessage::PARAM_EXT_ACK(PARAM_EXT_ACK_DATA { param_id:parameter.id().into(),param_value:parameter.encode(&camdef::Value::U32(2)).unwrap().into(),param_type:MavParamExtType::MAV_PARAM_EXT_TYPE_UINT32,param_result:ParamAck::PARAM_ACK_ACCEPTED }));
        let deadline=Instant::now()+Duration::from_secs(3);
        while form.session.as_ref().unwrap().is_pending("CAM_TEST") { form.poll(&link,false); assert!(Instant::now()<deadline); std::thread::sleep(Duration::from_millis(5)); }
        assert_eq!(form.session.as_ref().unwrap().values["CAM_TEST"],camdef::Value::U32(2));
        let ctx=egui::Context::default();
        let mut out=ctx.run_ui(egui::RawInput::default(),|ui| assert!(form.ui(ui.ctx(),Some(&link),true))); out.textures_delta.clear();
        form.waiting=None; let mut readonly=CameraSettingsForm::open(&link,(42,100),true);
        assert!(readonly.waiting.is_none() && readonly.session.is_none());
        assert!(!readonly.ui(&ctx,None,true));
    }
}
