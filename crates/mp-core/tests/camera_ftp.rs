//! Size/session/source checks over the production TCP link and FILE_TRANSFER_PROTOCOL.
use std::{io::Write, net::{TcpListener,TcpStream}, sync::{Arc,atomic::AtomicBool}, time::Duration};
use mp_core::{comms::ConnectionSpec, ftp::{Ftp,Op,Packet}, link::Link, mavlink::{self,dialects::ardupilotmega::*,MavHeader,MavlinkReader,MavlinkVersion}};
fn request(reader: &mut MavlinkReader<TcpStream>) -> Packet {
    loop { if let MavMessage::FILE_TRANSFER_PROTOCOL(f)=reader.read_any_message::<MavMessage>().unwrap().1 { assert_eq!((f.target_system,f.target_component),(42,100)); return Packet::decode(&f.payload); } }
}
fn reply(stream: &mut TcpStream, request: &Packet, session: u8, data: Vec<u8>, component: u8) {
    let packet=Packet { seq:request.seq.wrapping_add(1),session,opcode:128,size:data.len() as u8,req_opcode:request.opcode,burst_complete:0,offset:request.offset,data };
    let msg=MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA { target_network:0,target_system:255,target_component:190,payload:packet.encode() });
    mavlink::write_versioned_msg(stream,MavlinkVersion::V2,MavHeader { system_id:42,component_id:component,sequence:0 },&msg).unwrap(); stream.flush().unwrap();
}

#[test]
fn bounded_ftp_rejects_size_before_read_and_preserves_session_and_source() {
    for (advertised,session,cancel_open,nak) in [(u32::MAX,42,false,false),(3,0,false,false),(3,42,true,false),(3,42,false,true)] {
        let cancel=Arc::new(AtomicBool::new(false)); let peer_cancel=cancel.clone();
        let listener=TcpListener::bind("127.0.0.1:0").unwrap();
        let link=Link::connect_with_params(ConnectionSpec::Tcp { host:"127.0.0.1".into(),port:listener.local_addr().unwrap().port() },None,false);
        let (mut stream,_)=listener.accept().unwrap(); stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let peer=std::thread::spawn(move || {
            let mut reader=MavlinkReader::new(stream.try_clone().unwrap());
            let open=request(&mut reader); assert_eq!(open.opcode,Op::OpenFileRO as u8);
            if cancel_open { peer_cancel.store(true,std::sync::atomic::Ordering::Relaxed); }
            reply(&mut stream,&open,session,advertised.to_le_bytes().to_vec(),100);
            if advertised==3 && !cancel_open {
                let read=request(&mut reader); assert_eq!(read.opcode,Op::ReadFile as u8); assert_eq!(read.session,session);
                if nak {
                    let packet=Packet { seq:read.seq.wrapping_add(1),session:0,opcode:129,size:1,req_opcode:read.opcode,burst_complete:0,offset:0,data:vec![6] };
                    let msg=MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA { target_network:0,target_system:255,target_component:190,payload:packet.encode() });
                    mavlink::write_versioned_msg(&mut stream,MavlinkVersion::V2,MavHeader { system_id:42,component_id:100,sequence:1 },&msg).unwrap();
                } else {
                    reply(&mut stream,&read,0,b"bad".to_vec(),99); // unrelated source with matching sequence
                    reply(&mut stream,&read,99,b"bad".to_vec(),100); // wrong session, same camera and sequence
                    reply(&mut stream,&read,0,b"xml".to_vec(),100);
                }
            }
            let terminate=request(&mut reader); assert_eq!(terminate.opcode,Op::TerminateSession as u8); assert_eq!(terminate.session,session);
            reply(&mut stream,&terminate,session,vec![],100);
        });
        let mut ftp=Ftp::for_target(&link,42,100);
        let result=ftp.get_bounded("/camera.xml",1024,cancel,|_,_| {});
        if cancel_open { assert!(result.unwrap_err().contains("cancelled")); }
        else if nak { assert_eq!(result.unwrap_err(),"EOF"); }
        else if advertised==3 { assert_eq!(result.unwrap(),b"xml"); } else { assert!(result.unwrap_err().contains("size")); }
        peer.join().unwrap();
    }
}
