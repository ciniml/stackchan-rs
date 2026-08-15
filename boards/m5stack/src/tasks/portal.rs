//! Captive-portal helpers for AP (provisioning) mode: a minimal DHCP server and a
//! DNS responder that answers every query with the AP address, so phones pop their
//! "sign in to network" sheet and land on the setup page (the C++ firmware's
//! `captive_portal.cpp` analogue).

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use log::{info, warn};

/// AP address, also handed out as router + DNS (mirrors the esp-netif default).
pub const AP_ADDR: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);

const DHCP_SERVER_PORT: u16 = 67;
const DHCP_CLIENT_PORT: u16 = 68;
const DNS_PORT: u16 = 53;

/// Minimal DHCP server: DISCOVER → OFFER, REQUEST → ACK (or NAK when the client asks
/// for a lease from some other network, forcing it back to DISCOVER). Addresses are
/// derived from the client MAC, so every exchange is stateless and re-offers are
/// stable.
#[embassy_executor::task]
pub async fn dhcp_server(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 640];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buf = [0u8; 640];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    if let Err(e) = socket.bind(DHCP_SERVER_PORT) {
        warn!("DHCP bind failed: {:?}", e);
        return;
    }
    info!("DHCP server up on {}", AP_ADDR);
    let mut req = [0u8; 640];
    let mut resp = [0u8; 320];
    loop {
        let Ok((len, _meta)) = socket.recv_from(&mut req).await else {
            continue;
        };
        let Some(resp_len) = handle_dhcp(&req[..len], &mut resp) else {
            continue;
        };
        // Reply by broadcast: the client does not have an address yet.
        let dest = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), DHCP_CLIENT_PORT);
        if let Err(e) = socket.send_to(&resp[..resp_len], dest).await {
            warn!("DHCP send failed: {:?}", e);
        }
    }
}

/// Deterministic per-MAC address in 192.168.4.2..=251.
fn lease_for_mac(mac: &[u8]) -> Ipv4Address {
    Ipv4Address::new(192, 168, 4, 2 + mac[5] % 250)
}

/// Parse a BOOTP/DHCP request and build the reply. Returns the reply length, or
/// `None` when the packet is not something we answer.
fn handle_dhcp(req: &[u8], resp: &mut [u8; 320]) -> Option<usize> {
    // Fixed BOOTP header (236 bytes) + magic cookie + at least the message-type option.
    if req.len() < 243 || req[0] != 1 /* BOOTREQUEST */ || req[236..240] != [0x63, 0x82, 0x53, 0x63]
    {
        return None;
    }
    let chaddr = &req[28..34];
    // Walk the options for message type (53) and requested address (50).
    let mut msg_type = 0u8;
    let mut requested: Option<Ipv4Address> = None;
    let mut i = 240;
    while i + 1 < req.len() {
        let (opt, olen) = (req[i], req[i + 1] as usize);
        if opt == 255 {
            break;
        }
        if opt == 0 {
            i += 1;
            continue;
        }
        if i + 2 + olen > req.len() {
            break;
        }
        let val = &req[i + 2..i + 2 + olen];
        match opt {
            53 if olen == 1 => msg_type = val[0],
            50 if olen == 4 => requested = Some(Ipv4Address::new(val[0], val[1], val[2], val[3])),
            _ => {}
        }
        i += 2 + olen;
    }

    let lease = lease_for_mac(chaddr);
    let reply_type: u8 = match msg_type {
        1 => 2, // DISCOVER -> OFFER
        3 => {
            // REQUEST -> ACK, or NAK when the client wants an address we would not
            // assign (e.g. an old lease from a different network).
            match requested {
                Some(r) if r != lease => 6,
                _ => 5,
            }
        }
        _ => return None,
    };
    let nak = reply_type == 6;

    resp.fill(0);
    resp[0] = 2; // BOOTREPLY
    resp[1] = 1; // ethernet
    resp[2] = 6; // hlen
    resp[4..8].copy_from_slice(&req[4..8]); // xid
    resp[10] = 0x80; // broadcast flag
    if !nak {
        resp[16..20].copy_from_slice(&lease.octets()); // yiaddr
        resp[20..24].copy_from_slice(&AP_ADDR.octets()); // siaddr
    }
    resp[28..44].copy_from_slice(&req[28..44]); // chaddr
    resp[236..240].copy_from_slice(&[0x63, 0x82, 0x53, 0x63]);
    let mut o = 240;
    let mut push = |bytes: &[u8]| {
        resp[o..o + bytes.len()].copy_from_slice(bytes);
        o += bytes.len();
    };
    push(&[53, 1, reply_type]);
    push(&[54, 4]);
    push(&AP_ADDR.octets()); // server identifier
    if !nak {
        push(&[51, 4]);
        push(&86400u32.to_be_bytes()); // lease time
        push(&[1, 4, 255, 255, 255, 0]); // subnet mask
        push(&[3, 4]);
        push(&AP_ADDR.octets()); // router
        push(&[6, 4]);
        push(&AP_ADDR.octets()); // DNS
    }
    push(&[255]);
    Some(o)
}

/// Wildcard DNS responder: answers every A query with the AP address. Combined with
/// the HTTP catch-all redirect this triggers the OS captive-portal sheet.
#[embassy_executor::task]
pub async fn dns_server(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 640];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buf = [0u8; 640];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    if let Err(e) = socket.bind(DNS_PORT) {
        warn!("DNS bind failed: {:?}", e);
        return;
    }
    info!("DNS responder up (all names -> {})", AP_ADDR);
    let mut req = [0u8; 512];
    let mut resp = [0u8; 512];
    loop {
        let Ok((len, meta)) = socket.recv_from(&mut req).await else {
            continue;
        };
        let Some(resp_len) = handle_dns(&req[..len], &mut resp) else {
            continue;
        };
        let _ = socket.send_to(&resp[..resp_len], meta.endpoint).await;
    }
}

/// Build a response answering the first question with an A record for [`AP_ADDR`].
fn handle_dns(req: &[u8], resp: &mut [u8; 512]) -> Option<usize> {
    if req.len() < 12 || req[2] & 0x80 != 0 {
        return None; // not a query
    }
    let qdcount = u16::from_be_bytes([req[4], req[5]]);
    if qdcount == 0 {
        return None;
    }
    // Find the end of the first question (labels, then QTYPE + QCLASS).
    let mut i = 12;
    while i < req.len() && req[i] != 0 {
        i += 1 + req[i] as usize;
    }
    let q_end = i + 1 + 4;
    if i >= req.len() || q_end > req.len() {
        return None;
    }
    let answer_len = 16; // name ptr(2) type(2) class(2) ttl(4) rdlen(2) rdata(4)
    if q_end + answer_len > resp.len() {
        return None;
    }
    resp[..q_end].copy_from_slice(&req[..q_end]);
    resp[2] = 0x81; // response, recursion desired (echoed)
    resp[3] = 0x80; // recursion available, no error
    resp[4..6].copy_from_slice(&1u16.to_be_bytes()); // one question kept
    resp[6..8].copy_from_slice(&1u16.to_be_bytes()); // one answer
    resp[8..12].fill(0);
    let a = &mut resp[q_end..q_end + answer_len];
    a[0..2].copy_from_slice(&[0xC0, 0x0C]); // pointer to the question name
    a[2..4].copy_from_slice(&1u16.to_be_bytes()); // type A
    a[4..6].copy_from_slice(&1u16.to_be_bytes()); // class IN
    a[6..10].copy_from_slice(&60u32.to_be_bytes()); // TTL
    a[10..12].copy_from_slice(&4u16.to_be_bytes());
    a[12..16].copy_from_slice(&AP_ADDR.octets());
    Some(q_end + answer_len)
}
