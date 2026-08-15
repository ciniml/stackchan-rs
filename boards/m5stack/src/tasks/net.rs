//! Network task (core 0): Wi-Fi station + a minimal HTTP control API, or a
//! provisioning access point with a captive setup page.
//!
//! Credentials come from the persisted [`crate::config`] record when present, otherwise
//! from the `WIFI_SSID` / `WIFI_PASS` environment variables at build time
//! (`option_env!`). With neither, the device starts a WPA2 AP (`Stackchan-XXXXXX` /
//! `sc-xxxxxxxx`, both MAC-derived, 192.168.4.1) with captive-portal DHCP/DNS (see
//! [`portal`]); the setup form at `/` stores credentials and reboots into station mode.
//! In station mode DHCP is used for addressing; the obtained IP is logged.
//!
//! ## API (HTTP/1.1, port 80, one request per connection)
//!
//! - `GET  /api/status`             → `{"expression":"happy","pan":90,"tilt":90,...}`
//! - `POST /api/expression/<name>`  → set expression (neutral/happy/angry/sad/doubt/sleepy)
//! - `POST /api/head/<pan>/<tilt>`  → move head, degrees (e.g. `/api/head/120/85`)
//! - `POST /api/sound/<name>`       → play a sound (arpeggio/blip)
//! - `POST /api/volume/<0-100>`     → set speaker volume (persisted on reboot)
//! - `POST /api/wifi/<ssid>/<pass>` → set Wi-Fi credentials (persisted + applied on
//!   reboot; no URL-decoding — avoid `/`, `%`, spaces in credentials for now)
//! - `POST /api/face`               → hot-swap the face bytecode (binary `AVDS` v1 body,
//!   e.g. `curl --data-binary @face.avbc`; persisted on reboot, max 8 KiB);
//!   `POST /api/face/reset` restores the default (also persisted on reboot)
//! - `POST /api/balloon/<text>`     → show balloon text (UTF-8, Japanese OK;
//!   percent-encoded, `_` also renders as space); `POST /api/balloon/clear` clears it
//! - `POST /api/wifi/clear`         → clear stored credentials (after `/api/reboot`
//!   the device comes back in provisioning AP mode)
//! - `POST /api/reboot`             → persist changed settings, then software reset
//! - `GET  /` (or `/setup`)         → Wi-Fi setup page; `POST /setup` (form) stores
//!   credentials and reboots
//!
//! Settings are deliberately written to flash only in the reboot path: a flash write
//! while the Wi-Fi driver is active (cache disabled mid-write) reliably took the whole
//! device down, and just before a reset that hazard does not matter.

use core::fmt::Write as FmtWrite;

use alloc::boxed::Box;

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{
    Config as NetConfig, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write as IoWrite;
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{self, Interface, WifiController};
use log::{info, warn};
use m5stack_avatar_rs::Expression;
use static_cell::StaticCell;

use crate::config::{Config, ConfigStore};
use crate::head::DEG;
use crate::shared_state::{BALLOON_CAP, STATE, Sound};
use crate::tasks::portal;

const WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");
const WIFI_PASS: Option<&str> = option_env!("WIFI_PASS");

const HTTP_PORT: u16 = 80;
/// Idle pause after an API head move, matching the touch hold-off.
const API_INTERACT_HOLD_MS: u32 = 3000;

static RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();

fn expression_name(e: Expression) -> &'static str {
    match e {
        Expression::Neutral => "neutral",
        Expression::Happy => "happy",
        Expression::Angry => "angry",
        Expression::Sad => "sad",
        Expression::Doubt => "doubt",
        Expression::Sleepy => "sleepy",
    }
}

fn expression_from_name(name: &str) -> Option<Expression> {
    Some(match name {
        "neutral" => Expression::Neutral,
        "happy" => Expression::Happy,
        "angry" => Expression::Angry,
        "sad" => Expression::Sad,
        "doubt" => Expression::Doubt,
        "sleepy" => Expression::Sleepy,
        _ => return None,
    })
}

#[embassy_executor::task]
async fn net_runner(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

/// Keep the station associated: (re)connect whenever the link drops.
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(info) => {
                info!("Wi-Fi connected (ssid {:?}, channel {})", info.ssid, info.channel);
                let _ = controller.wait_for_disconnect_async().await;
                warn!("Wi-Fi disconnected; reconnecting");
            }
            Err(e) => {
                warn!("Wi-Fi connect failed: {:?}; retrying in 5 s", e);
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn net(
    spawner: Spawner,
    wifi: WIFI<'static>,
    rng: Rng,
    store: ConfigStore,
    config: Config,
) {
    let (ssid, pass): (&str, &str) = if !config.wifi_ssid.is_empty() {
        (config.wifi_ssid.as_str(), config.wifi_pass.as_str())
    } else if let Some(ssid) = WIFI_SSID {
        (ssid, WIFI_PASS.unwrap_or(""))
    } else {
        info!("no Wi-Fi credentials; starting provisioning AP");
        return access_point(spawner, wifi, rng, store, config).await;
    };

    let (mut controller, interfaces) = match wifi::new(wifi, Default::default()) {
        Ok(x) => x,
        Err(e) => {
            warn!("Wi-Fi init failed: {:?}; network disabled", e);
            return;
        }
    };
    let station_config = wifi::Config::Station(
        StationConfig::default()
            .with_ssid(ssid)
            .with_password(pass.into()),
    );
    if let Err(e) = controller.set_config(&station_config) {
        warn!("Wi-Fi config failed: {:?}; network disabled", e);
        return;
    }

    let seed = ((rng.random() as u64) << 32) | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        interfaces.station,
        NetConfig::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(net_runner(runner).unwrap());
    spawner.spawn(connection(controller).unwrap());

    stack.wait_config_up().await;
    if let Some(v4) = stack.config_v4() {
        info!("IP address: {}", v4.address.address());
    }

    // Boxed: the serve future carries ~12 KiB of buffers, and both this and the AP
    // branch would otherwise be laid out side by side in the net task future — which
    // is briefly built on the main stack at spawn time and overflowed it.
    Box::pin(serve(stack, store, config, false)).await
}

/// Provisioning mode (the C++ firmware's AP + captive portal): bring up a WPA2 AP with
/// MAC-derived credentials, hand out addresses, answer every DNS name with our own
/// address, and serve the setup page. Submitting the form stores the credentials and
/// reboots into station mode.
async fn access_point(
    spawner: Spawner,
    wifi: WIFI<'static>,
    mut rng: Rng,
    store: ConfigStore,
    config: Config,
) {
    // Same derivation as the C++ firmware: SSID from the STA MAC tail, password from
    // one more byte, so a phone that saw the device before reconnects seamlessly.
    let mac_addr =
        esp_hal::efuse::interface_mac_address(esp_hal::efuse::InterfaceMacAddress::Station);
    let mac = mac_addr.as_bytes();
    let mut ap_ssid: heapless::String<32> = heapless::String::new();
    let _ = write!(ap_ssid, "Stackchan-{:02X}{:02X}{:02X}", mac[3], mac[4], mac[5]);
    let mut ap_pass: heapless::String<32> = heapless::String::new();
    let _ = write!(
        ap_pass,
        "sc-{:02x}{:02x}{:02x}{:02x}",
        mac[2], mac[3], mac[4], mac[5]
    );

    let (mut controller, interfaces) = match wifi::new(wifi, Default::default()) {
        Ok(x) => x,
        Err(e) => {
            warn!("Wi-Fi init failed: {:?}; network disabled", e);
            return;
        }
    };
    let ap_config = wifi::Config::AccessPoint(
        AccessPointConfig::default()
            .with_ssid(ap_ssid.as_str())
            .with_password(ap_pass.as_str().into())
            .with_auth_method(wifi::AuthenticationMethod::Wpa2Personal)
            .with_channel(6)
            .with_max_connections(4),
    );
    if let Err(e) = controller.set_config(&ap_config) {
        warn!("AP config failed: {:?}; network disabled", e);
        return;
    }

    let seed = ((rng.random() as u64) << 32) | rng.random() as u64;
    let net_config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(portal::AP_ADDR, 24),
        gateway: Some(portal::AP_ADDR),
        dns_servers: Default::default(),
    });
    let (stack, runner) = embassy_net::new(
        interfaces.access_point,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(net_runner(runner).unwrap());
    spawner.spawn(portal::dhcp_server(stack).unwrap());
    spawner.spawn(portal::dns_server(stack).unwrap());

    info!("AP up: SSID \"{}\" pass \"{}\" IP {}", ap_ssid, ap_pass, portal::AP_ADDR);
    let mut notice: heapless::String<BALLOON_CAP> = heapless::String::new();
    let _ = write!(
        notice,
        "Wi-Fi設定: AP「{}」(パスワード {}) に接続して http://192.168.4.1/ を開いてください",
        ap_ssid, ap_pass
    );
    STATE.post_balloon(&notice);

    // Keep the controller alive for the lifetime of the AP (dropping it would tear
    // Wi-Fi down); `serve` never returns.
    let _controller = controller;
    Box::pin(serve(stack, store, config, true)).await
}

async fn serve(
    stack: Stack<'static>,
    mut store: ConfigStore,
    mut config: Config,
    ap_mode: bool,
) -> ! {
    let loaded_config = config.clone();
    // Face bytecode awaiting persistence (Some(empty) = clear). Like the config, it is
    // written to flash only on the way into a reboot (see module docs).
    let mut pending_face: Option<alloc::vec::Vec<u8>> = None;
    let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 1024];
    // Large enough for the request head plus a face-bytecode upload body (~2-4 KiB).
    let mut req = [0u8; 8192];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if let Err(e) = socket.accept(HTTP_PORT).await {
            warn!("accept failed: {:?}", e);
            continue;
        }

        // Read the request head.
        let mut used = 0usize;
        let head_end = loop {
            match socket.read(&mut req[used..]).await {
                Ok(0) => break None,
                Ok(n) => {
                    used += n;
                    if let Some(pos) = find_header_end(&req[..used]) {
                        break Some(pos);
                    }
                    if used == req.len() {
                        break None; // request head too large
                    }
                }
                Err(e) => {
                    warn!("read failed: {:?}", e);
                    break None;
                }
            }
        };

        // Parse the head into owned values so the buffer can keep receiving the body.
        let parsed = head_end.and_then(|head_end| {
            let head = core::str::from_utf8(&req[..head_end]).ok()?;
            let line = head.lines().next()?;
            let mut parts = line.split(' ');
            let method: heapless::String<8> =
                heapless::String::try_from(parts.next().unwrap_or("")).ok()?;
            // Percent-encoded Japanese balloon text runs 9 bytes per character, so the
            // path buffer is sized for BALLOON_CAP (256) fully-encoded + the prefix.
            let path: heapless::String<1024> =
                heapless::String::try_from(parts.next().unwrap_or("")).ok()?;
            let content_length: usize = head
                .lines()
                .find_map(|l| {
                    let (name, value) = l.split_once(':')?;
                    name.eq_ignore_ascii_case("content-length")
                        .then(|| value.trim().parse().ok())?
                })
                .unwrap_or(0);
            Some((head_end, method, path, content_length))
        });

        if let Some((head_end, method, path, content_length)) = parsed {
            let (method, path) = (method.as_str(), path.as_str());

            // Read the request body when Content-Length is present (bounded by `req`).
            let body_start = head_end + 4;
            let body_end = body_start + content_length;
            let body_ok = body_end <= req.len();
            while body_ok && used < body_end {
                match socket.read(&mut req[used..]).await {
                    Ok(0) => break,
                    Ok(n) => used += n,
                    Err(e) => {
                        warn!("body read failed: {:?}", e);
                        break;
                    }
                }
            }
            let request_body: &[u8] = if body_ok && used >= body_end {
                &req[body_start..body_end]
            } else {
                &[]
            };

            let (status, reply, reboot) =
                handle_request(method, path, request_body, &mut config, &mut pending_face, ap_mode);
            let mut resp: heapless::String<1024> = heapless::String::new();
            match &reply {
                Reply::Json(body) => {
                    let _ = write!(
                        resp,
                        "HTTP/1.1 {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        status,
                        body.len(),
                        body
                    );
                }
                Reply::Page(html) => {
                    let _ = write!(
                        resp,
                        "HTTP/1.1 {}\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                        status,
                        html.len()
                    );
                }
                Reply::Redirect(loc) => {
                    let _ = write!(
                        resp,
                        "HTTP/1.1 302 Found\r\nLocation: {}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                        loc
                    );
                }
            }
            let _ = socket.write_all(resp.as_bytes()).await;
            if let Reply::Page(html) = &reply {
                let _ = socket.write_all(html.as_bytes()).await;
            }
            if reboot {
                let _ = socket.flush().await;
                socket.close();
                Timer::after(Duration::from_millis(200)).await;
                // Persist pending changes now — the flash write may take the Wi-Fi
                // stack down, but we are resetting immediately anyway.
                if config != loaded_config
                    && let Err(e) = store.save(&config)
                {
                    warn!("config save failed: {:?}", e);
                }
                if let Some(face) = pending_face.take()
                    && let Err(e) = store.save_face(&face)
                {
                    warn!("face save failed: {:?}", e);
                }
                info!("rebooting (API request)");
                esp_hal::system::software_reset();
            }
        }
        let _ = socket.flush().await;
        socket.close();
        // Give the peer a moment to receive FIN before the socket is dropped/reused.
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Decode a percent-encoded component into UTF-8 text (`+` → space; with
/// `underscore_as_space` also `_` → space, kept from the balloon API's ASCII-only days
/// so existing scripts still work). Returns `None` on a malformed `%` escape or invalid
/// UTF-8; input beyond [`BALLOON_CAP`] bytes is truncated at a character boundary.
fn percent_decode(raw: &str, underscore_as_space: bool) -> Option<heapless::String<BALLOON_CAP>> {
    let mut bytes: heapless::Vec<u8, BALLOON_CAP> = heapless::Vec::new();
    let mut it = raw.bytes();
    while let Some(b) = it.next() {
        let decoded = match b {
            b'%' => {
                let hi = (it.next()? as char).to_digit(16)?;
                let lo = (it.next()? as char).to_digit(16)?;
                (hi * 16 + lo) as u8
            }
            b'_' if underscore_as_space => b' ',
            b'+' => b' ',
            _ => b,
        };
        if bytes.push(decoded).is_err() {
            break;
        }
    }
    // Truncate a UTF-8 sequence cut off by the capacity limit.
    let text = loop {
        match core::str::from_utf8(&bytes) {
            Ok(s) => break s,
            Err(e) if e.error_len().is_none() && !bytes.is_empty() => {
                bytes.truncate(e.valid_up_to());
            }
            Err(_) => return None,
        }
    };
    Some(heapless::String::try_from(text).ok()?)
}

fn find_header_end(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n")
}

/// Wi-Fi setup page (the C++ firmware's provisioning UI equivalent, static version).
const SETUP_HTML: &str = r#"<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ｽﾀｯｸﾁｬﾝ Wi-Fi設定</title>
<style>
body{font-family:sans-serif;max-width:26em;margin:2em auto;padding:0 1em;background:#fafafa}
h1{font-size:1.3em}
form{background:#fff;border:1px solid #ddd;border-radius:8px;padding:1em}
label{display:block;margin:.8em 0 .2em}
input{width:100%;box-sizing:border-box;padding:.5em;font-size:1em;border:1px solid #bbb;border-radius:4px}
button{margin-top:1.2em;width:100%;padding:.7em;font-size:1em;border:0;border-radius:4px;background:#e8a33d;color:#fff}
p.note{color:#666;font-size:.85em}
</style></head><body>
<h1>ｽﾀｯｸﾁｬﾝ Wi-Fi設定</h1>
<form method="post" action="/setup">
<label for="ssid">SSID</label>
<input id="ssid" name="ssid" maxlength="32" required>
<label for="pass">パスワード</label>
<input id="pass" name="pass" type="password" maxlength="64">
<button type="submit">保存して再起動</button>
</form>
<p class="note">保存後、ｽﾀｯｸﾁｬﾝは再起動して設定したWi-Fiに接続します。</p>
</body></html>
"#;

const SETUP_DONE_HTML: &str = r#"<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>設定完了</title></head>
<body style="font-family:sans-serif;max-width:26em;margin:2em auto;padding:0 1em">
<h1>設定を保存しました</h1>
<p>ｽﾀｯｸﾁｬﾝを再起動して、設定したWi-Fiに接続します。この画面は閉じてください。</p>
</body></html>
"#;

const SETUP_FAIL_HTML: &str = r#"<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>設定エラー</title></head>
<body style="font-family:sans-serif;max-width:26em;margin:2em auto;padding:0 1em">
<h1>設定内容が不正です</h1>
<p>SSID は 1〜32 文字、パスワードは 64 文字以内で入力してください。
<a href="/">戻る</a></p>
</body></html>
"#;

/// Response payload: a JSON body, a static HTML page, or a redirect.
enum Reply {
    Json(heapless::String<512>),
    Page(&'static str),
    Redirect(&'static str),
}

/// Route a request. Returns (status line, reply, reboot-after-response).
fn handle_request(
    method: &str,
    path: &str,
    request_body: &[u8],
    config: &mut Config,
    pending_face: &mut Option<alloc::vec::Vec<u8>>,
    ap_mode: bool,
) -> (&'static str, Reply, bool) {
    let (status, reply) = route(method, path, request_body, config, pending_face, ap_mode);
    // The setup form also reboots on success: the credentials are persisted on the way
    // down and the device comes back in station mode.
    let reboot = method == "POST"
        && (path == "/api/reboot" || path == "/setup")
        && status.starts_with("200");
    (status, reply, reboot)
}

fn route(
    method: &str,
    path: &str,
    request_body: &[u8],
    config: &mut Config,
    pending_face: &mut Option<alloc::vec::Vec<u8>>,
    ap_mode: bool,
) -> (&'static str, Reply) {
    match (method, path) {
        // The Wi-Fi setup page (served in both modes so credentials can also be
        // changed from a browser while on the network).
        ("GET", "/") | ("GET", "/setup") => return ("200 OK", Reply::Page(SETUP_HTML)),
        ("POST", "/setup") => return handle_setup_form(request_body, config),
        _ => {}
    }
    if ap_mode && method == "GET" {
        // Captive-portal detection probes (and any other unknown page) redirect to
        // the setup page so the OS pops its sign-in sheet.
        if !path.starts_with("/api/") {
            return ("302 Found", Reply::Redirect("http://192.168.4.1/"));
        }
    }
    let (status, body) = route_json(method, path, request_body, config, pending_face);
    (status, Reply::Json(body))
}

/// Parse the `application/x-www-form-urlencoded` setup form (`ssid`, `pass`) and stage
/// the credentials; the caller reboots on success and the reboot path persists them.
fn handle_setup_form(request_body: &[u8], config: &mut Config) -> (&'static str, Reply) {
    let Ok(form) = core::str::from_utf8(request_body) else {
        return ("400 Bad Request", Reply::Page(SETUP_FAIL_HTML));
    };
    let mut ssid: Option<heapless::String<BALLOON_CAP>> = None;
    let mut pass: Option<heapless::String<BALLOON_CAP>> = None;
    for pair in form.split('&') {
        let Some((key, value)) = pair.split_once('=') else {
            continue;
        };
        match key {
            "ssid" => ssid = percent_decode(value, false),
            "pass" => pass = percent_decode(value, false),
            _ => {}
        }
    }
    let (Some(ssid), Some(pass)) = (ssid, pass) else {
        return ("400 Bad Request", Reply::Page(SETUP_FAIL_HTML));
    };
    if ssid.is_empty() || ssid.len() > 32 || pass.len() > 64 {
        return ("400 Bad Request", Reply::Page(SETUP_FAIL_HTML));
    }
    config.wifi_ssid.clear();
    let _ = config.wifi_ssid.push_str(&ssid);
    config.wifi_pass.clear();
    let _ = config.wifi_pass.push_str(&pass);
    info!("setup form accepted (ssid {:?}); rebooting into station mode", ssid);
    ("200 OK", Reply::Page(SETUP_DONE_HTML))
}

fn route_json(
    method: &str,
    path: &str,
    request_body: &[u8],
    config: &mut Config,
    pending_face: &mut Option<alloc::vec::Vec<u8>>,
) -> (&'static str, heapless::String<512>) {
    let mut body: heapless::String<512> = heapless::String::new();
    match (method, path) {
        ("GET", "/api/status") => {
            let (pan, tilt, _) = STATE.head_target();
            let _ = write!(
                body,
                "{{\"expression\":\"{}\",\"pan\":{},\"tilt\":{},\"mouth\":{},\"volume\":{}}}",
                expression_name(STATE.expression()),
                pan / DEG,
                tilt / DEG,
                STATE.mouth_open_pct(),
                STATE.volume(),
            );
            ("200 OK", body)
        }
        ("POST", "/api/reboot") => {
            let _ = write!(body, "{{\"reboot\":true}}");
            ("200 OK", body)
        }
        ("POST", _) if path.starts_with("/api/volume/") => {
            let value = path["/api/volume/".len()..].parse::<u8>().ok();
            match value {
                Some(v) if v <= 100 => {
                    STATE.set_volume(v);
                    config.volume = v;
                    let _ = write!(body, "{{\"volume\":{}}}", v);
                    ("200 OK", body)
                }
                _ => {
                    let _ = write!(body, "{{\"error\":\"expected /api/volume/<0-100>\"}}");
                    ("400 Bad Request", body)
                }
            }
        }
        ("POST", "/api/wifi/clear") => {
            // Clear stored credentials; after /api/reboot the device comes back in
            // provisioning AP mode.
            config.wifi_ssid.clear();
            config.wifi_pass.clear();
            let _ = write!(body, "{{\"wifi\":null}}");
            ("200 OK", body)
        }
        ("POST", _) if path.starts_with("/api/wifi/") => {
            let rest = &path["/api/wifi/".len()..];
            let mut it = rest.splitn(2, '/');
            let ssid = it.next().unwrap_or("");
            let pass = it.next().unwrap_or("");
            if ssid.is_empty() || ssid.len() > 32 || pass.len() > 64 {
                let _ = write!(body, "{{\"error\":\"expected /api/wifi/<ssid>/<pass>\"}}");
                return ("400 Bad Request", body);
            }
            config.wifi_ssid.clear();
            let _ = config.wifi_ssid.push_str(ssid);
            config.wifi_pass.clear();
            let _ = config.wifi_pass.push_str(pass);
            let _ = write!(body, "{{\"ssid\":\"{}\",\"note\":\"reboot to apply\"}}", ssid);
            ("200 OK", body)
        }
        ("POST", _) if path.starts_with("/api/expression/") => {
            let name = &path["/api/expression/".len()..];
            match expression_from_name(name) {
                Some(e) => {
                    STATE.set_expression(e);
                    let _ = write!(body, "{{\"expression\":\"{}\"}}", name);
                    ("200 OK", body)
                }
                None => {
                    let _ = write!(body, "{{\"error\":\"unknown expression\"}}");
                    ("400 Bad Request", body)
                }
            }
        }
        ("POST", _) if path.starts_with("/api/head/") => {
            let rest = &path["/api/head/".len()..];
            let mut it = rest.split('/');
            let pan = it.next().and_then(|s| s.parse::<u32>().ok());
            let tilt = it.next().and_then(|s| s.parse::<u32>().ok());
            match (pan, tilt) {
                (Some(pan), Some(tilt)) if pan <= 180 && tilt <= 180 => {
                    STATE.set_head_target(pan * DEG, tilt * DEG);
                    STATE.note_interaction(
                        Instant::now().as_millis() as u32,
                        API_INTERACT_HOLD_MS,
                    );
                    let _ = write!(body, "{{\"pan\":{},\"tilt\":{}}}", pan, tilt);
                    ("200 OK", body)
                }
                _ => {
                    let _ = write!(body, "{{\"error\":\"expected /api/head/<pan>/<tilt>\"}}");
                    ("400 Bad Request", body)
                }
            }
        }
        ("POST", "/api/face/reset") => {
            STATE.post_face(&[]);
            *pending_face = Some(alloc::vec::Vec::new());
            let _ = write!(body, "{{\"face\":\"default\"}}");
            ("200 OK", body)
        }
        ("POST", "/api/face") => {
            // Body must be an `AVDS` v1 bytecode file from tools/avatar_dsl.
            match m5stack_avatar_rs::stackchan::vm::decode(request_body) {
                Ok(_) if request_body.len() <= crate::config::FACE_MAX_LEN => {
                    STATE.post_face(request_body);
                    *pending_face = Some(request_body.to_vec());
                    let _ = write!(body, "{{\"face\":\"loaded\",\"bytes\":{}}}", request_body.len());
                    ("200 OK", body)
                }
                Ok(_) => {
                    let _ = write!(
                        body,
                        "{{\"error\":\"too large (max {} bytes)\"}}",
                        crate::config::FACE_MAX_LEN
                    );
                    ("400 Bad Request", body)
                }
                Err(e) => {
                    let _ = write!(body, "{{\"error\":\"bad bytecode: {:?}\"}}", e);
                    ("400 Bad Request", body)
                }
            }
        }
        ("POST", "/api/balloon/clear") => {
            STATE.post_balloon("");
            let _ = write!(body, "{{\"balloon\":null}}");
            ("200 OK", body)
        }
        ("POST", _) if path.starts_with("/api/balloon/") => {
            let raw = &path["/api/balloon/".len()..];
            match percent_decode(raw, true) {
                Some(text) => {
                    STATE.post_balloon(&text);
                    let _ = write!(body, "{{\"balloon\":\"{}\"}}", text);
                    ("200 OK", body)
                }
                None => {
                    let _ = write!(body, "{{\"error\":\"bad percent-encoding or utf-8\"}}");
                    ("400 Bad Request", body)
                }
            }
        }
        ("POST", "/api/sound/arpeggio") => {
            STATE.post_sound(Sound::Arpeggio);
            let _ = write!(body, "{{\"sound\":\"arpeggio\"}}");
            ("200 OK", body)
        }
        ("POST", "/api/sound/blip") => {
            STATE.post_sound(Sound::Blip);
            let _ = write!(body, "{{\"sound\":\"blip\"}}");
            ("200 OK", body)
        }
        _ => {
            let _ = write!(body, "{{\"error\":\"not found\"}}");
            ("404 Not Found", body)
        }
    }
}
