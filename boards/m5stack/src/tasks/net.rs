//! Network task (core 0): Wi-Fi station + a minimal HTTP control API.
//!
//! Credentials come from the persisted [`crate::config`] record when present, otherwise
//! from the `WIFI_SSID` / `WIFI_PASS` environment variables at build time
//! (`option_env!`); with neither, the task exits and the firmware runs offline. DHCP is
//! used for addressing; the obtained IP is logged.
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
//! - `POST /api/reboot`             → persist changed settings, then software reset
//!
//! Settings are deliberately written to flash only in the reboot path: a flash write
//! while the Wi-Fi driver is active (cache disabled mid-write) reliably took the whole
//! device down, and just before a reset that hazard does not matter.

use core::fmt::Write as FmtWrite;

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config as NetConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write as IoWrite;
use esp_hal::peripherals::WIFI;
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{self, Interface, WifiController};
use log::{info, warn};
use m5stack_avatar_rs::Expression;
use static_cell::StaticCell;

use crate::config::{Config, ConfigStore};
use crate::head::DEG;
use crate::shared_state::{STATE, Sound};

const WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");
const WIFI_PASS: Option<&str> = option_env!("WIFI_PASS");

const HTTP_PORT: u16 = 80;
/// Idle pause after an API head move, matching the touch hold-off.
const API_INTERACT_HOLD_MS: u32 = 3000;

static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

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
        info!("no Wi-Fi credentials (config empty, WIFI_SSID unset); network disabled");
        return;
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

    serve(stack, store, config).await
}

async fn serve(stack: Stack<'static>, mut store: ConfigStore, mut config: Config) -> ! {
    let loaded_config = config.clone();
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut req = [0u8; 1024];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if let Err(e) = socket.accept(HTTP_PORT).await {
            warn!("accept failed: {:?}", e);
            continue;
        }

        // Read the request head (we only care about the request line; ignore body).
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

        if head_end.is_some()
            && let Ok(text) = core::str::from_utf8(&req[..used])
            && let Some(line) = text.lines().next()
        {
            let mut parts = line.split(' ');
            let method = parts.next().unwrap_or("");
            let path = parts.next().unwrap_or("");
            let (status, body, reboot) = handle_request(method, path, &mut config);
            let mut resp: heapless::String<512> = heapless::String::new();
            let _ = write!(
                resp,
                "HTTP/1.1 {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                status,
                body.len(),
                body
            );
            let _ = socket.write_all(resp.as_bytes()).await;
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

fn find_header_end(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n")
}

/// Route a request. Returns (status line, JSON body, reboot-after-response).
fn handle_request(
    method: &str,
    path: &str,
    config: &mut Config,
) -> (&'static str, heapless::String<256>, bool) {
    let (status, body) = route(method, path, config);
    let reboot = method == "POST" && path == "/api/reboot" && status.starts_with("200");
    (status, body, reboot)
}

fn route(
    method: &str,
    path: &str,
    config: &mut Config,
) -> (&'static str, heapless::String<256>) {
    let mut body: heapless::String<256> = heapless::String::new();
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
