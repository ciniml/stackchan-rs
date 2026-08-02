//! Persistent configuration in on-chip flash (the C++ firmware's NVS analogue).
//!
//! A single fixed-layout record is stored in the `nvs` partition region of the default
//! espflash partition table (0x9000, one 4 KiB sector) — this firmware does not use the
//! ESP-IDF NVS format, the region is simply repurposed. Writes park the second core
//! automatically (`multicore_auto_park`), so the servo task's core is safe while the
//! cache is disabled.
//!
//! Layout (little-endian): magic `u32` ("SCFG"), version `u16`, ssid_len `u8`,
//! pass_len `u8`, ssid `[u8;32]`, pass `[u8;64]`, volume `u8`, pad `[u8;3]`,
//! checksum `u32` (wrapping byte sum of everything before it).
//!
//! The face bytecode (the C++ firmware's `storage.cpp` analogue) lives in the same
//! region from the next 4 KiB sector (0xA000): magic `u32` ("SFCE"), version `u16`,
//! pad `u16`, len `u32`, checksum `u32` (wrapping byte sum of the payload), then the
//! raw `AVDS` file. `len == 0` means "use the built-in default face".

use alloc::vec;
use alloc::vec::Vec;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use log::{info, warn};

const CONFIG_ADDR: u32 = 0x9000;
const MAGIC: u32 = 0x5343_4647; // "SCFG"
const VERSION: u16 = 1;
const RECORD_LEN: usize = 4 + 2 + 1 + 1 + 32 + 64 + 1 + 3 + 4; // = 112

const FACE_ADDR: u32 = 0xA000;
const FACE_MAGIC: u32 = 0x5346_4345; // "SFCE"
const FACE_VERSION: u16 = 1;
const FACE_HEADER_LEN: usize = 16;
/// Maximum persisted face size; matches the HTTP request buffer so anything accepted
/// by `POST /api/face` fits (the nvs region has 20 KiB left past the config sector).
pub const FACE_MAX_LEN: usize = 8192;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Config {
    pub wifi_ssid: heapless::String<32>,
    pub wifi_pass: heapless::String<64>,
    /// Speaker volume, 0..=100.
    pub volume: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            wifi_ssid: heapless::String::new(),
            wifi_pass: heapless::String::new(),
            volume: 80,
        }
    }
}

pub struct ConfigStore {
    flash: FlashStorage<'static>,
}

fn checksum(bytes: &[u8]) -> u32 {
    bytes.iter().fold(0u32, |acc, &b| acc.wrapping_add(b as u32))
}

impl ConfigStore {
    pub fn new(flash: FLASH<'static>) -> Self {
        let flash = FlashStorage::new(flash).multicore_auto_park();
        Self { flash }
    }

    /// Load the stored config; returns the default when the record is missing or corrupt.
    pub fn load(&mut self) -> Config {
        let mut buf = [0u8; RECORD_LEN];
        if let Err(e) = self.flash.read(CONFIG_ADDR, &mut buf) {
            warn!("config read failed: {:?}; using defaults", e);
            return Config::default();
        }
        let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        let version = u16::from_le_bytes(buf[4..6].try_into().unwrap());
        let stored_sum = u32::from_le_bytes(buf[RECORD_LEN - 4..].try_into().unwrap());
        if magic != MAGIC || version != VERSION || stored_sum != checksum(&buf[..RECORD_LEN - 4]) {
            info!("no valid config record; using defaults");
            return Config::default();
        }
        let ssid_len = (buf[6] as usize).min(32);
        let pass_len = (buf[7] as usize).min(64);
        let mut config = Config::default();
        if let Ok(s) = core::str::from_utf8(&buf[8..8 + ssid_len]) {
            let _ = config.wifi_ssid.push_str(s);
        }
        if let Ok(s) = core::str::from_utf8(&buf[40..40 + pass_len]) {
            let _ = config.wifi_pass.push_str(s);
        }
        config.volume = buf[104].min(100);
        info!(
            "config loaded (ssid {:?}, volume {})",
            config.wifi_ssid, config.volume
        );
        config
    }

    pub fn save(&mut self, config: &Config) -> Result<(), esp_storage::FlashStorageError> {
        let mut buf = [0u8; RECORD_LEN];
        buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        buf[4..6].copy_from_slice(&VERSION.to_le_bytes());
        buf[6] = config.wifi_ssid.len() as u8;
        buf[7] = config.wifi_pass.len() as u8;
        buf[8..8 + config.wifi_ssid.len()].copy_from_slice(config.wifi_ssid.as_bytes());
        buf[40..40 + config.wifi_pass.len()].copy_from_slice(config.wifi_pass.as_bytes());
        buf[104] = config.volume.min(100);
        let sum = checksum(&buf[..RECORD_LEN - 4]);
        buf[RECORD_LEN - 4..].copy_from_slice(&sum.to_le_bytes());
        self.flash.write(CONFIG_ADDR, &buf)?;
        info!("config saved");
        Ok(())
    }

    /// Load the persisted face bytecode. `None` when absent, corrupt, or cleared
    /// (the caller should then use the built-in default face).
    pub fn load_face(&mut self) -> Option<Vec<u8>> {
        let mut header = [0u8; FACE_HEADER_LEN];
        if self.flash.read(FACE_ADDR, &mut header).is_err() {
            return None;
        }
        let magic = u32::from_le_bytes(header[0..4].try_into().unwrap());
        let version = u16::from_le_bytes(header[4..6].try_into().unwrap());
        let len = u32::from_le_bytes(header[8..12].try_into().unwrap()) as usize;
        let stored_sum = u32::from_le_bytes(header[12..16].try_into().unwrap());
        if magic != FACE_MAGIC || version != FACE_VERSION || len == 0 || len > FACE_MAX_LEN {
            return None;
        }
        let mut face = vec![0u8; len];
        if self
            .flash
            .read(FACE_ADDR + FACE_HEADER_LEN as u32, &mut face)
            .is_err()
            || checksum(&face) != stored_sum
        {
            warn!("face record corrupt; using default face");
            return None;
        }
        info!("face bytecode loaded from flash ({} bytes)", len);
        Some(face)
    }

    /// Persist the face bytecode (empty slice = clear, reverting to the default face).
    /// Like [`Self::save`], only call this on the way into a reset — flash writes while
    /// Wi-Fi is active take the device down.
    pub fn save_face(&mut self, face: &[u8]) -> Result<(), esp_storage::FlashStorageError> {
        let len = face.len().min(FACE_MAX_LEN);
        let mut buf = vec![0u8; FACE_HEADER_LEN + len];
        buf[0..4].copy_from_slice(&FACE_MAGIC.to_le_bytes());
        buf[4..6].copy_from_slice(&FACE_VERSION.to_le_bytes());
        buf[8..12].copy_from_slice(&(len as u32).to_le_bytes());
        buf[12..16].copy_from_slice(&checksum(&face[..len]).to_le_bytes());
        buf[FACE_HEADER_LEN..].copy_from_slice(&face[..len]);
        self.flash.write(FACE_ADDR, &buf)?;
        info!("face bytecode saved ({} bytes)", len);
        Ok(())
    }
}
