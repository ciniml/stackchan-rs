//! Audio task (core 0): plays short synthesized sounds (boot arpeggio, touch blip)
//! through the board speaker and drives lip sync.
//!
//! Each note is synthesized in full into a heap buffer and sent with a single async DMA
//! write, so there are no re-arm gaps inside a note. Notes start and end at zero
//! envelope amplitude, which masks the (microsecond) DMA re-arm between notes — this is
//! much simpler and more robust than the circular-DMA streaming approach, whose state
//! machine proved fragile across ring wrap-arounds.
//!
//! While a note plays, a fixed mouth openness is published to [`STATE`] for the render
//! task (lip sync), cleared when the sound ends.

use alloc::vec;
use embassy_time::{Duration, Ticker};
use log::warn;
use micromath::F32Ext;

use crate::board::SpeakerTy;
use crate::shared_state::{STATE, Sound};

/// I2S sample rate. Also used by the board bring-up to configure I2S and the amplifier.
pub const SAMPLE_RATE_HZ: u32 = 24_000;

/// Output amplitude relative to full scale.
const AMPLITUDE: f32 = 0.35;
/// Poll period for the sound-command mailbox.
const POLL_MS: u64 = 50;
/// Longest supported note. Bounds the synth buffer (96 bytes/ms at 24 kHz stereo 16-bit)
/// and must stay within the DMA descriptor budget (32 KiB) of the board bring-up.
const MAX_NOTE_MS: u32 = 300;

struct Note {
    freq_hz: f32,
    duration_ms: u32,
}

const ARPEGGIO: &[Note] = &[
    Note { freq_hz: 523.25, duration_ms: 120 }, // C5
    Note { freq_hz: 659.25, duration_ms: 120 }, // E5
    Note { freq_hz: 784.00, duration_ms: 120 }, // G5
    Note { freq_hz: 1046.50, duration_ms: 200 }, // C6
];

const BLIP: &[Note] = &[Note { freq_hz: 880.0, duration_ms: 60 }];

/// Linear attack (5 ms) then linear decay to zero over the note. `t` is 0..1 within the
/// note, `attack` the attack length as a fraction of the note. Starting and ending at
/// zero keeps note boundaries click-free.
fn envelope(t: f32, attack: f32) -> f32 {
    if t < attack {
        t / attack
    } else {
        let d = (t - attack) / (1.0 - attack);
        1.0 - d
    }
}

/// Synthesize `note` into `buf` (stereo 16-bit LE frames). Returns the used byte length.
fn synth_note(note: &Note, buf: &mut [u8]) -> usize {
    let volume = STATE.volume() as f32 / 100.0;
    let total = (SAMPLE_RATE_HZ * note.duration_ms.min(MAX_NOTE_MS) / 1000) as usize;
    let attack = (5.0 * SAMPLE_RATE_HZ as f32 / 1000.0) / total as f32;
    let phase_step = note.freq_hz / SAMPLE_RATE_HZ as f32;
    let mut phase: f32 = 0.0;
    for i in 0..total {
        let t = i as f32 / total as f32;
        let s = (phase * core::f32::consts::TAU).sin()
            * AMPLITUDE
            * volume
            * envelope(t, attack);
        phase += phase_step;
        if phase >= 1.0 {
            phase -= 1.0;
        }
        let bytes = ((s * 32767.0) as i16).to_le_bytes();
        let off = i * 4;
        // Same sample on both channels (the speakers are mono amps).
        buf[off] = bytes[0];
        buf[off + 1] = bytes[1];
        buf[off + 2] = bytes[0];
        buf[off + 3] = bytes[1];
    }
    total * 4
}

#[embassy_executor::task]
pub async fn audio(mut speaker: SpeakerTy) {
    let mut buf = vec![0u8; (SAMPLE_RATE_HZ * MAX_NOTE_MS / 1000) as usize * 4];
    let mut ticker = Ticker::every(Duration::from_millis(POLL_MS));
    loop {
        ticker.next().await;
        let Some(sound) = STATE.take_sound() else {
            continue;
        };
        let notes = match sound {
            Sound::Arpeggio => ARPEGGIO,
            Sound::Blip => BLIP,
        };
        for note in notes {
            let len = synth_note(note, &mut buf);
            STATE.set_mouth_open_pct(60);
            if let Err(e) = speaker.write_dma_async(&mut buf[..len]).await {
                warn!("I2S write failed: {:?}", e);
                break;
            }
        }
        STATE.set_mouth_open_pct(0);
    }
}
