//! Stackchan firmware for the M5Stack family. Choose the board variant via Cargo features:
//!
//!   `--features core2`  (default) — M5Stack Core2, ESP32, AXP192 + ILI9341
//!   `--no-default-features --features cores3` — M5Stack CoreS3 / SE, ESP32-S3, AXP2101 +
//!                                               AW9523B + ILI9342
//!
//! Build target must match the chip:
//!   Core2  -> `--target xtensa-esp32-none-elf`
//!   CoreS3 -> `--target xtensa-esp32s3-none-elf`
//!
//! Layout:
//!   `board/`       — per-board bring-up (BSP): power, display, servo bus. Returns a `Board`.
//!   `config`       — persistent settings in flash (Wi-Fi credentials, volume).
//!   `head`         — board-independent pan/tilt pose space (`HeadDriver` + `HeadLimits`).
//!   `shared_state` — lock-free state shared between tasks (pose targets, expression).
//!   `tasks/`       — embassy tasks: avatar rendering, servo motion, input, audio, network.
//!
//! Core assignment (both chips are dual-core Xtensa):
//!   core 0 — render + idle + input + audio + network. Slow, jitter-tolerant work.
//!   core 1 — the servo task, alone on its own executor, so real-time head motion is
//!            never delayed by rendering or communication.
//!
//! This does not use `#[esp_rtos::main]`: the scheduler (`esp_rtos::start`, inside
//! `board::init`) must be running before the embassy executor starts, and starting the
//! second core is cleanest before entering the executor as well. A plain `#[esp_hal::main]`
//! makes that ordering explicit.

#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::system::Stack;
use log::info;
use static_cell::StaticCell;

#[cfg(all(feature = "core2", feature = "cores3"))]
compile_error!("Enable only one of `core2` or `cores3`.");

#[cfg(not(any(feature = "core2", feature = "cores3")))]
compile_error!("Enable one of `core2` or `cores3`.");

mod board;
mod config;
mod head;
mod shared_state;
mod tasks;

// ESP-IDF app descriptor, required by the 2nd-stage bootloader / espflash 4.x.
esp_bootloader_esp_idf::esp_app_desc!();

const SERVO_CORE_STACK_SIZE: usize = 8192;
static SERVO_CORE_STACK: StaticCell<Stack<SERVO_CORE_STACK_SIZE>> = StaticCell::new();
static SERVO_CORE_EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
static MAIN_EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

#[esp_hal::main]
fn main() -> ! {
    // Brings up the chip and all peripherals; calls `esp_rtos::start` internally, so the
    // scheduler is running (and this context is the main task) from here on.
    let board = board::init();

    let mut config_store = config::ConfigStore::new(board.flash);
    let saved_config = config_store.load();
    shared_state::STATE.set_volume(saved_config.volume);

    // Real-time work goes to core 1: build the head driver there (its UART sharing must
    // not cross cores) and run the servo task on a dedicated executor.
    if board.head_present {
        let head_parts = board.head_parts;
        let limits = board.limits;
        let stack = SERVO_CORE_STACK.init(Stack::new());
        esp_rtos::start_second_core(
            board.cpu_ctrl,
            board.sw_int1,
            stack,
            move || {
                info!("servo core (core 1) up");
                let head = board::make_head(head_parts);
                let executor = SERVO_CORE_EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|s| s.spawn(tasks::servo::servo(head, limits).unwrap()))
            },
        );
    } else {
        info!("head not present; running avatar only");
    }

    let executor = MAIN_EXECUTOR.init(esp_rtos::embassy::Executor::new());
    executor.run(move |spawner| {
        if board.head_present {
            spawner.spawn(tasks::idle::idle(board.rng, board.limits).unwrap());
        }

        if let Some(touch) = board.touch {
            spawner.spawn(tasks::input::input(touch, board.limits).unwrap());
        }

        if let Some(speaker) = board.speaker {
            shared_state::STATE.post_sound(shared_state::Sound::Arpeggio);
            spawner.spawn(tasks::audio::audio(speaker).unwrap());
        }

        spawner.spawn(
            tasks::net::net(spawner, board.wifi, board.rng, config_store, saved_config).unwrap(),
        );

        spawner.spawn(tasks::render::render(board.display).unwrap());
    })
}
