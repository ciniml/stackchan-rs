# Toolchain environment for building the no_std boards (boards/m5stack).
#
# NOTE: this used to point at the 2021-era GCC 8.4.0 cross toolchains in ~/.espressif.
# That linker silently produces BROKEN images with current esp-hal linker scripts
# (symptoms: garbage .rodata/.data reads at runtime — wild LoadProhibited crashes in
# Uart::new / esp-sync, endless NUL output from log formatting). Always use the GCC
# bundled with the espup `esp` toolchain instead.
export LIBCLANG_PATH="/home/kenta/.espressif/tools/xtensa-esp32-elf-clang/esp-14.0.0-20220415-x86_64-unknown-linux-gnu/lib/"
export PATH="$HOME/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin:$PATH"
