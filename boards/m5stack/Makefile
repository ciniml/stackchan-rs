.PHONY: run

debug:
	cargo build
	scp target/xtensa-esp32-none-elf/debug/stackchan-m5stack intrepid-t:/home/kenta/wk/
	ssh intrepid-t /home/kenta/.cargo/bin/espflash --use-stub --speed 1500000 --flash-freq 40M --flash-mode DIO --monitor /dev/ttyUSB0 /home/kenta/wk/stackchan-m5stack

release:
	cargo build --release
	scp target/xtensa-esp32-none-elf/release/stackchan-m5stack intrepid-t:/home/kenta/wk/
	ssh intrepid-t /home/kenta/.cargo/bin/espflash --use-stub --speed 1500000 --flash-freq 40M --flash-mode DIO --monitor /dev/ttyUSB0 /home/kenta/wk/stackchan-m5stack

disasm:
	xtensa-esp32-elf-objdump -dSC target/xtensa-esp32-none-elf/release/stackchan-m5stack > dump.lst
	code dump.lst