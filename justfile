# These commands assume that ESP Rust tools are installed. I no longer how was is done exactly, but `espup` was used.

# Use bash to get `export` working
set shell := ["bash", "-uc"]

# Show commands as default
default:
	just --list

# Build ESP32 bridge firmware
bridge:
	. ~/export-esp.sh && cd bridge && cargo build

# Flash ESP32 board
flash:
	. ~/export-esp.sh && cd bridge && cargo run

# Pull cross compiler to build Synology NAS client executable
pull:
	docker pull messense/rust-musl-cross:armv7-musleabihf

# Build client for Synology NAS
client:
	docker run --rm -it -v "$(pwd)":/home/rust/src messense/rust-musl-cross:armv7-musleabihf cargo build --target armv7-unknown-linux-musleabihf -p client --release


# Run 'nohup ./client &' on NAS