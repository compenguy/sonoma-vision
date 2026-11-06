#!/bin/sh
# Cross-compile a release binary for SystemCore (aarch64 Linux).
#
# Needs the Rust target and an aarch64 C toolchain with its sysroot; on Debian or Ubuntu:
#   rustup target add aarch64-unknown-linux-gnu
#   sudo apt install gcc-aarch64-linux-gnu libc6-dev-arm64-cross libclang-dev
# v4l2r generates its V4L2 bindings with bindgen at build time, so clang must see the target's headers
# (the V4L2 headers themselves are vendored: see .cargo/config.toml).
set -eu
SYSROOT=${SYSROOT:-/usr/aarch64-linux-gnu}
export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=${LINKER:-aarch64-linux-gnu-gcc}
export BINDGEN_EXTRA_CLANG_ARGS_aarch64_unknown_linux_gnu="--sysroot=$SYSROOT -I$SYSROOT/include"
cargo build --release --target aarch64-unknown-linux-gnu "$@"
echo "built target/aarch64-unknown-linux-gnu/release/sonoma-vision"
