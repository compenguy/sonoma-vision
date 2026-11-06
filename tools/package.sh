#!/bin/sh
# Bundle a built release binary for distribution:
#
#   tools/package.sh <target triple> <bundle name> [--deploy]
#
# Writes dist/sonoma-vision-<version>-<bundle name>.tar.gz (and a .sha256 beside it), holding the
# binary, README.md, LICENSE, NOTICE, the field layouts, and the licenses of vendored code built into
# the binary; with --deploy, also deploy/ (the systemd unit and example configuration). Build first,
# e.g. `cargo build --release --target x86_64-unknown-linux-gnu` or tools/build-systemcore.sh.
set -eu
[ $# -ge 2 ] || { sed -n '2,10s/^# \{0,1\}//p' "$0" >&2; exit 2; }
target=$1
name=$2
deploy=${3:-}
cd "$(dirname "$0")/.."
version=$(sed -n 's/^version = "\(.*\)"$/\1/p' Cargo.toml | head -n 1)
bin=target/$target/release/sonoma-vision
[ -x "$bin" ] || { echo "no $bin: build it first" >&2; exit 1; }
dir=sonoma-vision-$version-$name
rm -rf "dist/$dir" "dist/$dir.tar.gz" "dist/$dir.tar.gz.sha256"
mkdir -p "dist/$dir/licenses"
cp "$bin" README.md LICENSE NOTICE "dist/$dir/"
cp -R fields "dist/$dir/"
cp vendor/napa-vision/LICENSE "dist/$dir/licenses/napa-vision-LICENSE"
cp vendor/napa-vision/NOTICE "dist/$dir/licenses/napa-vision-NOTICE"
cp vendor/apriltag/LICENSE.md "dist/$dir/licenses/apriltag-LICENSE.md"
if [ "$deploy" = --deploy ]; then
    cp -R deploy "dist/$dir/"
fi
tar -C dist -czf "dist/$dir.tar.gz" "$dir"
(cd dist && sha256sum "$dir.tar.gz" > "$dir.tar.gz.sha256")
rm -rf "dist/$dir"
echo "dist/$dir.tar.gz"
