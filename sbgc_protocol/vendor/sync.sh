#!/usr/bin/env bash
# Re-copy the vendored SimpleBGC32 protocol sources from upstream.
#
# These files are carried in-tree so the repository is self-contained. That
# means upstream fixes do not arrive on their own, and this is how they are
# taken deliberately rather than drifting in.
#
#   ./sync.sh                 # latest upstream main
#   ./sync.sh <commit-ish>    # a specific revision
#
# Afterwards run `make test`. Upstream's own byte-level suite is built here, so
# a revision that changes the wire format fails in this build rather than on a
# gimbal.
set -euo pipefail

UPSTREAM="https://github.com/magdang/simplebgc32-control.git"
REF="${1:-main}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Only the protocol is taken. Upstream's httpd, gamepad, applications and web
# UI belong to its standalone tools and have no place in a ROS driver.
SRC=(sbgc_api.c sbgc_params.c sbgc_gui_config.c)
HDR=(sbgc_api.h sbgc_params.h sbgc_gui_config.h)
TST=(test_sbgc_api.c sbgc_sim.py)

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

echo "fetching $UPSTREAM at $REF"
git clone --quiet --no-checkout "$UPSTREAM" "$tmp/up"
git -C "$tmp/up" checkout --quiet "$REF"
sha="$(git -C "$tmp/up" rev-parse HEAD)"

for f in "${SRC[@]}"; do cp "$tmp/up/src/$f"     "$HERE/src/$f";     done
for f in "${HDR[@]}"; do cp "$tmp/up/include/$f" "$HERE/include/$f"; done
for f in "${TST[@]}"; do cp "$tmp/up/test/$f"    "$HERE/test/$f";    done
cp "$tmp/up/LICENSE" "$HERE/LICENSE.upstream"

# The recorded commit is the whole point of the file: without it nobody can
# tell which upstream revision these copies correspond to.
sed -i -E "s/^\*\*Pinned at commit \`[0-9a-f]+\`\.\*\*$/**Pinned at commit \`$sha\`.**/" \
  "$HERE/README.md"

echo "synced to $sha"
echo "now run: make test"
git -C "$HERE" status --short -- "$HERE" 2>/dev/null || true
