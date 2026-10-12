#!/bin/bash
# Installs (or verifies, with --check) Valve's Steam Runtime 3c ARM64 SDK sysroot
# into ~/sdk/steamrt3c-arm64-sdk. Runs inside WSL; called by
# tools/setup_dev_env.ps1 -Frame. Idempotent.
#
# steamrt3c = "Steam Linux Runtime 3.0 ARM64 (Sniper)" (Debian 11: glibc 2.31,
# GCC 10 runtime libstdc++, plus GCC 14 headers/libstdc++.a in the SDK). It is the
# OLDEST runtime a Steam Frame devkit title can be launched in, so a build against
# it runs on every newer one too (Steam Linux Runtime 4: glibc 2.41; the host).
# Official source only: repo.steampowered.com (Valve), SHA256 from the same directory.
set -euo pipefail
BUILD_ID="3c.0.20260729.253765"
NAME="com.valvesoftware.SteamRuntime.Sdk-arm64-steamrt3c-sysroot.tar.gz"
BASE="https://repo.steampowered.com/steamrt3c/images/${BUILD_ID}"
ROOT="${PS_AARCH64_SYSROOT:-$HOME/sdk/steamrt3c-arm64-sdk}"
DL="$HOME/sdk/dl"
STAMP="$ROOT/.rg-sysroot-ready"

if [ "${1:-}" = "--check" ]; then
  if [ -f "$STAMP" ] && [ -f "$ROOT/usr/include/stdio.h" ]; then
    echo "OK $ROOT ($(cat "$STAMP"))"; exit 0
  fi
  echo "MISSING $ROOT"; exit 1
fi

if [ -f "$STAMP" ] && grep -q "$BUILD_ID" "$STAMP"; then echo "already installed: $ROOT"; exit 0; fi
mkdir -p "$DL"
cd "$DL"
curl -fsSL --http1.1 --retry 5 -C - -O "$BASE/$NAME"
curl -fsSL --http1.1 --retry 5 -o "SHA256SUMS.3c" "$BASE/SHA256SUMS"
want=$(grep " \*\?$NAME\$" SHA256SUMS.3c | awk '{print $1}')
got=$(sha256sum "$NAME" | awk '{print $1}')
if [ -z "$want" ] || [ "$want" != "$got" ]; then echo "SHA256 mismatch for $NAME (want '$want', got '$got')"; exit 2; fi
echo "sha256 ok $got"
rm -rf "$ROOT"; mkdir -p "$ROOT"
tar -xzf "$DL/$NAME" -C "$ROOT" --exclude="./dev/*"
# Absolute symlinks (libpthread.so -> /lib/aarch64-linux-gnu/...) dangle on the host
# and make the linker fall back to the static archives: rewrite them relative.
python3 - "$ROOT" <<'PY'
import os, sys
root = sys.argv[1]
n = 0
for d, dirs, files in os.walk(root):
    for f in files + dirs:
        p = os.path.join(d, f)
        if os.path.islink(p):
            t = os.readlink(p)
            if t.startswith('/'):
                new = os.path.relpath(root + t, d)
                os.unlink(p)
                os.symlink(new, p)
                n += 1
print("relinked %d absolute symlinks" % n)
PY
echo "$BUILD_ID" > "$STAMP"
echo "installed $ROOT"
