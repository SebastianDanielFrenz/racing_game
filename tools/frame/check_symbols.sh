#!/usr/bin/env bash
# Steam Frame build check: the ARM64 GDExtension must not need anything newer
# than glibc 2.31 (the Steam Linux Runtime 3.0 ARM64 "sniper" generation) and
# must dlopen with every symbol resolved.
#
#   tools/frame/check_symbols.sh <librg_godot.arm64.so> [sysroot] [entry_symbol]
#
# 1. Highest GLIBC_x.y / GLIBCXX_x.y / CXXABI_x.y / GCC_x.y version the library
#    requires (objdump -T and readelf -V) - fails on a GLIBC_ above 2.31.
# 2. Lists the undefined, unversioned, non-weak symbols (they bind to whatever
#    the host process provides; expected: libpthread/libdl entries of glibc 2.31).
# 3. With qemu-aarch64 and the sysroot: dlopen(RTLD_NOW) of the library in a glibc
#    2.31 environment (ld.so from the sysroot) - proves that every symbol resolves.
set -u
LIB="${1:?usage: check_symbols.sh <library.so> [sysroot]}"
SR="${2:-${PS_AARCH64_SYSROOT:-$HOME/sdk/steamrt3c-arm64-sdk}}"
MAX_GLIBC="2.31"
rc=0

echo "== $LIB ($(stat -c %s "$LIB") bytes)"
echo "-- required symbol versions (readelf -V, version needs)"
readelf -V "$LIB" | grep -oE "Name: (GLIBC|GLIBCXX|CXXABI|GCC)_[0-9.]+" | sed 's/Name: //' | sort -uV | tr '\n' ' '
echo
maxg=$(objdump -T "$LIB" | grep -oE "GLIBC_[0-9]+(\.[0-9]+)*" | sort -uV | tail -1)
maxc=$(objdump -T "$LIB" | grep -oE "GLIBCXX_[0-9]+(\.[0-9]+)*" | sort -uV | tail -1)
echo "highest GLIBC_   : ${maxg:-none}"
echo "highest GLIBCXX_ : ${maxc:-none (libstdc++ is linked statically)}"
if [ -n "${maxg:-}" ] && [ "$(printf '%s\n%s\n' "GLIBC_$MAX_GLIBC" "$maxg" | sort -V | tail -1)" != "GLIBC_$MAX_GLIBC" ]; then
  echo "FAIL: requires $maxg, newer than GLIBC_$MAX_GLIBC"; rc=1
fi
if readelf -d "$LIB" | grep -q "NEEDED.*libstdc++"; then
  echo "FAIL: NEEDED libstdc++.so (must be linked statically)"; rc=1
fi
echo "-- NEEDED"
readelf -d "$LIB" | grep NEEDED | sed 's/.*Shared library: //'

echo "-- undefined, unversioned, non-weak symbols (resolved from the host process)"
objdump -T "$LIB" | awk '$0 ~ /\*UND\*/ && $0 ~ /Base/ && $2 !~ /w/ {print $NF}' | sort -u | tr '\n' ' '
echo

if command -v qemu-aarch64 >/dev/null 2>&1 && [ -f "$SR/usr/include/stdio.h" ]; then
  tmp="$(mktemp -d)"
  cat > "$tmp/dl.c" <<'EOF'
#include <dlfcn.h>
#include <stdio.h>
int main(int argc, char** argv) {
  void* h = dlopen(argv[1], RTLD_NOW | RTLD_GLOBAL);
  if (!h) { fprintf(stderr, "dlopen FAILED: %s\n", dlerror()); return 1; }
  void* e = dlsym(h, argc > 2 ? argv[2] : "rg_gdextension_entry");
  printf("dlopen OK (RTLD_NOW), entry symbol %s\n", e ? "present" : "ABSENT");
  return e ? 0 : 2;
}
EOF
  clang --target=aarch64-linux-gnu --sysroot="$SR" -fuse-ld=lld -O1 -o "$tmp/dl" "$tmp/dl.c" -ldl -lpthread 2>&1 | tail -3
  entry="${3:-rg_godot_library_init}"
  echo "-- dlopen under qemu-aarch64 with the glibc 2.31 sysroot (entry: $entry)"
  qemu-aarch64 -L "$SR" "$tmp/dl" "$LIB" "$entry" 2>&1 | tail -5
  [ "${PIPESTATUS[0]}" = "0" ] || rc=1
  rm -rf "$tmp"
else
  echo "-- dlopen check skipped (no qemu-aarch64 or sysroot)"
fi
[ $rc -eq 0 ] && echo "RESULT: OK" || echo "RESULT: FAIL"
exit $rc
