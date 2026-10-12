// glibc 2.31 compatibility symbols for the Steam Frame ARM64 build
// (linux-arm64-release preset, cmake/toolchains/aarch64-linux-gnu-clang.cmake).
//
// The game is compiled against Valve's Steam Runtime 3c ("sniper") glibc 2.31
// headers/libraries, but statically links the C++ runtime (libstdc++.a / libgcc)
// of a newer GCC (the Ubuntu cross package; sniper's own GCC 10 lacks the C++20
// library the code needs). Those archives were built against a newer glibc and
// reference a handful of symbols glibc 2.31 does not export. They are defined
// here, with hidden visibility, so the link binds to these and the finished
// library has no undefined reference newer than glibc 2.31 (checked by
// tools/frame/check_symbols.sh, which tools/frame_install.ps1 runs).
//
// Only the static libstdc++/libgcc objects call these; the game's own code does
// not. On a newer glibc the real symbols exist but are not used (hidden, bound
// at link time).
#include <cerrno>
#include <cstddef>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sys/random.h>

#define RG_HIDDEN __attribute__((visibility("hidden")))

extern "C" {

// glibc 2.32. libstdc++ uses it to skip atomics in single-threaded programs; 0 =
// "may be multi-threaded", always correct.
RG_HIDDEN char __libc_single_threaded = 0;

// glibc 2.38: C23 strtol/strtoul (identical except for 0b prefixes).
RG_HIDDEN long __isoc23_strtol(const char* s, char** end, int base) { return std::strtol(s, end, base); }
RG_HIDDEN unsigned long __isoc23_strtoul(const char* s, char** end, int base) { return std::strtoul(s, end, base); }
RG_HIDDEN long long __isoc23_strtoll(const char* s, char** end, int base) { return std::strtoll(s, end, base); }
RG_HIDDEN unsigned long long __isoc23_strtoull(const char* s, char** end, int base) { return std::strtoull(s, end, base); }

// glibc 2.38: C23 scanf family (godot-cpp / libstdc++ call sscanf; differences
// are the %b conversion only).
RG_HIDDEN int __isoc23_vsscanf(const char* s, const char* fmt, va_list ap) { return std::vsscanf(s, fmt, ap); }
RG_HIDDEN int __isoc23_sscanf(const char* s, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int r = std::vsscanf(s, fmt, ap);
  va_end(ap);
  return r;
}

// glibc 2.36: libstdc++'s std::random_device. getrandom (glibc 2.25) is the same
// kernel source arc4random reads.
RG_HIDDEN uint32_t arc4random(void) {
  uint32_t v = 0;
  unsigned char* p = reinterpret_cast<unsigned char*>(&v);
  size_t got = 0;
  while (got < sizeof v) {
    ssize_t n = getrandom(p + got, sizeof v - got, 0);
    if (n < 0) {
      if (errno == EINTR) continue;
      std::abort();
    }
    got += static_cast<size_t>(n);
  }
  return v;
}

// glibc 2.35: libgcc's unwinder looks up the FDE of a pc with it and falls back
// to dl_iterate_phdr when it fails (-1), the pre-2.35 behaviour.
RG_HIDDEN int _dl_find_object(void*, void*) { return -1; }

}  // extern "C"
