# CMake toolchain file: clang cross-compiling racing_game's GDExtension to
# native Linux ARM64 (aarch64-linux-gnu) for the Steam Frame. Used by the
# `linux-arm64-release` preset (CMakePresets.json). Built in WSL; build dirs
# live under ~/build, never /mnt/s.
#
# It reuses physics_sim's toolchain file (external/physics_sim/cmake/toolchains/
# aarch64-linux-gnu-clang.cmake, pin 5ff0675 or later) and its environment
# variables, so both repos cross-build the same way:
#   PS_AARCH64_SYSROOT        root of the target sysroot (CMAKE_SYSROOT and the
#                             qemu-user -L path)
#
# Library policy (two parts, deliberately different generations):
#
# 1. C library = glibc 2.31, from Valve's Steam Runtime 3c ARM64 SDK sysroot.
#    A Steam Frame devkit upload runs a native ARM64 game in "Steam Linux
#    Runtime 3.0 ARM64 (Sniper)" (Valve's Steam Frame loadgames page;
#    repo.steampowered.com/steamrt3c): Debian 11, glibc 2.31. Linking WSL's own
#    cross libraries (Ubuntu 26.04, glibc 2.43) would need symbols that runtime
#    does not have. tools\setup_dev_env.ps1 -Frame installs the SDK to
#    ~/sdk/steamrt3c-arm64-sdk (the default below). Another sysroot needs
#    PS_AARCH64_SYSROOT; WSL's glibc needs RG_ALLOW_HOST_CROSS_SYSROOT=1 and is
#    for compile checks only, never for a build that goes to the headset.
#
# 2. C++ library = the newest GCC cross libstdc++ of the build host (Ubuntu
#    libstdc++-N-dev-arm64-cross, GCC 16 on Ubuntu 26.04), linked STATICALLY.
#    The sniper SDK ships GCC 10's libstdc++, too old for this code base (C++20
#    atomic<shared_ptr>, atomic wait/notify, floating from_chars, bit_cast).
#    Static linking means the library does not depend on the runtime's
#    libstdc++.so.6 at all. The few glibc >= 2.32 symbols that static archive
#    pulls in are provided by compat/glibc231_compat.cpp (RG_GLIBC_COMPAT_SOURCE,
#    added to rg_godot by godot_ext/CMakeLists.txt); tools/frame/check_symbols.sh
#    verifies the result needs nothing newer than glibc 2.31.
#    Override the GCC install directory with RG_AARCH64_GCC_INSTALL_DIR.
if(NOT DEFINED ENV{PS_AARCH64_SYSROOT} OR "$ENV{PS_AARCH64_SYSROOT}" STREQUAL "")
  set(_rg_default_sysroot "$ENV{HOME}/sdk/steamrt3c-arm64-sdk")
  if(EXISTS "${_rg_default_sysroot}/usr/include/stdio.h")
    set(ENV{PS_AARCH64_SYSROOT} "${_rg_default_sysroot}")
  elseif(NOT "$ENV{RG_ALLOW_HOST_CROSS_SYSROOT}" STREQUAL "1")
    message(FATAL_ERROR
      "No Steam Runtime 3c (sniper) ARM64 SDK sysroot: set PS_AARCH64_SYSROOT or run "
      "tools/setup_dev_env.ps1 -Frame (installs ${_rg_default_sysroot}). "
      "RG_ALLOW_HOST_CROSS_SYSROOT=1 allows WSL's newer glibc for a compile check only.")
  endif()
endif()

# physics_sim's file sets the system name/processor, the *_COMPILER_TARGET
# triple, lld, CMAKE_SYSROOT, the find-root modes and the qemu emulator.
include("${CMAKE_CURRENT_LIST_DIR}/../../external/physics_sim/cmake/toolchains/aarch64-linux-gnu-clang.cmake")

# The GCC installation clang takes libstdc++ (headers + libstdc++.a), libgcc and
# crtbegin from.
set(_rg_gcc_dir "$ENV{RG_AARCH64_GCC_INSTALL_DIR}")
if("${_rg_gcc_dir}" STREQUAL "")
  file(GLOB _rg_gcc_cands "/usr/lib/gcc-cross/aarch64-linux-gnu/*")
  list(SORT _rg_gcc_cands COMPARE NATURAL)
  list(REVERSE _rg_gcc_cands)
  foreach(_c IN LISTS _rg_gcc_cands)
    if(EXISTS "${_c}/libstdc++.a" AND EXISTS "${_c}/crtbeginS.o")
      set(_rg_gcc_dir "${_c}")
      break()
    endif()
  endforeach()
endif()
if("${_rg_gcc_dir}" STREQUAL "" OR NOT EXISTS "${_rg_gcc_dir}/libstdc++.a")
  message(FATAL_ERROR
    "No aarch64 cross libstdc++ (libstdc++.a) found under /usr/lib/gcc-cross/aarch64-linux-gnu. "
    "Run tools/setup_dev_env.ps1 -Frame (installs libstdc++-N-dev-arm64-cross in WSL) "
    "or set RG_AARCH64_GCC_INSTALL_DIR.")
endif()
# C++ headers: the GCC cross package's libstdc++ headers, named explicitly. NOT
# via --gcc-install-dir at compile time, because clang then also puts the
# package's /usr/aarch64-linux-gnu/include (glibc 2.43 headers) AHEAD of the
# sysroot's glibc 2.31 headers. With -nostdinc++ the C headers all come from the
# sysroot, the C++ ones from here (they sit first, so include_next works).
get_filename_component(_rg_gcc_ver "${_rg_gcc_dir}" NAME)
get_filename_component(_rg_cxx_inc "${_rg_gcc_dir}/../../../../aarch64-linux-gnu/include/c++/${_rg_gcc_ver}" REALPATH)
if(NOT EXISTS "${_rg_cxx_inc}/bits/c++config.h" AND NOT EXISTS "${_rg_cxx_inc}/aarch64-linux-gnu/bits/c++config.h")
  message(FATAL_ERROR "libstdc++ headers for ${_rg_gcc_dir} not found at ${_rg_cxx_inc} (package libstdc++-${_rg_gcc_ver}-dev-arm64-cross)")
endif()
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} -nostdinc++ -isystem ${_rg_cxx_inc} -isystem ${_rg_cxx_inc}/aarch64-linux-gnu -isystem ${_rg_cxx_inc}/backward")
# Link: libgcc, crt files and libstdc++.a of the same GCC.
set(_rg_link_flags "--gcc-install-dir=${_rg_gcc_dir} -static-libstdc++ -static-libgcc")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CMAKE_SHARED_LINKER_FLAGS_INIT} ${_rg_link_flags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${CMAKE_MODULE_LINKER_FLAGS_INIT} ${_rg_link_flags}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT} ${_rg_link_flags}")
# The compiler checks link a plain executable and must not pick the host's
# libstdc++.so (newer glibc symbol versions than the sysroot has). CMake did not
# carry the _INIT value into that link here, so set the plain variable as well.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${_rg_link_flags}")

set(RG_GLIBC_COMPAT_SOURCE "${CMAKE_CURRENT_LIST_DIR}/compat/glibc231_compat.cpp")
