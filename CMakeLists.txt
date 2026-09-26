cmake_minimum_required(VERSION 3.25)

# CMP0091 + CMAKE_MSVC_RUNTIME_LIBRARY must be set before project() - same
# reasoning as physics_sim's own top-level CMakeLists.txt (its comment block
# explains the ABI-detection try-compile ordering issue in detail; not
# repeated here since this file follows the same rule for the same reason,
# and every target here is eventually linked against physics_sim's own
# ps_core, which requires a matching CRT).
if(POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif()
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# clang-cl + AddressSanitizer runtime, linked explicitly (Windows "asan"
# preset only) - same issue and same fix as physics_sim's own top-level
# CMakeLists.txt (see its comment for the full story): CMake's Ninja
# generator for clang-cl invokes lld-link.exe/link.exe DIRECTLY as the link
# step, which does not understand "-fsanitize=address" and never links the
# ASan runtime in, so it must be named explicitly via /WHOLEARCHIVE. This is
# racing_game's OWN copy of that block, not a reuse of physics_sim's: CMake
# directory-scoped variables (a plain set() with no PARENT_SCOPE/CACHE, which
# is what physics_sim's own top-level CMakeLists.txt uses for
# CMAKE_EXE_LINKER_FLAGS) do not propagate out of the directory they were set
# in - physics_sim's block only ever affects targets built inside
# external/physics_sim's own add_subdirectory() scope, never racing_game's
# sibling directories (core/, godot_ext/, tests/unit/, tools/hash_check/).
# Unconditional (no top-level-only guard, unlike geo2map_engine's copy of
# this same block) because racing_game itself IS always the top-level
# CMakeLists.txt for its own build - CMAKE_SOURCE_DIR ==
# CMAKE_CURRENT_SOURCE_DIR is always true here.
if(CMAKE_CXX_COMPILER MATCHES "clang-cl" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-resource-dir
    OUTPUT_VARIABLE RG_CLANG_RESOURCE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(RG_CLANG_ASAN_LIB_DIR "${RG_CLANG_RESOURCE_DIR}/lib/windows")
  set(RG_ASAN_LINK_FLAGS " /LIBPATH:\"${RG_CLANG_ASAN_LIB_DIR}\" /WHOLEARCHIVE:clang_rt.asan-x86_64.lib \"${RG_CLANG_ASAN_LIB_DIR}/clang_rt.asan-x86_64.lib\" /WHOLEARCHIVE:clang_rt.asan_cxx-x86_64.lib \"${RG_CLANG_ASAN_LIB_DIR}/clang_rt.asan_cxx-x86_64.lib\" /INFERASANLIBS:NO")
  # Applied to EXE, SHARED and MODULE link steps alike - CMAKE_EXE_LINKER_FLAGS
  # alone (the original form of this block) covers ps_run/sweep/the test
  # executables but not godot_ext/'s rg_godot (add_library(rg_godot SHARED)):
  # that DLL link step went unexercised by every asan-preset build until R2.0
  # actually built it, at which point it failed with undefined __asan_*
  # symbols (lld-link never implicitly links the ASan runtime for clang-cl,
  # same root cause as the comment above, just a different linker-flags
  # variable for a non-EXE target type).
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}${RG_ASAN_LINK_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}${RG_ASAN_LINK_FLAGS}")
  set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS}${RG_ASAN_LINK_FLAGS}")
endif()

project(racing_game LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(RG_BUILD_TESTS "Build the Catch2 unit test suite for rg_core" ON)
option(RG_BUILD_GODOT_EXTENSION "Build the rg_godot GDExtension" ON)

# Same /arch:AVX2 (or -mavx2 -mfma) requirement as physics_sim, for the same
# reason (Jolt's DVec3Arg ABI, vault PHYS-003) - every target here that
# touches ps_core (directly or indirectly) must agree with how physics_sim
# itself was compiled. Applied project-wide (not just our own targets) so
# any third-party target we fetch here (godot-cpp, Catch2) inherits it too,
# matching physics_sim's own top-level CMakeLists.txt approach.
if(MSVC)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /clang:-ffp-contract=off /arch:AVX2")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
  endif()
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off -mavx2 -mfma")
endif()

include(FetchContent)

# ---------------------------------------------------------------------------
# physics_sim (submodule, pinned - PLAN.md 10.2). EXCLUDE_FROM_ALL: its own
# tools/ps_run, tools/sweep and tests/* targets exist but are not part of the
# default build here - built on request (e.g. `cmake --build --target
# ps_run`) for the hash-comparison check (tools/ci.ps1, tools/run.ps1).
# PS_BUILD_TESTS=OFF/PS_BUILD_GODOT_ADAPTER=OFF per PLAN.md 10.2: this repo
# never builds physics_sim's own Catch2 suite or its adapters/godot (a
# separate GDExtension living inside the read-only submodule - we build our
# own rg_godot instead, see godot_ext/).
# ---------------------------------------------------------------------------
set(PS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PS_BUILD_GODOT_ADAPTER OFF CACHE BOOL "" FORCE)
add_subdirectory(external/physics_sim EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------
# geo2map_engine (submodule, pinned - PLAN.md R2.0). EXCLUDE_FROM_ALL, same
# reasoning as physics_sim above: rg_core only needs g2m_server/g2m_client/
# g2m_layers, never geo2map_engine's own tests/apps/decoders.
# G2M_BUILD_TESTS/G2M_BUILD_APPS/G2M_BUILD_IMPORT/G2M_BUILD_FUZZERS=OFF: this
# repo never builds geo2map_engine's own Catch2 suite, its g2m_tiler CLI, its
# OSM/GeoTIFF decoder stack (libdeflate/zlib-ng/protozero/libosmium - none of
# that is needed by a pure consumer reading an already-built store) or its
# libFuzzer targets. This exact combination matches geo2map_engine's own
# CMakePresets.json "consumer-check" preset, so neither of its
# G2M_BUILD_APPS-requires-G2M_BUILD_IMPORT / G2M_BUILD_TESTS-requires-
# G2M_BUILD_IMPORT FATAL_ERROR guards fires.
# ---------------------------------------------------------------------------
set(G2M_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(G2M_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(G2M_BUILD_IMPORT OFF CACHE BOOL "" FORCE)
set(G2M_BUILD_FUZZERS OFF CACHE BOOL "" FORCE)

# geo2map_engine's own cmake/deps_core.cmake calls enable_language(C)
# UNCONDITIONALLY (zstd/SQLite are C) - unlike its cmake/deps_decoders.cmake
# (only included when G2M_BUILD_IMPORT=ON, which we force OFF above), it has
# no guard of its own forcing CMAKE_C_COMPILER to match CMAKE_CXX_COMPILER.
# Standalone, geo2map_engine's own CMakePresets.json sets CMAKE_C_COMPILER
# explicitly (its clang-cl-base preset), so this never bites there; consumed
# via add_subdirectory from a parent whose own presets set CMAKE_CXX_COMPILER
# but never CMAKE_C_COMPILER (racing_game's clang-cl presets - CXX-only
# LANGUAGES), CMake's C-language detection instead falls back to whatever
# bare "clang"/"cc" the VS dev shell's PATH exposes (the GNU-style driver
# bundled alongside clang-cl, not clang-cl itself) - that C compiler then
# fails CMake's own ABI-detection check outright ("Check for working C
# compiler ... - broken"), discovered by actually running the release
# preset's configure step, not by inspection. Fix mirrors deps_decoders.cmake's
# own guard exactly (see that file's comment for the full reasoning), applied
# here instead since we cannot edit the read-only geo2map_engine submodule.
if(CMAKE_CXX_COMPILER MATCHES "clang-cl" AND NOT CMAKE_C_COMPILER)
  set(CMAKE_C_COMPILER "${CMAKE_CXX_COMPILER}" CACHE FILEPATH "C compiler (forced to match clang-cl for geo2map_engine's deps_core.cmake - see CMakeLists.txt's own comment)" FORCE)
endif()

# With every G2M_BUILD_* option above OFF, geo2map_engine's own
# cmake/deps_core.cmake still FetchContent_Declare()s g2m_zstd/g2m_sqlite/
# g2m_json (needed by g2m_layers/g2m_server/g2m_client themselves, not just
# the importer) - the owner has not approved a network fetch for this
# increment. RG_G2M_DEPS_DIR points FETCHCONTENT_SOURCE_DIR_<NAME> at an
# already-populated geo2map_engine build's own _deps directory instead (its
# consumer-check/release preset on Windows, its linux-release preset under
# WSL - both already exist on this machine from geo2map_engine's own CI), so
# FetchContent finds the source already there and never reaches the network.
# Override with -DRG_G2M_DEPS_DIR=... for a machine where that path differs.
if(WIN32)
  set(RG_G2M_DEPS_DIR_DEFAULT "S:/claude_code/geo2map_engine/out/build/release/_deps")
else()
  set(RG_G2M_DEPS_DIR_DEFAULT "$ENV{HOME}/build/geo2map_engine/linux-release/_deps")
endif()
set(RG_G2M_DEPS_DIR "${RG_G2M_DEPS_DIR_DEFAULT}" CACHE PATH
    "Directory holding an already-populated geo2map_engine build's FetchContent _deps (avoids a network fetch for g2m_zstd/g2m_sqlite/g2m_json etc.)")

if(NOT EXISTS "${RG_G2M_DEPS_DIR}")
  message(WARNING "RG_G2M_DEPS_DIR (${RG_G2M_DEPS_DIR}) does not exist - "
                  "geo2map_engine's FetchContent deps (g2m_zstd/g2m_sqlite/g2m_json) "
                  "will try to reach the network. Build geo2map_engine's own "
                  "release/consumer-check preset first, or pass -DRG_G2M_DEPS_DIR=...")
else()
  # Defensive: also cover the decoder-only deps (libdeflate/zlib-ng/protozero/
  # libosmium) and catch2, in case a future increment turns G2M_BUILD_IMPORT
  # or G2M_BUILD_TESTS back on without revisiting this list. Harmless no-ops
  # today (none of these FetchContent_Declare calls execute while both stay
  # OFF).
  foreach(_rg_g2m_dep g2m_zstd g2m_sqlite g2m_json catch2 libdeflate libosmium protozero zlibng)
    set(_rg_g2m_dep_src "${RG_G2M_DEPS_DIR}/${_rg_g2m_dep}-src")
    if(EXISTS "${_rg_g2m_dep_src}")
      string(TOUPPER "${_rg_g2m_dep}" _rg_g2m_dep_upper)
      set(FETCHCONTENT_SOURCE_DIR_${_rg_g2m_dep_upper} "${_rg_g2m_dep_src}" CACHE PATH "" FORCE)
    endif()
  endforeach()
endif()

add_subdirectory(external/geo2map_engine EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------
# godot-cpp (branch 4.5, same pin as physics_sim's own adapter - PLAN.md
# 10.2: "the game fetches godot-cpp itself, same pin as physics_sim").
# ---------------------------------------------------------------------------
if(RG_BUILD_GODOT_EXTENSION)
  FetchContent_Declare(
    godot-cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG        4.5
  )
  FetchContent_MakeAvailable(godot-cpp)
endif()

# ---------------------------------------------------------------------------
# Catch2 v3.16.0 (rg_core unit tests only - same pin as physics_sim).
# ---------------------------------------------------------------------------
if(RG_BUILD_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.16.0
  )
  FetchContent_MakeAvailable(Catch2)
  if(MSVC)
    # Same /EHsc requirement as physics_sim's own top-level CMakeLists.txt
    # (vault TOOL-019) - Catch2's exception translator needs real C++ EH.
    target_compile_options(Catch2 PRIVATE /EHsc)
    target_compile_options(Catch2WithMain PRIVATE /EHsc)
  endif()
endif()

# ---------------------------------------------------------------------------
# rg_warnings / rg_options: same INTERFACE-target pattern as physics_sim's
# ps_warnings/ps_options (warnings-as-errors and arch/FP flags for OUR OWN
# targets only - never applied to Jolt, godot-cpp, Catch2 or physics_sim's
# own targets, which are physics_sim's business, not ours).
# ---------------------------------------------------------------------------
add_library(rg_warnings INTERFACE)
if(MSVC)
  target_compile_options(rg_warnings INTERFACE /W4 /WX)
else()
  target_compile_options(rg_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror)
endif()

add_library(rg_options INTERFACE)
if(MSVC)
  target_compile_options(rg_options INTERFACE /arch:AVX2 /fp:precise /EHsc)
else()
  target_compile_options(rg_options INTERFACE -mavx2 -mfma)
endif()

add_subdirectory(core)
add_subdirectory(tools/hash_check)
add_subdirectory(tools/lod_measure)
add_subdirectory(tools/route_check)

if(RG_BUILD_GODOT_EXTENSION)
  add_subdirectory(godot_ext)
endif()

if(RG_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/unit)
endif()
