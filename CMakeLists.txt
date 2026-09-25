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

if(RG_BUILD_GODOT_EXTENSION)
  add_subdirectory(godot_ext)
endif()

if(RG_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/unit)
endif()
