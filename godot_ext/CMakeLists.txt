# rg_godot — the one GDExtension DLL (PLAN.md 10.1/12 R0 scope: "One
# GDExtension DLL rg_godot containing ps_core + rg_core + a thin Godot
# layer"). Mirrors physics_sim's own adapters/godot/CMakeLists.txt in shape
# (same godot-cpp linkage/output-dir pattern, same "not linked against
# rg_warnings" reasoning - see below) but much smaller: two source files,
# no debug draw, no collision import, no haptics, no plain-C++ test
# executables (rg_core's own numeric logic is covered by tests/unit's
# Catch2 suite instead, since none of it needs to be Godot-free-but-outside-
# rg_core the way physics_sim's input_map_core/haptics_core do).

set(RG_GODOT_BIN_DIR "${CMAKE_SOURCE_DIR}/game/bin")

add_library(rg_godot SHARED
  src/register_types.cpp
  src/rg_simulation.cpp
  src/rg_simulation_shell.cpp
  src/rg_shell.cpp
  src/rg_garage.cpp
  src/rg_garage_browser.cpp
  src/rg_camera_math.cpp
  src/rg_terrain_view.cpp
  src/rg_engine_audio.cpp
  ${CMAKE_SOURCE_DIR}/external/physics_sim/adapters/godot/src/vehicle_voice.cpp
  ${CMAKE_SOURCE_DIR}/external/physics_sim/adapters/godot/src/spatial_audio.cpp
)

target_include_directories(rg_godot PRIVATE src
  ${CMAKE_SOURCE_DIR}/external/physics_sim/adapters/godot/src
  ${CMAKE_SOURCE_DIR}/external/physics_sim/tools/common)
if(WIN32)
  target_link_libraries(rg_godot PRIVATE ole32)
endif()

# RG_BUILD_CONFIG carries the CMake config name into RgSimulation::get_build_info()
# (rg_simulation.cpp) - so a debug-vs-optimised DLL mismatch (the whole point
# of tools/run.ps1's stale-DLL guard above it) shows up in the game's own log
# line instead of only being discoverable by inspecting the build dir.
# $<CONFIG> works for single-config generators too (Ninja here): it resolves
# to CMAKE_BUILD_TYPE at generate time.
execute_process(COMMAND git -C "${CMAKE_SOURCE_DIR}" rev-parse --short=10 HEAD OUTPUT_VARIABLE RG_SOURCE_REV OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git -C "${CMAKE_SOURCE_DIR}" rev-parse --git-path HEAD OUTPUT_VARIABLE RG_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git -C "${CMAKE_SOURCE_DIR}" symbolic-ref -q HEAD OUTPUT_VARIABLE RG_GIT_REF OUTPUT_STRIP_TRAILING_WHITESPACE)
if(RG_GIT_REF)
  execute_process(COMMAND git -C "${CMAKE_SOURCE_DIR}" rev-parse --git-path "${RG_GIT_REF}" OUTPUT_VARIABLE RG_GIT_REF_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/${RG_GIT_REF_PATH}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/${RG_GIT_HEAD}")
target_compile_definitions(rg_godot PRIVATE RG_BUILD_CONFIG="$<CONFIG>" RG_SOURCE_REV="${RG_SOURCE_REV}")

# NOT linked against rg_warnings - same reasoning as physics_sim's own
# adapters/godot/CMakeLists.txt (read and confirmed by building): godot-cpp
# branch 4.5's headers are unused-parameter-laden, and /W4 /WX from an
# INTERFACE library re-enables warnings clang already resolved by then
# (command-line order), so godot-cpp headers merely INCLUDED by our own
# .cpp files would fail our own warnings-as-errors even though the
# offending code is godot-cpp's, not ours.
target_link_libraries(rg_godot
  PRIVATE
    rg_core
    ps_audio
    godot-cpp
    rg_options
)

set_target_properties(rg_godot PROPERTIES
  OUTPUT_NAME "librg_godot"
  RUNTIME_OUTPUT_DIRECTORY "${RG_GODOT_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${RG_GODOT_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${RG_GODOT_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${RG_GODOT_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY "${RG_GODOT_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_DEBUG "${RG_GODOT_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${RG_GODOT_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${RG_GODOT_BIN_DIR}"
)
