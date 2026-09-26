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
  src/rg_terrain_view.cpp
)

target_include_directories(rg_godot PRIVATE src)

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
