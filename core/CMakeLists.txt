# rg_core (STATIC): engine-neutral game logic (rg::Session and friends).
# Public headers in include/rg; no Godot type anywhere in this target
# (PLAN.md 11.1, user memory "Engine-neutral logic"). Links ps_core
# PUBLICLY (rg::Session's own public header, session.h, includes
# ps/world/world.h directly - unlike physics_sim's own ps_core, which keeps
# every Jolt/Godot type out of its public headers, rg_core's contract only
# promises "no GODOT type", so exposing ps:: types in rg::Session's API is
# in scope and simplifies godot_ext considerably).
#
# Headers (plus one .cpp) are reused BY PATH from physics_sim's own
# adapters/godot/src, rather than copied, per this task's own instruction to
# state which and why (see session.h's top comment for the per-file
# reasoning): triple_buffer.h, origin_rebase.h, fall_detector.h (R2.2 R4:
# terrain mode's fall diagnostic), sim_thread.h (precise_sleep_until, used by
# fixed_rate_loop.cpp - rg::Session itself now runs on rg::FixedRateLoop),
# frame_convert_core.h/.cpp. All are Godot-free (verified by
# reading them - none includes any godot:: type), so reusing them by path
# creates no dependency on godot-cpp and no risk of drifting from the
# read-only submodule's own copy silently going stale (a future physics_sim
# fix to e.g. SimThread's catch-up logic is picked up here automatically on
# the next submodule pin bump, not duplicated-and-forgotten).

# g2m_phys/g2m_ps_bridge (geo2map_engine submodule, R2.2 R1): g2m_phys is
# ps-free (PhysicsTileGrid/ResidentHeightSet/fill_physics_heights/
# HeightTileLoader/PhysicsTerrainStreamer); g2m_ps_bridge (bridges/
# physics_sim/, built only because physics_sim's own add_subdirectory above
# runs before geo2map_engine's, so ps_core already exists as a target when
# geo2map_engine's `if(TARGET ps_core)` guard is evaluated) adds
# g2m::ps_bridge::G2mTerrainSource : ps::terrain::ITerrainSource. Linked
# PUBLIC, not PRIVATE: R2.2 R4 puts a G2mTerrainSource inside rg::Session,
# so it will be named in session.h (a PUBLIC header of this target) the same
# way ps::World already is - a PRIVATE link would compile rg_core itself but
# fail every downstream consumer of session.h (godot_ext, tests/unit,
# tools/hash_check et al.) with missing types/unresolved symbols the moment
# R4 lands. Matches the existing ps_core/g2m_server/g2m_client/g2m_layers/
# g2m_mesh PUBLIC precedent above for the same reason.
#
# g2m_server/g2m_client/g2m_layers/g2m_mesh (geo2map_engine submodule):
# rg::load_world_config (world_config.h/.cpp) validates data/world/
# world_config.json and expands its ${RG_G2M_HOME}/${RG_G2M_DERIVED}
# placeholders; rg::WorldTerrain (world_terrain.h/.cpp, PLAN.md R2.1) is the
# real consumer - it opens the source/derived TileStores, builds one offline
# g2m::Server + local release (g2m::builtin_local_release_params +
# g2m::make_local_release, the same wiring g2m_tiler's own `bake` subcommand
# and geo2map_engine's golden tests use), and serves g2m::mesh::
# TerrainChunkMesh chunks via g2m::mesh::select_chunks/gather_window/
# build_chunk. g2m_mesh landed on the submodule pin with G2.3 (bumped
# 2026-09-26, commit da07f4c) - it links only g2m_core + g2m_layers (always
# built, unconditional on every G2M_BUILD_* option this project forces OFF),
# so it is safe to link here even with G2M_BUILD_IMPORT/TESTS/APPS/FUZZERS
# all OFF.
set(PS_ADAPTER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/physics_sim/adapters/godot/src")

add_library(rg_core STATIC
    src/session.cpp
    src/player_mode.cpp
    src/terrain_mode.cpp
    src/world_config.cpp
    src/world_terrain.cpp
    src/road_classes.cpp
    src/buildings.cpp
    src/road_surface.cpp
    src/terrain_smoothing.cpp
    src/road_structures.cpp
    src/fixed_rate_loop.cpp
    src/terrain_view_streamer.cpp
    src/route_check.cpp
    "${PS_ADAPTER_SRC_DIR}/frame_convert_core.cpp"
)

target_include_directories(rg_core
    PUBLIC
        include
        "${PS_ADAPTER_SRC_DIR}"
)

# rg_warnings (/W4 /WX) deliberately NOT linked here, mirroring physics_sim's
# own adapters/godot/CMakeLists.txt precedent: frame_convert_core.cpp is
# reused, read-only, foreign-owned code (see the file-list comment above) -
# warnings-as-errors belongs to code we actually maintain, not a byte-for-
# byte copy of another target's build. session.cpp (ours) still gets full
# /W4 in the msvc-release/debug/release presets via rg_options' /fp:precise
# etc.; if we later want strict warnings on session.cpp specifically without
# reopening this decision, add a second OBJECT library or per-source
# COMPILE_OPTIONS for that one file.
target_link_libraries(rg_core
    PUBLIC
        ps_core
        g2m_server
        g2m_client
        g2m_layers
        g2m_mesh
        g2m_phys
        g2m_ps_bridge
    PRIVATE
        rg_options
        nlohmann_json::nlohmann_json
)
