# rg_core (STATIC): engine-neutral game logic (rg::Session and friends).
# Public headers in include/rg; no Godot type anywhere in this target
# (PLAN.md 11.1, user memory "Engine-neutral logic"). Links ps_core
# PUBLICLY (rg::Session's own public header, session.h, includes
# ps/world/world.h directly - unlike physics_sim's own ps_core, which keeps
# every Jolt/Godot type out of its public headers, rg_core's contract only
# promises "no GODOT type", so exposing ps:: types in rg::Session's API is
# in scope and simplifies godot_ext considerably).
#
# Four headers (plus one .cpp) are reused BY PATH from physics_sim's own
# adapters/godot/src, rather than copied, per this task's own instruction to
# state which and why (see session.h's top comment for the per-file
# reasoning): sim_thread.h, triple_buffer.h, origin_rebase.h,
# frame_convert_core.h/.cpp. All four are already Godot-free (verified by
# reading them - none includes any godot:: type), so reusing them by path
# creates no dependency on godot-cpp and no risk of drifting from the
# read-only submodule's own copy silently going stale (a future physics_sim
# fix to e.g. SimThread's catch-up logic is picked up here automatically on
# the next submodule pin bump, not duplicated-and-forgotten).

# g2m_server/g2m_client/g2m_layers (geo2map_engine submodule, PLAN.md R2.0):
# rg::load_world_config (world_config.h/.cpp) is the only rg_core file that
# touches geo2map_engine directly today (validates data/world/world_config.json
# and expands its ${RG_G2M_HOME} placeholder; does not yet open a store).
# g2m_mesh does NOT exist yet on geo2map_engine's current pin (main,
# 216775fd45a64014fc2fd89200555b3f1426509e) - only g2m_server/g2m_client/
# g2m_layers are linked; add g2m_mesh here once geo2map_engine's G2.3
# increment lands it.
set(PS_ADAPTER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/physics_sim/adapters/godot/src")

add_library(rg_core STATIC
    src/session.cpp
    src/world_config.cpp
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
    PRIVATE
        rg_options
        nlohmann_json::nlohmann_json
)
