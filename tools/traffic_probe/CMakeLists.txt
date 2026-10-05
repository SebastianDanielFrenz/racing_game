# traffic_probe: headless NPC traffic harness on the real home world (see main.cpp's header comment).
# Counts stuck NPC episodes by cause with the Session's own detector. Links rg_core only (no Godot).

add_executable(traffic_probe main.cpp)
target_link_libraries(traffic_probe PRIVATE rg_core rg_warnings rg_options)
target_compile_definitions(traffic_probe PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
