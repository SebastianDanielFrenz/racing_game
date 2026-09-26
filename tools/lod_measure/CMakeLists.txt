# lod_measure: PLAN.md R2.1's LOD-distance measurement sweep (see main.cpp's
# own header comment) - chunk count/vertex count/build_static_view wall time
# for max_distance_m in {6000, 12000, 20000} against the real committed
# data/world/world_config.json home-r1 store. Links rg_core only, same
# "no wall clock beyond std::chrono, no Godot" shape as tools/hash_check.

add_executable(lod_measure main.cpp)
target_link_libraries(lod_measure PRIVATE rg_core rg_warnings rg_options)
target_compile_definitions(lod_measure PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
