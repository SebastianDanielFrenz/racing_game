# route_check: checks a data/routes/*.json drive route against the real
# terrain store (see main.cpp's header comment). Thin main over rg_core's
# rg::check_route_on_world; links rg_core only (no Godot).

add_executable(route_check main.cpp)
target_link_libraries(route_check PRIVATE rg_core rg_warnings rg_options)
target_compile_definitions(route_check PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
