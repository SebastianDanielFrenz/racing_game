# rg_unit_tests: Catch2 v3 (this repo's own fetch - see top-level
# CMakeLists.txt for why physics_sim's own Catch2/ps_test_catch_main are not
# reused: PS_BUILD_TESTS=OFF on the submodule means neither target exists).

add_library(rg_test_catch_main STATIC catch_main.cpp)
target_link_libraries(rg_test_catch_main
  PUBLIC
    Catch2::Catch2
    ps_headless_env
)

add_executable(rg_unit_tests
  test_session.cpp
  test_world_config.cpp
  test_g2m_link_smoke.cpp
  test_g2m_ps_bridge_link_smoke.cpp
  test_world_terrain.cpp
  test_road_classes.cpp
  test_route_road_coverage.cpp
  test_route_road_render_path.cpp
  test_fixed_rate_loop.cpp
  test_terrain_view_streamer.cpp
  test_route_check.cpp
  test_terrain_mode.cpp
  test_session_terrain.cpp
  test_session_reinit.cpp
  test_player_mode.cpp
  test_vehicle_data.cpp
  test_route_grip.cpp
)

target_link_libraries(rg_unit_tests
  PRIVATE
    rg_core
    rg_test_catch_main
    rg_warnings
    rg_options
    # test_world_config.cpp builds its own fixture JSON directly with
    # nlohmann::json (same precedent as physics_sim's own
    # test_terrain_golden.cpp linking nlohmann_json into ps_unit_tests).
    nlohmann_json::nlohmann_json
)

target_compile_definitions(rg_unit_tests PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(rg_unit_tests)
