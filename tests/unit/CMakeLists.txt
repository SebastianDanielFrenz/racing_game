# rg_unit_tests: Catch2 v3 (this repo's own fetch - see top-level
# CMakeLists.txt for why physics_sim's own Catch2/ps_test_catch_main are not
# reused: PS_BUILD_TESTS=OFF on the submodule means neither target exists).

add_library(rg_test_catch_main STATIC catch_main.cpp)
target_link_libraries(rg_test_catch_main
  PUBLIC
    Catch2::Catch2
    ps_headless_env
    rg_core
)

add_executable(rg_unit_tests
  test_driver_actions.cpp
  test_electric_motor_sound.cpp
  test_environment.cpp
  test_crt_guard.cpp
  test_npc_truck.cpp
  test_traffic_stuck.cpp
  test_npc_traffic.cpp
  test_session.cpp
  test_world_config.cpp
  test_g2m_link_smoke.cpp
  test_g2m_ps_bridge_link_smoke.cpp
  test_world_terrain.cpp
  test_terrain_colours.cpp
  test_served_terrain_class.cpp
  test_road_surface.cpp
  test_underpass_decks.cpp
  test_road_structures.cpp
  test_road_overpass.cpp
  test_terrain_smoothing.cpp
  test_route_road_coverage.cpp
  test_route_road_render_path.cpp
  test_route_standalone_paths.cpp
  test_fixed_rate_loop.cpp
  test_terrain_view_streamer.cpp
  test_render_focus.cpp
  test_steering_assist.cpp
  test_address_search.cpp
  test_terrain_carve_state.cpp
  test_route_check.cpp
  test_route_follower.cpp
  test_road_network.cpp
  test_route_g3.cpp
  test_terrain_mode.cpp
  test_session_terrain.cpp
  test_session_reinit.cpp
  test_player_mode.cpp
  test_walker.cpp
  test_session_walker.cpp
  test_fpv_drone.cpp
  test_session_fpv.cpp
  test_vehicle_data.cpp
  test_route_grip.cpp
  test_tick_spikes.cpp
  test_credits.cpp
  test_settings.cpp
  test_lod_settings.cpp
  test_route_planner.cpp
  test_spawn_presets.cpp
  test_shell_flow.cpp
  test_time_weather.cpp
  test_exposure.cpp
  test_climate.cpp
  test_wet_look.cpp
  test_session_weather.cpp
  test_crops.cpp
  test_controls.cpp
  test_control_capture.cpp
  test_vehicle_catalog.cpp
  test_vehicle_setup.cpp
  test_slider_map.cpp
  test_rev_limit_extremes.cpp
  test_tyre_spec.cpp
  test_garage_dyno.cpp
  test_limit_advisor.cpp
  test_envelope_cost.cpp
  test_engine_garage.cpp
  test_turbo_kit.cpp
  test_engine_internals.cpp
  test_cooling_upgrades.cpp
  test_damage.cpp
  test_damage_engine.cpp
  test_engine_bore.cpp
  test_gearbox_swap.cpp
  test_automatic_gearbox_swap.cpp
  test_part_mass.cpp
  test_strip_visual.cpp
  test_drivetrain_swap.cpp
  test_garage.cpp
  test_car_browser.cpp
  test_car_browser_data.cpp
  test_road_ahead.cpp
  test_camera_math.cpp
  test_camera_motion.cpp
  test_tyre_sound.cpp
  test_cinematic_obstacles.cpp
  test_render_pose.cpp
  test_deck_installer.cpp
  test_deck_install_perf.cpp
  test_stutter_perf.cpp
  test_nitrous_spawn.cpp
  test_nitrous_gain.cpp
  test_tc_owner_repro.cpp
  test_engine_thermal_info.cpp
  test_game_car_thermal.cpp
  test_assist_info.cpp
  test_regen_info.cpp
  test_dashboard_info.cpp
  test_channel_mirror.cpp
  test_hyper_rear_travel.cpp
  test_hyper_power_jumps.cpp
  test_flat_ground.cpp
  test_surface_kinds.cpp
  test_turbo_map_extension.cpp
  test_abs_repro.cpp
  test_abs_fitment.cpp
  test_hybrid_electronics.cpp
  test_hybrid_selector.cpp
  test_hybrid_shift.cpp
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

# These diagnostics intentionally use standard C stdio/environment functions.
# MSVC labels them deprecated; keep /WX elsewhere and scope the CRT opt-out
# to the affected translation units (no change to product builds).
if(WIN32)
  set_property(SOURCE
    test_road_overpass.cpp test_exposure.cpp test_turbo_kit.cpp
    test_nitrous_gain.cpp test_tc_owner_repro.cpp test_turbo_map_extension.cpp test_abs_repro.cpp
    APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
endif()
target_compile_definitions(rg_unit_tests PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(rg_unit_tests)
