# rg_route_gen: generates data/routes/home_g3_junctions.json (the G3/R3 acceptance route,
# docs/g3_consumer_design.md S3) from the real road graph; see main.cpp.
add_executable(rg_route_gen main.cpp)
target_link_libraries(rg_route_gen PRIVATE rg_core nlohmann_json::nlohmann_json rg_warnings rg_options)
target_compile_definitions(rg_route_gen PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
