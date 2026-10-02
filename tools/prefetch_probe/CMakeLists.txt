add_executable(prefetch_probe main.cpp)
target_link_libraries(prefetch_probe PRIVATE rg_core rg_warnings rg_options)
target_compile_definitions(prefetch_probe PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
