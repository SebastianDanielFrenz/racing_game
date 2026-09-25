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
)

target_link_libraries(rg_unit_tests
  PRIVATE
    rg_core
    rg_test_catch_main
    rg_warnings
    rg_options
)

target_compile_definitions(rg_unit_tests PRIVATE RG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(rg_unit_tests)
