// Custom Catch2 entry point, same reasoning and pattern as physics_sim's
// own tests/support/catch_main.cpp (read for reference, not reused by path,
// since it links physics_sim's OWN Catch2 target - PS_BUILD_TESTS=OFF on
// the submodule per this repo's top-level CMakeLists.txt means physics_sim
// never builds a Catch2 target or its own ps_test_catch_main at all, so
// this repo needs its own copy of this ~10-line file rather than a by-path
// reuse): install the headless CRT handlers (ps_headless_env, from
// physics_sim's tools/common - that target IS always built, unconditional
// on PS_BUILD_TESTS, see tools/common/CMakeLists.txt) before running a
// single test, so a failed assert cannot pop a blocking modal dialog and
// hang a headless/CI run.

#include "headless_env.h"

#define CATCH_CONFIG_RUNNER
#include <catch2/catch_session.hpp>

int main(int argc, char* argv[]) {
    ps_tools::install_headless_crt_handlers();
    return Catch::Session().run(argc, argv);
}
