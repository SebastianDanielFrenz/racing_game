// rg::install_crt_guard: a debug-CRT assertion goes to stderr, never into a modal dialog (a headless --garage-test run
// once blocked the owner's screen on "cannot seek array iterator after end"). Sabotage: skipping the _CrtSetReportMode
// calls leaves the dialog mode on and the first CHECK fails.
#include <catch2/catch_test_macros.hpp>

#include "rg/crt_guard.h"

#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

TEST_CASE("CRT guard routes assertions and errors to stderr, not a dialog", "[crt_guard]") {
#if defined(_WIN32) && defined(_DEBUG)
    // Put the CRT back to its default (dialog) mode first: the test entry point already installed the handlers.
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_WNDW);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_WNDW);
    rg::install_crt_guard();
    rg::install_crt_guard(); // repeatable
    CHECK(_CrtSetReportMode(_CRT_ASSERT, _CRTDBG_REPORT_MODE) == _CRTDBG_MODE_FILE);
    CHECK(_CrtSetReportMode(_CRT_ERROR, _CRTDBG_REPORT_MODE) == _CRTDBG_MODE_FILE);
#else
    rg::install_crt_guard();
    SUCCEED("the debug CRT dialog exists only in a Windows debug build");
#endif
}
