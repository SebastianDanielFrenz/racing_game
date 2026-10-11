// rg/crt_guard.h - keep a debug-CRT assertion from ever opening a modal dialog (Windows; a no-op elsewhere).
//
// install_crt_guard() routes the debug CRT's assert/error/warn reports and abort() to stderr, silences the OS crash
// dialogs, and prints a symbolised stack trace for every CRT assertion so a headless run says WHERE it asserted.
// Called by the test entry point and by the Godot extension's library init (debug builds, and any run that passes
// --headless), so a --garage-test or CI run cannot block on a dialog.
#pragma once

namespace rg {

// Idempotent. `stack_trace` = also print the call stack of a CRT assertion to stderr.
void install_crt_guard(bool stack_trace = true);

} // namespace rg
