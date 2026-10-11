#include "rg/crt_guard.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <crtdbg.h>
#include <dbghelp.h>
#include <cstdio>
#include <cstdlib>

#pragma comment(lib, "dbghelp.lib")

namespace rg {
namespace {

void print_stack() {
    HANDLE process = GetCurrentProcess();
    static bool symbols = SymInitialize(process, nullptr, TRUE) != FALSE;
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
    void* frames[48];
    const USHORT n = CaptureStackBackTrace(1, 48, frames, nullptr);
    std::fprintf(stderr, "RG_CRT_STACK begin (%u frames)\n", static_cast<unsigned>(n));
    for (USHORT i = 0; i < n; ++i) {
        const auto address = reinterpret_cast<DWORD64>(frames[i]);
        char buffer[sizeof(SYMBOL_INFO) + 256] = {};
        auto* symbol = reinterpret_cast<SYMBOL_INFO*>(buffer);
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen = 255;
        DWORD64 displacement = 0;
        if (symbols && SymFromAddr(process, address, &displacement, symbol)) {
            IMAGEHLP_LINE64 line = {};
            line.SizeOfStruct = sizeof(line);
            DWORD line_displacement = 0;
            if (SymGetLineFromAddr64(process, address, &line_displacement, &line)) {
                std::fprintf(stderr, "  #%u %s (%s:%lu)\n", static_cast<unsigned>(i), symbol->Name, line.FileName, line.LineNumber);
            } else {
                std::fprintf(stderr, "  #%u %s\n", static_cast<unsigned>(i), symbol->Name);
            }
        } else {
            std::fprintf(stderr, "  #%u 0x%llx\n", static_cast<unsigned>(i), static_cast<unsigned long long>(address));
        }
    }
    std::fprintf(stderr, "RG_CRT_STACK end\n");
    std::fflush(stderr);
}

int crt_report_hook(int report_type, char*, int*) {
    if (report_type == _CRT_ASSERT || report_type == _CRT_ERROR) print_stack();
    return FALSE; // continue with the normal (stderr) report
}

} // namespace

void install_crt_guard(bool stack_trace) {
    static bool hook_installed = false;
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_WARN, _CRTDBG_FILE_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    if (stack_trace && !hook_installed) {
        hook_installed = true;
        _CrtSetReportHook(crt_report_hook);
    }
}

} // namespace rg
#else
namespace rg {
void install_crt_guard(bool) {}
} // namespace rg
#endif
