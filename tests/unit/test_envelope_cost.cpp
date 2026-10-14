// test_envelope_cost.cpp - hidden [.][perf][envelope] measurement of physics' full-load envelope build (pedal plan P1:
// ~30 held-crank dyno points, memoised per process, built when a turbo SimulatedEngine is constructed). The first flat
// Session of a car pays it, the second (same process, same key) hits the memo; the difference is the envelope cost.
// RG_DIAG_QUALITY (sketch|preview|full, default = the Session default) selects the engine-map quality.
#include "rg/session.h"
#include "rg/vehicle_catalog.h"


#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>

namespace {
const std::string kRepo = RG_SOURCE_DIR;
double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
} // namespace

TEST_CASE("full-load envelope cost: first Session of a turbo car vs second", "[.][perf][envelope]") {
    for (const char* car : {"data/vehicles/car_hyper.json", "data/vehicles/car_hyper_n2o.json"}) {
        rg::SessionConfig config;
        config.vehicle_json_path = kRepo + "/" + car;
        config.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
        config.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
        const double t0 = now_s();
        { rg::Session first(config); }
        const double t1 = now_s();
        { rg::Session second(config); }
        const double t2 = now_s();
        std::printf("ENVELOPE %s: first Session %.3f s, second (memo hit) %.3f s, envelope ~%.3f s\n", car, t1 - t0, t2 - t1, (t1 - t0) - (t2 - t1));
        std::fflush(stdout);
        CHECK(t1 > t0);
    }
}
