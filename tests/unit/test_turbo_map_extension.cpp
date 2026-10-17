// Turbo stage map extension (rg/turbo_map_extension.h): the policy file, finding the map file, generation once + cache hit
// on the hypercar's N2O twin turbo at 1.8 bar, the covered demand, the choke message of the setup check and the worker.
#include "rg/turbo_map_extension.h"

#include "rg/limit_advisor.h"
#include "rg/vehicle_load.h"

#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/drivetrain/turbo_stage_maps.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <variant>

namespace {

namespace fs = std::filesystem;
namespace pd = ps::drivetrain;
const std::string kRepo = RG_SOURCE_DIR;

rg::TurboMapExtensionConfig shipped() {
    std::string err;
    auto c = rg::load_turbo_map_extension_config(std::getenv("TM_CFG") ? std::string(std::getenv("TM_CFG")) : kRepo + "/data/engines/turbo_map_extension.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

std::string scratch(const std::string& name) {
    const fs::path p = fs::path(kRepo) / "out" / "test_tmp" / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    return p.generic_string();
}

const std::string kHyperN2o = kRepo + "/data/vehicles/car_hyper_n2o.json";

// The hypercar with its sketch-quality engine maps (the same quality the garage advisor uses), loaded once per process.
const ps::vehicle::VehicleDesc& hyper_desc() {
    static const ps::vehicle::VehicleDesc desc = [] {
        ps::io::EngineMapOptions maps;
        maps.quality = std::getenv("TM_FULL") ? pd::MapQuality::Full : pd::MapQuality::Sketch;
        const fs::path cache = fs::path(kRepo) / "out" / "test_tmp" / "turbomap_engine_maps";
        std::error_code ec;
        fs::create_directories(cache, ec);
        maps.cache_dir = cache.generic_string();
        return rg::load_vehicle(kHyperN2o, maps);
    }();
    return desc;
}

const pd::SimulatedEngineDesc& engine_of(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* s = std::get_if<pd::SimulatedEngineDesc>(&c.params)) return *s;
    FAIL("no simulated engine");
    throw 0;
}

} // namespace

TEST_CASE("turbo map extension: the policy file loads strictly", "[turbo_map][turbo_map_config]") {
    const auto c = shipped();
    CHECK(c.enabled);
    CHECK(c.policy.speed_step_rpm == 500.0);
    CHECK(c.policy.max_speed_lines == 129);
    CHECK(c.cache_subdir == "turbo_stage_maps");

    const std::string dir = scratch("turbomap_cfg");
    const auto write = [&](const std::string& text) {
        std::ofstream(dir + "/c.json") << text;
        std::string err;
        const auto r = rg::load_turbo_map_extension_config(dir + "/c.json", &err);
        return std::make_pair(r.has_value(), err);
    };
    CHECK_FALSE(write("{}").first);
    CHECK_FALSE(write(R"({"format":"rg.turbo_map_extension/1"})").first);
    auto text = [&](const std::string& policy_patch) {
        return R"({"format":"rg.turbo_map_extension/1","policy":{"flow_margin":0.1,"speed_margin":0.05,"speed_step_rpm":)" + policy_patch +
               R"(,"flow_step_kg_s":0.01,"max_speed_lines":129,"max_flow_points":257},"cache":{"subdir":"m"},"choke":{"rpm_step":250,"bisect_iterations":40,"flow_scan_max_factor":1.5}})";
    };
    CHECK(write(text("500")).first);
    const auto bad = write(text("-5"));
    CHECK_FALSE(bad.first);
    CHECK(bad.second.find("speed_step_rpm") != std::string::npos);
}

TEST_CASE("turbo map extension: the map file is found through the vehicle, engine, configuration and hardware files", "[turbo_map]") {
    const std::string path = rg::find_turbo_stage_map_path(kHyperN2o);
    INFO(path);
    REQUIRE_FALSE(path.empty());
    CHECK(fs::is_regular_file(path));
    CHECK(path.find("turbo_maps") != std::string::npos);
    CHECK(rg::find_turbo_stage_map_path(kRepo + "/data/vehicles/car_sedan.json").empty());
}

TEST_CASE("turbo map extension: an out-of-range demand generates once, the second load hits the cache, the map covers the demand except the choke",
          "[turbo_map]") {
    const auto cfg = shipped();
    const std::string cache = scratch("turbomap_cache");
    int generations = 0;
    rg::TurboExtensionHooks hooks;
    hooks.before_generate = [&] { ++generations; };

    ps::vehicle::VehicleDesc first = hyper_desc();
    const pd::TurboStageDemand original = pd::turbo_stage_demand(engine_of(first));
    REQUIRE(original.applies);
    int uncovered_original = 0;
    for (const auto& s : original.steps) uncovered_original += s.covered ? 0 : 1;
    REQUIRE(uncovered_original > 0); // the shipped map does not cover the 1.8 bar demand (the setup check's finding)

    const auto r1 = rg::extend_turbo_maps(first, kHyperN2o, cache, cfg, nullptr, nullptr, &hooks);
    INFO(r1.message);
    CHECK(r1.status == rg::TurboExtensionResult::Status::Generated);
    CHECK(r1.choke.choke_flow_kg_s > 0.50);
    CHECK(r1.choke.choke_flow_kg_s < r1.choke.demand_flow_kg_s);
    CHECK(generations == 1);
    CHECK(r1.uncovered_steps_before == uncovered_original);
    CHECK(r1.uncovered_steps_after < r1.uncovered_steps_before);
    REQUIRE(fs::is_regular_file(r1.cache_file));
    CHECK(r1.cache_file.find(cfg.cache_subdir) != std::string::npos);
    // No temp file left behind by the atomic write.
    for (const auto& e : fs::directory_iterator(fs::path(r1.cache_file).parent_path())) CHECK(e.path().extension() == ".json");

    // What stays uncovered is at the compressor's choke flow, nothing else: a bilinear cell needs four valid corners, so the
    // certified boundary lags the choke edge by up to three flow knots.
    const pd::TurboStageDemand after = pd::turbo_stage_demand(engine_of(first));
    for (const auto& s : after.steps)
        std::printf("  step flow %.4f rotor %.0f reached %d covered %d\n", s.flow_kg_s, s.rotor_rpm, int(s.reached), int(s.covered));
    for (const auto& s : after.steps) {
        INFO("step flow " << s.flow_kg_s << " choke " << r1.choke.choke_flow_kg_s);
        if (!s.covered) CHECK(s.flow_kg_s > r1.choke.choke_flow_kg_s - 3.0 * cfg.policy.flow_step_kg_s);
    }
    std::printf("turbo map: uncovered %d -> %d, choke %.4f kg/s of %.4f kg/s demanded per turbo at PR %.4f, generated in %.2f s\n",
                r1.uncovered_steps_before, r1.uncovered_steps_after, r1.choke.choke_flow_kg_s, r1.choke.demand_flow_kg_s, r1.choke.pressure_ratio,
                r1.seconds);

    // A second load of the same car hits the cache: no generation, the same map, the file untouched.
    const auto stamp = fs::last_write_time(r1.cache_file);
    ps::vehicle::VehicleDesc second = hyper_desc();
    const auto r2 = rg::extend_turbo_maps(second, kHyperN2o, cache, cfg, nullptr, nullptr, &hooks);
    CHECK(r2.status == rg::TurboExtensionResult::Status::CacheHit);
    CHECK(generations == 1);
    CHECK(r2.cache_key == r1.cache_key);
    CHECK(r2.uncovered_steps_after == r1.uncovered_steps_after);
    CHECK(fs::last_write_time(r1.cache_file) == stamp);
    CHECK(r2.seconds < r1.seconds);
    const auto* m1 = engine_of(first).turbo_pair->maps.get();
    const auto* m2 = engine_of(second).turbo_pair->maps.get();
    CHECK(m1->content_hash == m2->content_hash);
    CHECK(m1->content_hash != engine_of(hyper_desc()).turbo_pair->maps->content_hash);

    // A damaged cache file is a miss: regenerated and replaced.
    std::ofstream(r1.cache_file, std::ios::trunc) << "{ not json";
    ps::vehicle::VehicleDesc third = hyper_desc();
    const auto r3 = rg::extend_turbo_maps(third, kHyperN2o, cache, cfg, nullptr, nullptr, &hooks);
    CHECK(r3.status == rg::TurboExtensionResult::Status::Generated);
    CHECK(generations == 2);

    // A second pass over an already extended desc is a cache hit with the same key (the demand is read against the map file).
    const auto r4 = rg::extend_turbo_maps(first, kHyperN2o, cache, cfg, nullptr, nullptr, &hooks);
    CHECK(r4.status == rg::TurboExtensionResult::Status::CacheHit);
    CHECK(r4.cache_key == r1.cache_key);
    CHECK(generations == 2);

    // A car without a stage-map turbo is left alone.
    ps::vehicle::VehicleDesc other = hyper_desc();
    CHECK(rg::extend_turbo_maps(other, kRepo + "/data/vehicles/car_sedan.json", cache, cfg).status ==
          rg::TurboExtensionResult::Status::NotApplicable);
}

TEST_CASE("turbo map extension: the worker generates on its own thread and the submitter does not wait", "[turbo_map]") {
    const auto cfg = shipped();
    std::mutex m;
    std::condition_variable cv;
    bool release = false, started = false;
    rg::TurboExtensionHooks hooks;
    hooks.before_generate = [&] {
        std::unique_lock lock(m);
        started = true;
        cv.notify_all();
        cv.wait(lock, [&] { return release; });
    };
    rg::TurboMapWorker worker;
    // Loading a cold engine map and preparing disk storage are fixture work, not submit latency.
    const auto desc = hyper_desc();
    const auto cache = scratch("turbomap_worker");
    const auto t0 = std::chrono::steady_clock::now();
    const auto job = worker.submit(desc, kHyperN2o, cache, cfg, hooks);
    const double submit_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    {
        std::unique_lock lock(m);
        REQUIRE(cv.wait_for(lock, std::chrono::seconds(60), [&] { return started; }));
        // The worker is blocked inside "generation" and the submitting thread is free: submit returned, nothing is done.
        CHECK_FALSE(job->done.load());
        release = true;
    }
    cv.notify_all();
    CHECK(submit_ms < 1000.0); // copying a VehicleDesc and starting a thread; generation takes much longer
    CHECK(job->thread != std::this_thread::get_id());
    for (int i = 0; i < 1200 && !job->done.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(job->done.load());
    CHECK(job->thread != std::thread::id());
    CHECK(job->result.status == rg::TurboExtensionResult::Status::Generated);
    CHECK(job->result.uncovered_steps_after < job->result.uncovered_steps_before);
}

TEST_CASE("turbo map extension: a compressor choke is reported as the choke flow, not as a narrow map", "[turbo_map][turbo_map_choke][limit_advisor]") {
    std::string err;
    auto config = rg::load_advisor_config(kRepo + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(config.has_value());
    pd::BuildIssue issue;
    issue.rule = pd::BuildRule::TurboStageMapCoverage;
    issue.severity = pd::LimitSeverity::Near;
    issue.slot = "forced_induction";
    issue.field_pointer = "/turbo";
    issue.message = "turbo stage map: the 1.8 bar target ...";
    issue.source_label = "[C] test";
    issue.value = 0.9375;
    issue.limit = 1.0;
    issue.unit = pd::LimitUnit::Fraction;

    rg::AdviceInput in;
    in.target_boost_bar = 1.8;
    in.build_issues = {issue};
    const auto plain = rg::build_advice(in, *config);
    REQUIRE(plain.size() == 1);
    CHECK(plain[0].variant.empty());

    rg::TurboChoke choke;
    choke.chokes = true;
    choke.choke_flow_kg_s = 0.5186;
    choke.demand_flow_kg_s = 0.5336;
    choke.units = 2;
    choke.pressure_ratio = 2.7765;
    in.turbo_choke = choke;
    const auto items = rg::build_advice(in, *config);
    REQUIRE(items.size() == 1);
    CHECK(items[0].variant == "choke");
    CHECK(items[0].title == "Compressor chokes at this boost");
    REQUIRE(items[0].lines.size() >= 2);
    CHECK(items[0].lines[1].find("0.519 kg/s") != std::string::npos);
    CHECK(items[0].lines[1].find("0.534 kg/s") != std::string::npos);
    CHECK(items[0].severity == rg::AdviceSeverity::Advice);
    for (const auto& l : items[0].lines) CHECK(l.find("narrow") == std::string::npos);

    // Not choking (the extended map covers everything): the old text if physics still reports an issue.
    in.turbo_choke->chokes = false;
    CHECK(rg::build_advice(in, *config)[0].variant.empty());
}
