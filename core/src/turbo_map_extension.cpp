// rg/turbo_map_extension.cpp - see turbo_map_extension.h.
#include "rg/turbo_map_extension.h"

#include "json_util.h"

#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/drivetrain/turbo_stage_maps.h"
#include "ps/io/turbo_stage_io.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <variant>

namespace rg {

namespace fs = std::filesystem;
namespace pd = ps::drivetrain;
namespace ml = ps::drivetrain::meanline;
using detail::json;

const char* to_string(TurboExtensionResult::Status s) {
    switch (s) {
        case TurboExtensionResult::Status::NotApplicable: return "not_applicable";
        case TurboExtensionResult::Status::Covered: return "covered";
        case TurboExtensionResult::Status::CacheHit: return "cache_hit";
        case TurboExtensionResult::Status::Generated: return "generated";
        case TurboExtensionResult::Status::Failed: return "failed";
    }
    return "?";
}

// ---- config -------------------------------------------------------------------------------------------------------------

namespace {

bool cfg_fail(std::string* err, const std::string& origin, const std::string& msg) {
    if (err != nullptr) *err = origin + ": " + msg;
    return false;
}

bool read_num(const json& o, const char* key, double& out, double lo, double hi, const std::string& origin, std::string* err) {
    const auto it = o.find(key);
    if (it == o.end()) return cfg_fail(err, origin, std::string("\"") + key + "\" is missing");
    if (!it->is_number()) return cfg_fail(err, origin, std::string("\"") + key + "\" must be a number");
    const double v = it->get<double>();
    if (!(v >= lo && v <= hi)) return cfg_fail(err, origin, std::string("\"") + key + "\" is outside its range");
    out = v;
    return true;
}

bool read_int(const json& o, const char* key, int& out, int lo, int hi, const std::string& origin, std::string* err) {
    double v = 0;
    if (!read_num(o, key, v, lo, hi, origin, err)) return false;
    if (v != std::floor(v)) return cfg_fail(err, origin, std::string("\"") + key + "\" must be a whole number");
    out = static_cast<int>(v);
    return true;
}

} // namespace

std::optional<TurboMapExtensionConfig> load_turbo_map_extension_config(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) {
        cfg_fail(err, path, "cannot read the file");
        return std::nullopt;
    }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) {
        cfg_fail(err, path, "not a JSON object");
        return std::nullopt;
    }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.turbo_map_extension/1") {
        cfg_fail(err, path, "\"format\" must be \"rg.turbo_map_extension/1\"");
        return std::nullopt;
    }
    TurboMapExtensionConfig c;
    if (root->contains("enabled") && !detail::get_bool(*root, "enabled", c.enabled)) {
        cfg_fail(err, path, "\"enabled\" must be true or false");
        return std::nullopt;
    }
    const auto policy = root->find("policy");
    if (policy == root->end() || !policy->is_object()) {
        cfg_fail(err, path, "\"policy\" object is missing");
        return std::nullopt;
    }
    double flow_margin = 0, speed_margin = 0, speed_step = 0, flow_step = 0;
    int max_lines = 0, max_flows = 0;
    if (!read_num(*policy, "flow_margin", flow_margin, 0.0, 10.0, path + " policy", err) ||
        !read_num(*policy, "speed_margin", speed_margin, 0.0, 10.0, path + " policy", err) ||
        !read_num(*policy, "speed_step_rpm", speed_step, 1.0, 1.0e6, path + " policy", err) ||
        !read_num(*policy, "flow_step_kg_s", flow_step, 1.0e-6, 100.0, path + " policy", err) ||
        !read_int(*policy, "max_speed_lines", max_lines, 2, 4096, path + " policy", err) ||
        !read_int(*policy, "max_flow_points", max_flows, 2, 4096, path + " policy", err)) {
        return std::nullopt;
    }
    c.policy.flow_margin = flow_margin;
    c.policy.speed_margin = speed_margin;
    c.policy.speed_step_rpm = speed_step;
    c.policy.flow_step_kg_s = flow_step;
    c.policy.max_speed_lines = max_lines;
    c.policy.max_flow_points = max_flows;
    const auto cache = root->find("cache");
    if (cache == root->end() || !cache->is_object() || !detail::get_string(*cache, "subdir", c.cache_subdir) || c.cache_subdir.empty() ||
        c.cache_subdir.find_first_of("/\\") != std::string::npos) {
        cfg_fail(err, path, "\"cache\".\"subdir\" must be a plain directory name");
        return std::nullopt;
    }
    const auto choke = root->find("choke");
    if (choke == root->end() || !choke->is_object()) {
        cfg_fail(err, path, "\"choke\" object is missing");
        return std::nullopt;
    }
    if (!read_num(*choke, "rpm_step", c.choke_rpm_step, 1.0, 1.0e5, path + " choke", err) ||
        !read_int(*choke, "bisect_iterations", c.choke_bisect_iterations, 4, 100, path + " choke", err) ||
        !read_num(*choke, "flow_scan_max_factor", c.choke_flow_scan_max_factor, 1.0, 100.0, path + " choke", err)) {
        return std::nullopt;
    }
    return c;
}

// ---- finding the map file -----------------------------------------------------------------------------------------------

namespace {

// The value of a string key of a part file, looking through "extends" parents; `dir_out` = the directory of the file that
// defines it (relative paths in the value are relative to that file).
bool lookup_part_string(const fs::path& file, const char* key, std::string& value, fs::path& dir_out, int depth = 0) {
    if (depth > 8) return false;
    const auto text = detail::read_text_file(file.string());
    if (!text) return false;
    const auto j = detail::parse_json(*text);
    if (!j || !j->is_object()) return false;
    if (detail::get_string(*j, key, value)) {
        dir_out = file.parent_path();
        return true;
    }
    std::string parent;
    if (detail::get_string(*j, "extends", parent) && !parent.empty()) {
        return lookup_part_string(file.parent_path() / parent, key, value, dir_out, depth + 1);
    }
    return false;
}

fs::path resolve_from(const fs::path& dir, const std::string& ref) {
    const fs::path p(ref);
    return (p.is_absolute() ? p : dir / p).lexically_normal();
}

} // namespace

std::string find_turbo_stage_map_path(const std::string& vehicle_path) {
    const auto text = detail::read_text_file(vehicle_path);
    if (!text) return {};
    const auto vehicle = detail::parse_json(*text);
    if (!vehicle || !vehicle->is_object()) return {};
    const auto powertrain = vehicle->find("powertrain");
    if (powertrain == vehicle->end() || !powertrain->is_object()) return {};
    const auto components = powertrain->find("components");
    if (components == powertrain->end() || !components->is_array()) return {};
    const fs::path vdir = fs::path(vehicle_path).parent_path();
    for (const json& c : *components) {
        std::string type;
        if (!c.is_object() || !detail::get_string(c, "type", type) || type != "engine") continue;
        std::string config_ref;
        fs::path config_dir;
        if (detail::get_string(c, "forced_induction", config_ref)) {
            config_dir = vdir;
        } else {
            std::string engine_ref;
            if (!detail::get_string(c, "ref", engine_ref)) return {};
            fs::path engine_dir;
            if (!lookup_part_string(resolve_from(vdir, engine_ref), "default_forced_induction", config_ref, engine_dir)) return {};
            config_dir = engine_dir;
        }
        if (config_ref.empty() || config_ref == "none") return {};
        const fs::path config_file = resolve_from(config_dir, config_ref);
        std::string turbo_ref;
        fs::path turbo_dir;
        if (!lookup_part_string(config_file, "turbo", turbo_ref, turbo_dir)) return {};
        const fs::path hardware = resolve_from(turbo_dir, turbo_ref);
        std::string map_ref;
        fs::path map_dir;
        if (!lookup_part_string(hardware, "performance_map", map_ref, map_dir)) return {};
        return resolve_from(map_dir, map_ref).generic_string();
    }
    return {};
}

// ---- the extension --------------------------------------------------------------------------------------------------------

namespace {

pd::SimulatedEngineDesc* stage_map_engine(ps::vehicle::VehicleDesc& desc) {
    for (pd::ComponentDesc& c : desc.powertrain.components) {
        if (auto* s = std::get_if<pd::SimulatedEngineDesc>(&c.params)) {
            if (s->turbo_pair && s->turbo_pair->maps && s->turbo_pair->maps->stages) return s;
            return nullptr; // the first simulated engine decides
        }
    }
    return nullptr;
}

int count_uncovered(const pd::TurboStageDemand& d) {
    return static_cast<int>(std::count_if(d.steps.begin(), d.steps.end(), [](const pd::TurboStageDemand::Step& s) { return !s.covered; }));
}

std::string hex16(std::uint64_t v) {
    char buf[17];
    std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

// The largest per-turbo flow at which the compressor still reaches `target_pr` on some valid point: at one flow the valid
// points start at the choke edge and the pressure ratio there rises with the flow, so the target is reachable while the
// smallest valid pressure ratio over the speed (the edge's) is at or below it.
double largest_reachable_flow(const ml::StageGeometry& g, double target_pr, double rpm_lo, double rpm_hi, double flow_hi,
                              const TurboMapExtensionConfig& c) {
    // Smallest valid pressure ratio at this flow = the pressure ratio on the choke edge: the first valid speed of a coarse
    // scan, then the edge itself by bisection between that speed and the one below it.
    const auto reachable = [&](double flow) {
        double below = -1.0;
        for (double rpm = rpm_lo; rpm <= rpm_hi; rpm += c.choke_rpm_step) {
            const ml::CompressorPoint p = ml::compressor_point(g, rpm, flow);
            if (p.status != ml::CompressorStatus::Ok) {
                below = rpm;
                continue;
            }
            if (below < 0.0) return p.pressure_ratio <= target_pr;
            double lo = below, hi = rpm, pr = p.pressure_ratio;
            for (int i = 0; i < c.choke_bisect_iterations; ++i) {
                const double mid = 0.5 * (lo + hi);
                const ml::CompressorPoint q = ml::compressor_point(g, mid, flow);
                if (q.status == ml::CompressorStatus::Ok) {
                    hi = mid;
                    pr = q.pressure_ratio;
                } else {
                    lo = mid;
                }
            }
            return pr <= target_pr;
        }
        return false;
    };
    if (reachable(flow_hi)) return flow_hi;
    double lo = 0.0, hi = flow_hi;
    for (int i = 0; i < c.choke_bisect_iterations; ++i) {
        const double mid = 0.5 * (lo + hi);
        (reachable(mid) ? lo : hi) = mid;
    }
    return lo;
}

} // namespace

TurboExtensionResult extend_turbo_maps(ps::vehicle::VehicleDesc& desc, const std::string& vehicle_path, const std::string& cache_dir,
                                       const TurboMapExtensionConfig& config, ps::jobs::JobSystem* pool, const std::atomic<bool>* cancel,
                                       const TurboExtensionHooks* hooks) {
    const auto t0 = std::chrono::steady_clock::now();
    TurboExtensionResult out;
    const auto finish = [&](TurboExtensionResult::Status s, std::string message = {}) {
        out.status = s;
        if (!message.empty()) out.message = std::move(message);
        out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return out;
    };
    if (!config.enabled) return finish(TurboExtensionResult::Status::NotApplicable, "disabled");
    pd::SimulatedEngineDesc* engine = stage_map_engine(desc);
    if (engine == nullptr) return finish(TurboExtensionResult::Status::NotApplicable, "no engine turbo with a stage map");

    out.source_map_path = find_turbo_stage_map_path(vehicle_path);
    if (out.source_map_path.empty()) return finish(TurboExtensionResult::Status::NotApplicable, "the stage map file of the turbo was not found");
    if (cancel != nullptr && cancel->load()) return finish(TurboExtensionResult::Status::Failed, "cancelled");

    try {
        const auto map_text = detail::read_text_file(out.source_map_path);
        if (!map_text) return finish(TurboExtensionResult::Status::Failed, "cannot read " + out.source_map_path);
        // The demand is always taken against the map FILE the turbo names, never against maps already swapped into `desc`, so
        // the extended axes (and with them the cache key) depend on the car alone and a second pass over an extended desc
        // is a cache hit, not a new extension.
        pd::SimulatedEngineDesc original_engine = *engine;
        {
            auto original_maps = std::make_shared<pd::TurboPerformanceMaps>(*engine->turbo_pair->maps);
            original_maps->stages = ps::io::turbo_stage_maps_from_json(*map_text, engine->turbo_pair->maps->stages->units, out.source_map_path);
            original_engine.turbo_pair->maps = std::move(original_maps);
        }
        const pd::TurboStageDemand before = pd::turbo_stage_demand(original_engine);
        if (!before.applies || before.steps.empty()) return finish(TurboExtensionResult::Status::NotApplicable, "no stage-map demand");
        out.uncovered_steps_before = count_uncovered(before);
        out.choke.units = before.units;
        out.choke.pressure_ratio = before.pressure_ratio;
        out.choke.demand_flow_kg_s = before.steps.back().flow_kg_s;
        out.choke.choke_flow_kg_s = out.choke.demand_flow_kg_s;
        if (out.uncovered_steps_before == 0) return finish(TurboExtensionResult::Status::Covered);
        const ml::StageGeometry geometry = ps::io::parse_turbo_geometry_json(*map_text, out.source_map_path);
        const ml::Sampling current = ps::io::parse_turbo_stage_sampling_json(*map_text, out.source_map_path);
        const double kappa = (geometry.air.gamma - 1.0) / geometry.air.gamma;
        const ml::Sampling wide = ml::extend_sampling_for_demand(current, ml::demand_points(before, kappa), config.policy);
        out.cache_key = ml::cache_key(geometry, wide, nullptr);

        const int units = before.units;
        fs::path cache_file;
        if (!cache_dir.empty()) cache_file = fs::path(cache_dir) / config.cache_subdir / ("turbo_stage_" + hex16(out.cache_key) + ".json");
        std::shared_ptr<pd::TurboStageMaps> maps;
        bool hit = false;
        if (!cache_file.empty()) {
            std::error_code ec;
            if (fs::is_regular_file(cache_file, ec)) {
                if (const auto cached = detail::read_text_file(cache_file.string())) {
                    try {
                        maps = ps::io::turbo_stage_maps_from_json(*cached, units, cache_file.string());
                        hit = true;
                    } catch (const std::exception&) {
                        maps.reset(); // a damaged cache file is a miss: regenerated and replaced below
                    }
                }
            }
        }
        if (!hit) {
            if (hooks != nullptr && hooks->before_generate) hooks->before_generate();
            if (cancel != nullptr && cancel->load()) return finish(TurboExtensionResult::Status::Failed, "cancelled");
            const ml::StageMapResult generated = ml::generate(geometry, wide, pool);
            maps = ml::to_stage_maps(generated, units);
            if (!cache_file.empty()) {
                try {
                    std::error_code ec;
                    fs::create_directories(cache_file.parent_path(), ec);
                    ps::io::write_turbo_stage_maps_json(cache_file.string(), generated, out.cache_key); // atomic: temp + rename
                } catch (const std::exception& e) {
                    out.message = std::string("the extended map could not be cached: ") + e.what(); // driving does not need the file
                    cache_file.clear();
                }
            }
        }
        out.cache_file = cache_file.generic_string();

        // The vehicle uses the extended maps: a copy of the performance maps with the new stage maps; the content hash
        // (state hash input) moves with the cache key.
        auto replaced = std::make_shared<pd::TurboPerformanceMaps>(*engine->turbo_pair->maps);
        replaced->stages = maps;
        replaced->content_hash = (replaced->content_hash ^ out.cache_key) * 1099511628211ULL;
        engine->turbo_pair->maps = std::move(replaced);

        const pd::TurboStageDemand after = pd::turbo_stage_demand(*engine);
        out.uncovered_steps_after = count_uncovered(after);
        if (out.uncovered_steps_after > 0) {
            out.choke.chokes = true;
            double rpm_lo = std::numeric_limits<double>::max(), rpm_hi = 0.0;
            for (const auto& s : after.steps) {
                rpm_lo = std::min(rpm_lo, s.rotor_rpm);
                rpm_hi = std::max(rpm_hi, s.rotor_rpm);
            }
            rpm_hi = std::max(rpm_hi, after.top_speed_line_rpm) * 1.5;
            rpm_lo *= 0.5;
            out.choke.choke_flow_kg_s = largest_reachable_flow(geometry, after.pressure_ratio, rpm_lo, rpm_hi,
                                                               out.choke.demand_flow_kg_s * config.choke_flow_scan_max_factor, config);
            out.choke.choke_flow_kg_s = std::min(out.choke.choke_flow_kg_s, out.choke.demand_flow_kg_s);
        }
        return finish(hit ? TurboExtensionResult::Status::CacheHit : TurboExtensionResult::Status::Generated);
    } catch (const std::exception& e) {
        return finish(TurboExtensionResult::Status::Failed, e.what());
    }
}

// ---- the worker ------------------------------------------------------------------------------------------------------------

TurboMapWorker::~TurboMapWorker() { cancel_all(); }

void TurboMapWorker::cancel_all() {
    std::lock_guard lock(mutex_);
    for (auto& [job, thread] : jobs_) job->cancel.store(true);
    for (auto& [job, thread] : jobs_) {
        if (thread.joinable()) thread.join();
    }
    jobs_.clear();
}

std::shared_ptr<TurboMapWorker::Job> TurboMapWorker::submit(ps::vehicle::VehicleDesc desc, std::string vehicle_path, std::string cache_dir,
                                                            TurboMapExtensionConfig config, TurboExtensionHooks hooks) {
    auto job = std::make_shared<Job>();
    job->desc = std::move(desc);
    std::thread thread([job, vehicle_path = std::move(vehicle_path), cache_dir = std::move(cache_dir), config = std::move(config),
                        hooks = std::move(hooks)]() mutable {
        job->thread = std::this_thread::get_id();
        job->result = extend_turbo_maps(job->desc, vehicle_path, cache_dir, config, nullptr, &job->cancel, &hooks);
        job->done.store(true, std::memory_order_release);
    });
    std::lock_guard lock(mutex_);
    // Finished jobs are joined here so the list stays short.
    for (auto it = jobs_.begin(); it != jobs_.end();) {
        if (it->first->done.load(std::memory_order_acquire) && it->second.joinable()) {
            it->second.join();
            it = jobs_.erase(it);
        } else {
            ++it;
        }
    }
    jobs_.emplace_back(job, std::move(thread));
    return job;
}

} // namespace rg
