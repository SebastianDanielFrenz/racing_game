// rg/upgrade_hints.cpp - see upgrade_hints.h.
#include "rg/upgrade_hints.h"

#include "rg/cooling_upgrades.h"
#include "rg/limit_advisor.h"
#include "rg/turbo_map_extension.h"
#include "rg/vehicle_load.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/tyre_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace rg {

namespace pd = ps::drivetrain;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---- tyres -------------------------------------------------------------------------------------------------------------

// The largest |f(x)| over [lo, hi]: a coarse scan of `n` intervals, then golden-section refinement around the best sample.
template <class F>
double peak_abs(const F& f, double lo, double hi, int n) {
    const double step = (hi - lo) / n;
    double best = -1.0, best_x = lo;
    for (int i = 0; i <= n; ++i) {
        const double x = lo + step * i;
        const double v = std::fabs(f(x));
        if (v > best) {
            best = v;
            best_x = x;
        }
    }
    double a = std::max(lo, best_x - step), b = std::min(hi, best_x + step);
    constexpr double phi = 0.6180339887498949;
    double c = b - phi * (b - a), d = a + phi * (b - a);
    double fc = std::fabs(f(c)), fd = std::fabs(f(d));
    for (int i = 0; i < 48; ++i) {
        if (fc > fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - phi * (b - a);
            fc = std::fabs(f(c));
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + phi * (b - a);
            fd = std::fabs(f(d));
        }
    }
    return std::max({best, fc, fd});
}

pd::MapQuality quality_of(const std::string& q) {
    if (q == "full") return pd::MapQuality::Full;
    if (q == "preview") return pd::MapQuality::Preview;
    return pd::MapQuality::Sketch;
}

void check_cancel(const FigureEnv& env) {
    if (env.cancel != nullptr && env.cancel->load()) throw ps::io::GenerationCancelled();
}

// ---- the engine --------------------------------------------------------------------------------------------------------

HintEnginePeaks sweep_engine(const ps::vehicle::VehicleDesc& desc, const UpgradeReferences& refs, bool nitrous_armed, const FigureEnv& env) {
    HintEnginePeaks out;
    // The electric side first: it needs no sweep.
    double motor_w = 0.0, discharge_w = 0.0;
    bool has_battery = false;
    for (const pd::ComponentDesc& c : desc.powertrain.components) {
        if (const auto* m = std::get_if<pd::ElectricMotorDesc>(&c.params)) motor_w += m->max_power_w;
        if (const auto* b = std::get_if<pd::BatteryDesc>(&c.params)) {
            out.battery_kwh += b->capacity_kwh;
            discharge_w += b->max_discharge_w;
            has_battery = true;
        }
    }
    out.electric_kw = (has_battery ? std::min(motor_w, discharge_w) : motor_w) / 1000.0;

    const pd::ComponentParams* engine = nullptr;
    const pd::SimulatedEngineDesc* simulated = nullptr;
    double idle = 0.0, limiter = 0.0;
    for (const pd::ComponentDesc& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<pd::SimulatedEngineDesc>(&c.params)) {
            engine = &c.params;
            simulated = s;
            idle = s->idle_rpm;
            limiter = s->limiter.rpm;
            break;
        }
        if (const auto* m = std::get_if<pd::TorqueMapEngineDesc>(&c.params)) {
            if (engine == nullptr) {
                engine = &c.params;
                idle = m->idle_rpm;
                limiter = m->limiter.rpm;
            }
        }
    }
    if (engine == nullptr) return out;
    out.has_engine = true;

    pd::DynoSweepSpec spec;
    const double step = refs.dyno_rpm_step;
    const double end = limiter - refs.dyno_limiter_margin_rpm;
    double start = std::ceil(std::max(refs.dyno_rpm_min, idle) / step) * step;
    if (!(start < end)) start = std::ceil(idle / step) * step;
    for (double rpm = start; rpm < end - 1.0e-6; rpm += step) spec.rpm.push_back(rpm);
    if (end > idle) spec.rpm.push_back(end);
    if (spec.rpm.empty()) spec.rpm.push_back(std::max(idle, 1.0));
    spec.turbo_count = 1;
    if (nitrous_armed && simulated != nullptr) spec.nitrous = pd::nitrous_from_powertrain(desc.powertrain);

    const auto t0 = std::chrono::steady_clock::now();
    const pd::DynoResult dyno = pd::run_dyno_sweep(*engine, nullptr, spec, env.pool, env.cancel);
    out.sweep_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const pd::DynoPoint* peak = nullptr;
    for (const pd::DynoPoint& p : dyno.points) {
        if (!std::isfinite(p.power_kw)) continue;
        out.peak_torque_nm = std::max(out.peak_torque_nm, static_cast<double>(p.torque_nm));
        if (peak == nullptr || p.power_kw > peak->power_kw) peak = &p;
    }
    if (peak == nullptr) return out;
    out.peak_power_kw = peak->power_kw;
    out.peak_power_rpm = peak->rpm;

    // The engine thermal model's steady state at the peak-power point.
    if (simulated != nullptr && desc.powertrain.thermal.has_value() && !simulated->fuels.empty() && std::isfinite(peak->fuel_g_s) &&
        peak->fuel_g_s > 0.0) {
        const pd::EngineThermalConfig& thermal = *desc.powertrain.thermal;
        const double fuel_w = peak->fuel_g_s * 1.0e-3 * simulated->fuels.front().lhv_j_kg;
        const double wall_w = thermal.engine.wall_heat_fraction * fuel_w;
        const double friction_w = refs.thermal_friction_power_fraction * std::max(0.0, static_cast<double>(peak->power_kw)) * 1000.0;
        const SteadyThermal t = steady_thermal(thermal, peak->rpm, wall_w, friction_w, refs.thermal_ambient_c, refs.thermal_air_speed_kmh / 3.6);
        out.coolant_c = t.coolant_c;
        out.oil_c = t.oil_c;
    }
    return out;
}

} // namespace

// ---- tyres (public) ----------------------------------------------------------------------------------------------------

std::optional<TyreReference> load_tyre_reference(const UpgradeReferences& refs, std::string* err) {
    TyreReference out;
    try {
        const ps::io::SurfaceTable table(refs.surface_table);
        const ps::SurfaceId id = table.id_for(refs.surface);
        if (id == ps::kInvalidSurfaceId) {
            if (err != nullptr) *err = refs.surface_table + ": no surface \"" + refs.surface + "\"";
            return std::nullopt;
        }
        out.lambda_mu = table.lambda_mu_for(id);
        out.crr = table.crr_for(id);
    } catch (const std::exception& e) {
        if (err != nullptr) *err = std::string("surface table: ") + e.what();
        return std::nullopt;
    }
    out.speed_m_s = refs.rolling_resistance_speed_kmh / 3.6;
    return out;
}

std::vector<double> static_corner_loads(const std::vector<std::pair<double, double>>& wheel_xy, double weight_n) {
    const std::size_t n = wheel_xy.size();
    if (n == 0) return {};
    const std::vector<double> equal(n, weight_n / static_cast<double>(n));
    if (n < 3) return equal;
    // Smallest-norm F = A^T lambda with A F = (W, 0, 0), A = [1; x; y]: solve (A A^T) lambda = b.
    double g[3][3] = {{static_cast<double>(n), 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (const auto& [x, y] : wheel_xy) {
        g[0][1] += x;
        g[0][2] += y;
        g[1][1] += x * x;
        g[1][2] += x * y;
        g[2][2] += y * y;
    }
    g[1][0] = g[0][1];
    g[2][0] = g[0][2];
    g[2][1] = g[1][2];
    double m[3][4] = {{g[0][0], g[0][1], g[0][2], weight_n}, {g[1][0], g[1][1], g[1][2], 0.0}, {g[2][0], g[2][1], g[2][2], 0.0}};
    for (int col = 0; col < 3; ++col) { // Gauss-Jordan with partial pivoting
        int pivot = col;
        for (int r = col + 1; r < 3; ++r) {
            if (std::fabs(m[r][col]) > std::fabs(m[pivot][col])) pivot = r;
        }
        if (std::fabs(m[pivot][col]) < 1.0e-9) return equal; // the wheels lie on one line: no moment balance exists
        for (int k = 0; k < 4; ++k) std::swap(m[col][k], m[pivot][k]);
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double factor = m[r][col] / m[col][col];
            for (int k = col; k < 4; ++k) m[r][k] -= factor * m[col][k];
        }
    }
    const double l0 = m[0][3] / m[0][0], l1 = m[1][3] / m[1][1], l2 = m[2][3] / m[2][2];
    std::vector<double> loads;
    loads.reserve(n);
    for (const auto& [x, y] : wheel_xy) loads.push_back(l0 + l1 * x + l2 * y);
    return loads;
}

TyreFigures tyre_figures(const ps::vehicle::VehicleDesc& desc, double mass_kg, const TyreReference& reference) {
    if (desc.wheels.empty()) throw std::runtime_error("the vehicle has no wheels");
    if (!(mass_kg > 0.0)) throw std::runtime_error("the car's mass must be positive");
    TyreFigures out;
    out.weight_n = mass_kg * kHintGravity;
    std::vector<std::pair<double, double>> xy;
    for (const ps::vehicle::WheelDesc& w : desc.wheels) xy.emplace_back(w.attachment_local.x, w.attachment_local.y);
    out.corner_loads_n = static_corner_loads(xy, out.weight_n);
    for (std::size_t i = 0; i < desc.wheels.size(); ++i) {
        const ps::vehicle::WheelDesc& wheel = desc.wheels[i];
        const double fz = std::max(0.0, out.corner_loads_n[i]);
        const auto model = ps::vehicle::create_pacejka_tyre_model(wheel.tyre);
        ps::vehicle::TyreSlipInput in;
        in.fz = fz;
        in.gamma = wheel.static_camber;
        in.vx = reference.speed_m_s;
        in.lambda_mu = reference.lambda_mu;
        out.peak_fx_n += peak_abs(
            [&](double kappa) {
                ps::vehicle::TyreSlipInput s = in;
                s.kappa = kappa;
                return model->evaluate(s).fx;
            },
            -1.0, 1.0, 200);
        out.peak_fy_n += peak_abs(
            [&](double alpha) {
                ps::vehicle::TyreSlipInput s = in;
                s.alpha = alpha;
                return model->evaluate(s).fy;
            },
            -0.6, 0.6, 240);
        // Cruising at the reference speed: the wheel's own rolling-resistance law (physics_sim wheel_instance.cpp) at zero slip.
        const double fx_pure = model->evaluate(in).fx_pure;
        const double crr = wheel.tyre.qsy1 + reference.crr + wheel.tyre.qsy2 * std::fabs(fx_pure) / wheel.tyre.fz0 +
                           ps::vehicle::tyre_rolling_speed_term(wheel.tyre, reference.speed_m_s);
        out.rolling_n += fz * crr;
    }
    out.grip_longitudinal_g = out.peak_fx_n / out.weight_n;
    out.grip_lateral_g = out.peak_fy_n / out.weight_n;
    out.rolling_g = out.rolling_n / out.weight_n;
    return out;
}

// ---- figures of one setup ----------------------------------------------------------------------------------------------

FigureEnv make_figure_env(const UpgradeReferences& refs) {
    FigureEnv env;
    env.references = refs;
    if (const auto tyre = load_tyre_reference(refs, &env.tyre_error)) {
        env.tyre = *tyre;
        env.tyre_ok = true;
    }
    return env;
}

SetupFigures compute_setup_figures(const HintBase& base, const VehicleSetup& setup, const std::set<UpgradeQuantity>& wanted,
                                   bool nitrous_armed, const FigureEnv& env) {
    SetupFigures out;
    out.computed = wanted;
    const auto t0 = std::chrono::steady_clock::now();
    const auto done = [&] {
        out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return out;
    };
    const auto has = [&](UpgradeQuantity q) { return wanted.count(q) != 0; };
    const bool sweep_needed = has(UpgradeQuantity::Power) || has(UpgradeQuantity::Torque) || has(UpgradeQuantity::CoolantTemperature) ||
                              has(UpgradeQuantity::OilTemperature);
    const bool tyres_needed = has(UpgradeQuantity::GripLongitudinal) || has(UpgradeQuantity::GripLateral) || has(UpgradeQuantity::RollingResistance);
    const bool desc_needed = sweep_needed || tyres_needed || has(UpgradeQuantity::BatteryKwh);
    check_cancel(env);

    CompiledSetup compiled;
    std::string vehicle_path;
    std::optional<ScopedWorkDir> work;
    try {
        if (desc_needed) {
            work.emplace(base.context.work_root + "_hints", "fig");
            MaterialisedSetup material = materialise_setup(base.entry, base.table, base.context, setup, work->path());
            if (!material.ok) {
                out.error = material.error;
                return done();
            }
            compiled = std::move(material.compiled);
            vehicle_path = material.vehicle_path;
        } else {
            compiled = compile_setup(base.entry, base.table, base.context, setup);
            if (!compiled.ok) {
                out.error = compiled.error;
                return done();
            }
        }
    } catch (const ps::io::GenerationCancelled&) {
        throw;
    } catch (const std::exception& e) {
        out.error = e.what();
        return done();
    }

    // The whole car's mass: the part-mass tree total, else the catalog chassis plus the swap's delta.
    const double mass_kg = compiled.parts.ok ? compiled.parts.total_kg : base.entry.chassis.mass_kg + compiled.chassis_mass_delta_kg;
    if (has(UpgradeQuantity::Weight)) out.figures[UpgradeQuantity::Weight] = mass_kg;
    if (has(UpgradeQuantity::RpmTolerance) && compiled.internals) {
        double lowest = 0.0;
        for (const InternalPartBuild& p : compiled.internals->parts) {
            if (p.limit_rpm > 0.0 && (lowest <= 0.0 || p.limit_rpm < lowest)) lowest = p.limit_rpm;
        }
        if (lowest > 0.0) out.figures[UpgradeQuantity::RpmTolerance] = lowest;
    }
    if (!desc_needed) return done();
    check_cancel(env);

    try {
        ps::io::EngineMapOptions maps;
        maps.cancel = env.cancel;
        maps.pool = env.pool;
        if (sweep_needed) {
            maps.quality = quality_of(env.references.dyno_quality);
            maps.cache_dir = base.context.engine_map_cache_dir;
        } else {
            maps.generate = false; // validate only: tyres and batteries need no engine maps
        }
        ps::vehicle::VehicleDesc desc = rg::load_vehicle(vehicle_path, maps);
        if (sweep_needed && base.context.turbo_map_extension) {
            (void)extend_turbo_maps(desc, vehicle_path, base.context.engine_map_cache_dir, *base.context.turbo_map_extension, env.pool, env.cancel);
        }
        check_cancel(env);
        if (tyres_needed) {
            if (!env.tyre_ok) {
                out.error = env.tyre_error;
            } else {
                const TyreFigures t = tyre_figures(desc, mass_kg, env.tyre);
                if (has(UpgradeQuantity::GripLongitudinal)) out.figures[UpgradeQuantity::GripLongitudinal] = t.grip_longitudinal_g;
                if (has(UpgradeQuantity::GripLateral)) out.figures[UpgradeQuantity::GripLateral] = t.grip_lateral_g;
                if (has(UpgradeQuantity::RollingResistance)) out.figures[UpgradeQuantity::RollingResistance] = t.rolling_g;
            }
        }
        const HintEnginePeaks peaks = sweep_needed || has(UpgradeQuantity::BatteryKwh)
                                      ? (sweep_needed ? sweep_engine(desc, env.references, nitrous_armed, env) : [&] {
                                            HintEnginePeaks e;
                                            for (const pd::ComponentDesc& c : desc.powertrain.components) {
                                                if (const auto* b = std::get_if<pd::BatteryDesc>(&c.params)) e.battery_kwh += b->capacity_kwh;
                                            }
                                            return e;
                                        }())
                                      : HintEnginePeaks{};
        if (has(UpgradeQuantity::BatteryKwh)) out.figures[UpgradeQuantity::BatteryKwh] = peaks.battery_kwh;
        if (sweep_needed) {
            if (has(UpgradeQuantity::Power) && (peaks.has_engine || peaks.electric_kw > 0.0)) {
                out.figures[UpgradeQuantity::Power] = (peaks.has_engine ? peaks.peak_power_kw : 0.0) + peaks.electric_kw;
            }
            if (has(UpgradeQuantity::Torque) && peaks.has_engine) out.figures[UpgradeQuantity::Torque] = peaks.peak_torque_nm;
            if (has(UpgradeQuantity::CoolantTemperature) && std::isfinite(peaks.coolant_c)) out.figures[UpgradeQuantity::CoolantTemperature] = peaks.coolant_c;
            if (has(UpgradeQuantity::OilTemperature) && std::isfinite(peaks.oil_c)) out.figures[UpgradeQuantity::OilTemperature] = peaks.oil_c;
        }
    } catch (const ps::io::GenerationCancelled&) {
        throw;
    } catch (const std::exception& e) {
        out.error = e.what();
    }
    return done();
}

// ---- the cache and the background thread ------------------------------------------------------------------------------

UpgradeHints::UpgradeHints(UpgradeBadgeConfig config, int pool_workers)
    : config_(std::move(config)), env_(make_figure_env(config_.references)), pool_workers_(pool_workers), thread_([this] { run(); }) {}

UpgradeHints::~UpgradeHints() {
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
        if (cancel_) cancel_->store(true);
    }
    wake_.notify_all();
    thread_.join();
}

std::string UpgradeHints::key_of(const VehicleSetup& setup, bool nitrous) {
    return setup_to_json(setup) + (nitrous ? "|armed" : "|dry");
}

void UpgradeHints::set_base(HintBase base, VehicleSetup current) {
    std::lock_guard lock(mutex_);
    const bool same_car = have_base_ && base_.entry.id == base.entry.id && base_.context.engines == base.context.engines &&
                          base_.context.engine_internals == base.context.engine_internals &&
                          base_.context.cooling_upgrades == base.context.cooling_upgrades && base_.context.turbo_kits == base.context.turbo_kits;
    const bool same_current = same_car && setup_to_json(current_) == setup_to_json(current);
    if (!same_car) cache_.clear();
    base_ = std::move(base);
    have_base_ = true;
    if (same_current) return;
    current_ = std::move(current);
    ++base_generation_;
    ++request_generation_;
    pending_.reset(); // a candidate was meant against the previous current setup
    if (cancel_ && running_) {
        cancel_->store(true);
        ++stats_.jobs_cancelled;
    }
    wake_.notify_all();
}

HintAnswer UpgradeHints::request(const VehicleSetup& candidate, const std::vector<UpgradeQuantity>& metrics, bool nitrous_armed) {
    HintAnswer answer;
    std::lock_guard lock(mutex_);
    if (!have_base_) {
        answer.error = "no car is set";
        return answer;
    }
    const std::string current_key = key_of(current_, nitrous_armed);
    const std::string cand_key = key_of(candidate, nitrous_armed);
    Entry& installed = cache_[current_key];
    Entry& cand = cache_[cand_key];
    std::set<UpgradeQuantity> missing_installed, missing_candidate;
    for (UpgradeQuantity q : metrics) {
        if (installed.done.count(q) == 0) missing_installed.insert(q);
        if (cand.done.count(q) == 0) missing_candidate.insert(q);
        if (installed.done.count(q) == 0 || cand.done.count(q) == 0) answer.pending.insert(q);
    }
    answer.installed = installed.figures;
    answer.candidate = cand.figures;
    answer.error = !cand.error.empty() ? cand.error : installed.error;
    if (answer.pending.empty()) {
        ++stats_.cache_hits;
        return answer;
    }
    // Queue (or keep) the work. The same candidate asking again changes nothing; a different one replaces the queued job and
    // cancels a running candidate computation (a running baseline finishes: every later hint needs it).
    std::set<UpgradeQuantity> wanted = answer.pending;
    if (pending_ && pending_->cand_key == cand_key && pending_->nitrous == nitrous_armed) {
        pending_->wanted.insert(wanted.begin(), wanted.end());
        return answer;
    }
    if (running_ && running_cand_key_ == cand_key && !cancel_->load()) return answer; // already on it
    Job job;
    job.cand_key = cand_key;
    job.candidate = candidate;
    job.wanted = std::move(wanted);
    job.nitrous = nitrous_armed;
    pending_ = std::move(job);
    ++request_generation_;
    if (running_ && running_candidate_phase_ && running_cand_key_ != cand_key && cancel_) {
        cancel_->store(true);
        ++stats_.jobs_cancelled;
    }
    wake_.notify_all();
    return answer;
}

HintAnswer UpgradeHints::request_current(const std::vector<UpgradeQuantity>& metrics) {
    HintAnswer answer;
    std::lock_guard lock(mutex_);
    if (!have_base_) {
        answer.error = "no car is set";
        return answer;
    }
    const std::string key = key_of(current_, false);
    const Entry& e = cache_[key];
    std::set<UpgradeQuantity> missing;
    for (UpgradeQuantity q : metrics) {
        spec_wanted_.insert(q);
        if (e.done.count(q) == 0) missing.insert(q);
    }
    answer.installed = answer.candidate = e.figures;
    answer.error = e.error;
    answer.pending = missing;
    if (missing.empty()) {
        ++stats_.cache_hits;
        return answer;
    }
    if (pending_) return answer; // a queued job (a tile's) takes the specification figures along in its baseline
    if (running_) return answer; // the next poll queues what the running job did not cover
    Job job;
    job.cand_key = key;
    job.candidate = current_;
    job.wanted = std::move(missing);
    job.nitrous = false;
    pending_ = std::move(job);
    ++request_generation_;
    wake_.notify_all();
    return answer;
}

void UpgradeHints::cancel() {
    std::lock_guard lock(mutex_);
    ++request_generation_;
    pending_.reset();
    if (running_ && cancel_) {
        cancel_->store(true);
        ++stats_.jobs_cancelled;
    }
    wake_.notify_all();
}

bool UpgradeHints::wait_idle(int timeout_ms) {
    std::unique_lock lock(mutex_);
    return idle_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !pending_ && !running_; });
}

HintStats UpgradeHints::stats() const {
    std::lock_guard lock(mutex_);
    return stats_;
}

namespace {

// Figures that come from the part-mass tree alone (a compile, no engine maps, no disk): they are computed at once, without the
// debounce that protects the expensive sweeps from a flick over the tiles.
bool is_cheap_quantity(UpgradeQuantity q) { return q == UpgradeQuantity::Weight || q == UpgradeQuantity::RpmTolerance; }

} // namespace

void UpgradeHints::run() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    ps::jobs::JobSystemOptions pool_options;
    pool_options.background_priority = true;
    ps::jobs::JobSystem pool(pool_workers_ > 0 ? pool_workers_ : background_worker_count(), pool_options);

    // Computes the missing figures of `wanted` for the current setup, then the candidate; each into the cache.
    const auto fill = [&](const Job& job, const std::set<UpgradeQuantity>& wanted, const std::set<UpgradeQuantity>& spec_extra, const HintBase& base,
                          const VehicleSetup& current, std::uint64_t base_generation, const FigureEnv& env) {
        const auto store = [&](const std::string& key, const SetupFigures& f) {
            std::lock_guard lock(mutex_);
            if (base_generation != base_generation_) return; // the current setup changed meanwhile: these figures are not wanted
            Entry& e = cache_[key];
            for (const auto& [q, v] : f.figures) e.figures[q] = v;
            e.done.insert(f.computed.begin(), f.computed.end());
            if (!f.error.empty()) e.error = f.error;
            ++stats_.figure_sets_computed;
        };
        const auto missing_of = [&](const std::string& key, const std::set<UpgradeQuantity>* extra) {
            std::set<UpgradeQuantity> missing;
            std::lock_guard lock(mutex_);
            const Entry& e = cache_[key];
            for (UpgradeQuantity q : wanted) {
                if (e.done.count(q) == 0) missing.insert(q);
            }
            if (extra != nullptr) {
                for (UpgradeQuantity q : *extra) {
                    if (e.done.count(q) == 0) missing.insert(q);
                }
            }
            return missing;
        };
        const std::string current_key = key_of(current, job.nitrous);
        if (current_key != job.cand_key) {
            const auto missing = missing_of(current_key, &spec_extra);
            if (!missing.empty()) store(current_key, compute_setup_figures(base, current, missing, job.nitrous, env));
        }
        {
            std::lock_guard lock(mutex_);
            running_candidate_phase_ = true;
        }
        const auto missing = missing_of(job.cand_key, current_key == job.cand_key ? &spec_extra : nullptr);
        if (!missing.empty()) store(job.cand_key, compute_setup_figures(base, job.candidate, missing, job.nitrous, env));
    };

    for (;;) {
        Job job;
        HintBase base;
        VehicleSetup current;
        std::uint64_t base_generation = 0, generation = 0;
        std::shared_ptr<std::atomic<bool>> cancel;
        std::set<UpgradeQuantity> cheap, spec, spec_cheap;
        {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
            if (stopping_) return;
            job = *pending_; // stays queued: the debounce below may be interrupted by a newer request
            spec = job.nitrous ? std::set<UpgradeQuantity>{} : spec_wanted_; // the specification pane's figures (baseline only)
            base = base_;
            current = current_;
            base_generation = base_generation_;
            generation = request_generation_;
            for (UpgradeQuantity q : job.wanted) {
                if (is_cheap_quantity(q)) cheap.insert(q);
            }
            for (UpgradeQuantity q : spec) {
                if (is_cheap_quantity(q)) spec_cheap.insert(q);
            }
        }

        // Stage 1: the cheap figures, now.
        FigureEnv env = env_;
        env.pool = &pool;
        if (!cheap.empty()) {
            try {
                fill(job, cheap, spec_cheap, base, current, base_generation, env);
            } catch (const std::exception&) {
            }
        }
        {
            std::unique_lock lock(mutex_);
            running_candidate_phase_ = false;
            if (stopping_) return;
            if (request_generation_ != generation || !pending_) {
                idle_.notify_all();
                continue; // replaced or cancelled meanwhile: the newer request is handled by the next turn
            }
            if (cheap.size() == job.wanted.size()) { // nothing else to compute
                pending_.reset();
                ++stats_.jobs_started;
                ++stats_.jobs_finished;
                idle_.notify_all();
                continue;
            }
            // Stage 2: a hover must rest before a sweep starts - a flick over the tiles starts none.
            if (wake_.wait_for(lock, std::chrono::milliseconds(config_.references.dyno_debounce_ms),
                               [&] { return stopping_ || request_generation_ != generation; })) {
                if (stopping_) return;
                continue; // a newer request or a cancel arrived
            }
            job = std::move(*pending_);
            pending_.reset();
            if (!job.nitrous) spec = spec_wanted_;
            cancel = std::make_shared<std::atomic<bool>>(false);
            cancel_ = cancel;
            running_ = true;
            running_candidate_phase_ = false;
            running_cand_key_ = job.cand_key;
            ++stats_.jobs_started;
        }

        env.cancel = cancel.get();
        const auto t0 = std::chrono::steady_clock::now();
        bool cancelled = false;
        try {
            fill(job, job.wanted, spec, base, current, base_generation, env);
        } catch (const ps::io::GenerationCancelled&) {
            cancelled = true;
        } catch (const std::exception&) {
            cancelled = true; // an unexpected failure caches nothing: the next request tries again
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        {
            std::lock_guard lock(mutex_);
            running_ = false;
            running_candidate_phase_ = false;
            running_cand_key_.clear();
            if (!cancelled) {
                ++stats_.jobs_finished;
                stats_.last_job_seconds = seconds;
                stats_.total_seconds += seconds;
            }
            if (cache_.size() > 512) { // figures are small, but a long session hovers over a lot of setups
                for (auto it = cache_.begin(); it != cache_.end() && cache_.size() > 384;) {
                    it = cache_.erase(it);
                }
            }
        }
        idle_.notify_all();
        // A request that arrived while this job ran (cancelled or not) is picked up by the next loop turn.
    }
}

} // namespace rg
