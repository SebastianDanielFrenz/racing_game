#include "rg/garage_dyno.h"
#include "rg/vehicle_load.h"
#include "json_util.h"
#include "powertrain_swap_json.h"
#include "drivetrain/engine_simulated.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_dyno.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <cctype>
#include <cstdio>
#include <set>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <string_view>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
namespace rg {
namespace {
std::vector<DynoPoint> measure_dyno(const ps::drivetrain::SimulatedEngineDesc* simulated,
    const ps::drivetrain::TorqueMapEngineDesc* mapped,int count,const std::atomic<bool>* cancel,
    const std::function<void(const std::vector<DynoPoint>&,int)>& publish) {
    const double idle=simulated ? simulated->idle_rpm:mapped->idle_rpm;
    const double limit=simulated ? simulated->limiter.rpm:mapped->limiter.rpm;
    std::unique_ptr<ps::drivetrain::SimulatedEngine> engine;
    if(simulated)engine=std::make_unique<ps::drivetrain::SimulatedEngine>(*simulated);
    constexpr double pi=3.14159265358979323846, h=1.0/960.0;
    std::vector<int> order{0,32};
    for(int stride=16;stride>=1;stride/=2)for(int i=stride;i<32;i+=2*stride)order.push_back(i);
    std::vector<DynoPoint> points;
    for(int index=0;index<count;++index) {
        const int point=order[index];
        if(cancel->load()) throw ps::io::GenerationCancelled();
        const double rpm=idle+(limit-1.0-idle)*point/32.0;
        double torque=0, compressor_samples=0, turbine_samples=0, fallback_samples=0;
        const bool turbo_simulation=simulated && simulated->turbo_pair && simulated->turbo_pair->maps && simulated->turbo_pair->maps->stages;
        if(engine) {
            engine->reset(ps::drivetrain::EngineState::Running);
            ps::drivetrain::Sensors sensors;
            sensors.omega=rpm*pi/30.0; sensors.ignition=1;
            sensors.clutch_engaged_fraction=1; sensors.gearbox_in_neutral=false;
            ps::drivetrain::ThrottleCommand throttle; throttle.command=1;
            for(int step=0;step<15360;++step) {
                if((step&255)==0 && cancel->load()) throw ps::io::GenerationCancelled();
                auto pre=engine->pre_solve(sensors,throttle,ps::drivetrain::TcuRequest{},h);
                engine->post_solve(sensors.omega,h);
                if(step>=14400) {
                    torque+=pre.tau0-pre.friction_cap_kinetic;
                    compressor_samples+=pre.turbo_compressor_stage_used;
                    turbine_samples+=pre.turbo_turbine_stage_used;
                    fallback_samples+=pre.turbo_stage_map_fallback;
                }
            }
            torque/=960.0;
        } else {
            // These authored WOT curves already specify net crank torque.
            torque=mapped->wot_torque_nm_vs_rpm.evaluate(rpm);
        }
        if(!std::isfinite(torque)) throw std::runtime_error("Non-finite dyno torque");
        points.push_back({rpm,torque,torque*rpm*pi/30000.0,turbo_simulation,
            compressor_samples/960.0,turbine_samples/960.0,fallback_samples/960.0});
        std::sort(points.begin(),points.end(),[](const auto& a,const auto& b){return a.rpm<b.rpm;});
        publish(points,index+1);
    }
    return points;
}

bool ref_fail(std::string* err, const std::string& path, const std::string& msg) {
    if (err) *err = path + ": " + msg;
    return false;
}
bool power_of_two_plus_one(int n) { return n >= 3 && ((n - 1) & (n - 2)) == 0; }
// The bisection evaluation order of an n = 2^k + 1 point grid: ends, middle, quarters, ... (its first 2^j + 1 entries are the
// uniform coarse grid). Same order as measure_dyno's.
std::vector<int> bisection_order(int n) {
    std::vector<int> order{0, n - 1};
    for (int stride = (n - 1) / 2; stride >= 1; stride /= 2)
        for (int i = stride; i < n - 1; i += 2 * stride) order.push_back(i);
    return order;
}
std::string ordinal(int n) {
    const int mod100 = n % 100;
    const char* suffix = (mod100 >= 11 && mod100 <= 13) ? "th" : n % 10 == 1 ? "st" : n % 10 == 2 ? "nd" : n % 10 == 3 ? "rd" : "th";
    return std::to_string(n) + suffix;
}
std::string percent(double fraction) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.0f %%", fraction * 100.0);
    return buffer;
}
bool vehicle_file_is_electrified(const std::string& path) {
    const auto text = detail::read_text_file(path);
    if (!text) return false;
    const auto json = detail::parse_json(*text);
    return json && json->is_object() && detail::powertrain_kind(*json) != "ice";
}
// An option that changes the electric hardware the dyno measures: any motor, battery or fuel-cell stack (a pointer into such a
// component of the vehicle file, or a part file of such a kind). Generic on purpose: whichever option kind or pointer edits
// the hardware (a scale, a preset card, a part choice) reaches the dyno without being listed in the reference file.
bool changes_electric_hardware(const SetupOptionDef& option) {
    static const char* const kinds[] = {"[type=electric_motor]", "[type=battery]", "[type=fuel_cell]", "[type=hydrogen_tank]"};
    for (const auto& pointer : option.pointers)
        for (const char* kind : kinds)
            if (pointer.find(kind) != std::string::npos) return true;
    static const char* const formats[] = {"electric_motor", "battery", "fuel_cell"};
    for (const char* format : formats)
        if (option.part_format.find(format) != std::string::npos) return true;
    return false;
}
} // namespace

const char* to_string(DynoTopology t) {
    switch (t) {
    case DynoTopology::Combustion: return "combustion";
    case DynoTopology::Electric: return "electric";
    case DynoTopology::FuelCellElectric: return "fuel_cell_electric";
    case DynoTopology::Hybrid: return "hybrid";
    }
    return "combustion";
}

std::optional<DynoReference> load_dyno_reference(const std::string& path, std::string* err) {
    const auto text = detail::read_text_file(path);
    if (!text) { ref_fail(err, path, "cannot read the file"); return std::nullopt; }
    const auto root = detail::parse_json(*text);
    if (!root || !root->is_object()) { ref_fail(err, path, "not a JSON object"); return std::nullopt; }
    static const std::set<std::string> known{"format", "source", "battery_soc", "fuel_cell_soc_below_target", "hydrogen_fraction", "drive_mode", "pedal",
        "reference_gear_ratio", "points", "coarse_points", "motor_rpm_min", "motor_rpm_top_fraction", "hybrid_clutch_margin_rpm", "ambient",
        "electrified_setup_options"};
    for (auto it = root->begin(); it != root->end(); ++it)
        if (!known.count(it.key()) && !it.key().ends_with("_help")) { ref_fail(err, path, "unknown key \"" + it.key() + "\""); return std::nullopt; }
    std::string format;
    if (!detail::get_string(*root, "format", format) || format != "rg.garage_dyno_reference/1") {
        ref_fail(err, path, "\"format\" must be \"rg.garage_dyno_reference/1\""); return std::nullopt;
    }
    DynoReference r;
    if (!detail::get_string(*root, "source", r.source) || r.source.empty()) { ref_fail(err, path, "\"source\" is required"); return std::nullopt; }
    const auto number = [&](const nlohmann::json& o, const char* key, double& out, double lo, double hi, const std::string& where) {
        if (!detail::get_number(o, key, out) || !std::isfinite(out) || out < lo || out > hi)
            return ref_fail(err, path, where + "\"" + key + "\" must be a number in [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
        return true;
    };
    double points = 0, coarse = 0;
    if (!number(*root, "battery_soc", r.battery_soc, 0.0, 1.0, "") || !number(*root, "fuel_cell_soc_below_target", r.fuel_cell_soc_below_target, 0.0, 0.5, "") ||
        !number(*root, "hydrogen_fraction", r.hydrogen_fraction, 0.0, 1.0, "") ||
        !number(*root, "pedal", r.pedal, 0.0, 1.0, "") || !number(*root, "reference_gear_ratio", r.reference_gear_ratio, 0.05, 20.0, "") ||
        !number(*root, "points", points, 3, 129, "") || !number(*root, "coarse_points", coarse, 3, 129, "") ||
        !number(*root, "motor_rpm_min", r.motor_rpm_min, 1.0, 1.0e5, "") ||
        !number(*root, "motor_rpm_top_fraction", r.motor_rpm_top_fraction, 0.5, 1.0, "") ||
        !number(*root, "hybrid_clutch_margin_rpm", r.hybrid_clutch_margin_rpm, 0.0, 2000.0, ""))
        return std::nullopt;
    r.points = static_cast<int>(points);
    r.coarse_points = static_cast<int>(coarse);
    if (r.points != points || !power_of_two_plus_one(r.points)) { ref_fail(err, path, "\"points\" must be 2^k + 1 (5, 9, 17, 33, 65 or 129)"); return std::nullopt; }
    if (r.coarse_points != coarse || !power_of_two_plus_one(r.coarse_points) || r.coarse_points > r.points) {
        ref_fail(err, path, "\"coarse_points\" must be 2^j + 1 and not more than \"points\""); return std::nullopt;
    }
    static const char* modes[] = {"eco", "normal", "sport", "racing"};
    if (!detail::get_string(*root, "drive_mode", r.drive_mode)) { ref_fail(err, path, "\"drive_mode\" is required"); return std::nullopt; }
    r.drive_mode_index = -1;
    for (int i = 0; i < 4; ++i) if (r.drive_mode == modes[i]) r.drive_mode_index = i;
    if (r.drive_mode_index < 0) { ref_fail(err, path, "\"drive_mode\" must be eco, normal, sport or racing"); return std::nullopt; }
    const auto ambient = root->find("ambient");
    if (ambient == root->end() || !ambient->is_object()) { ref_fail(err, path, "\"ambient\" object is missing"); return std::nullopt; }
    for (auto it = ambient->begin(); it != ambient->end(); ++it)
        if (it.key() != "pressure_pa" && it.key() != "temperature_k" && !it.key().ends_with("_help")) {
            ref_fail(err, path, "unknown key \"ambient." + it.key() + "\""); return std::nullopt;
        }
    if (!number(*ambient, "pressure_pa", r.ambient_pressure_pa, 30000.0, 120000.0, "ambient ") ||
        !number(*ambient, "temperature_k", r.ambient_temperature_k, 200.0, 350.0, "ambient "))
        return std::nullopt;
    const auto options = root->find("electrified_setup_options");
    if (options == root->end() || !options->is_array()) { ref_fail(err, path, "\"electrified_setup_options\" must be an array of option ids"); return std::nullopt; }
    for (const auto& id : *options) {
        if (!id.is_string() || id.get<std::string>().empty()) { ref_fail(err, path, "\"electrified_setup_options\" must hold non-empty strings"); return std::nullopt; }
        r.electrified_setup_options.push_back(id.get<std::string>());
    }
    return r;
}

VehicleSetup dyno_setup(const SetupOptionTable& table, const CatalogEntry& entry, const VehicleSetup& working, const DynoReference* reference) {
    VehicleSetup out;
    out.vehicle_id = entry.id;
    bool electrified = false;
    bool conversion_chosen = false;
    for (const auto& option : table.options) {
        if (option.part_format != "rg.powertrain_swaps/1") continue;
        const auto value = working.values.find(option.id);
        if (value == working.values.end()) continue;
        conversion_chosen = true;
        if (const auto* choice = std::get_if<std::string>(&value->second)) electrified = *choice != "ice";
    }
    if (!conversion_chosen && reference) electrified = vehicle_file_is_electrified(entry.vehicle_path);
    for (const auto& option : table.options) {
        const bool engine = option.file == "engine" || option.file == "turbo" || option.part_format == "physics_sim.engine/1" ||
                            option.part_format == "physics_sim.turbo_configuration/1";
        const bool conversion = option.part_format == "rg.powertrain_swaps/1";
        const bool gearing = electrified && reference &&
            std::find(reference->electrified_setup_options.begin(), reference->electrified_setup_options.end(), option.id) !=
                reference->electrified_setup_options.end();
        const bool hardware = electrified && changes_electric_hardware(option);
        if (!engine && !conversion && !gearing && !hardware) continue;
        const auto value = working.values.find(option.id);
        if (value != working.values.end()) out.values.emplace(*value);
    }
    return out;
}

std::optional<ElectrifiedDynoPlan> plan_electrified_dyno(const ps::vehicle::VehicleDesc& vehicle, const DynoReference& reference) {
    namespace pd = ps::drivetrain;
    ElectrifiedDynoPlan plan;
    const pd::ElectricMotorDesc* axis_motor = nullptr;
    const pd::GearboxDesc* gearbox = nullptr;
    double idle = 0, limit = 0;
    bool engine = false, fuel_cell = false;
    double stack_target_soc = 1.0; // the lowest target of the car's stacks
    int motor_index = 0;
    for (const auto& c : vehicle.powertrain.components) {
        if (const auto* m = std::get_if<pd::ElectricMotorDesc>(&c.params)) {
            if (!m->crank_assist) {
                if (!axis_motor) { axis_motor = m; plan.axis_motor = motor_index; }
                plan.traction.push_back(motor_index);
                plan.labels.push_back(c.id);
            }
            ++motor_index;
        }
        if (const auto* g = std::get_if<pd::GearboxDesc>(&c.params)) gearbox = g;
        if (const auto* fc = std::get_if<pd::FuelCellDesc>(&c.params)) { fuel_cell = true; stack_target_soc = std::min(stack_target_soc, fc->target_soc); }
        // The sweep stops below the limiter's restore speed: the stepped engine latches a fuel cut on the first overshoot past
        // the cut speed and only restores below rpm - hysteresis, so a point held between the two would read the cut.
        const auto top_of = [](const pd::EngineLimiterDesc& l) {
            return l.type == pd::LimiterType::FuelCut ? l.rpm - l.hysteresis_rpm - 1.0 : l.rpm - 1.0;
        };
        if (const auto* e = std::get_if<pd::SimulatedEngineDesc>(&c.params)) { engine = true; idle = e->idle_rpm; limit = top_of(e->limiter); }
        if (const auto* e = std::get_if<pd::TorqueMapEngineDesc>(&c.params)) { engine = true; idle = e->idle_rpm; limit = top_of(e->limiter); }
    }
    if (plan.traction.empty()) return std::nullopt;
    // A manual-gearbox hybrid's supervisor always runs the auto-clutch; in its Launch law the clutch slips until the gearbox
    // input turns faster than rolling_reengage_rpm, so below that the crank does not turn at the axis speed (it sits at the
    // launch rpm). An automatic, dual-clutch or CVT box is held in the dyno gear with its starting clutch locked (physics_sim
    // AutoTransmission::set_dyno_hold); only its anti-stall dump (a starting clutch opens below launch_anti_stall_rpm) still
    // lets the crank leave the axis speed, so the sweep starts above that.
    double clutch_locks_rpm = 0;
    for (const auto& c : vehicle.controllers) {
        if (const auto* t = std::get_if<pd::ManualTcuDesc>(&c.params)) clutch_locks_rpm = t->auto_clutch.rolling_reengage_rpm;
        if (const auto* t = std::get_if<pd::AutomaticTcuDesc>(&c.params)) {
            const bool starting_clutch = gearbox && (gearbox->type == pd::GearboxType::DualClutch ||
                                                     (gearbox->type == pd::GearboxType::Cvt && !gearbox->converter));
            if (starting_clutch) clutch_locks_rpm = t->launch_anti_stall_rpm;
        }
    }
    plan.topology = engine ? DynoTopology::Hybrid : fuel_cell ? DynoTopology::FuelCellElectric : DynoTopology::Electric;
    plan.battery_soc = fuel_cell ? std::clamp(stack_target_soc - reference.fuel_cell_soc_below_target, 0.0, 1.0) : reference.battery_soc;
    std::string caption = "Hubs · full pedal · battery " + percent(plan.battery_soc);
    if (fuel_cell) caption += " · H₂ " + percent(reference.hydrogen_fraction);
    if (engine) {
        std::string mode = reference.drive_mode;
        if (!mode.empty()) mode[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(mode[0])));
        caption += " · " + mode + " mode";
    }
    const int n = reference.points;
    if (engine && gearbox && !gearbox->forward_ratios.empty()) {
        // Parallel hybrid: the crank's rpm in a stated gear; the motors run at the speed that gear gives them.
        plan.axis = DynoAxis::EngineRpm;
        int best = 0;
        for (int g = 1; g < static_cast<int>(gearbox->forward_ratios.size()); ++g)
            if (std::abs(gearbox->forward_ratios[g] - reference.reference_gear_ratio) <
                std::abs(gearbox->forward_ratios[best] - reference.reference_gear_ratio)) best = g;
        plan.gear = best + 1;
        // A CVT has one forward gear: the held "gear" is the virtual step whose ratio is nearest the reference, and the
        // variator is held at that ratio (physics_sim PowertrainDynoSpec::gear).
        if (gearbox->type == pd::GearboxType::Cvt) {
            char ratio[32];
            std::snprintf(ratio, sizeof ratio, "%.2f", gearbox->forward_ratios[static_cast<std::size_t>(plan.gear - 1)]);
            caption += std::string(" · engine rpm at ratio ") + ratio + ":1";
        } else {
            caption += " · engine rpm in " + ordinal(plan.gear);
        }
        const double start = std::max(idle, clutch_locks_rpm > 0 ? clutch_locks_rpm + reference.hybrid_clutch_margin_rpm : idle);
        if (!(limit > start)) {
            plan.unsupported_reason = "Dyno not available: the engine's speed range is below the clutch's lock-up speed";
            plan.caption = caption;
            return plan;
        }
        for (int i = 0; i < n; ++i) plan.axis_points.push_back(start + (limit - start) * i / (n - 1));
    } else {
        // EV, fuel-cell EV, or a hybrid without a gearbox (series): the axis motor's own rpm.
        plan.axis = DynoAxis::MotorRpm;
        caption += " · " + plan.labels.front() + " rpm";
        const double top = axis_motor->max_rpm * reference.motor_rpm_top_fraction;
        if (!(top > reference.motor_rpm_min)) {
            plan.unsupported_reason = "Dyno not available: the motor's speed range is below the reference start speed";
            plan.caption = caption;
            return plan;
        }
        for (int i = 0; i < n; ++i) plan.axis_points.push_back(reference.motor_rpm_min + (top - reference.motor_rpm_min) * i / (n - 1));
    }
    plan.caption = caption;
    return plan;
}

std::vector<DynoPoint> measure_electrified_dyno(const ps::vehicle::VehicleDesc& vehicle, const DynoReference& reference,
    const ElectrifiedDynoPlan& plan, ps::jobs::JobSystem* pool, const std::atomic<bool>* cancel,
    const std::function<void(const std::vector<DynoPoint>&, int)>& publish) {
    namespace pd = ps::drivetrain;
    if (!plan.unsupported_reason.empty()) return {};
    pd::PowertrainDynoSpec spec;
    spec.axis = plan.axis == DynoAxis::EngineRpm ? pd::PowertrainDynoAxis::EngineRpm : pd::PowertrainDynoAxis::MotorRpm;
    spec.motor_index = plan.axis_motor;
    spec.gear = plan.axis == DynoAxis::EngineRpm ? plan.gear : 1;
    spec.pedal = reference.pedal;
    spec.battery_soc = plan.battery_soc;
    spec.hydrogen_fraction = reference.hydrogen_fraction;
    spec.drive_mode = reference.drive_mode_index;
    spec.ambient.pressure_pa = reference.ambient_pressure_pa;
    spec.ambient.temperature_k = reference.ambient_temperature_k;
    const auto order = bisection_order(static_cast<int>(plan.axis_points.size()));
    const std::size_t coarse = std::min<std::size_t>(order.size(), static_cast<std::size_t>(reference.coarse_points));
    std::vector<DynoPoint> points;
    const auto convert = [&](const pd::PowertrainDynoPoint& p) {
        constexpr double pi = 3.14159265358979323846;
        const double omega = p.axis_value * pi / 30.0;
        DynoPoint d;
        d.rpm = p.axis_value;
        d.power_kw = p.system_power_w / 1000.0;
        d.torque_nm = p.system_torque_nm;
        d.engine_torque_nm = p.engine_torque_nm;
        d.engine_power_kw = p.engine_power_w / 1000.0;
        d.engine_rpm = p.engine_rpm;
        double electric_w = 0;
        for (const int index : plan.traction) {
            const auto& m = p.motors.at(static_cast<std::size_t>(index));
            electric_w += m.power_w;
            d.motors.push_back({m.rpm, m.torque_nm, m.power_w / 1000.0,
                                m.limit == pd::MotorLimit::BatteryDischarge || m.limit == pd::MotorLimit::Empty});
        }
        d.electric_power_kw = electric_w / 1000.0;
        d.electric_torque_nm = omega > 0 ? electric_w / omega : 0.0;
        d.fuel_cell_power_kw = p.fuel_cell_power_w / 1000.0;
        d.battery_power_kw = p.battery_power_w / 1000.0;
        d.battery_limited = p.battery_limited;
        d.engine_running = p.engine_running;
        d.converged = p.converged;
        d.gear = p.gear;
        if (!std::isfinite(d.power_kw) || !std::isfinite(d.torque_nm)) throw std::runtime_error("Non-finite powertrain dyno output");
        return d;
    };
    const auto run = [&](std::size_t from, std::size_t to) {
        spec.points.clear();
        for (std::size_t i = from; i < to; ++i) spec.points.push_back(plan.axis_points[static_cast<std::size_t>(order[i])]);
        const auto result = pd::run_powertrain_dyno(vehicle, nullptr, spec, pool, cancel);
        for (const auto& p : result.points) points.push_back(convert(p));
        std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.rpm < b.rpm; });
        publish(points, static_cast<int>(to));
    };
    run(0, coarse);
    if (coarse < order.size()) run(coarse, order.size());
    return points;
}

GarageDyno::GarageDyno():thread_([this]{run();}) {}
GarageDyno::~GarageDyno() {
    { std::lock_guard lock(mutex_); stopping_=true; if(cancel_) cancel_->store(true); }
    wake_.notify_one(); thread_.join();
}
void GarageDyno::request(DynoRequest request) {
    std::lock_guard lock(mutex_);
    if(requested_key_==request.key) return;
    if(cancel_) cancel_->store(true);
    if(result_.vehicle_id!=request.entry.id) result_=DynoResult{};
    requested_key_=request.key;
    ++revision_;
    result_.vehicle_id=request.entry.id;
    result_.busy=true; result_.error.clear(); result_.progress=0; result_.provisional_points.clear();
    pending_=std::move(request);
    wake_.notify_one();
}
void GarageDyno::cancel() {
    std::lock_guard lock(mutex_);
    ++revision_; requested_key_.clear(); pending_.reset();
    if(cancel_) cancel_->store(true);
    result_.busy=false; result_.provisional_points.clear();
}
DynoResult GarageDyno::result() const { std::lock_guard lock(mutex_); return result_; }
void GarageDyno::run() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
#endif
    ps::jobs::JobSystemOptions pool_options; pool_options.background_priority=true;
    // Leave four logical processors for rendering, audio and other work.
    int workers=static_cast<int>(std::max(1u,std::thread::hardware_concurrency()>4 ? std::thread::hardware_concurrency()-4 : 1u));
    if(const char* value=std::getenv("RG_ENGINE_MAP_WORKERS")) {
        const std::string_view text(value);
        int configured=0;
        const auto parsed=std::from_chars(text.data(),text.data()+text.size(),configured);
        if(parsed.ec==std::errc{} && parsed.ptr==text.data()+text.size() && configured>=1 && configured<=64)
            workers=configured;
    }
    ps::jobs::JobSystem pool(workers,pool_options);
    for(;;) {
        DynoRequest request;
        std::uint64_t revision;
        std::shared_ptr<std::atomic<bool>> cancel;
        { std::unique_lock lock(mutex_);
          wake_.wait(lock,[this]{return stopping_ || pending_.has_value();});
          if(stopping_) return;
          // Wait for slider edits to settle before starting another cold map.
          auto observed=revision_;
          while(wake_.wait_for(lock,std::chrono::milliseconds(350),[&]{return stopping_ || revision_!=observed;})) {
              if(stopping_)return;
              observed=revision_;
          }
          if(!pending_)continue;
          request=std::move(*pending_); pending_.reset(); revision=revision_;
          cancel=std::make_shared<std::atomic<bool>>(false); cancel_=cancel;
        }
        auto progress=[&](int value) {
            std::lock_guard lock(mutex_);
            if(revision==revision_) result_.progress=value;
        };
        std::vector<DynoPoint> points;
        std::string error;
        DynoResult meta; // the descriptive fields of the finished result
        try {
            ScopedWorkDir work(request.context.work_root+"_dyno","pull");
            const auto material=materialise_setup(request.entry,request.table,request.context,request.setup,work.path());
            if(!material.ok) throw std::runtime_error(material.error);
            // An electrified car's provisional curve is the system one, never the bare engine's.
            const bool electrified_file=vehicle_file_is_electrified(material.vehicle_path);
            ps::io::EngineMapOptions maps;
            maps.cache_dir=request.context.engine_map_cache_dir;
            maps.cancel=cancel.get(); maps.pool=&pool;
            maps.on_progress=[&](std::size_t done,std::size_t total){progress(int(60*done/std::max(std::size_t(1),total)));};
            int coarse_stage=0;
            maps.on_partial_engine=[&](const auto& coarse) {
                if(electrified_file)return;
                const bool first=(coarse_stage++==0);
                measure_dyno(&coarse,nullptr,9,cancel.get(),[&](const auto& partial,int done) {
                    if(!first && done<9)return;
                    std::lock_guard lock(mutex_);
                    if(revision==revision_ && !cancel->load())result_.provisional_points=partial;
                });
            };
            auto desc=rg::load_vehicle(material.vehicle_path,maps);
            if(request.context.turbo_map_extension) // the dyno uses the same extended (cached) stage map as the drive
                (void)rg::extend_turbo_maps(desc,material.vehicle_path,request.context.engine_map_cache_dir,*request.context.turbo_map_extension,&pool,cancel.get());
            const ps::drivetrain::SimulatedEngineDesc* simulated=nullptr;
            const ps::drivetrain::TorqueMapEngineDesc* mapped=nullptr;
            for(const auto& component:desc.powertrain.components) {
                if(auto e=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&component.params)) simulated=e;
                if(auto e=std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&component.params)) mapped=e;
            }
            std::optional<ElectrifiedDynoPlan> plan;
            if(request.reference) plan=plan_electrified_dyno(desc,*request.reference);
            else for(const auto& component:desc.powertrain.components)
                if(auto m=std::get_if<ps::drivetrain::ElectricMotorDesc>(&component.params); m && !m->crank_assist)
                    throw std::runtime_error("Electric dyno reference unavailable (data/garage/dyno_reference.json)");
            if(plan) {
                meta.topology=plan->topology; meta.axis=plan->axis; meta.reference_gear=plan->gear;
                meta.motor_labels=plan->labels; meta.caption=plan->caption; meta.unsupported_reason=plan->unsupported_reason;
                const int total=static_cast<int>(plan->axis_points.size());
                points=measure_electrified_dyno(desc,*request.reference,*plan,&pool,cancel.get(),[&](const auto& partial,int done) {
                    std::lock_guard lock(mutex_);
                    if(revision!=revision_ || cancel->load())return;
                    if(partial.size()>=result_.provisional_points.size())result_.provisional_points=partial;
                    result_.progress=60+done*40/std::max(1,total);
                });
            } else {
                if(!simulated && !mapped) throw std::runtime_error("No engine or traction motor in this setup");
                points=measure_dyno(simulated,mapped,33,cancel.get(),[&](const auto& partial,int done) {
                    std::lock_guard lock(mutex_);
                    if(revision!=revision_ || cancel->load())return;
                    if(partial.size()>=result_.provisional_points.size())result_.provisional_points=partial;
                    result_.progress=60+done*40/33;
                });
            }
        } catch(const std::exception& e) { error=e.what(); }
        { std::lock_guard lock(mutex_);
          if(revision!=revision_ || cancel->load()) continue;
          result_.busy=false; result_.error=error;result_.provisional_points.clear();
          if(error.empty()) {
              result_.points=std::move(points); result_.revision=revision;
              result_.key=request.key; result_.progress=100;
              result_.topology=meta.topology; result_.axis=meta.axis; result_.reference_gear=meta.reference_gear;
              result_.motor_labels=std::move(meta.motor_labels); result_.caption=std::move(meta.caption);
              result_.unsupported_reason=std::move(meta.unsupported_reason);
          }
        }
    }
}
} // namespace rg
