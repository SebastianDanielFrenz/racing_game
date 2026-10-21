// rg/upgrade_hints.h: the whole-car figures behind the garage's upgrade hints and their offset rule (candidate whole car minus
// the CURRENT whole car), the cache and the cancellation. Every figure is checked against an independent computation.
//
// Recorded sabotages (each made the named test fail, then reverted):
//   S1 static_corner_loads without the pitch-moment row (equal share per wheel)      -> "static corner loads ..." fails
//   S2 tyre peak scan over kappa in [0, 0.05] only                                    -> "tyre figures ..." fails
//   S3 the hint taken against the stock setup instead of the current one              -> "the offset rule ..." fails
//   S4 the nitrous flag ignored (kit never armed)                                      -> "N2O ..." fails
//   S5 request() queues a job per call (no replacement of the pending one)             -> "newest request wins" fails
//   S6 cache key without the nitrous flag                                              -> "N2O ..." fails
//   S7 Garage::set_option re-applies an option when its value is already installed       -> "the installed option ..." (both cases) fail
//      (found by the hybrid case: re-selecting the installed powertrain reset the engine/turbo/gearbox/fuel choices)
//   S8 Garage::edits_installed always false                                              -> "the installed option ..." fails
//   S9 "battery_kwh" back in the powertrain_install metrics                              -> "a powertrain swap shows ..." fails
//   S10 request_current answers from the stock setup (key of an empty setup)             -> "the specification pane reports ..." fails
//   S11 Garage::stats keeps the stock gear count after a gearbox swap                     -> "... the gears of the installed gearbox" fails
#include "rg/garage.h"
#include "rg/upgrade_hints.h"
#include "rg/turbo_map_extension.h"
#include "rg/vehicle_load.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/tyre_model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <deque>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <thread>

namespace {

namespace fs = std::filesystem;
using Catch::Approx;
using namespace rg;
const std::string kRoot = RG_SOURCE_DIR;

struct TempDirs {
    std::string base;
    TempDirs() {
        base = (fs::temp_directory_path() / ("rg_hints_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)))
                   .generic_string();
        fs::create_directories(base + "/user");
        fs::create_directories(base + "/work");
    }
    ~TempDirs() {
        std::error_code ec;
        fs::remove_all(fs::path(base), ec);
    }
};

const UpgradeBadgeConfig& badge_config() {
    static const UpgradeBadgeConfig cfg = [] {
        std::string err;
        auto loaded = load_upgrade_badge_config(kRoot + "/data/garage/upgrade_badges.json", &err);
        if (!loaded) FAIL(err);
        return *loaded;
    }();
    return cfg;
}

struct Rig {
    TempDirs dirs;
    std::unique_ptr<Garage> garage;
    CatalogEntry entry;
    explicit Rig(const std::string& car) {
        GarageConfig c;
        c.repo_root = kRoot;
        c.catalog_path = kRoot + "/data/vehicles/catalog.json";
        c.options_path = kRoot + "/data/vehicles/setup_options.json";
        c.set_path = kRoot + "/data/garage/garage_set.json";
        c.tyre_families_path = kRoot + "/data/tyres/families.json";
        c.user_dir = dirs.base + "/user";
        c.work_root = dirs.base + "/work";
        c.engine_map_cache_dir = kRoot + "/out/godot_engine_cache";
        std::string err;
        garage = Garage::open(c, &err);
        INFO(err);
        REQUIRE(garage != nullptr);
        REQUIRE(garage->begin_edit(car, &err));
        entry = *garage->catalog().find(car);
    }
    HintBase base() const { return HintBase{entry, garage->options(), garage->context_for(entry)}; }
    // A stable copy: the garage rebuilds its model on every edit, which would invalidate pointers into it.
    const OptionView* view(const std::string& id) const {
        for (const OptionView& v : garage->model().options) {
            if (v.def.id == id) {
                views_.push_back(v);
                return &views_.back();
            }
        }
        return nullptr;
    }
    mutable std::deque<OptionView> views_;
    // The setup with `edits` applied to the working copy, as the real clicks would leave it.
    VehicleSetup with(const std::vector<std::pair<std::string, SetupValue>>& edits) {
        std::string err;
        auto s = garage->candidate_setup(edits, &err);
        INFO(err);
        REQUIRE(s.has_value());
        return *s;
    }
};

FigureEnv env_with(const UpgradeBadgeConfig& cfg) {
    FigureEnv env = make_figure_env(cfg.references);
    REQUIRE(env.tyre_ok);
    return env;
}

SetupFigures figures_of(const Rig& rig, const VehicleSetup& setup, std::set<UpgradeQuantity> wanted, bool armed = false) {
    return compute_setup_figures(rig.base(), setup, wanted, armed, env_with(badge_config()));
}

// A UpgradeHints with the debounce shortened for tests.
std::unique_ptr<UpgradeHints> make_hints(int debounce_ms = 0, const std::string& quality = "") {
    UpgradeBadgeConfig cfg = badge_config();
    cfg.references.dyno_debounce_ms = debounce_ms;
    if (!quality.empty()) cfg.references.dyno_quality = quality;
    return std::make_unique<UpgradeHints>(cfg, 0);
}

HintAnswer wait_answer(UpgradeHints& hints, const VehicleSetup& candidate, const std::vector<UpgradeQuantity>& metrics, bool armed = false,
                       int timeout_ms = 600000) {
    hints.request(candidate, metrics, armed);
    REQUIRE(hints.wait_idle(timeout_ms));
    return hints.request(candidate, metrics, armed);
}

} // namespace

// ---- the static corner loads ----------------------------------------------------------------------------------------------

TEST_CASE("hints: static corner loads carry the weight and have no moment", "[upgrade_hints]") {
    const double w = 15000.0;
    // A 60/40 car: front axle 1.2 m ahead of the centre of mass, rear axle 1.8 m behind (x forward, y left).
    const std::vector<std::pair<double, double>> wheels = {{1.2, 0.8}, {1.2, -0.8}, {-1.8, 0.8}, {-1.8, -0.8}};
    const auto loads = static_corner_loads(wheels, w);
    REQUIRE(loads.size() == 4);
    double sum = 0.0, mx = 0.0, my = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        sum += loads[i];
        mx += loads[i] * wheels[i].first;
        my += loads[i] * wheels[i].second;
    }
    CHECK(sum == Approx(w));
    CHECK(mx == Approx(0.0).margin(1.0e-6));
    CHECK(my == Approx(0.0).margin(1.0e-6));
    // The hand solution: the front axle carries 1.8/3.0 = 60 % of the weight, shared equally left and right.
    CHECK(loads[0] == Approx(0.30 * w));
    CHECK(loads[1] == Approx(0.30 * w));
    CHECK(loads[2] == Approx(0.20 * w));
    CHECK(loads[3] == Approx(0.20 * w));
    // Three wheels: the unique solution. Degenerate layouts fall back to an equal share.
    const auto three = static_corner_loads({{1.0, 0.0}, {-1.0, 1.0}, {-1.0, -1.0}}, 3000.0);
    CHECK(three[0] == Approx(1500.0));
    CHECK(three[1] + three[2] == Approx(1500.0));
    const auto line = static_corner_loads({{1.0, 0.0}, {0.0, 0.0}, {-1.0, 0.0}}, 3000.0);
    CHECK(line[0] == Approx(1000.0));
    CHECK(static_corner_loads({}, 1.0).empty());
}

// ---- the tyres ------------------------------------------------------------------------------------------------------------

TEST_CASE("hints: tyre figures equal a fine-grid computation with independently solved corner loads", "[upgrade_hints][tyres]") {
    Rig rig("car_sedan");
    TempDirs dirs;
    const VehicleSetup stock = rig.garage->working();
    const MaterialisedSetup m = materialise_setup(rig.entry, rig.garage->options(), rig.garage->context_for(rig.entry), stock, dirs.base + "/m");
    REQUIRE(m.ok);
    ps::io::EngineMapOptions maps;
    maps.generate = false;
    const ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, maps);
    const double mass = m.compiled.parts.ok ? m.compiled.parts.total_kg : rig.entry.chassis.mass_kg + m.compiled.chassis_mass_delta_kg;

    std::string err;
    const auto ref = load_tyre_reference(badge_config().references, &err);
    INFO(err);
    REQUIRE(ref.has_value());
    CHECK(ref->lambda_mu > 0.5);
    const TyreFigures got = tyre_figures(desc, mass, *ref);

    // The independent reference: axle loads from the wheelbase geometry by hand (two axles, left/right equal), a 1e-4 grid.
    double front_x = -1.0e9, rear_x = 1.0e9;
    for (const auto& w : desc.wheels) {
        front_x = std::max(front_x, w.attachment_local.x);
        rear_x = std::min(rear_x, w.attachment_local.x);
    }
    const double weight = mass * 9.80665;
    double fx_sum = 0.0, fy_sum = 0.0;
    for (const auto& w : desc.wheels) {
        const bool front = std::fabs(w.attachment_local.x - front_x) < 1.0e-6;
        const double axle_share = front ? (0.0 - rear_x) / (front_x - rear_x) : (front_x - 0.0) / (front_x - rear_x);
        const double fz = weight * axle_share / 2.0;
        const auto model = ps::vehicle::create_pacejka_tyre_model(w.tyre);
        ps::vehicle::TyreSlipInput in;
        in.fz = fz;
        in.gamma = w.static_camber;
        in.vx = ref->speed_m_s;
        in.lambda_mu = ref->lambda_mu;
        double best_fx = 0.0, best_fy = 0.0;
        for (int i = -10000; i <= 10000; ++i) {
            ps::vehicle::TyreSlipInput s = in;
            s.kappa = i * 1.0e-4;
            best_fx = std::max(best_fx, std::fabs(model->evaluate(s).fx));
        }
        for (int i = -6000; i <= 6000; ++i) {
            ps::vehicle::TyreSlipInput s = in;
            s.alpha = i * 1.0e-4;
            best_fy = std::max(best_fy, std::fabs(model->evaluate(s).fy));
        }
        fx_sum += best_fx;
        fy_sum += best_fy;
    }
    CHECK(got.weight_n == Approx(weight));
    CHECK(got.peak_fx_n == Approx(fx_sum).epsilon(2.0e-4));
    CHECK(got.peak_fy_n == Approx(fy_sum).epsilon(2.0e-4));
    CHECK(got.grip_longitudinal_g == Approx(fx_sum / weight).epsilon(2.0e-4));
    CHECK(got.grip_lateral_g == Approx(fy_sum / weight).epsilon(2.0e-4));
    // A road tyre on dry asphalt grips about one g, and rolls with a coefficient of order 1 %.
    CHECK(got.grip_longitudinal_g > 0.8);
    CHECK(got.grip_longitudinal_g < 1.6);
    CHECK(got.grip_lateral_g > 0.8);
    CHECK(got.grip_lateral_g < 1.6);
    CHECK(got.rolling_g > 0.005);
    CHECK(got.rolling_g < 0.05);
    // The corner loads of the car sum to its weight.
    double sum = 0.0;
    for (double f : got.corner_loads_n) sum += f;
    CHECK(sum == Approx(weight));
    std::printf("HINTS car_sedan tyres: mass %.1f kg  grip long %.4f g lat %.4f g  rolling %.5f g\n", mass, got.grip_longitudinal_g, got.grip_lateral_g,
                got.rolling_g);
    std::fflush(stdout);
}

TEST_CASE("hints: a wider, stickier tyre raises the grip figures and the rolling resistance", "[upgrade_hints][tyres]") {
    Rig rig("car_sedan");
    const OptionView* front = rig.view("tyre_front");
    REQUIRE(front != nullptr);
    REQUIRE(front->choices.size() >= 2);
    std::printf("HINTS car_sedan tyre choices: %zu, first %s, last %s\n", front->choices.size(), front->choices.front().c_str(), front->choices.back().c_str());
    // Pick the narrowest and the widest choice by the numbers in the id (compound_WIDTH_ASPECTrRIM).
    const auto width = [](const std::string& id) {
        const auto a = id.find('_');
        return a == std::string::npos ? 0 : std::atoi(id.c_str() + a + 1);
    };
    std::string narrow = front->choices.front(), wide = front->choices.front();
    for (const std::string& c : front->choices) {
        if (width(c) < width(narrow)) narrow = c;
        if (width(c) > width(wide)) wide = c;
    }
    REQUIRE(width(wide) > width(narrow));
    const VehicleSetup a = rig.with({{"tyre_front", narrow}, {"tyre_rear", narrow}});
    const VehicleSetup b = rig.with({{"tyre_front", wide}, {"tyre_rear", wide}});
    const std::set<UpgradeQuantity> wanted = {UpgradeQuantity::GripLongitudinal, UpgradeQuantity::GripLateral, UpgradeQuantity::RollingResistance};
    const SetupFigures fa = figures_of(rig, a, wanted);
    const SetupFigures fb = figures_of(rig, b, wanted);
    INFO(fa.error << " / " << fb.error);
    REQUIRE(fa.error.empty());
    REQUIRE(fb.error.empty());
    REQUIRE(fa.figures.size() == 3);
    REQUIRE(fb.figures.size() == 3);
    std::printf("HINTS tyres %s -> %s: grip long %+.4f g, lat %+.4f g, rolling %+.1f %%\n", narrow.c_str(), wide.c_str(),
                fb.figures.at(UpgradeQuantity::GripLongitudinal) - fa.figures.at(UpgradeQuantity::GripLongitudinal),
                fb.figures.at(UpgradeQuantity::GripLateral) - fa.figures.at(UpgradeQuantity::GripLateral),
                (fb.figures.at(UpgradeQuantity::RollingResistance) / fa.figures.at(UpgradeQuantity::RollingResistance) - 1.0) * 100.0);
    std::fflush(stdout);
    CHECK(fb.figures.at(UpgradeQuantity::GripLongitudinal) > fa.figures.at(UpgradeQuantity::GripLongitudinal));
    CHECK(fb.figures.at(UpgradeQuantity::GripLateral) > fa.figures.at(UpgradeQuantity::GripLateral));
    CHECK(fb.figures.at(UpgradeQuantity::RollingResistance) > fa.figures.at(UpgradeQuantity::RollingResistance));
}

// ---- weight and rpm tolerance (compile only) -----------------------------------------------------------------------------

TEST_CASE("hints: the weight figure is the part-mass tree total and a part's hint is its mass difference", "[upgrade_hints][weight]") {
    Rig rig("car_sedan");
    const VehicleSetup stock = rig.garage->working();
    const CompiledSetup stock_compiled = compile_setup(rig.entry, rig.garage->options(), rig.garage->context_for(rig.entry), stock);
    REQUIRE(stock_compiled.ok);
    REQUIRE(stock_compiled.parts.ok);
    const SetupFigures f0 = figures_of(rig, stock, {UpgradeQuantity::Weight});
    REQUIRE(f0.figures.count(UpgradeQuantity::Weight) == 1);
    CHECK(f0.figures.at(UpgradeQuantity::Weight) == Approx(stock_compiled.parts.total_kg));

    const OptionView* spare = rig.view("trim_spare_wheel");
    REQUIRE(spare != nullptr);
    const SetupPart* removed = nullptr;
    const SetupPart* installed_part = nullptr;
    for (const SetupPart& p : spare->parts) {
        if (p.removed) removed = &p;
        if (p.mass_kg >= 0.0 && !p.removed && installed_part == nullptr) installed_part = &p;
    }
    REQUIRE(removed != nullptr);
    REQUIRE(installed_part != nullptr);
    const VehicleSetup without = rig.with({{"trim_spare_wheel", removed->id}});
    const SetupFigures f1 = figures_of(rig, without, {UpgradeQuantity::Weight});
    // The hint = the candidate whole car minus the current one = the removed mass.
    const double delta = f1.figures.at(UpgradeQuantity::Weight) - f0.figures.at(UpgradeQuantity::Weight);
    CHECK(delta < 0.0);
    CHECK(delta == Approx(removed->mass_kg - installed_part->mass_kg).margin(1.0e-9));
    const auto badges = upgrade_badges(badge_config(), "Weight reduction", f1.figures, f0.figures);
    REQUIRE(badges.size() == 1);
    CHECK_FALSE(badges[0].worse);
}

TEST_CASE("hints: rpm tolerance is the weakest internal's limit and follows an internals swap", "[upgrade_hints][internals]") {
    Rig rig("car_hyper");
    const OptionView* springs = rig.view("engine_valve_springs");
    const OptionView* rods = rig.view("engine_rods");
    if (springs == nullptr || rods == nullptr || springs->parts.size() < 2) SKIP("the car has no swappable internals");
    const VehicleSetup stock = rig.garage->working();
    const auto weakest = [&](const VehicleSetup& s) {
        const CompiledSetup c = compile_setup(rig.entry, rig.garage->options(), rig.garage->context_for(rig.entry), s);
        REQUIRE(c.ok);
        REQUIRE(c.internals.has_value());
        double lowest = 0.0;
        for (const InternalPartBuild& p : c.internals->parts) {
            if (p.limit_rpm > 0.0 && (lowest <= 0.0 || p.limit_rpm < lowest)) lowest = p.limit_rpm;
        }
        return lowest;
    };
    const SetupFigures base = figures_of(rig, stock, {UpgradeQuantity::RpmTolerance});
    REQUIRE(base.figures.count(UpgradeQuantity::RpmTolerance) == 1);
    CHECK(base.figures.at(UpgradeQuantity::RpmTolerance) == Approx(weakest(stock)));
    int checked = 0;
    for (const SetupPart& p : rods->parts) {
        if (!p.fits || p.id.empty()) continue;
        const VehicleSetup cand = rig.with({{"engine_rods", p.id}});
        const SetupFigures f = figures_of(rig, cand, {UpgradeQuantity::RpmTolerance});
        REQUIRE(f.figures.count(UpgradeQuantity::RpmTolerance) == 1);
        CHECK(f.figures.at(UpgradeQuantity::RpmTolerance) == Approx(weakest(cand)));
        std::printf("HINTS car_hyper rods %s: weakest %.0f rpm (stock %.0f)\n", p.id.c_str(), f.figures.at(UpgradeQuantity::RpmTolerance),
                    base.figures.at(UpgradeQuantity::RpmTolerance));
        ++checked;
    }
    std::fflush(stdout);
    CHECK(checked >= 1);
}

// ---- the offset rule, the cache and the cancellation ----------------------------------------------------------------------

TEST_CASE("hints: the offset rule - the hint is against the CURRENT configuration, not stock", "[upgrade_hints][offset]") {
    Rig rig("car_sedan");
    const OptionView* front = rig.view("tyre_front");
    REQUIRE(front != nullptr);
    REQUIRE(front->choices.size() >= 3);
    const std::string stock_tyre = std::get<std::string>(rig.garage->current_value("tyre_front"));
    // A non-stock current configuration: different tyres AND a removed spare wheel; candidate: yet another tyre.
    std::string other = front->choices.front(), cand = front->choices.back();
    for (const std::string& c : front->choices) {
        if (c != stock_tyre && c != cand) {
            other = c;
            break;
        }
    }
    REQUIRE(other != cand);
    REQUIRE(other != stock_tyre);
    const OptionView* spare = rig.view("trim_spare_wheel");
    REQUIRE(spare != nullptr);
    std::string removed_id;
    for (const SetupPart& p : spare->parts) {
        if (p.removed) removed_id = p.id;
    }
    const VehicleSetup stock = rig.garage->working();
    const VehicleSetup current = rig.with({{"tyre_front", other}, {"tyre_rear", other}, {"trim_spare_wheel", removed_id}});
    // The candidate is swapped into the CURRENT car: its edits are made on top of the current setup.
    VehicleSetup candidate = current;
    candidate.values["tyre_front"] = cand;
    candidate.values["tyre_rear"] = cand;

    const std::set<UpgradeQuantity> wanted = {UpgradeQuantity::GripLongitudinal, UpgradeQuantity::RollingResistance, UpgradeQuantity::Weight};
    const SetupFigures fc = figures_of(rig, current, wanted);
    const SetupFigures fn = figures_of(rig, candidate, wanted);
    const SetupFigures fs = figures_of(rig, stock, wanted);
    REQUIRE(fc.error.empty());
    REQUIRE(fn.error.empty());
    REQUIRE(fs.error.empty());

    auto hints = make_hints();
    hints->set_base(rig.base(), current);
    const HintAnswer a = wait_answer(*hints, candidate, {UpgradeQuantity::GripLongitudinal, UpgradeQuantity::RollingResistance, UpgradeQuantity::Weight});
    CHECK(a.pending.empty());
    CHECK(a.error.empty());
    for (UpgradeQuantity q : wanted) {
        INFO(upgrade_quantity_name(q));
        REQUIRE(a.candidate.count(q) == 1);
        REQUIRE(a.installed.count(q) == 1);
        CHECK(a.candidate.at(q) == Approx(fn.figures.at(q)));
        CHECK(a.installed.at(q) == Approx(fc.figures.at(q))); // the baseline is the CURRENT car, not stock
        CHECK(a.installed.at(q) != Approx(fs.figures.at(q)));
    }
    // The shown offset is candidate - current; against stock it would read differently.
    const auto badges = upgrade_badges(badge_config(), "Tyres", a.candidate, a.installed);
    REQUIRE_FALSE(badges.empty());
    const UpgradeBadge& g = badges.front();
    REQUIRE(g.quantity == UpgradeQuantity::GripLongitudinal);
    CHECK(g.delta == Approx(fn.figures.at(UpgradeQuantity::GripLongitudinal) - fc.figures.at(UpgradeQuantity::GripLongitudinal)).margin(1.0e-9));
    CHECK(std::fabs(g.delta - (fn.figures.at(UpgradeQuantity::GripLongitudinal) - fs.figures.at(UpgradeQuantity::GripLongitudinal))) > 1.0e-4);
}

TEST_CASE("hints: a repeated request is answered from the cache and a new current setup drops the old work", "[upgrade_hints][cache]") {
    Rig rig("car_sedan");
    const OptionView* front = rig.view("tyre_front");
    REQUIRE(front != nullptr);
    REQUIRE(front->choices.size() >= 2);
    const VehicleSetup stock = rig.garage->working();
    VehicleSetup candidate = stock;
    candidate.values["tyre_front"] = front->choices.back();
    candidate.values["tyre_rear"] = front->choices.back();
    auto hints = make_hints();
    hints->set_base(rig.base(), stock);
    const std::vector<UpgradeQuantity> metrics = {UpgradeQuantity::GripLongitudinal};

    const HintAnswer first = hints->request(candidate, metrics, false);
    CHECK_FALSE(first.pending.empty()); // nothing cached yet: a placeholder
    REQUIRE(hints->wait_idle(120000));
    const HintStats after_first = hints->stats();
    CHECK(after_first.jobs_started == 1);
    CHECK(after_first.jobs_finished == 1);
    CHECK(after_first.figure_sets_computed == 2); // the current car and the candidate

    const HintAnswer second = hints->request(candidate, metrics, false);
    CHECK(second.pending.empty());
    CHECK(second.candidate.size() == 1);
    const HintStats after_second = hints->stats();
    CHECK(after_second.jobs_started == 1); // no new work
    CHECK(after_second.cache_hits == after_first.cache_hits + 1);

    // Another candidate on the same current car reuses the current car's figures: one new figure set only.
    VehicleSetup other = stock;
    other.values["trim_spare_wheel"] = std::string("removed");
    const OptionView* spare = rig.view("trim_spare_wheel");
    REQUIRE(spare != nullptr);
    for (const SetupPart& p : spare->parts) {
        if (p.removed) other.values["trim_spare_wheel"] = p.id;
    }
    hints->request(other, metrics, false);
    REQUIRE(hints->wait_idle(120000));
    CHECK(hints->stats().figure_sets_computed == after_first.figure_sets_computed + 1);

    // A new current setup: the old cache entries stay valid for their own setup, but queued work for the old baseline is dropped.
    hints->set_base(rig.base(), candidate);
    const HintAnswer rebased = hints->request(stock, metrics, false);
    // `stock` and `candidate` were both computed before: answered at once, now with the roles swapped.
    CHECK(rebased.pending.empty());
    CHECK(rebased.installed.at(UpgradeQuantity::GripLongitudinal) == Approx(second.candidate.at(UpgradeQuantity::GripLongitudinal)));
    CHECK(rebased.candidate.at(UpgradeQuantity::GripLongitudinal) == Approx(second.installed.at(UpgradeQuantity::GripLongitudinal)));
}

TEST_CASE("hints: the newest request wins and a flick over the tiles starts one job at most", "[upgrade_hints][cache]") {
    Rig rig("car_sedan");
    const VehicleSetup stock = rig.garage->working();
    const OptionView* spare = rig.view("trim_spare_wheel");
    const OptionView* audio = rig.view("trim_audio");
    REQUIRE(spare != nullptr);
    REQUIRE(audio != nullptr);
    std::string spare_removed, audio_removed;
    for (const SetupPart& p : spare->parts) {
        if (p.removed) spare_removed = p.id;
    }
    for (const SetupPart& p : audio->parts) {
        if (p.removed) audio_removed = p.id;
    }
    VehicleSetup a = stock, b = stock;
    a.values["trim_spare_wheel"] = spare_removed;
    b.values["trim_audio"] = audio_removed;
    auto hints = make_hints(300); // a hover must rest 300 ms
    hints->set_base(rig.base(), stock);
    hints->request(a, {UpgradeQuantity::GripLongitudinal}, false);
    hints->request(b, {UpgradeQuantity::GripLongitudinal}, false); // the pointer moved on within the debounce
    REQUIRE(hints->wait_idle(60000));
    const HintStats s = hints->stats();
    CHECK(s.jobs_started == 1); // only the last tile was worked on
    CHECK(hints->request(b, {UpgradeQuantity::GripLongitudinal}, false).pending.empty());
    CHECK_FALSE(hints->request(a, {UpgradeQuantity::GripLongitudinal}, false).pending.empty()); // a was never computed (and is queued now)
    hints->cancel();
    CHECK(hints->wait_idle(60000));
}

TEST_CASE("hints: weight arrives at once, the expensive figures wait for the debounce", "[upgrade_hints][cache]") {
    Rig rig("car_sedan");
    const VehicleSetup stock = rig.garage->working();
    const OptionView* spare = rig.view("trim_spare_wheel");
    REQUIRE(spare != nullptr);
    VehicleSetup lighter = stock;
    for (const SetupPart& p : spare->parts) {
        if (p.removed) lighter.values["trim_spare_wheel"] = p.id;
    }
    auto hints = make_hints(60000); // a debounce that never ends within the test
    hints->set_base(rig.base(), stock);
    const std::vector<UpgradeQuantity> metrics = {UpgradeQuantity::GripLongitudinal, UpgradeQuantity::Weight};
    hints->request(lighter, metrics, false);
    HintAnswer a;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    for (;;) {
        a = hints->request(lighter, metrics, false);
        if (a.pending.count(UpgradeQuantity::Weight) == 0 || std::chrono::steady_clock::now() > deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(a.pending.count(UpgradeQuantity::Weight) == 0); // weight is settled ...
    CHECK(a.pending.count(UpgradeQuantity::GripLongitudinal) == 1); // ... the sweep-class figure is still queued behind the debounce
    CHECK(a.candidate.at(UpgradeQuantity::Weight) < a.installed.at(UpgradeQuantity::Weight));
    const auto badges = upgrade_badges(badge_config(), "Tyres", a.candidate, a.installed, &a.pending);
    REQUIRE(badges.size() >= 1);
    CHECK(badges[0].pending); // the grip badge is the placeholder
    hints->cancel();
    CHECK(hints->wait_idle(60000));
}

TEST_CASE("hints: a failing candidate reports an error and the worker stays usable", "[upgrade_hints][cache]") {
    Rig rig("car_sedan");
    const VehicleSetup stock = rig.garage->working();
    auto hints = make_hints();
    hints->set_base(rig.base(), stock);
    VehicleSetup bad = stock;
    bad.values["tyre_front"] = std::string("no_such_tyre_123_45r17");
    const HintAnswer a = wait_answer(*hints, bad, {UpgradeQuantity::Weight, UpgradeQuantity::GripLongitudinal});
    CHECK_FALSE(a.error.empty());
    VehicleSetup ok = stock;
    const HintAnswer b = wait_answer(*hints, ok, {UpgradeQuantity::Weight});
    CHECK(b.error.empty());
    CHECK(b.pending.empty());
}

TEST_CASE("hints: candidate_setup replays the clicks without touching the working copy", "[upgrade_hints][garage]") {
    Rig rig("car_hyper");
    const std::string before = setup_to_json(rig.garage->working());
    const bool valid_before = rig.garage->validation().ok;
    const auto t0 = std::chrono::steady_clock::now();
    std::string err;
    const auto cand = rig.garage->candidate_setup({{"engine_rev_limit", 1.05}}, &err);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    INFO(err);
    REQUIRE(cand.has_value());
    CHECK(setup_to_json(rig.garage->working()) == before);
    CHECK(rig.garage->validation().ok == valid_before);
    CHECK(setup_to_json(*cand) != before);
    // The real click gives the same setup.
    REQUIRE(rig.garage->set_option("engine_rev_limit", 1.05).accepted);
    CHECK(setup_to_json(rig.garage->working()) == setup_to_json(*cand));
    // A rejected edit: no candidate and a message.
    std::string bad_err;
    CHECK_FALSE(rig.garage->candidate_setup({{"no_such_option", 1.0}}, &bad_err).has_value());
    CHECK_FALSE(bad_err.empty());
    std::printf("HINTS candidate_setup (car_hyper, one edit): %.2f ms on the calling thread\n", ms);
    std::fflush(stdout);
}

// ---- the engine figures (dyno) --------------------------------------------------------------------------------------------

namespace {

// The independent reference: materialise, load with the same map quality, sweep the same rpm grid by hand, take the maxima.
struct DynoRef {
    double peak_power_kw = 0.0, peak_torque_nm = 0.0, peak_rpm = 0.0;
    std::vector<ps::drivetrain::DynoPoint> points;
};
DynoRef dyno_reference(const Rig& rig, const VehicleSetup& setup, bool armed, const std::string& quality) {
    TempDirs dirs;
    const MaterialisedSetup m = materialise_setup(rig.entry, rig.garage->options(), rig.garage->context_for(rig.entry), setup, dirs.base + "/m");
    REQUIRE(m.ok);
    ps::io::EngineMapOptions maps;
    maps.quality = quality == "full" ? ps::drivetrain::MapQuality::Full : ps::drivetrain::MapQuality::Sketch;
    maps.cache_dir = kRoot + "/out/godot_engine_cache";
    ps::vehicle::VehicleDesc desc = rg::load_vehicle(m.vehicle_path, maps);
    if (rig.garage->context_for(rig.entry).turbo_map_extension) {
        (void)extend_turbo_maps(desc, m.vehicle_path, maps.cache_dir, *rig.garage->context_for(rig.entry).turbo_map_extension, nullptr, nullptr);
    }
    const ps::drivetrain::ComponentParams* engine = nullptr;
    const ps::drivetrain::SimulatedEngineDesc* sim = nullptr;
    double idle = 0.0, limiter = 0.0;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            engine = &c.params;
            sim = s;
            idle = s->idle_rpm;
            limiter = s->limiter.rpm;
            break;
        }
    }
    if (engine == nullptr) {
        for (const auto& c : desc.powertrain.components) {
            if (const auto* m = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
                engine = &c.params;
                idle = m->idle_rpm;
                limiter = m->limiter.rpm;
                break;
            }
        }
    }
    REQUIRE(engine != nullptr);
    ps::drivetrain::DynoSweepSpec spec;
    for (double rpm = std::ceil(std::max(2000.0, idle) / 250.0) * 250.0; rpm < limiter - 50.0 - 1.0e-6; rpm += 250.0) spec.rpm.push_back(rpm);
    spec.rpm.push_back(limiter - 50.0);
    spec.turbo_count = 1;
    if (armed) spec.nitrous = ps::drivetrain::nitrous_from_powertrain(desc.powertrain);
    (void)sim;
    DynoRef out;
    out.points = ps::drivetrain::run_dyno_sweep(*engine, nullptr, spec, nullptr, nullptr).points;
    for (const auto& p : out.points) {
        out.peak_torque_nm = std::max(out.peak_torque_nm, static_cast<double>(p.torque_nm));
        if (p.power_kw > out.peak_power_kw) {
            out.peak_power_kw = p.power_kw;
            out.peak_rpm = p.rpm;
        }
    }
    return out;
}

} // namespace

TEST_CASE("hints: peak power and torque equal an independent dyno sweep (car_sedan, sketch maps)", "[upgrade_hints][dyno][long]") {
    Rig rig("car_sedan");
    const VehicleSetup stock = rig.garage->working();
    const auto t0 = std::chrono::steady_clock::now();
    const SetupFigures f = figures_of(rig, stock, {UpgradeQuantity::Power, UpgradeQuantity::Torque, UpgradeQuantity::CoolantTemperature, UpgradeQuantity::OilTemperature});
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    INFO(f.error);
    REQUIRE(f.error.empty());
    const DynoRef ref = dyno_reference(rig, stock, false, "sketch");
    REQUIRE(f.figures.count(UpgradeQuantity::Power) == 1);
    CHECK(f.figures.at(UpgradeQuantity::Power) == Approx(ref.peak_power_kw).epsilon(1.0e-9));
    CHECK(f.figures.at(UpgradeQuantity::Torque) == Approx(ref.peak_torque_nm).epsilon(1.0e-9));
    // Power is torque x angular speed at the peak point.
    CHECK(ref.peak_power_kw == Approx(ref.peak_torque_nm * 0 + [&] {
        for (const auto& p : ref.points) {
            if (p.rpm == ref.peak_rpm) return p.torque_nm * p.rpm * 2.0 * 3.14159265358979 / 60.0 / 1000.0;
        }
        return 0.0;
    }()).epsilon(1.0e-3));
    std::printf("HINTS car_sedan stock: peak %.1f kW at %.0f rpm, torque %.0f N*m", f.figures.at(UpgradeQuantity::Power), ref.peak_rpm,
                f.figures.at(UpgradeQuantity::Torque));
    if (f.figures.count(UpgradeQuantity::CoolantTemperature)) {
        std::printf(", coolant %.1f C, oil %.1f C", f.figures.at(UpgradeQuantity::CoolantTemperature) , f.figures.at(UpgradeQuantity::OilTemperature));
    }
    std::printf(" (%.2f s)\n", secs);
    std::fflush(stdout);
}

TEST_CASE("hints: N2O power is measured with the kit armed, the unarmed figure ignores the kit (car_hyper_n2o)", "[upgrade_hints][dyno][n2o][long]") {
    Rig rig("car_hyper_n2o");
    const OptionView* kit = rig.view("nitrous_install_hyper");
    REQUIRE(kit != nullptr);
    std::string half_id;
    for (const SetupPart& p : kit->parts) {
        if (p.id == "nitrous_hyper_half") half_id = p.id;
    }
    REQUIRE_FALSE(half_id.empty());
    const VehicleSetup current = rig.garage->working(); // the full-flow kit
    const VehicleSetup half = rig.with({{"nitrous_install_hyper", half_id}});
    const auto t0 = std::chrono::steady_clock::now();
    const SetupFigures wet_full = figures_of(rig, current, {UpgradeQuantity::Power}, true);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const SetupFigures wet_half = figures_of(rig, half, {UpgradeQuantity::Power}, true);
    const SetupFigures dry_full = figures_of(rig, current, {UpgradeQuantity::Power}, false);
    const SetupFigures dry_half = figures_of(rig, half, {UpgradeQuantity::Power}, false);
    INFO(wet_full.error << wet_half.error << dry_full.error << dry_half.error);
    REQUIRE(wet_full.error.empty());
    REQUIRE(wet_half.error.empty());
    REQUIRE(dry_full.error.empty());
    REQUIRE(dry_half.error.empty());
    // Against an independent sweep with the same kit armed / disarmed.
    const DynoRef ref_wet = dyno_reference(rig, current, true, "sketch");
    const DynoRef ref_dry = dyno_reference(rig, current, false, "sketch");
    CHECK(wet_full.figures.at(UpgradeQuantity::Power) == Approx(ref_wet.peak_power_kw).epsilon(1.0e-9));
    CHECK(dry_full.figures.at(UpgradeQuantity::Power) == Approx(ref_dry.peak_power_kw).epsilon(1.0e-9));
    // The kit adds power only while armed; a smaller kit adds less; disarmed the two cars are the same car.
    CHECK(wet_full.figures.at(UpgradeQuantity::Power) > dry_full.figures.at(UpgradeQuantity::Power) + 10.0);
    CHECK(wet_half.figures.at(UpgradeQuantity::Power) > dry_half.figures.at(UpgradeQuantity::Power) + 10.0);
    CHECK(wet_half.figures.at(UpgradeQuantity::Power) < wet_full.figures.at(UpgradeQuantity::Power));
    CHECK(dry_half.figures.at(UpgradeQuantity::Power) == Approx(dry_full.figures.at(UpgradeQuantity::Power)).epsilon(1.0e-6));
    std::printf("HINTS car_hyper_n2o: dry %.1f kW, full kit armed %.1f kW (gain %+.1f), half kit armed %.1f kW (gain %+.1f); armed sweep incl. maps %.2f s\n",
                dry_full.figures.at(UpgradeQuantity::Power), wet_full.figures.at(UpgradeQuantity::Power),
                wet_full.figures.at(UpgradeQuantity::Power) - dry_full.figures.at(UpgradeQuantity::Power), wet_half.figures.at(UpgradeQuantity::Power),
                wet_half.figures.at(UpgradeQuantity::Power) - dry_half.figures.at(UpgradeQuantity::Power), secs);
    std::fflush(stdout);

    // Through the cache, the armed and the dry figure are kept apart (same setup, different flag) - the N2O category asks armed.
    auto hints = make_hints();
    hints->set_base(rig.base(), current);
    const HintAnswer armed = wait_answer(*hints, half, {UpgradeQuantity::Power}, true);
    const HintAnswer plain = wait_answer(*hints, half, {UpgradeQuantity::Power}, false);
    CHECK(armed.installed.at(UpgradeQuantity::Power) == Approx(wet_full.figures.at(UpgradeQuantity::Power)).epsilon(1.0e-9));
    CHECK(armed.candidate.at(UpgradeQuantity::Power) == Approx(wet_half.figures.at(UpgradeQuantity::Power)).epsilon(1.0e-9));
    CHECK(plain.installed.at(UpgradeQuantity::Power) == Approx(dry_full.figures.at(UpgradeQuantity::Power)).epsilon(1.0e-9));
    // The hint of the N2O tile: the smaller kit is a power LOSS with the kit armed (red); disarmed there would be nothing to show.
    const auto badges = upgrade_badges(badge_config(), "N2O", armed.candidate, armed.installed);
    REQUIRE(badges.size() == 1);
    CHECK(badges[0].worse);
    CHECK(upgrade_badges(badge_config(), "N2O", plain.candidate, plain.installed).empty());
}

TEST_CASE("hints: changing the selection cancels the running sweep", "[upgrade_hints][cancel][long]") {
    Rig rig("car_hyper");
    const VehicleSetup stock = rig.garage->working();
    VehicleSetup cand = stock;
    cand.values["engine_rev_limit"] = 1.02;
    auto hints = make_hints(0, "full"); // Full maps: the job is long enough to be cancelled while it runs
    hints->set_base(rig.base(), stock);
    hints->request(cand, {UpgradeQuantity::Power}, false);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (hints->stats().jobs_started < 1 && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    REQUIRE(hints->stats().jobs_started == 1);
    const auto t0 = std::chrono::steady_clock::now();
    hints->cancel();
    REQUIRE(hints->wait_idle(60000));
    const double latency = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const HintStats s = hints->stats();
    CHECK(s.jobs_cancelled == 1);
    CHECK(s.jobs_finished == 0);
    CHECK(hints->request(cand, {UpgradeQuantity::Power}, false).pending.size() == 1); // nothing was cached from the cancelled job
    hints->cancel();
    hints->wait_idle(60000);
    std::printf("HINTS cancel latency of a running Full-map job: %.2f s\n", latency);
    std::fflush(stdout);
}

// Measurement, not a check: how far the reference map quality (sketch) is from Full on the hyper, and what a figure costs.
// Run with: rg_unit_tests "hints: sketch versus full*" (hidden).
TEST_CASE("hints: sketch versus full peak power and temperatures (diag)", "[.][diag][hints_diag]") {
    for (const char* car : {"car_hyper", "car_hyper_n2o", "car_sedan_gen_n2o"}) {
        Rig rig(car);
        const VehicleSetup stock = rig.garage->working();
        std::vector<std::pair<std::string, VehicleSetup>> setups = {{"stock", stock}};
        const OptionView* v = rig.view("turbo_install");
        if (v == nullptr || !v->available) v = rig.view("turbo_install_n2o");
        if (v != nullptr && v->available) {
            for (const SetupPart& p : v->parts) {
                if (p.fits && !p.id.empty()) setups.emplace_back("turbo_install=" + p.id, rig.with({{v->def.id, p.id}}));
            }
        }
        for (const auto& [name, setup] : setups) {
            double power[2] = {0, 0}, cool[2] = {0, 0}, oil[2] = {0, 0}, secs[2] = {0, 0};
            for (int q = 0; q < 2; ++q) {
                UpgradeBadgeConfig cfg = badge_config();
                cfg.references.dyno_quality = q == 0 ? "sketch" : "full";
                FigureEnv env = make_figure_env(cfg.references);
                const SetupFigures f = compute_setup_figures(rig.base(), setup, {UpgradeQuantity::Power, UpgradeQuantity::CoolantTemperature, UpgradeQuantity::OilTemperature}, false, env);
                if (!f.error.empty()) {
                    std::printf("HINTS DIAG %s %s: error %s\n", car, name.c_str(), f.error.c_str());
                    continue;
                }
                power[q] = f.figures.count(UpgradeQuantity::Power) ? f.figures.at(UpgradeQuantity::Power) : 0.0;
                cool[q] = f.figures.count(UpgradeQuantity::CoolantTemperature) ? f.figures.at(UpgradeQuantity::CoolantTemperature) : 0.0;
                oil[q] = f.figures.count(UpgradeQuantity::OilTemperature) ? f.figures.at(UpgradeQuantity::OilTemperature) : 0.0;
                secs[q] = f.seconds;
            }
            std::printf("HINTS DIAG %s %-28s power sketch %.1f / full %.1f kW (%+.2f %%)  coolant %.1f/%.1f C  oil %.1f/%.1f C  time %.2f s / %.2f s\n", car, name.c_str(),
                        power[0], power[1], power[1] > 0 ? (power[0] / power[1] - 1.0) * 100.0 : 0.0, cool[0], cool[1], oil[0], oil[1], secs[0], secs[1]);
            std::fflush(stdout);
        }
    }
}

// Measurement, not a check: what the fuel option does to the peak power figure on the hyper (the Swaps / Fuel tile's hint).
TEST_CASE("hints: fuel option effect on peak power (diag)", "[.][diag][hints_diag]") {
    Rig rig("car_hyper");
    const OptionView* v = rig.view("fuel");
    REQUIRE(v != nullptr);
    std::printf("HINTS DIAG fuel option value %s\n", std::get<std::string>(rig.garage->current_value("fuel")).c_str());
    for (const SetupPart& p : v->parts) {
        const SetupFigures f = figures_of(rig, rig.with({{"fuel", p.id}}), {UpgradeQuantity::Power});
        std::printf("HINTS DIAG fuel %-12s power %.2f kW %s\n", p.id.c_str(), f.figures.count(UpgradeQuantity::Power) ? f.figures.at(UpgradeQuantity::Power) : -1.0, f.error.c_str());
    }
    std::fflush(stdout);
}

// ---- the installed option (owner 2026-10-19: no hint, and re-selecting it must not reset anything) ----------------------------

namespace {

// Every available option of the edited car re-selected with its CURRENT value (plus, with an ICE conversion installed, the
// conversion tile's two-edit form): the setup must stay byte-identical, edits_installed must say so (that is the rule that
// hides the hint chips of the installed tile) and the candidate must be the working copy itself.
void check_installed_is_nothing(Rig& rig, const std::string& label) {
    const std::string before = setup_to_json(rig.garage->working());
    std::vector<std::string> ids;
    for (const OptionView& v : rig.garage->model().options) {
        if (v.available) ids.push_back(v.def.id);
    }
    REQUIRE_FALSE(ids.empty());
    for (const std::string& id : ids) {
        INFO(label << " option " << id);
        const SetupValue now = rig.garage->current_value(id);
        CHECK(rig.garage->edits_installed({{id, now}}));
        std::string err;
        const auto cand = rig.garage->candidate_setup({{id, now}}, &err);
        INFO(err);
        REQUIRE(cand.has_value());
        CHECK(setup_to_json(*cand) == before);
        const EditResult r = rig.garage->set_option(id, now); // the real click
        CHECK(r.accepted);
        CHECK(setup_to_json(rig.garage->working()) == before);
    }
    const SetupValue pt = rig.garage->current_value("powertrain_install");
    if (std::holds_alternative<std::string>(pt) && std::get<std::string>(pt) == "ice") {
        const std::vector<std::pair<std::string, SetupValue>> tile = {{"powertrain_install", pt}, {"engine_install", rig.garage->current_value("engine_install")}};
        CHECK(rig.garage->edits_installed(tile));
        CHECK(setup_to_json(*rig.garage->candidate_setup(tile)) == before);
    }
    // A real change is still a change.
    CHECK_FALSE(rig.garage->edits_installed({}));
}

} // namespace

TEST_CASE("hints: the installed option of every category changes nothing on every car", "[upgrade_hints][garage]") {
    for (const char* car : {"car_hyper", "car_hyper_n2o", "car_sedan", "car_electric"}) {
        Rig rig(car);
        check_installed_is_nothing(rig, car);
        // Not the stock setup: a few upgrades on top, as a player's car has.
        if (rig.view("engine_rev_limit") != nullptr && rig.view("engine_rev_limit")->available) {
            REQUIRE(rig.garage->set_option("engine_rev_limit", 1.05).accepted);
            check_installed_is_nothing(rig, std::string(car) + " +rev limit");
        }
    }
}

TEST_CASE("hints: the installed hybrid keeps its sub-settings when it is selected again", "[upgrade_hints][garage]") {
    Rig rig("car_sedan");
    REQUIRE(rig.garage->set_option("powertrain_install", std::string("hybrid_battery")).accepted);
    // Component choices made after the conversion (they sit under the powertrain choice and a conversion resets them).
    std::string chosen;
    for (const char* id : {"engine_rev_limit", "final_drive"}) {
        const OptionView* v = rig.view(id);
        if (v != nullptr && v->available) {
            REQUIRE(rig.garage->set_option(id, 1.05).accepted);
            chosen += std::string(id) + " ";
        }
    }
    if (const OptionView* e = rig.view("engine_install"); e != nullptr && e->available) {
        for (const SetupPart& p : e->parts) {
            if (p.id != std::get<std::string>(rig.garage->current_value("engine_install")) && rig.garage->set_option("engine_install", p.id).accepted) {
                chosen += "engine_install ";
                break;
            }
        }
    }
    INFO("sub-settings set: " << chosen);
    REQUIRE_FALSE(chosen.empty());
    const std::string before = setup_to_json(rig.garage->working());
    const EditResult r = rig.garage->set_option("powertrain_install", std::string("hybrid_battery"));
    CHECK(r.accepted);
    CHECK(r.notice.empty()); // no "component upgrades were reset" notice
    CHECK(setup_to_json(rig.garage->working()) == before);
    // The tile's hint: no chips, and the candidate is the car as it is.
    CHECK(rig.garage->edits_installed({{"powertrain_install", std::string("hybrid_battery")}}));
    check_installed_is_nothing(rig, "car_sedan hybrid");
    // Choosing ANOTHER powertrain still converts (and resets), then the way back is a real change again.
    REQUIRE(rig.garage->set_option("powertrain_install", std::string("ice")).accepted);
    CHECK(setup_to_json(rig.garage->working()) != before);
}

TEST_CASE("hints: a powertrain swap shows peak power and weight only", "[upgrade_hints][upgrade_badges]") {
    const UpgradeCategoryMetrics* c = upgrade_category_for(badge_config(), "powertrain_install", "Swaps");
    REQUIRE(c != nullptr);
    // No battery figure (owner 2026-10-19: an EV conversion carrying a battery is what its name says).
    CHECK(c->metrics == std::vector<UpgradeQuantity>{UpgradeQuantity::Power, UpgradeQuantity::Weight});
}

// ---- the specification pane follows the working setup (owner 2026-10-19) ----------------------------------------------------

TEST_CASE("hints: the specification pane reports the configuration being edited, not the stock car", "[upgrade_hints][garage]") {
    Rig rig("car_hyper");
    auto hints = make_hints(0);
    hints->set_base(rig.base(), rig.garage->working());
    const std::vector<UpgradeQuantity> spec = {UpgradeQuantity::Power, UpgradeQuantity::Torque};
    hints->request_current(spec);
    REQUIRE(hints->wait_idle(600000));
    const HintAnswer stock = hints->request_current(spec);
    REQUIRE(stock.pending.empty());
    const double stock_kw = stock.installed.at(UpgradeQuantity::Power);
    const double stock_nm = stock.installed.at(UpgradeQuantity::Torque);
    const double stock_mass = rig.garage->stats("car_hyper", nullptr)->mass_kg;

    // The tile's hint for the single turbo (against the stock car) ...
    const VehicleSetup candidate = rig.with({{"turbo_install", std::string("hyper_single_compact")}});
    const HintAnswer tile = wait_answer(*hints, candidate, {UpgradeQuantity::Power});
    REQUIRE(tile.pending.empty());
    const double hint_delta_kw = tile.candidate.at(UpgradeQuantity::Power) - tile.installed.at(UpgradeQuantity::Power);
    REQUIRE(hint_delta_kw < -50.0);

    // ... then the real click: the pane (request_current on the new current setup) shows the new car.
    REQUIRE(rig.garage->set_option("turbo_install", std::string("hyper_single_compact")).accepted);
    hints->set_base(rig.base(), rig.garage->working());
    hints->request_current(spec);
    REQUIRE(hints->wait_idle(600000));
    const HintAnswer now = hints->request_current(spec);
    REQUIRE(now.pending.empty());
    const double now_kw = now.installed.at(UpgradeQuantity::Power);
    CHECK(now_kw != Approx(stock_kw).margin(1.0));
    CHECK(now_kw - stock_kw == Approx(hint_delta_kw).margin(1.0e-6)); // the pane and the tile hint are the same figure
    CHECK(now.installed.at(UpgradeQuantity::Torque) < stock_nm);

    // Mass follows the working setup too (a weight-reduction click), gears follow a gearbox swap.
    const double mass_before = rig.garage->stats("car_hyper", nullptr)->mass_kg;
    CHECK(mass_before != Approx(stock_mass)); // the single turbo is lighter: the pane mass follows the swap
    REQUIRE(rig.garage->set_option("trim_audio", std::string("removed")).accepted);
    CHECK(rig.garage->stats("car_hyper", nullptr)->mass_kg < mass_before - 1.0);
}

TEST_CASE("hints: the specification pane shows the gears of the installed gearbox", "[upgrade_hints][garage]") {
    Rig rig("car_sedan");
    const int stock_gears = rig.garage->stats("car_sedan", nullptr)->gear_count;
    REQUIRE(stock_gears > 0);
    REQUIRE(rig.garage->set_option("gearbox_install", std::string("econ_5mt")).accepted);
    CHECK(rig.garage->stats("car_sedan", nullptr)->gear_count == 5);
    REQUIRE(rig.garage->set_option("gearbox_install", std::string("hyper_7mt")).accepted);
    CHECK(rig.garage->stats("car_sedan", nullptr)->gear_count == 7);
}

TEST_CASE("hints: the pane never starves a hovered tile's job and vice versa", "[upgrade_hints]") {
    Rig rig("car_sedan");
    auto hints = make_hints(0);
    hints->set_base(rig.base(), rig.garage->working());
    const VehicleSetup candidate = rig.with({{"gearbox_install", std::string("heavy_duty_6mt")}});
    const HintAnswer first = hints->request(candidate, {UpgradeQuantity::Weight}, false); // a tile hover
    (void)first;
    for (int i = 0; i < 20; ++i) {
        hints->request_current({UpgradeQuantity::Power, UpgradeQuantity::Torque}); // the pane polling
        hints->request(candidate, {UpgradeQuantity::Weight}, false);               // the tile polling
    }
    REQUIRE(hints->wait_idle(600000));
    // One more pane poll queues whatever the running job did not cover.
    hints->request_current({UpgradeQuantity::Power, UpgradeQuantity::Torque});
    REQUIRE(hints->wait_idle(600000));
    CHECK(hints->request_current({UpgradeQuantity::Power, UpgradeQuantity::Torque}).pending.empty());
    CHECK(hints->request(candidate, {UpgradeQuantity::Weight}, false).pending.empty());
}
