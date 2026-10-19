// Garage limit advisor (rg/limit_advisor.h): the config file, the pure advice builder (grouping, merging, severity,
// templates), the Find-fix text, and - [long] - the real advisor on the hypercar with the owner's extreme turbo setup.
#include "rg/limit_advisor.h"

#include "rg/vehicle_catalog.h"
#include "rg/vehicle_setup.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <thread>
#include <variant>

namespace {

namespace pd = ps::drivetrain;
const std::string kRepo = RG_SOURCE_DIR;

rg::AdvisorConfig shipped_config() {
    std::string err;
    auto c = rg::load_advisor_config(kRepo + "/data/garage/advisor.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return *c;
}

pd::LimitFinding F(pd::LimitKind kind, pd::LimitSeverity s, double rpm, double value, double limit, pd::LimitUnit unit, const std::string& field = "") {
    pd::LimitFinding f;
    f.kind = kind;
    f.severity = s;
    f.rpm = rpm;
    f.value = value;
    f.limit = limit;
    f.unit = unit;
    f.part.field = field;
    return f;
}

pd::DynoPoint P(double rpm, double charge_bar) {
    pd::DynoPoint p;
    p.rpm = rpm;
    p.charge_boost_bar = charge_bar;
    return p;
}

// A sweep 2000..8000 in 250 steps, boost rising linearly to `top` bar.
std::vector<pd::DynoPoint> sweep_points(double top) {
    std::vector<pd::DynoPoint> pts;
    for (double rpm = 2000; rpm <= 8000; rpm += 250) pts.push_back(P(rpm, top * (rpm - 1500) / 6500.0));
    return pts;
}

std::string joined(const rg::AdviceItem& i) {
    std::string s = i.title;
    for (const auto& l : i.lines) s += "\n" + l;
    if (!i.suggestion.empty()) s += "\n" + i.suggestion;
    return s;
}

const rg::AdviceItem* find_item(const std::vector<rg::AdviceItem>& items, const std::string& id) {
    for (const auto& i : items) {
        if (i.id == id) return &i;
    }
    return nullptr;
}

} // namespace

TEST_CASE("advisor config: the shipped file loads and names every kind the advisor can produce", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config();
    CHECK(c.debounce_ms == 300);
    CHECK(c.sweep.quality == "sketch");
    for (const pd::LimitKind k : {pd::LimitKind::BoostBelowTarget, pd::LimitKind::BoostAboveTarget, pd::LimitKind::WastegateSaturated,
                                  pd::LimitKind::TurboRotorCap, pd::LimitKind::CompressorChoke, pd::LimitKind::CompressorSurge,
                                  pd::LimitKind::RevLimiter, pd::LimitKind::FuelCut, pd::LimitKind::MeanPistonSpeed, pd::LimitKind::MapGridEdge,
                                  pd::LimitKind::ClutchCapacity, pd::LimitKind::GearboxInputTorque, pd::LimitKind::Knock,
                                  pd::LimitKind::KitFuelLean, pd::LimitKind::N2OBottleDuration, pd::LimitKind::N2OBottlePressureDrop}) {
        INFO(pd::to_string(k));
        const rg::AdviceKindConfig* kc = c.find(k);
        REQUIRE(kc != nullptr);
        CHECK_FALSE(kc->text.title.empty());
        CHECK_FALSE(kc->text.lines.empty());
    }
    // What the game itself adds on top of the limit policy: the limiter kinds are hidden (the sweep ends under the
    // limiter, so they only restate the rev limit setting) and MapGridEdge needs Exceeded. Every other kind has NO
    // floor of its own - the policy's warn_at decides (it would only duplicate policy.warns()).
    CHECK(c.find(pd::LimitKind::RevLimiter)->hidden);
    CHECK(c.find(pd::LimitKind::FuelCut)->hidden);
    REQUIRE(c.find(pd::LimitKind::MapGridEdge)->min_severity.has_value());
    CHECK(*c.find(pd::LimitKind::MapGridEdge)->min_severity == pd::LimitSeverity::Exceeded);
    CHECK_FALSE(c.find(pd::LimitKind::MapGridEdge)->hidden);
    for (const rg::AdviceKindConfig& k : c.kinds) {
        INFO(pd::to_string(k.kind));
        if (k.kind == pd::LimitKind::RevLimiter || k.kind == pd::LimitKind::FuelCut || k.kind == pd::LimitKind::MapGridEdge) continue;
        CHECK_FALSE(k.hidden);
        CHECK_FALSE(k.min_severity.has_value());
    }
}

TEST_CASE("advisor config: the game's limit policy extends physics' default and changes nothing", "[limit_advisor][limit_policy]") {
    const rg::AdvisorConfig c = shipped_config();
    CHECK(c.policy_file == "limit_policy.json");
    const pd::LimitPolicy& d = pd::default_limit_policy();
    CHECK(c.policy.refuse_at == d.refuse_at);
    CHECK(c.policy.warn_at == d.warn_at);
    CHECK(c.policy.gate_near_opening == d.gate_near_opening);
    CHECK(c.policy.piston_speed_limiting_mps == d.piston_speed_limiting_mps);
    CHECK(c.policy.driveline_torque_margin == d.driveline_torque_margin);
    CHECK(c.policy.gearbox_rating_over_exceeded == d.gearbox_rating_over_exceeded);
    CHECK(c.policy.gearbox_rating_remedy_margin == d.gearbox_rating_remedy_margin);
    // The garage tells everything from Near and would refuse only Exceeded.
    CHECK(c.policy.warns(pd::LimitSeverity::Near));
    CHECK_FALSE(c.policy.refuses(pd::LimitSeverity::Limiting));
    CHECK(c.policy.refuses(pd::LimitSeverity::Exceeded));
}

TEST_CASE("advisor config: a user layer is merged over the game's policy, and a bad one is reported", "[limit_advisor][limit_policy]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "rg_advisor_policy_test";
    fs::create_directories(dir);
    const std::string user = (dir / "limit_policy.json").generic_string();
    {
        std::ofstream out(user);
        out << R"({"warn_at":"limiting","refuse_at":"limiting","turbo":{"gate_near_opening":0.8}})";
    }
    CHECK(rg::user_limit_policy_path(dir.generic_string()) == user);
    CHECK(rg::user_limit_policy_path("").empty());
    std::string err;
    auto c = rg::load_advisor_config(kRepo + "/data/garage/advisor.json", &err, user);
    INFO(err);
    REQUIRE(c.has_value());
    CHECK(c->policy.warn_at == pd::WarnAt::Limiting);
    CHECK_FALSE(c->policy.warns(pd::LimitSeverity::Near));
    CHECK(c->policy.refuses(pd::LimitSeverity::Limiting));
    CHECK(c->policy.gate_near_opening == Catch::Approx(0.8));
    CHECK(c->policy.gate_full_opening == pd::default_limit_policy().gate_full_opening); // everything else: the game's (= physics') value

    // The advice follows the policy: a Near finding is told by the default policy, not by warn_at limiting.
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    in.findings.push_back(F(pd::LimitKind::CompressorChoke, pd::LimitSeverity::Near, 6000.0, 0.92, 1.0, pd::LimitUnit::Fraction, "/max_mass_flow_kg_s"));
    in.findings.push_back(F(pd::LimitKind::CompressorSurge, pd::LimitSeverity::Limiting, 6000.0, 0.95, 1.0, pd::LimitUnit::Dimensionless, "/surge_line"));
    CHECK(rg::build_advice(in, shipped_config()).size() == 2);
    const auto strict = rg::build_advice(in, *c);
    REQUIRE(strict.size() == 1);
    CHECK(strict[0].kind == pd::LimitKind::CompressorSurge);
    // warn_at never: nothing is told at all - but the findings themselves are still there (the engine never drops them).
    rg::AdvisorConfig mute = shipped_config();
    mute.policy.warn_at = pd::WarnAt::Never;
    CHECK(rg::build_advice(in, mute).empty());
    CHECK(pd::has_refusing_finding({F(pd::LimitKind::CompressorSurge, pd::LimitSeverity::Limiting, 6000.0, 0.95, 1.0, pd::LimitUnit::Dimensionless)}, &c->policy));

    // A broken user layer is an error naming the file, not a silent default.
    {
        std::ofstream out(user);
        out << R"({"turbo":{"no_such_entry":1}})";
    }
    CHECK_FALSE(rg::load_advisor_config(kRepo + "/data/garage/advisor.json", &err, user).has_value());
    CHECK(err.find("limit policy") != std::string::npos);
    CHECK(err.find("no_such_entry") != std::string::npos);
    fs::remove_all(dir);
}

TEST_CASE("advisor config: rejections", "[limit_advisor]") {
    std::string good;
    {
        std::ifstream in(kRepo + "/data/garage/advisor.json");
        good.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const auto replaced = [&](const std::string& from, const std::string& to) {
        std::string s = good;
        const auto at = s.find(from);
        REQUIRE(at != std::string::npos);
        s.replace(at, from.size(), to);
        return s;
    };
    std::string err;
    CHECK(rg::parse_advisor_config(good, "good", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config("{", "bad", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("rg.garage_advisor/1", "rg.garage_advisor/9"), "fmt", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"BoostBelowTarget\"", "\"NoSuchKind\""), "kind", &err).has_value());
    CHECK(err.find("LimitKind") != std::string::npos);
    CHECK_FALSE(rg::parse_advisor_config(replaced("{peak_rpm}", "{peek_rpm}"), "tpl", &err).has_value());
    CHECK(err.find("{peek_rpm}") != std::string::npos);
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"never\"", "\"sometimes\""), "sev", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"quality\": \"sketch\"", "\"quality\": \"ultra\""), "q", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"debounce_ms\": 300", "\"debounce_ms\": -1"), "d", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"merge_gap_rpm\": 300", "\"merge_gap_rpm\": 300, \"typo\": 1"), "key", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("{target}", "{target"), "brace", &err).has_value());
    CHECK_FALSE(rg::parse_advisor_config(replaced("\"limit_policy\": \"limit_policy.json\"", "\"limit_policy\": 5"), "pol", &err).has_value());
    CHECK(err.find("limit_policy") != std::string::npos);
}

TEST_CASE("advice text: the owner's boost example, to the line", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(3.4);
    in.target_boost_bar = 15.5;
    // Exceeded at every swept rpm: the target is never reached.
    for (const auto& p : in.points)
        in.findings.push_back(F(pd::LimitKind::BoostBelowTarget, pd::LimitSeverity::Exceeded, p.rpm, p.charge_boost_bar * 1e5, 15.5e5, pd::LimitUnit::Pascal));
    const auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    const rg::AdviceItem& i = items[0];
    CHECK(i.severity == rg::AdviceSeverity::Critical);
    CHECK(i.title == "Excessive turbo boost target");
    REQUIRE(i.lines.size() == 3);
    CHECK(i.lines[0] == "Boost target is set to 15.5 bar.");
    CHECK(i.lines[1] == "The current setup achieved");
    CHECK(i.lines[2] == "3.4 bar @ 8000 RPM."); // the best the setup reached over the sweep
    CHECK(i.suggestion == "Bigger turbos or a lower boost target.");
    CHECK(i.option_id == "turbo_boost_target");
    CHECK(i.variant.empty());
    REQUIRE(i.ranges.size() == 1);
    CHECK(i.rpm_from == 2000);
    CHECK(i.rpm_to == 8000);
    CHECK(i.remedy_state == rg::RemedyState::None);
    CHECK(joined(i) == "Excessive turbo boost target\nBoost target is set to 15.5 bar.\nThe current setup achieved\n3.4 bar @ 8000 RPM.\nBigger turbos or a lower boost target.");
}

TEST_CASE("advice grouping: contiguous rpm ranges merge, far ones stay apart, worst severity first", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config(); // merge gap 300
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    in.target_boost_bar = 1.8;
    // CompressorChoke: 3000, 3250, 3500 merge; 6000 is a second range.
    for (const double rpm : {3000.0, 3250.0, 3500.0, 6000.0})
        in.findings.push_back(F(pd::LimitKind::CompressorChoke, pd::LimitSeverity::Near, rpm, 0.92, 1.0, pd::LimitUnit::Fraction, "/max_mass_flow_kg_s"));
    in.findings.push_back(F(pd::LimitKind::CompressorChoke, pd::LimitSeverity::Limiting, 6000.0, 1.03, 1.0, pd::LimitUnit::Fraction, "/max_mass_flow_kg_s"));
    in.findings.push_back(F(pd::LimitKind::TurboRotorCap, pd::LimitSeverity::Exceeded, 7000.0, 190000, 150000, pd::LimitUnit::RevPerMinute));
    in.findings.push_back(F(pd::LimitKind::RevLimiter, pd::LimitSeverity::Exceeded, 8000.0, 8000, 7900, pd::LimitUnit::RevPerMinute));  // hidden kind
    in.findings.push_back(F(pd::LimitKind::MapGridEdge, pd::LimitSeverity::Near, 5000.0, 250e3, 255e3, pd::LimitUnit::Pascal, "/cycle/grid_p_im_kpa")); // below its min
    const auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 2);
    // Worst severity first: Critical rotor cap, then the warning choke.
    CHECK(items[0].kind == pd::LimitKind::TurboRotorCap);
    CHECK(items[0].severity == rg::AdviceSeverity::Critical);
    CHECK(items[1].kind == pd::LimitKind::CompressorChoke);
    CHECK(items[1].severity == rg::AdviceSeverity::Warning);
    REQUIRE(items[1].ranges.size() == 2);
    CHECK(items[1].ranges[0].from_rpm == 3000);
    CHECK(items[1].ranges[0].to_rpm == 3500);
    CHECK(items[1].ranges[1].from_rpm == 6000);
    CHECK(items[1].ranges[1].to_rpm == 6000);
    CHECK(items[1].findings.size() == 5);
    CHECK(items[1].lines[0] == "The compressor runs at 103 %");   // the worst finding speaks
    CHECK(items[1].lines[1] == "of its flow capacity @ 6000 RPM.");
    CHECK(rg::format_rpm_ranges(items[1].ranges, c) == "3000-3500, 6000 RPM");
}

TEST_CASE("advice grouping: the config's minimum severity filters before merging", "[limit_advisor]") {
    rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    for (const double rpm : {3000.0, 3250.0, 3500.0})
        in.findings.push_back(F(pd::LimitKind::CompressorChoke, rpm == 3250.0 ? pd::LimitSeverity::Near : pd::LimitSeverity::Limiting, rpm, 1.0, 1.0, pd::LimitUnit::Fraction));
    for (auto& k : c.kinds) {
        if (k.kind == pd::LimitKind::CompressorChoke) k.min_severity = pd::LimitSeverity::Limiting;
    }
    const auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    REQUIRE(items[0].ranges.size() == 2); // the Near point between them is gone: 3000 and 3500 are 500 apart (> the 300 gap)
    CHECK(items[0].findings.size() == 2);
    // A kind absent from the config is never shown.
    c.kinds.clear();
    c.kinds.push_back(rg::AdviceKindConfig{});
    CHECK(rg::build_advice(in, c).empty());
}

TEST_CASE("advice variants: boost shortfall at the start of the sweep is spool-up, at the end a fall-off", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(1.8);
    in.target_boost_bar = 1.8;
    for (const double rpm : {2000.0, 2250.0, 2500.0})
        in.findings.push_back(F(pd::LimitKind::BoostBelowTarget, pd::LimitSeverity::Near, rpm, 1.4e5, 1.8e5, pd::LimitUnit::Pascal));
    auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].variant == "spool");
    CHECK(items[0].title == "Slow turbo spool-up");
    CHECK(items[0].severity == rg::AdviceSeverity::Advice);
    CHECK(items[0].lines[0] == "Boost target is set to 1.8 bar.");
    CHECK(items[0].lines[1] == "The turbos reach it from 2750 RPM."); // the first swept rpm after the shortfall
    CHECK(items[0].lines[2] == "Below that the setup achieved 1.4 bar @ 2000 RPM.");

    in.findings.clear();
    for (const double rpm : {7500.0, 7750.0, 8000.0})
        in.findings.push_back(F(pd::LimitKind::BoostBelowTarget, pd::LimitSeverity::Limiting, rpm, 1.5e5, 1.8e5, pd::LimitUnit::Pascal));
    items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].variant == "falloff");
    CHECK(items[0].lines[1] == "The turbos hold it up to 7250 RPM.");

    in.findings.clear();
    in.findings.push_back(F(pd::LimitKind::BoostBelowTarget, pd::LimitSeverity::Near, 5000.0, 1.5e5, 1.8e5, pd::LimitUnit::Pascal));
    items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].variant.empty()); // a dip in the middle keeps the base text
}

TEST_CASE("advice text: every kind renders without a missing fact", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config();
    struct Row {
        pd::LimitKind kind;
        pd::LimitSeverity severity;
        double value, limit;
        pd::LimitUnit unit;
        const char* field;
        const char* expect_in_text; // a number that must show up
    };
    const Row rows[] = {
        {pd::LimitKind::BoostBelowTarget, pd::LimitSeverity::Exceeded, 1.0e5, 1.8e5, pd::LimitUnit::Pascal, "", "bar"},
        {pd::LimitKind::BoostAboveTarget, pd::LimitSeverity::Exceeded, 2.4e5, 1.8e5, pd::LimitUnit::Pascal, "/wastegate_area_m2", "2.4 bar"},
        {pd::LimitKind::WastegateSaturated, pd::LimitSeverity::Limiting, 0.99, 0.98, pd::LimitUnit::Fraction, "/wastegate_area_m2", "99 % open"},
        {pd::LimitKind::TurboRotorCap, pd::LimitSeverity::Exceeded, 190000, 150000, pd::LimitUnit::RevPerMinute, "", "190000 RPM"},
        {pd::LimitKind::CompressorChoke, pd::LimitSeverity::Limiting, 1.04, 1.0, pd::LimitUnit::Fraction, "/max_mass_flow_kg_s", "104 %"},
        {pd::LimitKind::CompressorSurge, pd::LimitSeverity::Exceeded, 0.95, 1.0, pd::LimitUnit::Dimensionless, "/surge_line", "0.95x"},
        {pd::LimitKind::MeanPistonSpeed, pd::LimitSeverity::Limiting, 26.2, 25.0, pd::LimitUnit::MetrePerSecond, "/limiter/rpm", "26.2 m/s"},
        {pd::LimitKind::MapGridEdge, pd::LimitSeverity::Exceeded, 320e3, 300e3, pd::LimitUnit::Pascal, "/cycle/grid_p_im_kpa", "320 kPa"},
        {pd::LimitKind::MapGridEdge, pd::LimitSeverity::Exceeded, 8300, 8250, pd::LimitUnit::RevPerMinute, "/cycle/grid_rpm", "8300 RPM"},
        {pd::LimitKind::ClutchCapacity, pd::LimitSeverity::Limiting, 1800, 1500, pd::LimitUnit::NewtonMetre, "/capacity_nm", "1800 N*m"},
        {pd::LimitKind::GearboxInputTorque, pd::LimitSeverity::Limiting, 1500, 1400, pd::LimitUnit::NewtonMetre, "/rated_input_torque_nm", "1400 N*m"},
        {pd::LimitKind::Knock, pd::LimitSeverity::Limiting, 6.5, 5.0, pd::LimitUnit::CrankDegree, "/knock/ron", "6.5 deg"},
        {pd::LimitKind::KitFuelLean, pd::LimitSeverity::Exceeded, 1.15, 0.9, pd::LimitUnit::Dimensionless, "/fuel_jet_diameter_mm", "1.15"},
        {pd::LimitKind::N2OBottleDuration, pd::LimitSeverity::Limiting, 6.0, 10.0, pd::LimitUnit::Second, "/capacity_kg", "6.0 s"},
        {pd::LimitKind::N2OBottlePressureDrop, pd::LimitSeverity::Limiting, 0.75, 0.9, pd::LimitUnit::Dimensionless, "/pressure", "75 %"},
    };
    for (const Row& r : rows) {
        INFO(pd::to_string(r.kind) << " " << r.field);
        rg::AdviceInput in;
        in.points = sweep_points(2.0);
        in.target_boost_bar = 1.8;
        pd::LimitFinding f = F(r.kind, r.severity, 6500.0, r.value, r.limit, r.unit, r.field);
        in.findings.push_back(f);
        const auto items = rg::build_advice(in, c);
        REQUIRE(items.size() == 1);
        const std::string text = joined(items[0]);
        INFO(text);
        CHECK(text.find('?') == std::string::npos);
        CHECK(text.find('{') == std::string::npos);
        CHECK(text.find(r.expect_in_text) != std::string::npos);
        if (r.kind != pd::LimitKind::BoostBelowTarget) CHECK(text.find("6500 RPM") != std::string::npos); // BoostBelowTarget names the best rpm of the sweep
        CHECK(items[0].lines.size() >= 1);
    }
}

TEST_CASE("advice text: the gearbox overload, to the line (owner format)", "[limit_advisor][gearbox]") {
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    // A synthetic finding as check_driveline_limits makes it: rpm = the peak point, value = the peak, limit = the rating,
    // a Closed remedy (required_min = peak, required_margin = 1.1 x peak).
    pd::LimitFinding f = F(pd::LimitKind::GearboxInputTorque, pd::LimitSeverity::Limiting, 6000.0, 1371.0, 1300.0, pd::LimitUnit::NewtonMetre, "/rated_input_torque_nm");
    f.part.slot = "gearbox";
    pd::Remedy r;
    r.current = 1300.0;
    r.required_min = 1371.0;
    r.required_margin = 1508.1;
    r.unit = pd::LimitUnit::NewtonMetre;
    r.cost = pd::RemedyCost::Closed;
    f.remedy = r;
    in.findings = {f};
    const auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    const rg::AdviceItem& i = items[0];
    CHECK(i.kind == pd::LimitKind::GearboxInputTorque);
    CHECK(i.severity == rg::AdviceSeverity::Warning);
    CHECK(i.option_id == "gearbox_install");
    CHECK(i.title == "Gearbox overloaded");
    REQUIRE(i.lines.size() == 2);
    CHECK(i.lines[0] == "Gearbox is rated for 1300 N*m.");
    CHECK(i.lines[1] == "The engine makes 1371 N*m @ 6000 RPM.");
    CHECK(i.suggestion == "A stronger gearbox.");
    CHECK(i.remedy_state == rg::RemedyState::Closed);
    CHECK(i.remedy_text == "A gearbox rated for about 1508 N*m holds it with margin (now 1300 N*m).");
    CHECK_FALSE(i.can_solve);
    // Exceeded is critical; the policy decides what is told: a Near overload is advice, hidden by warn_at limiting.
    in.findings[0].severity = pd::LimitSeverity::Near;
    CHECK(rg::build_advice(in, c)[0].severity == rg::AdviceSeverity::Advice);
    rg::AdvisorConfig strict = c;
    strict.policy.warn_at = pd::WarnAt::Limiting;
    CHECK(rg::build_advice(in, strict).empty());
}

TEST_CASE("advisor sweep: the policy and the gearbox rating reach the physics checks", "[limit_advisor][gearbox][limit_policy]") {
    // physics_sim's torque-map car_sedan: no maps to generate. Its gearbox has no rating, so no gearbox finding; give it
    // one far below the engine's peak and the sweep reports GearboxInputTorque with the part file we pass.
    ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(kRepo + "/external/physics_sim/data/vehicles/car_sedan.json");
    ps::drivetrain::GearboxDesc* gearbox = nullptr;
    for (auto& comp : desc.powertrain.components) {
        if (auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&comp.params)) gearbox = g;
    }
    REQUIRE(gearbox != nullptr);
    CHECK(gearbox->rated_input_torque_nm == 0.0);
    ps::jobs::JobSystem pool(2);
    rg::AdvisorConfig c = shipped_config();

    const auto has_gearbox_finding = [](const rg::AdviceSweep& s) {
        return std::any_of(s.input.findings.begin(), s.input.findings.end(), [](const pd::LimitFinding& f) { return f.kind == pd::LimitKind::GearboxInputTorque; });
    };
    CHECK_FALSE(has_gearbox_finding(rg::run_advice_sweep(desc, c, &pool, nullptr, "clutch.json", "gearbox.json"))); // unrated: nothing

    gearbox->rated_input_torque_nm = 100.0; // the sedan's peak is about 212 N*m
    const rg::AdviceSweep sweep = rg::run_advice_sweep(desc, c, &pool, nullptr, "clutch.json", "gearbox.json");
    const pd::LimitFinding* found = nullptr;
    for (const pd::LimitFinding& f : sweep.input.findings) {
        if (f.kind == pd::LimitKind::GearboxInputTorque) found = &f;
    }
    REQUIRE(found != nullptr);
    CHECK(found->severity == pd::LimitSeverity::Exceeded);
    CHECK(found->part.slot == "gearbox");
    CHECK(found->part.file == "gearbox.json");
    CHECK(found->part.field == "/rated_input_torque_nm");
    CHECK(found->limit == Catch::Approx(100.0));
    CHECK(found->value > 150.0);
    REQUIRE(found->remedy.has_value());
    CHECK(found->remedy->required_margin == Catch::Approx(1.1 * found->value)); // the policy's gearbox_rating_remedy_margin
    const auto items = rg::build_advice(sweep.input, c);
    const rg::AdviceItem* item = find_item(items, "GearboxInputTorque@/rated_input_torque_nm");
    REQUIRE(item != nullptr);
    CHECK(item->severity == rg::AdviceSeverity::Critical);
    CHECK(item->lines[0] == "Gearbox is rated for 100 N*m.");

    // The policy reaches the check: a 3x remedy margin comes back in the finding. (The AdvisorConfig outlives the sweep.)
    c.policy.gearbox_rating_remedy_margin = 3.0;
    const rg::AdviceSweep wide = rg::run_advice_sweep(desc, c, &pool, nullptr);
    for (const pd::LimitFinding& f : wide.input.findings) {
        if (f.kind != pd::LimitKind::GearboxInputTorque) continue;
        REQUIRE(f.remedy.has_value());
        CHECK(f.remedy->required_margin == Catch::Approx(3.0 * f.value));
        CHECK(f.part.file.empty());
    }
    CHECK(wide.remedy_request.spec.policy == &c.policy);
}

TEST_CASE("advice remedies: Closed shows at once, Simulated waits for Find fix, solved and failed text", "[limit_advisor]") {
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    in.target_boost_bar = 1.8;
    // Clutch: Closed remedy, capacity 1500 -> at least 1800 (x1.3 margin 2340).
    pd::LimitFinding clutch = F(pd::LimitKind::ClutchCapacity, pd::LimitSeverity::Limiting, 6000.0, 1800, 1500, pd::LimitUnit::NewtonMetre, "/capacity_nm");
    pd::Remedy cr;
    cr.current = 1500;
    cr.required_min = 1800;
    cr.required_margin = 2340;
    cr.unit = pd::LimitUnit::NewtonMetre;
    cr.cost = pd::RemedyCost::Closed;
    clutch.remedy = cr;
    // Wastegate: Simulated and unsolved.
    pd::LimitFinding gate = F(pd::LimitKind::WastegateSaturated, pd::LimitSeverity::Limiting, 7000.0, 0.99, 0.98, pd::LimitUnit::Fraction, "/wastegate_area_m2");
    pd::Remedy sr;
    sr.current = 1.18e-3;
    sr.unit = pd::LimitUnit::SquareMetre;
    sr.cost = pd::RemedyCost::Simulated;
    sr.solved = false;
    sr.required_min = sr.required_margin = std::numeric_limits<double>::quiet_NaN();
    gate.remedy = sr;
    // Knock: a Simulated placeholder physics cannot solve: no button.
    pd::LimitFinding knock = F(pd::LimitKind::Knock, pd::LimitSeverity::Limiting, 6000.0, 6.0, 5.0, pd::LimitUnit::CrankDegree, "/knock/ron");
    knock.remedy = sr;
    in.findings = {clutch, gate, knock};
    auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 3);
    const auto* ci = find_item(items, "ClutchCapacity@/capacity_nm");
    REQUIRE(ci != nullptr);
    CHECK(ci->remedy_state == rg::RemedyState::Closed);
    CHECK(ci->remedy_text == "A clutch rated for about 2340 N*m holds it with margin (now 1500 N*m).");
    CHECK_FALSE(ci->can_solve);
    auto* gi = const_cast<rg::AdviceItem*>(find_item(items, "WastegateSaturated@/wastegate_area_m2"));
    REQUIRE(gi != nullptr);
    CHECK(gi->remedy_state == rg::RemedyState::Available);
    CHECK(gi->can_solve);
    CHECK(gi->remedy_text.empty());
    const auto* ki = find_item(items, "Knock@/knock/ron");
    REQUIRE(ki != nullptr);
    CHECK(ki->remedy_state == rg::RemedyState::None);
    CHECK_FALSE(ki->can_solve);

    pd::Remedy a = sr, b = sr;
    a.solved = true;
    a.required_min = 1.18e-3 * 1.215;
    a.required_margin = a.required_min * 1.1;
    b.solved = true;
    b.required_min = 1.18e-3 * 1.32;
    b.required_margin = b.required_min * 1.1;
    rg::apply_solved_remedy(*gi, {a, b}, c);
    CHECK(gi->remedy_state == rg::RemedyState::Solved);
    CHECK(gi->remedy_text == "Wastegate needs about 1.32x the area, 1.45x to be safe."); // the larger need wins
    rg::apply_solved_remedy(*gi, {a, sr}, c);
    CHECK(gi->remedy_state == rg::RemedyState::Failed);
    CHECK(gi->remedy_text.find("8x") != std::string::npos);
    rg::apply_solved_remedy(*gi, {}, c);
    CHECK(gi->remedy_state == rg::RemedyState::Failed);
}

TEST_CASE("advice severity vocabulary", "[limit_advisor]") {
    CHECK(rg::advice_severity(pd::LimitSeverity::Near) == rg::AdviceSeverity::Advice);
    CHECK(rg::advice_severity(pd::LimitSeverity::Limiting) == rg::AdviceSeverity::Warning);
    CHECK(rg::advice_severity(pd::LimitSeverity::Exceeded) == rg::AdviceSeverity::Critical);
    CHECK(std::string(rg::to_string(rg::AdviceSeverity::Warning)) == "warning");
}

// ---- the real thing ------------------------------------------------------------------------------------------------------

namespace {

namespace fs = std::filesystem;

struct World {
    rg::VehicleCatalog catalog;
    rg::SetupOptionTable table;
    rg::SetupContext ctx;
    std::string work_root;
};

World make_world() {
    World w;
    std::string err;
    auto catalog = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
    REQUIRE(catalog.has_value());
    w.catalog = *catalog;
    auto table = rg::load_setup_options(kRepo + "/data/vehicles/setup_options.json", &err);
    REQUIRE(table.has_value());
    w.table = *table;
    w.work_root = (fs::temp_directory_path() / "rg_advisor_test").generic_string();
    w.ctx.tyre_families_path = kRepo + "/data/tyres/families.json";
    w.ctx.work_root = w.work_root;
    w.ctx.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
    auto store = rg::EngineStore::open(kRepo + "/data/engines/engine_catalog.json", kRepo, "", &err);
    REQUIRE(store != nullptr);
    w.ctx.engines = std::shared_ptr<const rg::EngineStore>(std::move(store));
    return w;
}

rg::AdvisorRequest make_request(const World& w, const std::string& car, const std::string& setup_json, const std::string& key) {
    rg::AdvisorRequest r;
    const rg::CatalogEntry* e = w.catalog.find(car);
    REQUIRE(e != nullptr);
    r.entry = *e;
    r.table = w.table;
    r.context = w.ctx;
    std::string err;
    auto setup = rg::parse_setup(setup_json, w.table, "test", &err);
    INFO(err);
    REQUIRE(setup.has_value());
    r.setup = *setup;
    r.key = key;
    return r;
}

rg::AdvisorResult wait_done(rg::LimitAdvisor& advisor, double timeout_s = 180.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
        const auto r = advisor.result();
        if (!r.busy && !r.solving) return r;
        REQUIRE(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void print_items(const rg::AdvisorResult& r) {
    std::printf("ADVISOR maps %.2f s, sweep %.2f s, %zu items\n", r.maps_seconds, r.sweep_seconds, r.items.size());
    for (const auto& i : r.items) {
        std::printf("ADVISOR [%s] %s  (%s, option %s, remedy %s)\n", rg::to_string(i.severity), i.id.c_str(), i.variant.c_str(), i.option_id.c_str(),
                    rg::to_string(i.remedy_state));
        std::printf("%s\n", joined(i).c_str());
        if (!i.remedy_text.empty()) std::printf("  fix: %s\n", i.remedy_text.c_str());
    }
    std::fflush(stdout);
}

// The owner's setup (garage/setups/car_hyper_n2o.json, 2026-10-13): the huge turbos on a 15 bar boost target, an 8x clutch.
const char* kOwnerSetup = R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_hyper_n2o","options":{
  "clutch_capacity":8.0,"turbo_boost_target":8.600083819796385,"turbo_install_n2o":"hyper_twin_large_n2o"}})";
const char* kStockSetup = R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_hyper_n2o","options":{}})";

} // namespace

TEST_CASE("advisor: a failing setup reports an error and keeps the thread usable", "[limit_advisor]") {
    World w = make_world();
    rg::AdvisorConfig cfg = shipped_config();
    cfg.debounce_ms = 5;
    rg::LimitAdvisor advisor(cfg);
    auto bad = make_request(w, "car_hyper_n2o", kStockSetup, "bad");
    bad.setup.values["turbo_install_n2o"] = std::string("not_a_part");
    advisor.request(bad);
    const auto r = wait_done(advisor, 30.0);
    CHECK_FALSE(r.error.empty());
    CHECK_FALSE(r.have_result);
    advisor.request(bad); // the same key is one request
    advisor.cancel();
    CHECK_FALSE(advisor.result().busy);
}

TEST_CASE("advisor: the owner's extreme turbo setup is flagged (boost, rotor, choke), stock has no rotor/choke items", "[limit_advisor][long]") {
    World w = make_world();
    rg::LimitAdvisor advisor(shipped_config());
    advisor.request(make_request(w, "car_hyper_n2o", kOwnerSetup, "owner"));
    const auto owner = wait_done(advisor);
    INFO(owner.error);
    REQUIRE(owner.error.empty());
    REQUIRE(owner.have_result);
    print_items(owner);
    const auto* boost = find_item(owner.items, "BoostBelowTarget");
    REQUIRE(boost != nullptr);
    CHECK(boost->severity == rg::AdviceSeverity::Critical);
    CHECK(boost->title == "Excessive turbo boost target");
    CHECK(boost->lines[0].find("bar.") != std::string::npos);
    const auto* rotor = find_item(owner.items, "TurboRotorCap");
    REQUIRE(rotor != nullptr);
    CHECK(rotor->severity == rg::AdviceSeverity::Critical);
    CHECK(rotor->can_solve);
    const auto* choke = find_item(owner.items, "CompressorChoke@/max_mass_flow_kg_s");
    REQUIRE(choke != nullptr);
    CHECK(choke->severity >= rg::AdviceSeverity::Warning);

    // The Find fix button: wastegate search on the worker.
    advisor.solve(rotor->id);
    CHECK(advisor.result().solving); // set when the solve is queued, so wait_done cannot miss it
    const auto solved = wait_done(advisor);
    const auto* after = find_item(solved.items, "TurboRotorCap");
    REQUIRE(after != nullptr);
    CHECK((after->remedy_state == rg::RemedyState::Solved || after->remedy_state == rg::RemedyState::Failed));
    CHECK_FALSE(after->remedy_text.empty());
    std::printf("ADVISOR fix (%.2f s): %s\n", solved.last_solve_seconds, after->remedy_text.c_str());

    // Stock: the newest setup wins. The stock N2O car is not quiet (its gate nearly saturates, the spool-up is slow) but
    // the owner-setup problems must be absent: no rotor cap, no choke.
    advisor.request(make_request(w, "car_hyper_n2o", kStockSetup, "stock"));
    const auto stock = wait_done(advisor);
    REQUIRE(stock.error.empty());
    print_items(stock);
    CHECK(stock.key == "stock");
    CHECK(find_item(stock.items, "TurboRotorCap") == nullptr);
    CHECK(find_item(stock.items, "CompressorChoke@/max_mass_flow_kg_s") == nullptr);
    for (const auto& i : stock.items)
        if (i.variant == "spool") CHECK(i.severity == rg::AdviceSeverity::Advice);
}

// ---- stage-map coverage (physics BuildRule::TurboStageMapCoverage) and engine loader warnings ----

namespace {
pd::BuildIssue stage_issue(pd::LimitSeverity s, double coverage) {
    pd::BuildIssue b;
    b.rule = pd::BuildRule::TurboStageMapCoverage;
    b.severity = s;
    b.slot = "forced_induction";
    b.field_pointer = "/turbo";
    b.value = coverage;
    b.limit = 1.0;
    b.unit = pd::LimitUnit::Fraction;
    b.message = "turbo stage map: the 1.8 bar target ...";
    return b;
}
} // namespace

TEST_CASE("advice stage map: the physics issue becomes a finding with the coverage number (owner format)", "[limit_advisor][stage_map]") {
    const rg::AdvisorConfig c = shipped_config();
    REQUIRE(c.find_rule("TurboStageMapCoverage") != nullptr);
    rg::AdviceInput in;
    in.points = sweep_points(1.8);
    in.target_boost_bar = 1.8;
    in.build_issues.push_back(stage_issue(pd::LimitSeverity::Near, 0.688));
    auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].id == "TurboStageMapCoverage");
    CHECK(items[0].severity == rg::AdviceSeverity::Advice);
    CHECK(items[0].option_id == "turbo_boost_target");
    const std::string text = joined(items[0]);
    INFO(text);
    CHECK(text.find("68.8 %") != std::string::npos);
    CHECK(text.find("1.8 bar") != std::string::npos);
    CHECK(text.find('?') == std::string::npos);
    CHECK(text.find('{') == std::string::npos);

    // Limiting (needs a rotor speed above the map) uses the beyond_map variant and is louder.
    in.build_issues = {stage_issue(pd::LimitSeverity::Limiting, 0.25)};
    items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].severity == rg::AdviceSeverity::Warning);
    CHECK(items[0].variant == "beyond_map");
    CHECK(joined(items[0]).find("25.0 %") != std::string::npos);

    // The limit policy decides what is told: a policy that does not warn at Near drops it.
    rg::AdvisorConfig quiet = c;
    quiet.policy.warn_at = pd::WarnAt::Limiting;
    in.build_issues = {stage_issue(pd::LimitSeverity::Near, 0.688)};
    CHECK(rg::build_advice(in, quiet).empty());
}

TEST_CASE("advice loader warnings: passed through as lines, the stage-map advisory is not doubled", "[limit_advisor][stage_map]") {
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(1.0);
    in.loader_warnings = {"engine: Near: turbo stage map: the 1.8 bar target ...", "engine: map grid is coarse", "engine: second notice"};
    const auto items = rg::build_advice(in, c);
    REQUIRE(items.size() == 1);
    CHECK(items[0].id == "LoaderNotice");
    REQUIRE(items[0].lines.size() == 2);
    CHECK(items[0].lines[0] == "engine: map grid is coarse");
    in.loader_warnings = {"engine: Near: turbo stage map: x"};
    CHECK(rg::build_advice(in, c).empty());
}

TEST_CASE("advisor: the hyper n2o setup shows the stage-map finding, a stock sedan does not", "[limit_advisor][stage_map]") {
    World w = make_world();
    rg::LimitAdvisor advisor(shipped_config());
    advisor.request(make_request(w, "car_hyper_n2o", kStockSetup, "hyper"));
    const auto hyper = wait_done(advisor, 600.0);
    INFO(hyper.error);
    REQUIRE(hyper.error.empty());
    print_items(hyper);
    const auto* stage = find_item(hyper.items, "TurboStageMapCoverage");
    REQUIRE(stage != nullptr);
    CHECK(stage->severity == rg::AdviceSeverity::Advice);
    // The coverage is physics' share of the engine's air flow (16 flow steps: a multiple of 6.25 %), shown with one decimal.
    // 68.8 % was measured with the full-quality maps; the advisor's sketch maps give another VE and so another step.
    REQUIRE(stage->lines.size() >= 2);
    CHECK(stage->lines[1].find("The turbo map covers ") == 0);
    CHECK(stage->lines[1].find(".") != std::string::npos);
    CHECK(stage->lines[1].find(" % of the engine's air flow.") != std::string::npos);

    advisor.request(make_request(w, "car_sedan", R"({"format":"racing_game.vehicle_setup/1","vehicle":"car_sedan","options":{}})", "sedan"));
    const auto sedan = wait_done(advisor, 600.0);
    REQUIRE(sedan.error.empty());
    CHECK(find_item(sedan.items, "TurboStageMapCoverage") == nullptr);
}

TEST_CASE("advisor: a dual-clutch gearbox's shift clutches are checked and an uprated clutch is offered", "[limit_advisor][gearbox]") {
    // Owner 2026-10-18: the hyper_7dct's shift clutches hold 1900 N*m (static x1.15 = 2185) but the N2O shot makes about
    // 3150 N*m; the car has no ClutchDesc, so physics_sim's check_driveline_limits never saw them.
    pd::PowertrainDesc powertrain;
    pd::ComponentDesc comp;
    comp.id = "gearbox";
    pd::GearboxDesc gb;
    gb.type = pd::GearboxType::DualClutch;
    gb.shift_clutch.capacity_nm = 1900.0;
    gb.shift_clutch.static_factor = 1.15;
    comp.params = gb;
    powertrain.components.push_back(comp);
    const auto points = [](double peak) {
        std::vector<pd::DynoPoint> pts(3);
        pts[0].rpm = 3000; pts[0].torque_nm = 0.6 * peak;
        pts[1].rpm = 6000; pts[1].torque_nm = peak;
        pts[2].rpm = 8000; pts[2].torque_nm = 0.8 * peak;
        return pts;
    };
    const pd::LimitPolicy policy = pd::default_limit_policy();

    CHECK(rg::check_shift_clutch_limits(points(1387.0), powertrain, "g.json", &policy).empty()); // stock V8: holds with margin
    const auto found = rg::check_shift_clutch_limits(points(3150.0), powertrain, "g.json", &policy);
    REQUIRE(found.size() == 1);
    CHECK(found[0].kind == pd::LimitKind::ClutchCapacity);
    CHECK(found[0].severity == pd::LimitSeverity::Limiting);
    CHECK(found[0].rpm == Catch::Approx(6000.0));
    CHECK(found[0].value == Catch::Approx(3150.0));
    CHECK(found[0].limit == Catch::Approx(1900.0 * 1.15));
    CHECK(found[0].part.slot == "gearbox");
    CHECK(found[0].part.file == "g.json");
    CHECK(found[0].part.field == "/shift_clutch/capacity_nm");
    REQUIRE(found[0].remedy.has_value());
    CHECK(found[0].remedy->current == Catch::Approx(1900.0));
    CHECK(found[0].remedy->required_min == Catch::Approx(3150.0 / 1.15));
    CHECK(found[0].remedy->required_margin == Catch::Approx(policy.driveline_torque_margin * 3150.0 / 1.15));
    // A manual-box car (shift_clutch left at 0) yields nothing.
    pd::PowertrainDesc manual;
    pd::ComponentDesc mc;
    mc.id = "gearbox";
    mc.params = pd::GearboxDesc{};
    manual.components.push_back(mc);
    CHECK(rg::check_shift_clutch_limits(points(3150.0), manual, "", &policy).empty());

    // The advice names the gearbox's clutches and points at their garage option, not the manual clutch's.
    const rg::AdvisorConfig c = shipped_config();
    rg::AdviceInput in;
    in.points = sweep_points(2.0);
    in.findings = found;
    const auto items = rg::build_advice(in, c);
    const rg::AdviceItem* item = find_item(items, "ClutchCapacity@/shift_clutch/capacity_nm");
    REQUIRE(item != nullptr);
    CHECK(item->variant == "shift_clutch");
    CHECK(item->option_id == "shift_clutch_capacity");
    CHECK(item->title == "The gearbox clutches will slip");
    REQUIRE(item->lines.size() == 3);
    CHECK(item->lines[0] == "The gearbox clutches hold about 2185 N*m.");
    CHECK(item->remedy_state == rg::RemedyState::Closed);
    CHECK(item->remedy_text.find("Gearbox clutches rated for about") == 0);
    // The manual clutch keeps its own text and option.
    in.findings[0].part.field = "/capacity_nm";
    const auto manual_items = rg::build_advice(in, c);
    const rg::AdviceItem* mi = find_item(manual_items, "ClutchCapacity@/capacity_nm");
    REQUIRE(mi != nullptr);
    CHECK(mi->option_id == "clutch_capacity");
    CHECK(mi->title == "The clutch will slip");
}
