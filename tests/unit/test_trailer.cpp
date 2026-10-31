// test_trailer.cpp - the engine-neutral half of towing (rg/trailer.h, farm attachment plan step A2): the config, the
// hook-up planning, the pose alignment, the overload rule, the parking leg, the advice lines, the save record and the
// coupled-set authority. Tag [trailer]. Each test names its sabotage ("sabotage:"); the ones marked "run" were really
// applied to the source and the test failed, the numbers are in the line.
#include "rg/trailer.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;
using Catch::Approx;

std::shared_ptr<const rg::TrailerConfig> load_config() {
    std::string err;
    auto c = rg::load_trailer_config(kRepo + "/data/trailers/trailers.json", &err);
    INFO(err);
    REQUIRE(c.has_value());
    return std::make_shared<const rg::TrailerConfig>(std::move(*c));
}

ps::machine::HitchPointDesc point(const char* name, ps::machine::HitchRole role, const char* cls, ps::Vec3 at) {
    ps::machine::HitchPointDesc p;
    p.name = name;
    p.role = role;
    p.hitch.name = cls;
    p.hitch.max_latch_error_m = 0.15;
    p.position_local = at;
    return p;
}

ps::Pose pose_at(double x, double y, double z, double yaw = 0.0) {
    ps::Pose p;
    p.position = ps::Vec3{x, y, z};
    p.orientation = ps::Quat::from_axis_angle(ps::Vec3::unit_z(), yaw);
    return p;
}

struct Pair {
    std::vector<ps::machine::HitchPointDesc> car, trailer;
    Pair() {
        car.push_back(point("socket_tow_rear", ps::machine::HitchRole::Towing, "ball", ps::Vec3{-2.2, 0.0, -0.2}));
        trailer.push_back(point("eye", ps::machine::HitchRole::Towed, "ball", ps::Vec3{4.5, 0.0, -0.63}));
    }
    rg::HookupGeometry geometry(ps::Pose car_pose, ps::Pose trailer_pose) const {
        rg::HookupGeometry g;
        g.towing_points = &car;
        g.towed_points = &trailer;
        g.car = car_pose;
        g.trailer = trailer_pose;
        g.trailer_pivot_local = ps::Vec3{-0.45, 0.0, -0.51};
        return g;
    }
};

} // namespace

TEST_CASE("trailer config: the shipped file loads with its classes, kinds, loads and policy", "[trailer][io]") {
    const auto c = load_config();
    REQUIRE(c->classes.size() >= 2);
    CHECK(c->find_class("ball") != nullptr);
    CHECK(c->find_class("drawbar_pin") != nullptr);
    CHECK(c->find_class("fifth_wheel") == nullptr); // a class that is not listed is never offered
    const rg::TrailerKind* k = c->find_kind("trailer_tipper");
    REQUIRE(k != nullptr);
    CHECK(k->loads.size() == 3);
    REQUIRE(k->find_load(k->default_load) != nullptr);
    CHECK(k->find_load("full")->mass_kg > k->find_load("empty")->mass_kg);
    CHECK(std::filesystem::exists(k->vehicle_path));
    CHECK(std::filesystem::exists(k->model_path));
    CHECK(c->hookup.reach_m > 0.0);
    CHECK(c->hookup.max_speed_m_s > 0.0);
    // sabotage: a loader that skips resolving vehicle/model against the config's folder fails the two exists() checks
}

TEST_CASE("trailer config: every malformed field is rejected with the field named", "[trailer][io]") {
    std::string good_text;
    {
        std::ifstream in(kRepo + "/data/trailers/trailers.json", std::ios::binary);
        good_text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string dir = kRepo + "/data/trailers";
    const auto replace_once = [&](const std::string& from, const std::string& to) {
        std::string t = good_text;
        const auto at = t.find(from);
        REQUIRE(at != std::string::npos);
        t.replace(at, from.size(), to);
        return t;
    };
    const auto rejects = [&](const std::string& text, const char* needle) {
        std::string err;
        const auto c = rg::parse_trailer_config(text, dir, "t.json", &err);
        INFO(err);
        CHECK_FALSE(c.has_value());
        CHECK(err.find(needle) != std::string::npos);
    };
    {
        std::string err;
        CHECK(rg::parse_trailer_config(good_text, dir, "t.json", &err).has_value());
    }
    rejects(replace_once("rg.trailers/1", "rg.trailers/2"), "format");
    rejects(replace_once("\"default_trailer\": \"trailer_tipper\"", "\"default_trailer\": \"nope\""), "default_trailer");
    rejects(replace_once("\"default_load\":", "\"default_loadx\":"), "default_load");
    rejects(replace_once("\"reach_m\": 3.5", "\"reach_m\": -1.0"), "reach_m");
    rejects(replace_once("\"break_d_multiple\": 4.0", "\"break_d_multiple\": 0.0"), "break_d_multiple");
    rejects(replace_once("\"channel_scope\": \"trailer\"", "\"channel_scope\": \"trailer.x\""), "channel_scope");
    rejects("{ not json", "JSON");
    // sabotage (run): dropping the `reach_m > 0` check makes the reach_m case load, so the rejects() CHECK(has_value false) fails
}

TEST_CASE("trailer hook-up planning: the nearest mating pair, and a verdict for each way it can be refused", "[trailer]") {
    const auto cfg = load_config();
    const Pair pair;
    // The car at the origin facing +x: tow point at (-2.2, 0, -0.2). The trailer's eye 1.0 m behind it, at the same height.
    const auto trailer_at = [&](double eye_x, double eye_z, double yaw = 0.0) {
        // Place the trailer so its eye (local 4.5, 0, -0.63) lands at (eye_x, 0, eye_z).
        ps::Pose p = pose_at(0, 0, 0, yaw);
        const ps::Vec3 off = p.orientation.rotate(ps::Vec3{4.5, 0.0, -0.63});
        p.position = ps::Vec3{eye_x - off.x, -off.y, eye_z - off.z};
        return p;
    };
    SECTION("ok: one metre behind, level") {
        const rg::HookupPlan plan = rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2)));
        CHECK(plan.verdict == rg::HookupVerdict::Ok);
        CHECK(plan.distance_m == Approx(1.0).margin(1e-9));
        CHECK(plan.towing_point == "socket_tow_rear");
        CHECK(plan.towed_point == "eye");
        CHECK(plan.class_name == "ball");
        CHECK_FALSE(plan.within_latch);
    }
    SECTION("within the latch limit") {
        const rg::HookupPlan plan = rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-2.3, -0.2)));
        CHECK(plan.verdict == rg::HookupVerdict::Ok);
        CHECK(plan.within_latch);
    }
    SECTION("too far") {
        const rg::HookupPlan plan = rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-2.2 - cfg->hookup.reach_m - 0.5, -0.2)));
        CHECK(plan.verdict == rg::HookupVerdict::TooFar);
        CHECK_FALSE(plan.message.empty());
    }
    SECTION("pointing the wrong way") {
        const rg::HookupPlan plan = rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2, 2.0)));
        CHECK(plan.verdict == rg::HookupVerdict::HeadingOff);
        CHECK(plan.heading_deg == Approx(2.0 * 180.0 / 3.14159265358979).margin(1e-6));
    }
    SECTION("moving") {
        rg::HookupGeometry g = pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2));
        g.car_velocity = ps::Vec3{cfg->hookup.max_speed_m_s + 0.3, 0.0, 0.0};
        CHECK(rg::plan_hookup(*cfg, g).verdict == rg::HookupVerdict::TooFast);
        g.car_velocity = ps::Vec3{};
        g.trailer_velocity = ps::Vec3{0.0, cfg->hookup.max_speed_m_s + 0.3, 0.0};
        CHECK(rg::plan_hookup(*cfg, g).verdict == rg::HookupVerdict::TooFast);
    }
    SECTION("higher or lower than the jack can wind") {
        const double too_high = cfg->hookup.max_vertical_m + 0.1;
        CHECK(rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2 - too_high))).verdict == rg::HookupVerdict::OutOfJackRange);
        CHECK(rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2 + too_high))).verdict == rg::HookupVerdict::OutOfJackRange);
        const double fine = cfg->hookup.max_vertical_m - 0.05;
        CHECK(rg::plan_hookup(*cfg, pair.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2 - fine))).verdict == rg::HookupVerdict::Ok);
    }
    SECTION("no pair: another class, or two towing points") {
        Pair other;
        other.trailer[0].hitch.name = "drawbar_pin";
        CHECK(rg::plan_hookup(*cfg, other.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2))).verdict == rg::HookupVerdict::NoPair);
        Pair same_role;
        same_role.trailer[0].role = ps::machine::HitchRole::Towing;
        CHECK(rg::plan_hookup(*cfg, same_role.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2))).verdict == rg::HookupVerdict::NoPair);
        Pair unlisted;
        unlisted.car[0].hitch.name = unlisted.trailer[0].hitch.name = "fifth_wheel";
        CHECK(rg::plan_hookup(*cfg, unlisted.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2))).verdict == rg::HookupVerdict::NoPair);
    }
    SECTION("the nearest of two mating pairs is the one planned") {
        Pair two;
        two.trailer.push_back(point("far_eye", ps::machine::HitchRole::Towed, "ball", ps::Vec3{4.1, 0.0, -0.63}));
        two.car.push_back(point("socket_tow_front", ps::machine::HitchRole::Towing, "ball", ps::Vec3{2.2, 0.0, -0.2}));
        const rg::HookupPlan plan = rg::plan_hookup(*cfg, two.geometry(pose_at(0, 0, 0), trailer_at(-3.2, -0.2)));
        CHECK(plan.towing_point == "socket_tow_rear");
        CHECK(plan.towed_point == "eye");
    }
    // sabotage (run): `<=` -> `>=` in the nearest-pair choice picks far_eye / the front point and fails the last section;
    // comparing distance_m with reach_m after the heading test reorders the verdicts and fails "pointing the wrong way".
}

TEST_CASE("trailer pose alignment: the eye lands on the target to a tenth of a millimetre, the axles keep their place", "[trailer]") {
    const ps::Vec3 eye_local{4.5, 0.0, -0.63};
    const ps::Vec3 pivot_local{-0.45, 0.0, -0.51};
    const ps::Vec3 target{-2.9, 0.3, 0.55};
    for (const double yaw : {0.0, 0.4, -1.1, 3.0}) {
        for (const double dz : {-0.3, -0.1, 0.0, 0.12, 0.32}) {
            ps::Pose start = pose_at(-8.0, 1.0, 0.81, yaw);
            const ps::Vec3 eye0 = start.transform_point(eye_local);
            const ps::Vec3 want{target.x, target.y, eye0.z + dz};
            const ps::Pose done = rg::align_trailer_pose(start, eye_local, pivot_local, want);
            const ps::Vec3 eye = done.transform_point(eye_local);
            INFO("yaw " << yaw << " dz " << dz);
            CHECK(eye.x == Approx(want.x).margin(1e-4));
            CHECK(eye.y == Approx(want.y).margin(1e-4));
            CHECK(eye.z == Approx(want.z).margin(1e-4));
            // The orientation is a pure pitch about the trailer's lateral axis: no roll, no yaw change.
            const ps::Vec3 lateral0 = start.orientation.rotate(ps::Vec3::unit_y());
            const ps::Vec3 lateral = done.orientation.rotate(ps::Vec3::unit_y());
            CHECK(lateral.x == Approx(lateral0.x).margin(1e-6));
            CHECK(lateral.y == Approx(lateral0.y).margin(1e-6));
            CHECK(lateral.z == Approx(0.0).margin(1e-6));
        }
    }
    // The pivot (the middle of the axles) moves by the shift in plan only, never in height beyond the pitch's own cos term.
    ps::Pose start = pose_at(-8.0, 1.0, 0.81, 0.0);
    const ps::Pose done = rg::align_trailer_pose(start, eye_local, pivot_local, ps::Vec3{target.x, target.y, start.transform_point(eye_local).z + 0.2});
    CHECK(std::fabs(done.transform_point(pivot_local).z - start.transform_point(pivot_local).z) < 1e-6);
    // sabotage (run): pitching about the chassis origin instead of pivot_local lifts the axles 5 cm in the last check
    // (pivot z moved 0.0483); a sign flip on `rate` puts the eye 2x dz off in every dz != 0 case.
}

TEST_CASE("trailer: heading_yaw reads the forward axis, and the side axis when the nose points up", "[trailer]") {
    CHECK(rg::heading_yaw(pose_at(0, 0, 0, 0.7).orientation) == Approx(0.7).margin(1e-12));
    CHECK(rg::heading_yaw(pose_at(0, 0, 0, -2.5).orientation) == Approx(-2.5).margin(1e-12));
    // Standing on its tail: forward is vertical, the left axis (yaw + 90 deg) carries the heading.
    const ps::Quat tail = ps::Quat::from_axis_angle(ps::Vec3::unit_z(), 0.9) * ps::Quat::from_axis_angle(ps::Vec3::unit_y(), -1.5707963267948966);
    CHECK(rg::heading_yaw(tail) == Approx(0.9).margin(1e-9));
    // sabotage: always using the forward axis returns atan2(~0, ~0) = garbage on the tail case
}

TEST_CASE("trailer overload rule: D value and the hold time", "[trailer]") {
    // ISO 7641-1: D = g * T * R / (T + R), T the towing vehicle, R the trailer.
    CHECK(rg::iso_d_value_n(1500.0, 1500.0) == Approx(9.81 * 750.0));
    CHECK(rg::iso_d_value_n(1300.0, 4500.0) == Approx(9.81 * 1300.0 * 4500.0 / 5800.0));
    CHECK(rg::iso_d_value_n(0.0, 4500.0) == 0.0);
    CHECK(rg::iso_d_value_n(-1.0, -1.0) == 0.0);

    rg::HitchOverload o;
    const double limit = 1000.0, hold = 0.05, dt = 1.0 / 240.0;
    // Under the limit never trips.
    for (int i = 0; i < 1000; ++i) CHECK_FALSE(o.update(900.0, limit, hold, dt));
    // A spike shorter than the hold is a shock the hitch survives, and a gap restarts the count.
    for (int round = 0; round < 5; ++round) {
        for (int i = 0; i < 9; ++i) CHECK_FALSE(o.update(5000.0, limit, hold, dt)); // 9 ticks = 37.5 ms < 50 ms
        CHECK_FALSE(o.update(900.0, limit, hold, dt));
    }
    // Held above it trips exactly when the hold is reached: ceil(0.05 * 240) = 12 ticks.
    int ticks = 0;
    bool tripped = false;
    while (!tripped && ticks < 100) {
        tripped = o.update(5000.0, limit, hold, dt);
        ++ticks;
    }
    CHECK(tripped);
    CHECK(ticks == 12);
    // And re-arms.
    CHECK_FALSE(o.update(5000.0, limit, hold, dt));
    // sabotage (run): not resetting over_s on a drop below the limit trips on the 3rd round of the spike loop; a `>=` on the
    // force trips at exactly the limit in the first loop
}

TEST_CASE("trailer parking leg: pushes up only while deployed with the foot under the ground, capped, damped", "[trailer]") {
    rg::TrailerJackSpec spec;
    spec.foot_local = ps::Vec3{5.0, 0.0, -0.8};
    spec.stiffness_n_m = 200000.0;
    spec.damping_n_s_m = 8000.0;
    spec.max_force_n = 60000.0;
    rg::TrailerJack jack(ps::Frame(ps::BodyId{}), spec);
    ps::ForceElementContext ctx;
    ctx.frame_pose = pose_at(0, 0, 1.0); // foot at z = 0.2
    // Not deployed: nothing.
    jack.set_ground(false, true, 0.5);
    CHECK(jack.evaluate(ctx).force.z == 0.0);
    // No ground below: nothing.
    jack.set_ground(true, false, 0.5);
    CHECK(jack.evaluate(ctx).force.z == 0.0);
    // Foot above the ground: nothing.
    jack.set_ground(true, true, 0.1);
    CHECK(jack.evaluate(ctx).force.z == 0.0);
    // 5 cm deep: k * depth, at rest, with the moment of a force at the foot.
    jack.set_ground(true, true, 0.25);
    const ps::Wrench w = jack.evaluate(ctx);
    CHECK(w.force.z == Approx(200000.0 * 0.05));
    CHECK(w.force.x == 0.0);
    CHECK(w.torque.y == Approx(-5.0 * 10000.0)); // arm (5, 0, -0.8) x (0, 0, F) = (0, -5F, 0)
    // Sinking in (foot moving down at 0.1 m/s) pushes harder, rising pushes less, and never pulls.
    ctx.frame_motion.linear = ps::Vec3{0.0, 0.0, -0.1};
    CHECK(jack.evaluate(ctx).force.z == Approx(10000.0 + 800.0));
    ctx.frame_motion.linear = ps::Vec3{0.0, 0.0, 5.0};
    CHECK(jack.evaluate(ctx).force.z == 0.0);
    // Capped.
    ctx.frame_motion.linear = ps::Vec3{};
    jack.set_ground(true, true, 1.0);
    CHECK(jack.evaluate(ctx).force.z == Approx(60000.0));
    // sabotage (run): dropping the clamp's lower bound lets the damper pull (-0.0 -> -10000 in the rising case); dropping the
    // `deployed_` test fails the first check; the torque arm taken from the frame origin instead of the foot fails the torque check
}

TEST_CASE("trailer advice lines: only what the policy warns about, with the configured text", "[trailer]") {
    const auto cfg = load_config();
    std::vector<ps::drivetrain::LimitFinding> raw;
    ps::drivetrain::LimitFinding mass;
    mass.kind = ps::drivetrain::LimitKind::TowedMass;
    mass.severity = ps::drivetrain::LimitSeverity::Exceeded;
    mass.value = 4500.0;
    mass.limit = 1200.0;
    raw.push_back(mass);
    ps::drivetrain::LimitFinding tongue = mass;
    tongue.kind = ps::drivetrain::LimitKind::TongueLoad;
    tongue.severity = ps::drivetrain::LimitSeverity::Near;
    tongue.value = 900.0;
    tongue.limit = 1000.0;
    raw.push_back(tongue);
    ps::drivetrain::LimitFinding other = mass; // a finding of another kind is not towing advice
    other.kind = ps::drivetrain::LimitKind::RevLimiter;
    raw.push_back(other);

    const auto lines = rg::tow_findings(*cfg, raw, "Sedan", "Tipper");
    REQUIRE(lines.size() == 2);
    CHECK(lines[0].kind == ps::drivetrain::LimitKind::TowedMass);
    CHECK(lines[0].text == "The trailer (4500 kg) is over this car's towing rating of 1200 kg.");
    CHECK(lines[1].kind == ps::drivetrain::LimitKind::TongueLoad);
    CHECK(lines[1].text == "The nose weight (900 N) is close to this car's tongue rating of 1000 N.");

    // A policy that does not warn at Near drops the second line.
    auto strict = std::make_shared<rg::TrailerConfig>(*cfg);
    strict->policy.warn_at = ps::drivetrain::WarnAt::Exceeded;
    CHECK(rg::tow_findings(*strict, raw, "Sedan", "Tipper").size() == 1);
    // sabotage (run): not filtering by kind adds the RevLimiter line (3 != 2); ignoring the policy keeps the Near line
}

TEST_CASE("trailer text templates: known placeholders fill, unknown ones stay as written", "[trailer]") {
    CHECK(rg::render_trailer_text("a {x} b {y} c {x}", {{"x", "1"}, {"y", "two"}}) == "a 1 b two c 1");
    CHECK(rg::render_trailer_text("{unknown} {x", {{"x", "1"}}) == "{unknown} {x");
    CHECK(rg::render_trailer_text("", {{"x", "1"}}).empty());
}

TEST_CASE("trailer record: save and load round trip, bad files are refused", "[trailer][io]") {
    const std::string path = (std::filesystem::temp_directory_path() / "rg_trailer_record_test" / "trailer.json").generic_string();
    rg::TrailerRecord r;
    r.present = true;
    r.kind = "trailer_tipper";
    r.load = "half";
    r.coupled = true;
    std::string err;
    REQUIRE(rg::save_trailer_record(path, r, &err));
    const auto back = rg::load_trailer_record(path, &err);
    REQUIRE(back.has_value());
    CHECK(back->present);
    CHECK(back->kind == "trailer_tipper");
    CHECK(back->load == "half");
    CHECK(back->coupled);
    CHECK_FALSE(rg::load_trailer_record(path + ".missing", &err).has_value());
    CHECK_FALSE(rg::parse_trailer_record("{\"format\": \"other\"}", &err).has_value());
    CHECK_FALSE(rg::parse_trailer_record("{\"format\": \"rg.trailer_state/1\", \"present\": true}", &err).has_value());
    CHECK_FALSE(rg::parse_trailer_record("[1]", &err).has_value());
    std::filesystem::remove_all(std::filesystem::path(path).parent_path());
    // sabotage: writing "coupled" under another key fails the round trip's coupled check
}

TEST_CASE("coupled set authority: one authority per set, the towing vehicle's owner", "[trailer]") {
    rg::CoupledSetAuthority a;
    CHECK(a.register_vehicle(1, 100));
    CHECK(a.register_vehicle(1, 100)); // again, same peer
    CHECK_FALSE(a.register_vehicle(1, 200));
    CHECK(a.register_vehicle(2, 200));
    CHECK(a.register_vehicle(10, 100)); // peer 100's trailer

    CHECK(a.authority(1) == 100);
    CHECK(a.authority(999) == 0);
    // Only the owner of the towing vehicle builds a set.
    CHECK_FALSE(a.couple(1, 10, 200));
    CHECK_FALSE(a.couple(2, 10, 200)); // peer 200 does not own the trailer either
    CHECK(a.couple(1, 10, 100));
    CHECK(a.authority(10) == 100);
    // A trailer already in a set cannot be taken by another vehicle, not even by the same peer; no self, no loop.
    CHECK_FALSE(a.couple(2, 10, 200));
    CHECK(a.register_vehicle(3, 100));
    CHECK_FALSE(a.couple(3, 10, 100));
    CHECK_FALSE(a.couple(1, 1, 100));
    CHECK_FALSE(a.couple(10, 1, 100)); // 1 tows 10: 10 towing 1 would be a loop
    // A chain: 3 tows 1 (1 is not a trailer), so 1 and 10 follow 3's owner - the same peer.
    CHECK(a.couple(3, 1, 100));
    CHECK(a.authority(10) == 100);
    // Uncoupling needs the set's authority.
    CHECK_FALSE(a.uncouple(10, 200));
    CHECK(a.uncouple(10, 100));
    CHECK_FALSE(a.uncouple(10, 100)); // no longer coupled
    // A peer leaving releases its vehicles and the trailers behind them; another peer's vehicles stay.
    CHECK(a.couple(1, 10, 100));
    const auto released = a.release_peer(100);
    CHECK(released.size() >= 3);
    CHECK(a.authority(1) == 0);
    CHECK(a.authority(10) == 0);
    CHECK(a.authority(2) == 200);
    CHECK(a.vehicle_count() == 1);
    // sabotage (run): letting couple() skip the owner test accepts the first `couple(1, 10, 200)`; a root_of() that stops at the
    // first hop gives the wrong authority for the chain 3 -> 1 -> 10
}
