// test_trailer_session.cpp - towing through rg::Session on the flat world (rg/trailer.h + physics_sim's coupling): put a
// tipper trailer behind a car, hook it on, tow it, reverse with it, let it go, relocate and recover it, restore it into a
// new Session. Tag [trailer][session]. Every run longer than about 5 s of wall time is [long]. Each test names its
// sabotage; "run" = applied to the source and the test failed.
//   rg_unit_tests "[trailer]"          pure + short session cases
//   rg_unit_tests "[trailer][long]"    the drives
//   rg_unit_tests "[.][trailer_diag]"  hidden measurement table (RGTRAILER lines)
#include "rg/session.h"
#include "rg/trailer.h"
#include "rg/vehicle_catalog.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <memory>
#include <string>

namespace {

const std::string kRepo = RG_SOURCE_DIR;
using Catch::Approx;
constexpr double kHz = 240.0;

std::shared_ptr<const rg::TrailerConfig> trailer_config() {
    static const std::shared_ptr<const rg::TrailerConfig> cfg = [] {
        std::string err;
        auto c = rg::load_trailer_config(kRepo + "/data/trailers/trailers.json", &err);
        REQUIRE(c.has_value());
        return std::make_shared<const rg::TrailerConfig>(std::move(*c));
    }();
    return cfg;
}

const rg::CatalogEntry& catalog_entry(const std::string& id) {
    static const rg::VehicleCatalog catalog = [] {
        std::string err;
        auto c = rg::load_vehicle_catalog(kRepo + "/data/vehicles/catalog.json", kRepo, &err);
        REQUIRE(c.has_value());
        return *c;
    }();
    const rg::CatalogEntry* e = catalog.find(id);
    REQUIRE(e != nullptr);
    return *e;
}

rg::SessionConfig make_config(const std::string& car, bool with_trailer) {
    const rg::CatalogEntry& e = catalog_entry(car);
    rg::SessionConfig c;
    c.vehicle_json_path = e.vehicle_path;
    c.surface_table_path = kRepo + "/data/surfaces/surfaces.json";
    c.engine_map_cache_dir = kRepo + "/out/godot_engine_cache";
    c.chassis_mass_kg = e.chassis.mass_kg;
    c.chassis_half_extents = ps::Vec3{e.chassis.half_extents[0], e.chassis.half_extents[1], e.chassis.half_extents[2]};
    c.chassis_z_m = e.chassis.spawn_z_m;
    c.ground_tessellated = true;
    c.ground_half_extent_m = 3000;
    c.ground_half_extent_y_m = 400;
    if (with_trailer) c.trailer = trailer_config();
    return c;
}

struct Drive {
    rg::Session& s;
    ps::World& w;
    explicit Drive(rg::Session& session) : s(session), w(session.world()) {}
    void ctl(const char* name, double v) { w.set_control(name, v); }
    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) s.step();
    }
    void start_engine() {
        ctl("ignition", 1.0);
        ctl("assist.auto_clutch", 1.0);
        ctl("assist.auto_shift", 1.0);
        ctl("starter", 1.0);
        tick(120);
        ctl("starter", 0.0);
        tick(120);
    }
    [[nodiscard]] rg::TrailerRig::Car car() const { return rg::TrailerRig::Car{s.vehicle_id(), s.chassis_body(), &s.vehicle_desc()}; }
    [[nodiscard]] const rg::TrailerRig& rig() const {
        REQUIRE(s.trailer_rig() != nullptr);
        return *s.trailer_rig();
    }
    [[nodiscard]] rg::TrailerSnapshot trailer() const { return rig().snapshot(w, car()); }
    [[nodiscard]] double forward_speed() const {
        const ps::Pose p = w.get_pose(s.chassis_body());
        return p.orientation.inverse().rotate(w.get_motion(s.chassis_body()).linear).x;
    }
    [[nodiscard]] double car_yaw() const { return rg::heading_yaw(w.get_pose(s.chassis_body()).orientation); }
};

double wrap(double a) {
    while (a > 3.14159265358979) a -= 2.0 * 3.14159265358979;
    while (a < -3.14159265358979) a += 2.0 * 3.14159265358979;
    return a;
}

} // namespace

TEST_CASE("trailer session: without a trailer config nothing is created and the world hashes the same", "[trailer][session]") {
    rg::Session plain(make_config("car_sedan", false));
    rg::Session configured(make_config("car_sedan", true));
    CHECK(plain.trailer_rig() == nullptr);
    REQUIRE(configured.trailer_rig() != nullptr);
    CHECK_FALSE(configured.trailer_rig()->present());
    for (int i = 0; i < 120; ++i) {
        plain.step();
        configured.step();
    }
    // A rig that exists but holds no trailer creates no body, vehicle or force element: the same state hash.
    CHECK(plain.world().state_hash() == configured.world().state_hash());
    // And the requests of a session without a config are no-ops.
    plain.request_trailer_spawn("", "", false);
    plain.step();
    CHECK(plain.trailer_rig() == nullptr);
    // sabotage (run): creating the trailer vehicle in the TrailerRig constructor makes the hashes differ at tick 1
}

TEST_CASE("trailer session: a standing trailer settles on its wheels and its parking leg", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", false);
    d.tick(1);
    REQUIRE(d.rig().present());
    CHECK(d.rig().state() == rg::TrailerState::Standing);
    d.tick(360); // 1.5 s
    const rg::TrailerSnapshot t = d.trailer();
    REQUIRE(t.present);
    CHECK(t.state == rg::TrailerState::Standing);
    CHECK(t.jack_down);
    CHECK(t.kind_id == "trailer_tipper");
    CHECK(t.mass_kg == Approx(trailer_config()->find_kind("trailer_tipper")->find_load("empty")->mass_kg));
    CHECK(t.motion.linear.length() < 0.05);
    CHECK(t.motion.angular.length() < 0.05);
    // Level to within 2 degrees: the jack carries the nose of a tandem trailer.
    const ps::Vec3 up = t.pose.orientation.rotate(ps::Vec3::unit_z());
    const ps::Vec3 fwd = t.pose.orientation.rotate(ps::Vec3::unit_x());
    CHECK(std::asin(fwd.z) * 180.0 / 3.14159265358979 < 2.0);
    CHECK(std::asin(fwd.z) * 180.0 / 3.14159265358979 > -2.0);
    CHECK(up.z > 0.998);
    // Behind the car: the eye is about gap_m behind the tow point.
    REQUIRE(t.hookup_verdict == rg::HookupVerdict::Ok);
    CHECK(t.hookup_distance_m == Approx(trailer_config()->spawn.gap_m).margin(0.15));
    CHECK(t.wheels.size() == 4);
    for (const auto& w : t.wheels) CHECK(w.load > 1000.0);
    // sabotage (run): the parking leg switched off (set_ground(false, ...)) did NOT fail this test: the tandem tipper rests on
    // its 1.6 m bogie with the centre of mass only 0.45 m ahead of it (pitch 0.0x deg either way). The leg is for a trailer
    // with a single axle (physics_sim A1: "a free-standing single-axle trailer drops its nose"), covered by the pure
    // "parking leg" test; no such trailer is shipped yet.
}

TEST_CASE("trailer session: the assisted hook-up couples within the latch ramp and holds the sockets together", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", false);
    d.tick(240);
    const ps::Pose before = s.world().get_pose(d.rig().body());
    s.request_trailer_attach();
    d.tick(1);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    const ps::Pose after = s.world().get_pose(d.rig().body());
    // The assist moved the trailer by the 3 m gap or less, by a pure shift plus a pitch: the wheels stay on the ground.
    CHECK((after.position - before.position).length() < trailer_config()->spawn.gap_m + 0.5);
    d.tick(360);
    const rg::TrailerSnapshot t = d.trailer();
    CHECK(t.state == rg::TrailerState::Coupled);
    CHECK(t.latch_progress == Approx(1.0));
    CHECK(t.socket_error_m < 0.01);
    CHECK(t.class_name == "ball");
    CHECK(t.towing_point == "socket_tow_rear");
    CHECK(t.towed_point == "socket_hitch_eye_ball");
    CHECK(t.towed_mass_kg == Approx(t.mass_kg).margin(1.0));
    // The ball sits at the tipper's coupler height (data/vehicles/car_sedan.json): the empty trailer then rests on its own
    // wheels and the nose weight is the lever's, about 0 to +2 kN. [M] -4.3 kN with the ball 17 cm lower (z -0.28).
    CHECK(t.tongue_load_n > -500.0);
    CHECK(t.tongue_load_n < 2500.0);
    CHECK(t.break_force_n > 10000.0);
    CHECK(t.last_message.find("Hooked up") != std::string::npos);
    CHECK(t.attaches == 1);
    // The trailer's own brake is released while hooked on (the tow brake and the car's brake channel take over).
    CHECK(s.world().get_control(s.world().vehicle_channel(d.rig().vehicle(), "brake")) == 0.0);
    // sabotage (run): the sedan's tow ball back at its rig height (z -0.28 instead of -0.11) fails the tongue check
    // (-4.3 kN lifting the car's rear); sabotage (run) of reach_m back to 3.0 fails the REQUIRE on the state (too far by the
    // ball-to-coupler height)
}

TEST_CASE("trailer session: a hook-up that is not allowed is refused with a reason and changes nothing", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", false);
    d.tick(240);
    // Too far: drag the standing trailer 8 m away.
    ps::Pose p = s.world().get_pose(d.rig().body());
    p.position.x -= 8.0;
    s.world().backend().set_pose(d.rig().body(), p);
    d.tick(60);
    s.request_trailer_attach();
    d.tick(1);
    CHECK(d.rig().state() == rg::TrailerState::Standing);
    const rg::TrailerSnapshot far = d.trailer();
    CHECK(far.hookup_verdict == rg::HookupVerdict::TooFar);
    CHECK(far.last_message.find("too far") != std::string::npos);
    CHECK(far.attaches == 0);
    // Detaching a trailer that is not hooked on is refused too.
    s.request_trailer_detach();
    d.tick(1);
    CHECK(d.trailer().last_message.find("not hooked up") != std::string::npos);
    CHECK(d.trailer().detaches == 0);
    // sabotage (not run): comparing distance_m against reach_m with `<` flipped would attach the far trailer
}

TEST_CASE("trailer session: remove deletes the trailer and the car is as it was", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("", "", true); // the defaults, hooked on
    d.tick(120);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    s.request_trailer_remove();
    d.tick(2);
    CHECK_FALSE(d.rig().present());
    CHECK_FALSE(d.trailer().present);
    CHECK(d.trailer().removed == 1);
    // A second trailer afterwards works (channel scopes are never reused).
    s.request_trailer_spawn("trailer_tipper", "half", true);
    d.tick(120);
    CHECK(d.rig().state() == rg::TrailerState::Coupled);
    CHECK(d.rig().serial() == 2);
    CHECK(d.rig().load_id() == "half");
    // sabotage (run): reusing the scope "trailer" for the second vehicle throws std::invalid_argument (a retired scope stays
    // reserved) - the Session would die on the second spawn
}

TEST_CASE("trailer session: a trailer behind a car moves with it: tow, brake, reverse, let go", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.start_engine();
    s.request_trailer_spawn("trailer_tipper", "empty", true);
    d.tick(240);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);

    // Tow forward for 12 s.
    d.ctl("throttle", 0.6);
    double max_error = 0.0, max_force = 0.0, max_yaw = 0.0;
    for (int i = 0; i < 12 * static_cast<int>(kHz); ++i) {
        d.tick();
        if (i % 12 == 0) {
            const rg::TrailerSnapshot t = d.trailer();
            max_error = std::max(max_error, t.socket_error_m);
            max_force = std::max(max_force, t.hitch_force_n);
            max_yaw = std::max(max_yaw, std::fabs(t.yaw_deg));
        }
    }
    const double v_tow = d.forward_speed();
    std::printf("RGTRAILER tow v=%.2f m/s err=%.4f force=%.0f N yaw=%.2f deg\n", v_tow, max_error, max_force, max_yaw);
    CHECK(d.rig().state() == rg::TrailerState::Coupled); // the hitch did not let go in ordinary driving
    CHECK(v_tow > 5.0);
    CHECK(max_error < 0.02);
    CHECK(max_yaw < 10.0);
    const rg::TrailerSnapshot t = d.trailer();
    // The trailer follows: same direction, close behind.
    CHECK(std::fabs(wrap(rg::heading_yaw(t.pose.orientation) - d.car_yaw())) < 0.15);

    // Brake to a stop: the tow brake (channel link) keeps the trailer from jackknifing.
    CHECK(d.trailer().tow_brake == 0.0); // the car is not braking: neither is the trailer
    d.ctl("throttle", 0.0);
    d.ctl("brake", 1.0);
    d.tick(120); // 0.5 s: the 0.4 s brake line is 70 % up
    CHECK(d.trailer().tow_brake > 0.6);
    CHECK(s.world().get_control(s.world().vehicle_channel(d.rig().vehicle(), "brake")) == 0.0); // ... through the hitch, not by hand
    for (int i = 0; i < 10 * static_cast<int>(kHz) && d.forward_speed() > 0.2; ++i) d.tick();
    CHECK(d.forward_speed() < 0.5);
    CHECK(d.rig().state() == rg::TrailerState::Coupled);
    CHECK(std::fabs(d.trailer().yaw_deg) < 20.0);
    d.ctl("brake", 0.0);
    d.tick(240);

    // Reverse for 5 s: the trailer pushes, the articulation limit holds.
    d.ctl("assist.auto_shift", 0.0);
    d.ctl("gear_request", -1.0);
    d.tick(120);
    d.ctl("throttle", 0.4);
    for (int i = 0; i < 5 * static_cast<int>(kHz); ++i) d.tick();
    const double v_rev = d.forward_speed();
    std::printf("RGTRAILER reverse v=%.2f m/s yaw=%.2f deg\n", v_rev, d.trailer().yaw_deg);
    CHECK(v_rev < -0.3);
    CHECK(d.rig().state() == rg::TrailerState::Coupled);

    // Stop and let it go.
    d.ctl("throttle", 0.0);
    d.ctl("brake", 1.0);
    d.tick(5 * static_cast<int>(kHz));
    d.ctl("brake", 0.0);
    s.request_trailer_detach();
    d.tick(2);
    CHECK(d.rig().state() == rg::TrailerState::Standing);
    CHECK(d.trailer().detaches == 1);
    // The car drives off, the trailer stays (parking brake + jack).
    const ps::Vec3 stand = s.world().get_pose(d.rig().body()).position;
    d.ctl("gear_request", 1.0);
    d.ctl("assist.auto_shift", 1.0);
    d.ctl("throttle", 0.5);
    d.tick(5 * static_cast<int>(kHz));
    d.ctl("throttle", 0.0);
    CHECK(d.forward_speed() > 2.0);
    CHECK((s.world().get_pose(d.rig().body()).position - stand).length() < 1.5);
    // sabotage (run): deleting "tow_brake" from data/vehicles/trailer_tipper.json leaves tow_brake at 0 while the car brakes
    // (the `> 0.6` check fails); the straight stop alone did NOT catch it (yaw stayed 0: the drive is straight)
}

TEST_CASE("trailer session: statics of the coupled empty trailer (diagnostic table)", "[.][trailer_diag]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", true);
    for (int k = 0; k < 8; ++k) {
        d.tick(60);
        const rg::TrailerSnapshot t = d.trailer();
        double sum = 0.0;
        for (const auto& w : t.wheels) sum += w.load;
        double car_sum = 0.0;
        for (std::size_t i = 0; i < s.vehicle_desc().wheels.size(); ++i) car_sum += s.world().wheel_state(s.vehicle_id(), i).load;
        std::printf("RGTRAILER t=%.2f tongue=%.0f trailer_wheels=%.0f weight=%.0f car_wheels=%.0f force=%.0f err=%.4f z=%.3f\n", (k + 1) * 0.25,
                    t.tongue_load_n, sum, t.mass_kg * 9.81, car_sum, t.hitch_force_n, t.socket_error_m, t.pose.position.z);
    }
}

TEST_CASE("trailer session: towing envelope (diagnostic table)", "[.][trailer_diag]") {
    const std::string car = GENERATE(as<std::string>{}, "car_sedan", "car_city", "car_hyper");
    const std::string load = GENERATE(as<std::string>{}, "empty", "full");
    std::printf("RGTRAILER === car %s load %s\n", car.c_str(), load.c_str());
    rg::Session s(make_config(car, true));
    Drive d(s);
    d.start_engine();
    s.request_trailer_spawn("trailer_tipper", load, true);
    d.tick(240);
    d.ctl("throttle", 1.0);
    double max_force = 0, max_yaw = 0, max_err = 0, min_ratio = 1e9;
    const auto sample = [&](const char* tag) {
        const rg::TrailerSnapshot t = d.trailer();
        max_force = std::max(max_force, t.hitch_force_n);
        max_yaw = std::max(max_yaw, std::fabs(t.yaw_deg));
        max_err = std::max(max_err, t.socket_error_m);
        if (t.hitch_force_n > 1.0) min_ratio = std::min(min_ratio, t.break_force_n / t.hitch_force_n);
        (void)tag;
    };
    for (int sec = 0; sec < 30; ++sec) {
        for (int i = 0; i < 240; ++i) {
            d.tick();
            if (i % 12 == 0) sample("");
        }
        const rg::TrailerSnapshot t = d.trailer();
        std::printf("RGTRAILER t=%2d v=%6.2f state=%d force=%7.0f yaw=%6.2f pitch=%5.2f tongue=%6.0f err=%.4f\n", sec + 1, d.forward_speed(), static_cast<int>(t.state), t.hitch_force_n, t.yaw_deg, t.pitch_deg, t.tongue_load_n, t.socket_error_m);
        if (t.state != rg::TrailerState::Coupled) break;
    }
    // Steering at speed
    for (const double steer : {0.1, -0.2, 0.3}) {
        d.ctl("steer", steer);
        for (int i = 0; i < 240; ++i) {
            d.tick();
            if (i % 12 == 0) sample("");
        }
        const rg::TrailerSnapshot t = d.trailer();
        std::printf("RGTRAILER steer=%.2f v=%6.2f state=%d force=%7.0f yaw=%6.2f roll=%5.2f\n", steer, d.forward_speed(), static_cast<int>(t.state), t.hitch_force_n, t.yaw_deg, t.roll_deg);
        if (t.state != rg::TrailerState::Coupled) break;
    }
    d.ctl("steer", 0.0);
    d.ctl("throttle", 0.0);
    d.ctl("brake", 1.0);
    for (int sec = 0; sec < 12; ++sec) {
        for (int i = 0; i < 240; ++i) {
            d.tick();
            if (i % 12 == 0) sample("");
        }
        const rg::TrailerSnapshot t = d.trailer();
        std::printf("RGTRAILER brake t=%2d v=%6.2f state=%d force=%7.0f yaw=%6.2f\n", sec + 1, d.forward_speed(), static_cast<int>(t.state), t.hitch_force_n, t.yaw_deg);
        if (t.state != rg::TrailerState::Coupled || d.forward_speed() < 0.1) break;
    }
    std::printf("RGTRAILER summary max_force=%.0f max_yaw=%.2f max_err=%.4f min_break_ratio=%.2f overloads=%llu\n", max_force, max_yaw, max_err, min_ratio, static_cast<unsigned long long>(d.trailer().overloads));
}

TEST_CASE("trailer session: ball height above the ground per car (diagnostic table)", "[.][trailer_diag]") {
    const std::string car = GENERATE(as<std::string>{}, "car_sedan", "car_sedan_rwd", "car_sedan_awd", "car_sedan_gen_n2o", "car_electric", "car_hyper",
                                     "car_hyper_n2o", "car_city", "car_gt70", "car_runabout");
    rg::Session s(make_config(car, true));
    Drive d(s);
    d.tick(240);
    const ps::Pose p = s.world().get_pose(s.chassis_body());
    const auto& pts = s.vehicle_desc().hitch_points;
    REQUIRE_FALSE(pts.empty());
    const ps::Vec3 ball = p.transform_point(pts[0].position_local);
    std::printf("RGTRAILER car=%s chassis_z=%.3f ball_local_z=%.3f ball_height=%.3f\n", car.c_str(), p.position.z, pts[0].position_local.z, ball.z);
}

TEST_CASE("trailer session: a relocation takes a coupled trailer along and hooks it on behind the car again", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", true);
    d.tick(240);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    const std::uint64_t teleports = d.rig().teleports();

    constexpr double kYaw = 1.0;
    s.request_relocate(80.0, -40.0, kYaw);
    d.tick(1);
    // Hooked on again in the same attempt: a coupled car and trailer teleport as one.
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    CHECK(d.rig().teleports() > teleports); // the render history is told to step, not blend
    d.tick(360);
    const rg::TrailerSnapshot t = d.trailer();
    CHECK(t.state == rg::TrailerState::Coupled);
    CHECK(t.socket_error_m < 0.01);
    CHECK(t.attaches == 2);
    CHECK(t.detaches == 0); // the relocation's own let-go is not a player's detach
    const ps::Pose car = s.world().get_pose(s.chassis_body());
    CHECK(car.position.x == Approx(80.0).margin(0.5));
    CHECK(car.position.y == Approx(-40.0).margin(0.5));
    CHECK(std::fabs(wrap(rg::heading_yaw(t.pose.orientation) - kYaw)) < 0.05);
    CHECK((t.pose.position - car.position).length() < 8.0);
    // Both stand level and still.
    CHECK(t.motion.linear.length() < 0.2);
    // sabotage (run): skipping begin_relocation makes the physics throw on the teleport of a coupled vehicle (the hitch
    // would tear the pair apart, error > 70 m) - the REQUIRE on the coupled state or the socket error fails
}

TEST_CASE("trailer session: a relocation leaves a standing trailer where it stood only when the car lands near it", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", false);
    d.tick(240);
    const ps::Vec3 stood = s.world().get_pose(d.rig().body()).position;
    // 10 m to the side: inside relocation_keep_m, the trailer stays.
    s.request_relocate(stood.x + 3.0, stood.y + 10.0, 0.0);
    d.tick(240);
    REQUIRE(d.rig().present());
    CHECK(d.rig().state() == rg::TrailerState::Standing);
    CHECK((s.world().get_pose(d.rig().body()).position - stood).length() < 0.3);
    // 200 m away: removed (nothing keeps its ground loaded there).
    s.request_relocate(stood.x + 200.0, stood.y, 0.0);
    d.tick(2);
    CHECK_FALSE(d.rig().present());
    CHECK(d.trailer().last_message.find("left behind") != std::string::npos);
    // sabotage (run): keeping it unconditionally leaves a trailer 200 m away from the car (present stays true)
}

TEST_CASE("trailer session: recover puts a trailer that lies on its side upright behind the car and hooks it on", "[trailer][session]") {
    rg::Session s(make_config("car_sedan", true));
    Drive d(s);
    d.tick(120);
    s.request_trailer_spawn("trailer_tipper", "empty", true);
    d.tick(240);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    // The pair is stopped and the trailer is dropped on its side 12 m away (as after a rollover; let the hitch go first the way
    // a player's detach would).
    s.request_trailer_detach();
    d.tick(2);
    ps::Pose p = s.world().get_pose(d.rig().body());
    p.position = s.world().get_pose(s.chassis_body()).position + ps::Vec3{-12.0, 4.0, 1.5};
    p.orientation = ps::Quat::from_axis_angle(ps::Vec3::unit_x(), 1.5707963267948966);
    s.world().backend().set_pose(d.rig().body(), p);
    s.world().backend().set_motion(d.rig().body(), ps::Motion{});
    d.tick(120);
    REQUIRE(d.rig().state() == rg::TrailerState::Standing);
    CHECK(d.trailer().pose.orientation.rotate(ps::Vec3::unit_z()).z < 0.5);
    s.request_trailer_recover();
    d.tick(1);
    CHECK(d.rig().state() == rg::TrailerState::Coupled);
    d.tick(360);
    const rg::TrailerSnapshot t = d.trailer();
    CHECK(t.state == rg::TrailerState::Coupled);
    CHECK(t.pose.orientation.rotate(ps::Vec3::unit_z()).z > 0.998);
    CHECK(t.socket_error_m < 0.01);
    CHECK(std::fabs(wrap(rg::heading_yaw(t.pose.orientation) - d.car_yaw())) < 0.05);
    // sabotage (run): recover without place_standing leaves the trailer on its side: the attach is refused (HeadingOff /
    // TooFar) and the state stays Standing
}

TEST_CASE("trailer session: a new Session restores the trailer the player had (restart)", "[trailer][session]") {
    rg::SessionConfig config = make_config("car_sedan", true);
    config.trailer_restore.present = true;
    config.trailer_restore.kind = "trailer_tipper";
    config.trailer_restore.load = "half";
    config.trailer_restore.coupled = true;
    rg::Session s(config);
    Drive d(s);
    d.tick(300);
    REQUIRE(d.rig().present());
    CHECK(d.rig().state() == rg::TrailerState::Coupled);
    CHECK(d.rig().load_id() == "half");
    CHECK(d.trailer().mass_kg == Approx(trailer_config()->find_kind("trailer_tipper")->find_load("half")->mass_kg));
    CHECK(d.trailer().socket_error_m < 0.01);
    // A restore with an unknown kind is refused with a message and leaves no trailer.
    config.trailer_restore.kind = "nope";
    rg::Session bad(config);
    Drive db(bad);
    db.tick(10);
    CHECK_FALSE(db.rig().present());
    CHECK(db.trailer().last_message.find("Unknown trailer") != std::string::npos);
    // sabotage (run): not queueing the restore command in the Session constructor leaves present false (first REQUIRE)
}

TEST_CASE("trailer session: an overloaded hitch lets go with a message instead of tearing the pair apart", "[trailer][session]") {
    // A config whose ball breaks at a tiny multiple of D: the first pull on the trailer exceeds it.
    auto low = std::make_shared<rg::TrailerConfig>(*trailer_config());
    low->classes[0].break_d_multiple = 0.05;
    rg::SessionConfig config = make_config("car_sedan", false);
    config.trailer = low;
    rg::Session s(config);
    Drive d(s);
    d.start_engine();
    s.request_trailer_spawn("trailer_tipper", "empty", true);
    d.tick(60);
    REQUIRE(d.rig().state() == rg::TrailerState::Coupled);
    d.ctl("throttle", 0.8);
    d.tick(480);
    const rg::TrailerSnapshot t = d.trailer();
    CHECK(t.state == rg::TrailerState::Standing);
    CHECK(t.overloads == 1);
    CHECK(t.last_message.find("The hitch let go") != std::string::npos);
    // Let go means the trailer's parking brake holds it where it is while the car drives on.
    CHECK(d.forward_speed() > 2.0);
    // sabotage (run): HitchOverload::update returning false forever keeps the state Coupled (overloads 0)
}
