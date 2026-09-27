#include "rg/session.h"

#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/vehicle_reset.h"

// ps_godot::FallDetector, reused BY PATH from physics_sim's adapter (see
// session.h's top comment) - Godot-free.
#include "fall_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace rg {

// Mirrors external/physics_sim/adapters/godot/demo/scripts/main.gd's own
// control-channel set (see that file for the assist.* semantics) so a
// GDScript input_map.gd ported from main.gd's pattern needs no channel-name
// changes. set_control on any name not in this list is a documented no-op
// (see session.h's doc comment) rather than growing control_channels_ from
// a possibly-concurrent caller.
const char* const kControlChannelNames[] = {
    "steer",
    "throttle",
    "brake",
    "handbrake",
    "clutch",
    "shift_up_count",
    "shift_down_count",
    "ignition",
    "starter",
    "assist.auto_shift",
    "assist.auto_clutch",
    "assist.auto_blip",
};
const std::size_t kControlChannelCount = sizeof(kControlChannelNames) / sizeof(kControlChannelNames[0]);

// Terrain mode's streaming stack (R2.2 R4). Declaration order is the
// destruction contract: the streamer holds references to the resident set
// and the loader, so it goes first; the loader's destructor joins its
// workers (waiting out a fetch in flight). The World keeps its own
// shared_ptr to `source` (and through it to `resident`), so both outlive
// this struct when ~Session destroys terrain_ before world_.
struct Session::Terrain {
    Terrain(const TerrainModeConfig& cfg, ps::SurfaceId surface)
        : config(cfg),
          grid(cfg.frame),
          resident(std::make_shared<g2m::phys::ResidentHeightSet>()),
          loader(cfg.fetch, loader_config(cfg, /*require_roads=*/false)),
          streamer(grid, *resident, loader, g2m::phys::StreamerConfig{}),
          source(std::make_shared<g2m::ps_bridge::G2mTerrainSource>(grid, resident, surface)) {
        interest_points.reserve(4);
    }

    // Road mode (G2.5a-grip R-c): grip comes from per-cell OSM road classes
    // (g2m::ps_bridge::G2mTerrainSource's own road-mode constructor) instead
    // of one uniform SurfaceId - see Session::setup_terrain for how `road`
    // is built from physics.road_surfaces via this Session's SurfaceTable.
    // require_roads=true on the loader (roads ride on the SAME residency
    // entry as heights): an Ok height fetch whose L2 tile's roads come back
    // null is retried/failed exactly like a height-side 500.
    Terrain(const TerrainModeConfig& cfg, g2m::ps_bridge::RoadSurfaceConfig road)
        : config(cfg),
          grid(cfg.frame),
          resident(std::make_shared<g2m::phys::ResidentHeightSet>()),
          loader(cfg.fetch, loader_config(cfg, /*require_roads=*/true)),
          streamer(grid, *resident, loader, g2m::phys::StreamerConfig{}),
          source(std::make_shared<g2m::ps_bridge::G2mTerrainSource>(grid, resident, std::move(road))) {
        interest_points.reserve(4);
    }

    static g2m::phys::LoaderConfig loader_config(const TerrainModeConfig& cfg, bool require_roads) {
        g2m::phys::LoaderConfig lc;
        lc.workers = cfg.physics.loader_workers;
        lc.require_roads = require_roads;
        return lc;
    }

    TerrainModeConfig config;
    g2m::phys::PhysicsTileGrid grid;
    std::shared_ptr<g2m::phys::ResidentHeightSet> resident;
    g2m::phys::HeightTileLoader loader;
    g2m::phys::PhysicsTerrainStreamer streamer;
    std::shared_ptr<g2m::ps_bridge::G2mTerrainSource> source;
    ps_godot::FallDetector fall_detector;
    std::vector<g2m::phys::InterestPoint> interest_points; // Session::physics_interest_points' buffer
};

namespace {

using Clock = std::chrono::steady_clock;

// std::getenv is deprecated under the Windows UCRT (a /WX error), same as
// world_config.cpp's safe_getenv; empty when unset.
std::string env_or_empty(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return std::string();
    std::string value(buf);
    std::free(buf);
    return value;
#else
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string();
#endif
}

double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// A FallDetector height sample that touches a hole (NoData or Absent: the
// source writes ps::kHeightfieldNoCollision = FLT_MAX there) is not a ground
// height - a "fall" measured against it is the detector's artefact, not a
// fall through the ground.
constexpr double kHoleHeightThreshold = 1.0e6;

// physics.road_surfaces.{field} -> a SurfaceId valid as a road-mode grip
// value: present in the SurfaceTable AND < 255 (the Jolt heightfield
// material index is 8 bits, slot 255 reserved for kInvalidSurfaceId -
// physics_sim/core/src/backend/jolt/jolt_backend.cpp's
// shared_surface_material_list()). The pre-existing uniform
// physics.terrain_surface path has no such check and must keep none, so
// this helper is used only for the three road_surfaces names.
ps::SurfaceId resolve_grip_surface(const ps::io::SurfaceTable& table, const char* field, const std::string& name) {
    const ps::SurfaceId id = table.id_for(name);
    if (id == ps::kInvalidSurfaceId) {
        throw std::invalid_argument(std::string("Session: physics.road_surfaces.") + field + " '" + name +
                                    "' is not in the surface table");
    }
    if (id >= 255) {
        throw std::invalid_argument(std::string("Session: physics.road_surfaces.") + field + " '" + name +
                                    "' has surface id " + std::to_string(id) + " (must be < 255)");
    }
    return id;
}

constexpr double kPi = 3.14159265358979323846;

// The heading Session::request_flip_upright relocates to: the chassis local
// +X (forward) axis rotated to world and projected onto the horizontal
// plane, or - when the car is standing on its nose or tail and that
// projection is too short (< 0.2) to trust - the local +Y (left) axis
// projected instead, offset by -pi/2 so it reads as the same heading a level
// car with that left axis would have (see request_flip_upright's doc
// comment). Both degenerate cannot happen for a unit rotation; falls back to
// yaw 0 defensively.
double flip_upright_yaw(const ps::Quat& orientation) {
    const ps::Vec3 fwd = orientation.rotate(ps::Vec3::unit_x());
    const double fwd_h = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
    if (fwd_h >= 0.2) return ps::math::atan2(fwd.y, fwd.x);
    const ps::Vec3 left = orientation.rotate(ps::Vec3::unit_y());
    const double left_h = std::sqrt(left.x * left.x + left.y * left.y);
    if (left_h >= 0.2) return ps::math::atan2(left.y, left.x) - kPi * 0.5;
    return 0.0;
}

} // namespace

std::unique_ptr<ps::World> Session::make_world(const SessionConfig& config) {
    ps::WorldConfig world_config;
    world_config.tick_rate_hz = config.tick_rate_hz;
    world_config.substep_rate_hz = config.substep_rate_hz;
    world_config.gravity = config.gravity;
    world_config.job_workers = static_cast<int>(config.job_workers);
    return std::make_unique<ps::World>(world_config);
}

Session::Session(const SessionConfig& config)
    : config_(config), world_(make_world(config)), loop_(config.tick_rate_hz) {
    build_world_contents(config);

    control_channels_.reserve(kControlChannelCount);
    for (std::size_t i = 0; i < kControlChannelCount; ++i) {
        control_channels_[kControlChannelNames[i]].store(0.0, std::memory_order_relaxed);
    }
}

Session::~Session() { stop(); }

void Session::build_world_contents(const SessionConfig& config) {
    surface_table_ = std::make_shared<ps::io::SurfaceTable>(config.surface_table_path);
    world_->set_surface_table(surface_table_);

    ps::Pose chassis_pose;
    chassis_pose.position = ps::Vec3{0.0, 0.0, config.chassis_z_m};

    if (config.terrain) {
        chassis_pose = setup_terrain(*config.terrain);
        spawn_x_ = config.terrain->spawn_x;
        spawn_y_ = config.terrain->spawn_y;
        spawn_yaw_rad_ = config.terrain->spawn_yaw_rad;
    } else {
        // Ground: one large flat static box, top face at world z=0 - mirrors
        // external/physics_sim/data/scenarios/vehicle_step_steer.json's own
        // "ground" body exactly (see session.h's SessionConfig comment).
        ps::BodyDesc desc;
        desc.shape = ps::BoxShape{ps::Vec3{config.ground_half_extent_m, config.ground_half_extent_m, 0.5}};
        desc.motion = ps::BodyMotionType::Static;
        desc.pose.position = ps::Vec3{0.0, 0.0, -0.5};
        desc.friction = config.ground_friction;
        ground_body_ = world_->create_body(desc);
    }

    // Chassis: mirrors the same scenario's "chassis" body exactly (mass,
    // half_extents, z, friction, allow_sleep=false, gravity_enabled=true -
    // every field SessionConfig does not override is BodyDesc's own
    // default, same as the scenario JSON leaving it unspecified).
    {
        ps::BodyDesc desc;
        desc.shape = ps::BoxShape{config.chassis_half_extents};
        desc.motion = ps::BodyMotionType::Dynamic;
        desc.mass = config.chassis_mass_kg;
        desc.pose = chassis_pose;
        desc.velocity.linear = config.chassis_initial_velocity;
        desc.friction = config.chassis_friction;
        desc.allow_sleep = false;
        chassis_body_ = world_->create_body(desc);
    }

    vehicle_desc_ = ps::io::load_vehicle_json(config.vehicle_json_path);
    vehicle_id_ = world_->create_vehicle(vehicle_desc_, chassis_body_);
    have_vehicle_ = true;
    spawn_tick_ = world_->tick();
}

ps::Pose Session::setup_terrain(const TerrainModeConfig& tm) {
    const WorldConfig::PhysicsTerrainConfig& phys = tm.physics;
    if (tm.fetch == nullptr) throw std::invalid_argument("Session: terrain mode needs a height-tile fetch");
    if (!(phys.radius_m > 0.0)) throw std::invalid_argument("Session: physics.radius_m must be > 0");
    if (phys.max_tile_fills_per_tick < 1) {
        throw std::invalid_argument("Session: physics.max_tile_fills_per_tick must be >= 1");
    }
    if (!(phys.startup_timeout_s > 0.0)) throw std::invalid_argument("Session: physics.startup_timeout_s must be > 0");

    // R9: the cancel flag is also checked around the (in a debug build
    // slow, uncancellable) TileManager pool construction, not only in the
    // gate and priming loops below.
    throw_if_cancelled();
    if (phys.road_surfaces.enabled) {
        // G2.5a-grip R-c: grip comes from per-cell OSM road classes instead
        // of one uniform terrain_surface. Both hard errors below fail fast
        // rather than silently falling back to "everything off-road".
        if (!tm.road_layer_available) {
            throw std::invalid_argument(
                "Session: physics.road_surfaces.enabled but this release has no g2m.src.osm road layer");
        }
        if (!tm.fetch->provides_roads()) {
            throw std::invalid_argument(
                "Session: physics.road_surfaces.enabled but the terrain fetch does not provide roads");
        }
        const ps::SurfaceId paved = resolve_grip_surface(*surface_table_, "paved", phys.road_surfaces.paved);
        const ps::SurfaceId unpaved = resolve_grip_surface(*surface_table_, "unpaved", phys.road_surfaces.unpaved);
        const ps::SurfaceId off_road = resolve_grip_surface(*surface_table_, "off_road", phys.road_surfaces.off_road);
        const g2m::RoadValueLut lut = g2m::RoadValueLut::by_land_class(
            off_road, {{g2m::LandClass::PavedRoad, paved}, {g2m::LandClass::UnpavedRoad, unpaved}});
        terrain_ = std::make_unique<Terrain>(tm, g2m::ps_bridge::RoadSurfaceConfig{lut, off_road});
    } else {
        const ps::SurfaceId surface = surface_table_->id_for(phys.terrain_surface);
        if (surface == ps::kInvalidSurfaceId) {
            throw std::invalid_argument("Session: physics.terrain_surface '" + phys.terrain_surface +
                                        "' is not in the surface table");
        }
        terrain_ = std::make_unique<Terrain>(tm, surface);
    }
    world_->set_terrain_source(terrain_->source,
                               g2m::ps_bridge::make_terrain_config(phys.radius_m, phys.max_tile_fills_per_tick));

    throw_if_cancelled();
    StartupProgress* progress = config_.startup.get();
    if (progress) progress->stage.store(StartupProgress::WaitingForGate, std::memory_order_relaxed);

    // 1. Start-up: block until the gate around the spawn point is ready.
    // Failed keys are retried here (a transient 5xx at start-up must not be
    // fatal); the timeout is the one hard error (a cancellation, R9, throws
    // SessionCancelled instead).
    const Clock::time_point t0 = Clock::now();
    while (!gate_check()) {
        throw_if_cancelled();
        if (status_.failed.load(std::memory_order_relaxed) > 0) terrain_->streamer.retry_failed();
        if (ms_since(t0) > phys.startup_timeout_s * 1000.0) {
            throw std::runtime_error(
                "Session: terrain start-up timed out after " + std::to_string(phys.startup_timeout_s) +
                " s (gate missing " + std::to_string(status_.missing_required.load()) + ", inflight " +
                std::to_string(status_.inflight.load()) + ", failed " + std::to_string(status_.failed.load()) + ")");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    status_.startup_ms.store(ms_since(t0), std::memory_order_relaxed);

    // 2. Prime: step K ticks with no vehicle so TileManager fills its whole
    // square (at most max_tile_fills_per_tick per tick) before anything can
    // touch the ground.
    const int r_tm = static_cast<int>(std::ceil(phys.radius_m / terrain_->grid.tile_size_m()));
    const std::size_t side = static_cast<std::size_t>(2 * r_tm + 1);
    const std::size_t square = side * side;
    const int prime_ticks = tm.prime_ticks > 0
                                ? tm.prime_ticks
                                : static_cast<int>((square + phys.max_tile_fills_per_tick - 1) /
                                                   phys.max_tile_fills_per_tick) +
                                      1;
    if (progress) {
        progress->prime_total.store(static_cast<std::uint32_t>(prime_ticks), std::memory_order_relaxed);
        progress->stage.store(StartupProgress::Priming, std::memory_order_relaxed);
    }
    for (int k = 0; k < prime_ticks; ++k) {
        throw_if_cancelled();
        step_blocking("terrain priming");
        if (progress) progress->prime_done.store(static_cast<std::uint32_t>(k + 1), std::memory_order_relaxed);
    }
    status_.prime_ticks.store(static_cast<std::uint32_t>(prime_ticks), std::memory_order_relaxed);

    const std::size_t resident = world_->terrain_resident_tile_count();
    const std::size_t starved = world_->terrain_starved_tile_count();
    const std::uint64_t misses = terrain_->source->fill_miss_count();
    if (resident < square || starved != 0 || misses != 0) {
        throw std::runtime_error("Session: terrain priming failed after " + std::to_string(prime_ticks) +
                                 " ticks (resident tiles " + std::to_string(resident) + " of " +
                                 std::to_string(square) + ", starved " + std::to_string(starved) +
                                 ", fill misses " + std::to_string(misses) + ")");
    }

    throw_if_cancelled();
    if (progress) progress->stage.store(StartupProgress::Spawning, std::memory_order_relaxed);

    // 3. Spawn: five rays straight down (ray_spawn_pose).
    ps::Pose pose;
    double miss_x = 0.0, miss_y = 0.0;
    if (!ray_spawn_pose(tm.spawn_x, tm.spawn_y, tm.spawn_yaw_rad, phys.spawn_clearance_m, pose, miss_x, miss_y)) {
        char msg[160];
        std::snprintf(msg, sizeof msg, "Session: spawn over NoData (no ground under (%.2f, %.2f))", miss_x, miss_y);
        throw std::runtime_error(msg);
    }
    if (progress) progress->stage.store(StartupProgress::Done, std::memory_order_relaxed);
    return pose;
}

bool Session::ray_spawn_pose(double x, double y, double yaw_rad, double clearance_m, ps::Pose& out, double& miss_x,
                             double& miss_y) const {
    // Five rays straight down (centre + the four yaw-rotated chassis-
    // footprint corners), from z = +3000 over 6000 m; the chassis centre
    // goes chassis_z_m + clearance_m above the highest hit.
    double s_yaw = 0.0, c_yaw = 1.0;
    ps::math::sincos(yaw_rad, s_yaw, c_yaw);
    const double hx = config_.chassis_half_extents.x;
    const double hy = config_.chassis_half_extents.y;
    const double corners[5][2] = {{0.0, 0.0}, {hx, hy}, {hx, -hy}, {-hx, hy}, {-hx, -hy}};
    double z_max = -1.0e300;
    for (const auto& lc : corners) {
        const double wx = x + c_yaw * lc[0] - s_yaw * lc[1];
        const double wy = y + s_yaw * lc[0] + c_yaw * lc[1];
        const ps::RayCastHit hit =
            world_->backend().ray_cast(ps::Vec3{wx, wy, 3000.0}, ps::Vec3{0.0, 0.0, -1.0}, 6000.0);
        if (!hit.hit) {
            miss_x = wx;
            miss_y = wy;
            return false;
        }
        z_max = std::max(z_max, static_cast<double>(hit.point.z));
    }

    double s_half = 0.0, c_half = 1.0;
    ps::math::sincos(0.5 * yaw_rad, s_half, c_half);
    out.position = ps::Vec3{x, y, z_max + config_.chassis_z_m + clearance_m};
    out.orientation = ps::Quat{0.0, 0.0, s_half, c_half};
    return true;
}

void Session::request_relocate(double x, double y, double yaw_rad) {
    {
        std::lock_guard<std::mutex> lock(relocate_mutex_);
        relocate_request_ = RelocateTarget{x, y, yaw_rad};
    }
    relocate_pending_.store(true, std::memory_order_release);
}

void Session::request_reset_to_spawn() { request_relocate(spawn_x_, spawn_y_, spawn_yaw_rad_); }

void Session::request_flip_upright() { flip_upright_pending_.store(true, std::memory_order_release); }

void Session::take_relocate_request() {
    if (!have_vehicle_) return;
    if (relocate_pending_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(relocate_mutex_);
        relocate_pending_.store(false, std::memory_order_relaxed);
        relocation_ = relocate_request_; // a newer request replaces one still waiting for its gate
        // An explicit relocate already lands upright (ray_spawn_pose's
        // pure-yaw orientation) - drop a same-tick flip request rather than
        // carry it into a later tick against a pose the relocate has already
        // moved past (request_flip_upright's doc comment).
        flip_upright_pending_.store(false, std::memory_order_relaxed);
        return;
    }
    if (flip_upright_pending_.exchange(false, std::memory_order_acq_rel)) {
        const ps::Pose pose = world_->get_pose(chassis_body_);
        relocation_ = RelocateTarget{pose.position.x, pose.position.y, flip_upright_yaw(pose.orientation)};
    }
}

void Session::finish_relocation() {
    const RelocateTarget target = *relocation_;
    relocation_.reset();

    // Park the chassis far above the target (above the spawn rays' start, so
    // they never hit it), at rest.
    ps::Pose park;
    park.position = ps::Vec3{target.x, target.y, kRelocateParkZ};
    const ps::Motion still{};
    const auto park_chassis = [&] {
        world_->backend().set_pose(chassis_body_, park);
        world_->backend().set_motion(chassis_body_, still);
    };
    park_chassis();

    double clearance = 0.0;
    if (terrain_) {
        // The gate around the target is ready; TileManager's interest point
        // is already there (gate_check). Step its priming ticks with the
        // chassis held parked so the pool fills the target's square before
        // anything touches the ground - the start-up priming, repeated.
        const WorldConfig::PhysicsTerrainConfig& phys = terrain_->config.physics;
        clearance = phys.spawn_clearance_m;
        const int r_tm = static_cast<int>(std::ceil(phys.radius_m / terrain_->grid.tile_size_m()));
        const std::size_t side = static_cast<std::size_t>(2 * r_tm + 1);
        const int ticks = terrain_->config.prime_ticks > 0
                              ? terrain_->config.prime_ticks
                              : static_cast<int>((side * side + phys.max_tile_fills_per_tick - 1) /
                                                 phys.max_tile_fills_per_tick) +
                                    1;
        for (int k = 0; k < ticks; ++k) {
            world_->step();
            park_chassis();
        }
    }

    ps::Pose pose;
    double miss_x = 0.0, miss_y = 0.0;
    if (!ray_spawn_pose(target.x, target.y, target.yaw_rad, clearance, pose, miss_x, miss_y)) {
        // Not over ground: leave the car where it was asked to go, parked,
        // and say so - the caller picked a target outside the data.
        status_.relocate_failures.fetch_add(1, std::memory_order_relaxed);
        std::fprintf(stderr, "rg::Session: relocation to (%.1f, %.1f) failed: no ground under (%.2f, %.2f)\n",
                     target.x, target.y, miss_x, miss_y);
        return;
    }
    world_->backend().set_pose(chassis_body_, pose);
    world_->backend().set_motion(chassis_body_, still);

    // Drivetrain at rest: neutral, hubs stopped, slip relaxation cleared; an
    // engine that was not off is left running (a stalled one restarts).
    ps::vehicle::VehicleResetOptions opts;
    opts.gear = 0;
    opts.hubs_roll_with_chassis = false;
    opts.clutch_locked = false;
    opts.engine_state = world_->powertrain_state(vehicle_id_).engine_state == ps::drivetrain::EngineState::Off
                            ? ps::drivetrain::EngineState::Off
                            : ps::drivetrain::EngineState::Running;
    world_->reset_vehicle(vehicle_id_, opts);
    status_.relocations.fetch_add(1, std::memory_order_relaxed);
}

std::span<const g2m::phys::InterestPoint> Session::physics_interest_points() {
    // Every physics actor that needs resident terrain gets one entry. R4 has
    // exactly one: the chassis (or, before the vehicle exists, the spawn
    // point). Later actors (a walker, a parked car, a followed vehicle) are
    // appended here; the TileManager pool (make_terrain_config, 49 tiles at
    // r = 400 m) is sized for ONE point and must be re-sized with them.
    Terrain& t = *terrain_;
    std::vector<g2m::phys::InterestPoint>& out = t.interest_points;
    out.clear(); // capacity reserved in Terrain's constructor: no allocation per tick
    double x = t.config.spawn_x, y = t.config.spawn_y, vx = 0.0, vy = 0.0;
    if (relocation_) {
        // A relocation in progress: the terrain must come to the target
        // before the car can (R2.2 R9, request_relocate).
        x = relocation_->x;
        y = relocation_->y;
    } else if (have_vehicle_) {
        const ps::Vec3 p = world_->get_pose(chassis_body_).position;
        const ps::Vec3 v = world_->get_motion(chassis_body_).linear;
        x = p.x;
        y = p.y;
        vx = v.x;
        vy = v.y;
    }
    out.push_back(g2m::phys::InterestPoint{0, x, y, vx, vy, t.config.physics.radius_m});
    return out;
}

bool Session::gate_check() {
    Terrain& t = *terrain_;
    if (retry_failed_requested_.exchange(false, std::memory_order_relaxed)) t.streamer.retry_failed();

    const std::span<const g2m::phys::InterestPoint> points = physics_interest_points();
    const Clock::time_point t_update = Clock::now();
    const g2m::phys::GateStatus gs = t.streamer.update(points);
    last_gate_update_ms_ = ms_since(t_update);
    const Clock::time_point t_interest = Clock::now();
    // TileManager keeps one interest point per id; the ids are the streamer's
    // own (removing a point that disappears from the list is not needed while
    // the list is fixed at the chassis).
    for (const g2m::phys::InterestPoint& ip : points) {
        world_->set_terrain_interest_point(ip.id, ps::Vec3{ip.x, ip.y, 0.0}, ip.radius_m);
    }
    last_gate_interest_ms_ = ms_since(t_interest);

    status_.ready.store(gs.ready, std::memory_order_relaxed);
    status_.missing_required.store(gs.missing_required, std::memory_order_relaxed);
    status_.inflight.store(gs.inflight, std::memory_order_relaxed);
    status_.resident_l0.store(gs.resident, std::memory_order_relaxed);
    status_.failed.store(gs.failed, std::memory_order_relaxed);
    if (StartupProgress* progress = config_.startup.get(); progress != nullptr && !have_vehicle_) {
        progress->missing_required.store(gs.missing_required, std::memory_order_relaxed);
        progress->inflight.store(gs.inflight, std::memory_order_relaxed);
        progress->resident_l0.store(gs.resident, std::memory_order_relaxed);
        progress->failed.store(gs.failed, std::memory_order_relaxed);
    }
    return gs.ready;
}

bool Session::step_once(bool from_loop) {
    // Per-phase timing for the tick-spike queue (session.h); only real-time
    // loop attempts are judged - step()'s synchronous retries have no cadence.
    const Clock::time_point t_start = Clock::now();
    TickSpike spike;
    Clock::time_point t_phase = t_start;
    const auto lap = [&t_phase]() {
        const Clock::time_point now = Clock::now();
        const double ms = std::chrono::duration<double, std::milli>(now - t_phase).count();
        t_phase = now;
        return ms;
    };
    const auto finish = [&](TickSpikeKind kind, bool result) {
        if (!from_loop) return result;
        spike.kind = kind;
        spike.total_ms = ms_since(t_start);
        spike.gap_ms = have_last_attempt_
                           ? std::chrono::duration<double, std::milli>(t_start - last_attempt_start_).count()
                           : 0.0;
        spike.prev_total_ms = last_attempt_total_ms_;
        const bool is_spike = spike.total_ms > kTickSpikeAttemptMs || spike.gap_ms > kTickSpikeGapMs;
        have_last_attempt_ = true;
        last_attempt_start_ = t_start;
        last_attempt_total_ms_ = spike.total_ms;
        if (is_spike) {
            spike.tick = world_->tick();
            spike.wall_s = std::chrono::duration<double>(t_start - loop_started_).count();
            if (have_vehicle_) {
                const ps::Vec3 p = world_->get_pose(chassis_body_).position;
                spike.x = p.x;
                spike.y = p.y;
                spike.speed_mps = world_->get_motion(chassis_body_).linear.length();
            }
            if (terrain_) spike.resident_tiles = world_->terrain_resident_tile_count();
            std::lock_guard<std::mutex> lock(spike_mutex_);
            if (spikes_.size() < kTickSpikeCapacity) {
                spikes_.push_back(spike);
            } else {
                ++spike_overflow_;
            }
        }
        return result;
    };

    take_relocate_request();
    spike.take_relocate_ms = lap();
    if (terrain_) {
        const bool ready = gate_check();
        spike.gate_update_ms = last_gate_update_ms_;
        spike.gate_interest_ms = last_gate_interest_ms_;
        lap();
        if (!ready) {
            status_.frozen.store(true, std::memory_order_relaxed);
            if (have_vehicle_) {
                status_.frozen_attempts.fetch_add(1, std::memory_order_relaxed);
                if (!in_freeze_) {
                    in_freeze_ = true;
                    status_.freeze_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
            return finish(TickSpikeKind::Frozen, false); // frozen: the World is untouched
        }
        status_.frozen.store(false, std::memory_order_relaxed);
        in_freeze_ = false;
    }

    if (relocation_) {
        // This attempt moves the car instead of driving it (its priming
        // ticks step the World); the next attempt drives again.
        finish_relocation();
        spike.step_ms = lap();
        post_step(from_loop);
        spike.post_ms = lap();
        return finish(TickSpikeKind::Relocation, true);
    }

    if (have_vehicle_ && drive_script_) {
        drive_script_->apply(DriveTickContext{drive_tick(), chassis_body_, vehicle_id_}, *world_);
    } else if (from_loop) {
        apply_live_controls();
    }
    spike.controls_ms = lap();

    world_->step();
    spike.step_ms = lap();
    post_step(from_loop);
    spike.post_ms = lap();
    return finish(TickSpikeKind::Stepped, true);
}

std::vector<Session::TickSpike> Session::drain_tick_spikes(std::uint64_t* overflow) {
    std::vector<TickSpike> out;
    std::lock_guard<std::mutex> lock(spike_mutex_);
    out.swap(spikes_);
    spikes_.reserve(kTickSpikeCapacity);
    if (overflow != nullptr) *overflow = spike_overflow_;
    spike_overflow_ = 0;
    return out;
}

std::string Session::format_tick_spike(const TickSpike& s) {
    const char* kind = s.kind == TickSpikeKind::Frozen ? "frozen" : s.kind == TickSpikeKind::Relocation ? "relocation"
                                                                                                         : "stepped";
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "tick=%llu wall_s=%.3f kind=%s gap_ms=%.2f prev_total_ms=%.2f total_ms=%.2f take_ms=%.2f "
                  "gate_update_ms=%.2f gate_interest_ms=%.2f controls_ms=%.2f step_ms=%.2f post_ms=%.2f "
                  "pos=(%.1f,%.1f) speed_kmh=%.1f resident_tiles=%llu",
                  static_cast<unsigned long long>(s.tick), s.wall_s, kind, s.gap_ms, s.prev_total_ms, s.total_ms,
                  s.take_relocate_ms, s.gate_update_ms, s.gate_interest_ms, s.controls_ms, s.step_ms, s.post_ms, s.x,
                  s.y, s.speed_mps * 3.6, static_cast<unsigned long long>(s.resident_tiles));
    return std::string(buf);
}

void Session::apply_live_controls() {
    // Copies this session's own control_channels_ into ps::World right
    // before the tick (session.h's set_control doc comment explains why this
    // hand-off has to happen on the stepping thread).
    for (auto& [name, value] : control_channels_) {
        world_->set_control(name, value.load(std::memory_order_relaxed));
    }
    if (vehicle_control_.load(std::memory_order_relaxed) != VehicleControl::Unattended || !have_vehicle_) return;
    // Unattended (R2.2 R9): the driving channels come from the policy, not
    // from the player's input (rg/player_mode.h's unattended_controls).
    const ps::Vec3 v = world_->get_motion(chassis_body_).linear;
    const UnattendedControls u = unattended_controls(v.length());
    world_->set_control("steer", u.steer);
    world_->set_control("throttle", u.throttle);
    world_->set_control("brake", u.brake);
    world_->set_control("handbrake", u.handbrake);
    world_->set_control("clutch", u.clutch);
    world_->set_control("starter", u.starter);
}

void Session::throw_if_cancelled() const {
    if (have_vehicle_) return; // cancellation is a start-up seam only
    if (config_.startup && config_.startup->cancel.load(std::memory_order_relaxed)) throw SessionCancelled();
}

void Session::post_step(bool from_loop) {
    if (from_loop) {
        snapshot_buffer_.write_slot() = capture_frame_snapshot();
        snapshot_buffer_.publish();
    }
    if (!terrain_) return;

    Terrain& t = *terrain_;
    if (have_vehicle_) {
        const ps::Vec3 p = world_->get_pose(chassis_body_).position;
        if (t.fall_detector.update(t.source.get(), p) &&
            t.fall_detector.last_terrain_height() < kHoleHeightThreshold) {
            status_.falls.fetch_add(1, std::memory_order_relaxed);
            const std::string report = t.fall_detector.format_report(vehicle_desc_.name, *world_, vehicle_id_,
                                                                     chassis_body_, t.source.get(), surface_table_.get());
            std::fprintf(stderr, "%s\n", report.c_str());
        }
    }
    status_.fill_misses.store(t.source->fill_miss_count(), std::memory_order_relaxed);
    status_.nodata_fills.store(t.source->nodata_fill_count(), std::memory_order_relaxed);
    status_.resident_tiles.store(world_->terrain_resident_tile_count(), std::memory_order_relaxed);
    status_.starved_tiles.store(world_->terrain_starved_tile_count(), std::memory_order_relaxed);
    status_.relief_overflow.store(world_->terrain_relief_overflow_count(), std::memory_order_relaxed);
}

void Session::step_blocking(const char* what) {
    const Clock::time_point t0 = Clock::now();
    const double timeout_ms = terrain_ ? terrain_->config.physics.startup_timeout_s * 1000.0 : 0.0;
    while (!step_once(false)) {
        throw_if_cancelled();
        if (ms_since(t0) > timeout_ms) {
            throw std::runtime_error(std::string("Session: ") + what + ": terrain gate stayed frozen for " +
                                     std::to_string(timeout_ms / 1000.0) + " s (missing " +
                                     std::to_string(status_.missing_required.load()) + ", failed " +
                                     std::to_string(status_.failed.load()) + ")");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Session::step() {
    if (terrain_) {
        step_blocking("step");
    } else {
        step_once(false);
    }
}

bool Session::try_step() { return step_once(true); }

void Session::start() {
    loop_started_ = Clock::now();
    have_last_attempt_ = false;
    spikes_.reserve(kTickSpikeCapacity);
    // Diagnostics: RG_WORLD_CSV=<path> records ps::World's per-tick telemetry
    // (incl. every pipeline stage's stage.<name>.ms) for the whole session.
    if (const std::string csv = env_or_empty("RG_WORLD_CSV"); !csv.empty()) {
        world_->telemetry().start_csv(csv);
        // stdout, not stderr: tools/smoke_test.ps1 redirects a headless
        // Godot run's stderr with a plain `2>`, and Windows PowerShell 5.1
        // wraps every line a native process writes to stderr in a
        // NativeCommandError record when captured that way - this line
        // would otherwise show up as a spurious "Godot printed error
        // line(s)" smoke failure whenever RG_WORLD_CSV is set.
        std::printf("rg::Session: RG_WORLD_CSV -> %s\n", csv.c_str());
        std::fflush(stdout);
    }
    loop_.start([this] { return step_once(true); });
}

void Session::stop() { loop_.stop(); }

void Session::set_drive_script(DriveScript script) {
    if (running()) throw std::logic_error("Session::set_drive_script while running");
    script.rewind();
    drive_script_ = std::move(script);
}

void Session::clear_drive_script() {
    if (running()) throw std::logic_error("Session::clear_drive_script while running");
    drive_script_.reset();
}

StreamingStatus Session::streaming_status() const {
    StreamingStatus s;
    s.terrain_mode = terrain_ != nullptr;
    s.ready = status_.ready.load(std::memory_order_relaxed);
    s.frozen = status_.frozen.load(std::memory_order_relaxed);
    s.missing_required = status_.missing_required.load(std::memory_order_relaxed);
    s.inflight = status_.inflight.load(std::memory_order_relaxed);
    s.resident_l0 = status_.resident_l0.load(std::memory_order_relaxed);
    s.failed = status_.failed.load(std::memory_order_relaxed);
    s.frozen_attempts = status_.frozen_attempts.load(std::memory_order_relaxed);
    s.freeze_count = status_.freeze_count.load(std::memory_order_relaxed);
    s.fill_misses = status_.fill_misses.load(std::memory_order_relaxed);
    s.nodata_fills = status_.nodata_fills.load(std::memory_order_relaxed);
    s.falls = status_.falls.load(std::memory_order_relaxed);
    s.resident_tiles = status_.resident_tiles.load(std::memory_order_relaxed);
    s.starved_tiles = status_.starved_tiles.load(std::memory_order_relaxed);
    s.relief_overflow = status_.relief_overflow.load(std::memory_order_relaxed);
    s.startup_ms = status_.startup_ms.load(std::memory_order_relaxed);
    s.prime_ticks = status_.prime_ticks.load(std::memory_order_relaxed);
    s.relocations = status_.relocations.load(std::memory_order_relaxed);
    s.relocate_failures = status_.relocate_failures.load(std::memory_order_relaxed);
    if (terrain_) {
        s.road_surfaces = terrain_->config.physics.road_surfaces.enabled;
        if (terrain_->config.world_terrain) {
            const WorldTerrain::OsmFetchStats stats = terrain_->config.world_terrain->osm_fetch_stats();
            s.osm_ok = stats.ok;
            s.osm_fail = stats.fail;
        }
    }
    return s;
}

const g2m::ps_bridge::G2mTerrainSource* Session::terrain_source() const {
    return terrain_ ? terrain_->source.get() : nullptr;
}

std::shared_ptr<WorldTerrain> Session::world_terrain() const {
    return terrain_ ? terrain_->config.world_terrain : nullptr;
}

std::string Session::terrain_surface_name() const {
    return terrain_ ? terrain_->config.physics.terrain_surface : std::string();
}

void Session::set_control(const std::string& channel, double value) {
    const auto it = control_channels_.find(channel);
    if (it == control_channels_.end()) return; // unknown channel - see doc comment
    it->second.store(value, std::memory_order_relaxed);
}

double Session::get_control(const std::string& channel) const {
    const auto it = control_channels_.find(channel);
    return it == control_channels_.end() ? 0.0 : it->second.load(std::memory_order_relaxed);
}

FrameSnapshot Session::capture_frame_snapshot() const {
    FrameSnapshot snap;
    snap.tick = world_->tick();
    snap.sim_time = world_->sim_time();
    snap.chassis_pose = world_->get_pose(chassis_body_);
    snap.chassis_motion = world_->get_motion(chassis_body_);
    snap.powertrain = world_->powertrain_state(vehicle_id_);

    const std::size_t wheel_count = world_->vehicle_wheel_count(vehicle_id_);
    snap.wheels.reserve(wheel_count);
    for (std::size_t i = 0; i < wheel_count; ++i) {
        WheelSnapshot ws;
        ws.name = world_->wheel_name(vehicle_id_, i);
        ws.state = world_->wheel_state(vehicle_id_, i);
        snap.wheels.push_back(std::move(ws));
    }
    return snap;
}

std::unique_ptr<Session> make_session(const SessionConfig& config, std::string* error, SessionFailure* failure) {
    SessionFailure why = SessionFailure::Error;
    std::string text;
    try {
        auto session = std::make_unique<Session>(config);
        if (failure != nullptr) *failure = SessionFailure::None;
        if (error != nullptr) error->clear();
        return session;
    } catch (const SessionCancelled& e) {
        why = SessionFailure::Cancelled;
        text = e.what();
    } catch (const std::exception& e) {
        text = e.what();
    } catch (...) {
        text = "Session: constructor threw a non-std exception";
    }
    if (failure != nullptr) *failure = why;
    if (error != nullptr) *error = std::move(text);
    return nullptr;
}

} // namespace rg
