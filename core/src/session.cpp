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
#include <filesystem>
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
    "nitrous_arm", // N2O arm switch, a plain 0/1 level (owner 2026-10-05; only cars with a nitrous kit declare it)
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
    world_config.air_density = sample_environment(config.environment,config.chassis_z_m,0).air_density;
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

Session::~Session() { stop(); truck_cancel_.store(true); traffic_cancel_.store(true); if(truck_worker_.joinable())truck_worker_.join(); if(traffic_worker_.joinable())traffic_worker_.join(); }

void Session::build_world_contents(const SessionConfig& config) {
    traffic_stuck_.clear();traffic_stuck_summary_time_=0;
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

    if (config.vehicle_definition) {
        vehicle_desc_ = *config.vehicle_definition;
    } else {
        ps::io::EngineMapOptions options;
        std::size_t generated = 0;
        options.generation_count_out = &generated;
        if (config.engine_map_cache_enabled) {
            // Vehicle files live in <physics_sim>/data/vehicles; use the
            // same persistent cache location as physics_sim's Godot demo.
            if (!config.engine_map_cache_dir.empty()) {
                options.cache_dir = config.engine_map_cache_dir;
            } else {
                const auto vehicle_path = std::filesystem::absolute(config.vehicle_json_path);
                const auto library_root = vehicle_path.parent_path().parent_path().parent_path();
                options.cache_dir = (library_root / "out" / "godot_engine_cache").string();
            }
        }
        vehicle_desc_ = ps::io::load_vehicle_json(config.vehicle_json_path, options);
        std::fprintf(stderr, "RG_ENGINE_MAP_CACHE enabled=%s generated=%zu dir=%s\n",
                     config.engine_map_cache_enabled ? "yes" : "no", generated, options.cache_dir.c_str());
    }
    vehicle_id_ = world_->create_vehicle(vehicle_desc_, chassis_body_);
    have_vehicle_ = true;
    walker_wheels_.clear();
    for (const auto& wheel : vehicle_desc_.wheels) {
        walker_wheels_.push_back(WheelFootprint{wheel.attachment_local.x, wheel.attachment_local.y, wheel.wheel_radius,
                                                0.5 * wheel.wheel_width});
    }
    if(!vehicle_desc_.aero.fans.empty()&&config_.environment.fan_battery_energy_j>0)
        world_->credit_aero_fan_energy(vehicle_id_,config_.environment.fan_battery_energy_j);
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
    // The loader gate must cover every footprint a background shape may read.
    if (phys.prefetch_margin_tiles > static_cast<std::uint32_t>(g2m::phys::StreamerConfig{}.gate_margin_tiles) ||
        phys.prefetch_max_tiles > 1024 ||
        ((phys.prefetch_margin_tiles == 0) != (phys.prefetch_max_tiles == 0))) {
        throw std::invalid_argument("Session: invalid terrain prefetch margin/cap (margin exceeds streamer gate)");
    }
    // The TileManager pool is sized for TWO interest points (R9b: the player's
    // car plus a followed NPC/truck, physics_interest_points): twice the
    // single-square pool make_terrain_config computes. The priming check below
    // still compares against ONE square - priming runs with one point.
    ps::terrain::TerrainConfig terrain_config = g2m::ps_bridge::make_terrain_config(
        phys.radius_m, phys.max_tile_fills_per_tick, phys.prefetch_margin_tiles, phys.prefetch_max_tiles);
    terrain_config.max_resident_tiles *= 2;
    world_->set_terrain_source(terrain_->source, terrain_config);

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

    sync_road_decks();

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
        relocation_ = RelocateTarget{pose.position.x, pose.position.y, flip_upright_yaw(pose.orientation),false};
    }
}

void Session::finish_relocation() {
    // Priming advances World without traffic controllers: freeze actor motion so
    // preserved NPC route stations and poses remain in agreement after F.
    for(const auto& actor:traffic_actors_)world_->backend().set_motion(actor.body,ps::Motion{});
    if(truck_state_.active)world_->backend().set_motion(truck_body_,ps::Motion{});
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
    if (walker_ && walker_->active()) walker_->halt(); // the priming ticks above stepped the World
    status_.relocations.fetch_add(1, std::memory_order_relaxed);
}

std::span<const g2m::phys::InterestPoint> Session::physics_interest_points() {
    // Every physics actor that needs resident terrain gets one entry: the
    // chassis (or, before the vehicle exists, the spawn point or a relocation
    // target) as entry 0, and the followed vehicle (R9b) as entry 1 while one
    // is followed. Later actors (a walker, a parked car) are appended here; the
    // TileManager pool (setup_terrain: twice make_terrain_config's 49 tiles at
    // r = 400 m) is sized for TWO points and must be re-sized for more.
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
    if (walker_ && walker_->active()) {
        // On foot (R9c): the walker is point 0 and the parked car (or the relocation
        // target it is being moved to) keeps its terrain resident as the second
        // point. A followed vehicle cannot coexist with a walker (the mode machine
        // resets the drone target on entering OnFoot); it is ignored meanwhile.
        const ps::Vec3 feet = walker_->state().feet;
        const ps::Vec3 wv = walker_->state().velocity;
        out.push_back(g2m::phys::InterestPoint{kPlayerInterestId, feet.x, feet.y, wv.x, wv.y, t.config.physics.radius_m});
        out.push_back(g2m::phys::InterestPoint{kParkedCarInterestId, x, y, vx, vy, t.config.physics.radius_m});
        return out;
    }
    out.push_back(g2m::phys::InterestPoint{kPlayerInterestId, x, y, vx, vy, t.config.physics.radius_m});
    if (const std::uint64_t followed = followed_id_.load(std::memory_order_relaxed); followed != 0) {
        ps::Vec3 p, v;
        if (locate_followed(followed, p, v)) {
            out.push_back(g2m::phys::InterestPoint{kFollowedInterestId, p.x, p.y, v.x, v.y, t.config.physics.radius_m});
        }
    }
    return out;
}

bool Session::locate_followed(std::uint64_t id, ps::Vec3& position, ps::Vec3& velocity) const {
    if (id == 0) return false;
    if (id == kNpcTruckVehicleId) {
        if (!truck_state_.active) return false;
        position = world_->get_pose(truck_body_).position;
        velocity = world_->get_motion(truck_body_).linear;
        return true;
    }
    for (const TrafficActor& actor : traffic_actors_) {
        if (actor.id != id) continue;
        position = world_->get_pose(actor.body).position;
        velocity = world_->get_motion(actor.body).linear;
        return true;
    }
    return false;
}

void Session::check_followed_alive() {
    std::uint64_t id = followed_id_.load(std::memory_order_relaxed);
    if (id == 0) return;
    ps::Vec3 p, v;
    if (locate_followed(id, p, v)) return;
    // Lost: clear it (unless another thread already replaced it) and make it observable.
    if (followed_id_.compare_exchange_strong(id, 0, std::memory_order_relaxed)) {
        status_.followed_lost_id.store(id, std::memory_order_relaxed);
        status_.followed_lost.fetch_add(1, std::memory_order_release);
    }
}

void Session::set_followed_vehicle(std::optional<std::uint64_t> id) {
    followed_id_.store(id.value_or(0), std::memory_order_relaxed);
}

Session::FollowLoss Session::followed_loss() const {
    FollowLoss loss;
    loss.count = status_.followed_lost.load(std::memory_order_acquire);
    loss.id = status_.followed_lost_id.load(std::memory_order_relaxed);
    return loss;
}

std::optional<std::uint64_t> Session::followed_vehicle() const {
    const std::uint64_t id = followed_id_.load(std::memory_order_relaxed);
    if (id == 0) return std::nullopt;
    return id;
}

std::span<const g2m::phys::InterestPoint> Session::last_interest_points() const {
    if (!terrain_) return {};
    return terrain_->interest_points;
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
    // The followed point comes and goes (set_followed_vehicle, a lost actor):
    // release its TileManager point when it is no longer in the list.
    const bool has_follow_point = points.size() > 1;
    if (follow_point_active_ && !has_follow_point) world_->remove_terrain_interest_point(kFollowedInterestId);
    follow_point_active_ = has_follow_point;
    status_.interest_points.store(static_cast<std::uint32_t>(points.size()), std::memory_order_relaxed);
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
            if (terrain_) {
                spike.resident_tiles = world_->terrain_resident_tile_count();
                spike.prefetch = world_->terrain_prefetch_stats();
            }
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
    if(relocation_&&relocation_->clear_traffic)update_traffic(true);
    if(relocation_&&relocation_->clear_traffic&&(truck_state_.active||truck_state_.loading)){truck_request_.store(-1);update_npc_truck();}
    check_followed_alive(); // a relocation just cleared the traffic and the truck
    spike.take_relocate_ms = lap();
    if (terrain_) {
        const bool ready = gate_check() && sync_road_decks(1);
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
            return finish(TickSpikeKind::Frozen, false); // frozen: vehicle states and simulation time do not advance
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

    // On foot (R9c): a pending get-out/get-in is handled here, past the gate and
    // any relocation, so a spawn only ever happens with the car's terrain resident.
    process_walker_request();

    if (have_vehicle_ && drive_script_) {
        drive_script_->apply(DriveTickContext{drive_tick(), chassis_body_, vehicle_id_}, *world_);
    } else if (from_loop) {
        apply_live_controls();
    }
    spike.controls_ms = lap();

    // Everything below reads the chassis (or actors placed around it). Terrain
    // start-up primes the TileManager with World ticks BEFORE the chassis
    // exists (setup_terrain -> step_blocking): chassis_body_ is still the
    // default BodyId{} there, which in a release build silently resolved to
    // backend slot 0 (the first heightfield pool body) and planned the first
    // traffic scan around that body's position. Priming ticks only step.
    if (have_vehicle_) {
        update_npc_truck();
        update_traffic();
        check_followed_alive(); // despawned at its route end / truck removed this tick
        update_walker();
        environment_sample_=sample_environment(config_.environment,world_->get_pose(chassis_body_).position.z,world_->sim_time());
        if(config_.environment.enabled)world_->set_ambient({environment_sample_.pressure_pa,environment_sample_.temperature_k});
        world_->set_aero_environment({environment_sample_.air_density,environment_sample_.wind_world_m_s});
        const auto motion=world_->get_motion(chassis_body_);
        const auto rotation=world_->get_pose(chassis_body_).orientation;
        const double forward_speed=rotation.inverse().rotate(motion.linear-environment_sample_.wind_world_m_s).x;
        const double brake=std::clamp(static_cast<double>(world_->get_control("brake")),0.0,1.0);
        for(std::size_t i=0;i<vehicle_desc_.aero.surfaces.size();++i) {
            const auto& surface=vehicle_desc_.aero.surfaces[i];
            if(surface.name!="rear_wing"||!config_.environment.automatic_rear_wing)continue;
            const auto& c=config_.environment;
            double target=c.cruise_wing_offset_deg*std::clamp((forward_speed-10)/20,0.0,1.0);
            if(forward_speed>=c.airbrake_min_speed_m_s) {
                const double demand=std::clamp((brake-c.airbrake_threshold)/(1-c.airbrake_threshold),0.0,1.0);
                target+=(c.airbrake_wing_offset_deg-target)*demand;
            }
            const double radians=target*3.141592653589793/180;
            const double limit=radians>=0?surface.max_offset_rad:-surface.min_offset_rad;
            world_->set_aero_surface_command(vehicle_id_,i,limit>0?std::clamp(radians/limit,-1.0,1.0):0.0);
        }
        for(std::size_t i=0;i<vehicle_desc_.aero.fans.size();++i)
            world_->set_aero_fan_command(vehicle_id_,i,config_.environment.fan_command);
    }
    world_->step();
    spike.step_ms = lap();
    post_step(from_loop);
    spike.post_ms = lap();
    return finish(TickSpikeKind::Stepped, true);
}

bool Session::sync_road_decks(int budget) {
    if(!config_.terrain||!config_.terrain->world_terrain) return true;
    auto& terrain=*config_.terrain->world_terrain;
    const auto focus=relocation_?ps::Vec3{relocation_->x,relocation_->y,0}:have_vehicle_?world_->get_pose(chassis_body_).position:ps::Vec3{config_.terrain->spawn_x,config_.terrain->spawn_y,0};
    const double e=focus.x+terrain.frame().e0_m(),n=focus.y+terrain.frame().n0_m();
    const double radius=config_.terrain->physics.radius_m+255;
    const double prefetch_radius=radius+kDeckPrefetchMarginM;
    if(!deck_installer_) {
        DeckInstaller::Options options;
        options.workers=kDeckBuildWorkers;
        deck_installer_=std::make_unique<DeckInstaller>(*world_,std::move(options));
    }
    // Decks in range are REQUIRED (installed in (way, start, end) order before the clock runs); decks a margin
    // further out are only built, so their shape is ready by the time the car gets there (S1, design 5.2 option B).
    std::vector<DeckCandidate> required,prefetch;
    for(const auto& deck:terrain.road_decks()) {
        const DeckKey key=deck_key(*deck);
        if(deck_installer_->installed(key)) continue;
        const double dx=std::max({deck->min_easting-e,e-deck->max_easting,0.0});
        const double dy=std::max({deck->min_northing-n,n-deck->max_northing,0.0});
        const double d2=dx*dx+dy*dy;
        if(d2>(config_.legacy_deck_install?radius:prefetch_radius)*(config_.legacy_deck_install?radius:prefetch_radius)) continue;
        DeckCandidate c;
        c.key=key;c.deck=deck;
        c.surface=surface_table_->id_for(deck->land_class==g2m::LandClass::PavedRoad?config_.terrain->physics.road_surfaces.paved:config_.terrain->physics.road_surfaces.unpaved);
        c.x=deck->mesh.origin[0]-terrain.frame().e0_m();
        c.y=deck->mesh.origin[1]-terrain.frame().n0_m();
        (d2<=radius*radius?required:prefetch).push_back(std::move(c));
    }
    const auto by_key=[](const DeckCandidate& a,const DeckCandidate& b){return a.key<b.key;};
    std::stable_sort(required.begin(),required.end(),by_key);
    std::stable_sort(prefetch.begin(),prefetch.end(),by_key);
    if(config_.legacy_deck_install) // reference path: the old pacing was one deck per attempt, built on this thread
        return deck_installer_->update_legacy(*world_,required,budget>0?1:0);
    if(budget<=0) { // start-up / relocation: block until every deck in range is installed
        (void)deck_installer_->update(*world_,{},prefetch,0);
        deck_installer_->wait_until_installed(*world_,required);
        return true;
    }
    return deck_installer_->update(*world_,required,prefetch,kDeckInstallBudget);
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
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "tick=%llu wall_s=%.3f kind=%s gap_ms=%.2f prev_total_ms=%.2f total_ms=%.2f take_ms=%.2f "
                  "gate_update_ms=%.2f gate_interest_ms=%.2f controls_ms=%.2f step_ms=%.2f post_ms=%.2f "
                  "pos=(%.1f,%.1f) speed_kmh=%.1f resident_tiles=%llu "
                  "prefetch_enqueued=%llu prefetch_installed=%llu prefetch_late_sync=%llu prefetch_late_wait=%llu prefetch_late_wait_ns_total=%llu prefetch_late_wait_ns_max=%llu prefetch_stale_inputs=%llu prefetch_cancelled=%llu prefetch_skipped_suppression=%llu",
                  static_cast<unsigned long long>(s.tick), s.wall_s, kind, s.gap_ms, s.prev_total_ms, s.total_ms,
                  s.take_relocate_ms, s.gate_update_ms, s.gate_interest_ms, s.controls_ms, s.step_ms, s.post_ms, s.x,
                  s.y, s.speed_mps * 3.6, static_cast<unsigned long long>(s.resident_tiles),
                  static_cast<unsigned long long>(s.prefetch.enqueued),
                  static_cast<unsigned long long>(s.prefetch.installed),
                  static_cast<unsigned long long>(s.prefetch.late_sync),
                  static_cast<unsigned long long>(s.prefetch.late_wait),
                  static_cast<unsigned long long>(s.prefetch.late_wait_ns_total),
                  static_cast<unsigned long long>(s.prefetch.late_wait_ns_max),
                  static_cast<unsigned long long>(s.prefetch.stale_inputs),
                  static_cast<unsigned long long>(s.prefetch.cancelled),
                  static_cast<unsigned long long>(s.prefetch.skipped_suppression));
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

void Session::set_engine_audio_publisher(EngineAudioPublisher publisher) {
    std::lock_guard<std::mutex> lock(engine_audio_mutex_);
    engine_audio_publisher_ = std::move(publisher);
}
void Session::post_step(bool from_loop) {
    if (from_loop) {
        auto& frame = snapshot_buffer_.write_slot();
        frame = capture_frame_snapshot();
        {
            std::lock_guard<std::mutex> lock(engine_audio_mutex_);
            if(engine_audio_publisher_ && !frame.powertrain.engines.empty())
                engine_audio_publisher_(frame.powertrain.engines.front(), frame.sim_time);
        }
        pose_history_scratch_[0] = frame.chassis_pose; // before publish(): the slot belongs to readers afterwards
        const std::uint64_t published_tick = frame.tick;
        snapshot_buffer_.publish();
        // Nominal-clock pose history (session.h sample_render_poses). Only on the
        // real-time loop thread: tick_nominal_time() is meaningless for a plain
        // try_step() call from a test.
        if (loop_.running()) {
            const std::uint64_t relocations = status_.relocations.load(std::memory_order_relaxed);
            pose_history_teleport_[0] = relocations != pose_history_relocations_ ? 1 : 0;
            pose_history_relocations_ = relocations;
            pose_history_.push(loop_.tick_nominal_time(), published_tick, loop_.tick_clock_break(), pose_history_scratch_, pose_history_teleport_);
        }
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
    const auto prefetch = world_->terrain_prefetch_stats();
    status_.prefetch_enqueued.store(prefetch.enqueued, std::memory_order_relaxed);
    status_.prefetch_installed.store(prefetch.installed, std::memory_order_relaxed);
    status_.prefetch_late_sync.store(prefetch.late_sync, std::memory_order_relaxed);
    status_.prefetch_late_wait.store(prefetch.late_wait, std::memory_order_relaxed);
    status_.prefetch_late_wait_ns_total.store(prefetch.late_wait_ns_total, std::memory_order_relaxed);
    status_.prefetch_late_wait_ns_max.store(prefetch.late_wait_ns_max, std::memory_order_relaxed);
    status_.prefetch_stale_inputs.store(prefetch.stale_inputs, std::memory_order_relaxed);
    status_.prefetch_cancelled.store(prefetch.cancelled, std::memory_order_relaxed);
    status_.prefetch_skipped_suppression.store(prefetch.skipped_suppression, std::memory_order_relaxed);
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
    pose_history_.reset(1); // chassis only, before the loop thread exists
    pose_history_relocations_ = status_.relocations.load(std::memory_order_relaxed);
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
    loop_.start([this] {
        // Paused: report "held back" - the loop resyncs its deadline, so
        // resuming never bursts (set_paused()).
        if (paused_.load(std::memory_order_relaxed)) return false;
        return step_once(true);
    });
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
    s.prefetch.enqueued = status_.prefetch_enqueued.load(std::memory_order_relaxed);
    s.prefetch.installed = status_.prefetch_installed.load(std::memory_order_relaxed);
    s.prefetch.late_sync = status_.prefetch_late_sync.load(std::memory_order_relaxed);
    s.prefetch.late_wait = status_.prefetch_late_wait.load(std::memory_order_relaxed);
    s.prefetch.late_wait_ns_total = status_.prefetch_late_wait_ns_total.load(std::memory_order_relaxed);
    s.prefetch.late_wait_ns_max = status_.prefetch_late_wait_ns_max.load(std::memory_order_relaxed);
    s.prefetch.stale_inputs = status_.prefetch_stale_inputs.load(std::memory_order_relaxed);
    s.prefetch.cancelled = status_.prefetch_cancelled.load(std::memory_order_relaxed);
    s.prefetch.skipped_suppression = status_.prefetch_skipped_suppression.load(std::memory_order_relaxed);
    s.startup_ms = status_.startup_ms.load(std::memory_order_relaxed);
    s.prime_ticks = status_.prime_ticks.load(std::memory_order_relaxed);
    s.relocations = status_.relocations.load(std::memory_order_relaxed);
    s.relocate_failures = status_.relocate_failures.load(std::memory_order_relaxed);
    // Lost count first (acquire), then its id: a reader that sees a count has the id that goes with it.
    s.followed_lost = status_.followed_lost.load(std::memory_order_acquire);
    s.followed_lost_id = status_.followed_lost_id.load(std::memory_order_relaxed);
    s.followed_id = followed_id_.load(std::memory_order_relaxed);
    s.interest_points = status_.interest_points.load(std::memory_order_relaxed);
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
    snap.traffic.config=traffic_config_;snap.traffic.target=traffic_population_target_;snap.traffic.queued=static_cast<int>(traffic_ready_.trips.size());snap.traffic.loading=traffic_loading_;snap.traffic.message=traffic_message_;
    for(const auto& actor:traffic_actors_)snap.traffic.actors.push_back({actor.id,actor.trip.truck,world_->get_pose(actor.body),actor.speed,actor.trip.destination});
    snap.truck=truck_state_;
    if(truck_state_.active)snap.truck.pose=world_->get_pose(truck_body_);
    if(walker_&&walker_->active()) {
        const WalkerState& w=walker_->state();
        WalkerSnapshot& out=snap.walker;
        out.active=true;out.feet=w.feet;out.velocity=w.velocity;out.yaw_rad=w.yaw_rad;out.grounded=w.grounded;out.blocked=w.blocked;out.hold=w.hold;
        out.can_enter=walker_can_enter_;out.enter_distance_m=walker_enter_distance_m_;
        out.car_x=walker_footprint_.cx;out.car_y=walker_footprint_.cy;out.car_half_x=walker_footprint_.half_x;out.car_half_y=walker_footprint_.half_y;
        out.car_yaw_rad=ps::math::atan2(walker_footprint_.sin_yaw,walker_footprint_.cos_yaw);
    }
    snap.aero.environment = environment_sample_;
    const auto& aero=world_->vehicle_aero_telemetry(vehicle_id_);
    snap.aero.enabled=!vehicle_desc_.aero.surfaces.empty()||!vehicle_desc_.aero.fans.empty()||vehicle_desc_.aero.body.reference_area_m2>0;
    snap.aero.airspeed_m_s=aero.body_airspeed_m_s;
    snap.aero.downforce_n=aero.downforce_n;snap.aero.front_balance=aero.front_balance;
    snap.aero.fan_power_w=aero.fan_power_w;
    snap.aero.fan_energy_remaining_j=aero.fan_energy_remaining_j;
    snap.aero.force_world=aero.wrench_world.force;snap.aero.moment_world=aero.wrench_world.torque;
    const auto force_local=snap.chassis_pose.orientation.inverse().rotate(aero.wrench_world.force);
    snap.aero.side_force_n=force_local.y;
    if(vehicle_desc_.wheels.size()>1&&std::abs(aero.downforce_n)>1e-6) {
        double front=-1e30,rear=1e30;
        for(const auto& wheel:vehicle_desc_.wheels){front=std::max(front,static_cast<double>(wheel.attachment_local.x));rear=std::min(rear,static_cast<double>(wheel.attachment_local.x));}
        const double pitch=snap.chassis_pose.orientation.inverse().rotate(aero.wrench_world.torque).y;
        if(front-rear>1e-6)snap.aero.front_balance=(pitch-rear*aero.downforce_n)/((front-rear)*aero.downforce_n);
    }
    if(aero.body_airspeed_m_s>1e-9)snap.aero.drag_n=-ps::dot(force_local,aero.body_relative_airflow_local/aero.body_airspeed_m_s);
    const auto arm=snap.chassis_pose.orientation.rotate(vehicle_desc_.aero.body.position_local);
    const double undisturbed=(snap.chassis_motion.velocity_at(arm)-environment_sample_.wind_world_m_s).length_squared();
    snap.aero.wake_factor=undisturbed>1e-9?aero.body_airspeed_m_s*aero.body_airspeed_m_s/undisturbed:1;
    for(std::size_t i=0;i<vehicle_desc_.aero.surfaces.size();++i) {
        const auto& st=aero.surfaces[i];
        snap.aero.surfaces.push_back({vehicle_desc_.aero.surfaces[i].name,st.alpha_rad,st.beta_rad,st.cl,st.cd,st.clearance_m,st.ground_multiplier,st.dynamic_pressure_pa,st.wrench_world.force});
        if(vehicle_desc_.aero.surfaces[i].name=="rear_wing") { snap.aero.wing_pitch_offset_deg=st.offset_rad*180/3.141592653589793; snap.aero.wing_lift_m=st.lift_m; }
    }
    if (terrain_) {
        const auto& frame = terrain_->config.frame;
        const auto& position = snap.chassis_pose.position;
        const auto key = frame.tile_at({position.x, position.y}, 0);
        const auto forward = snap.chassis_pose.orientation.rotate(ps::Vec3::unit_x());
        const g2m::geom::PointMm point{std::llround(frame.grid_easting(position.x) * 1000.0),
                                      std::llround(frame.grid_northing(position.y) * 1000.0)};
        const bool want_road_ahead = road_ahead_wanted_.load(std::memory_order_relaxed);
        std::vector<std::shared_ptr<const std::vector<g2m::RoadSegment>>> road_tiles; // kept alive for the trace
        if (key) {
            if (want_road_ahead) road_tiles.reserve(9);
            // Read already-resident metadata only; no fetches or terrain locks.
            for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
                auto neighbor = *key;
                neighbor.x += x;
                neighbor.y += y;
                const g2m::HeightTile* height = nullptr;
                std::shared_ptr<const std::vector<g2m::RoadSegment>> roads;
                terrain_->resident->find(neighbor, &height, &roads);
                if (!roads) continue;
                if (want_road_ahead) road_tiles.push_back(roads);
                const auto candidate = g2m::match_road_speed_limit(*roads, point, forward.x, forward.y);
                if (candidate.distance_mm < snap.road_speed_limit.distance_mm ||
                    (candidate.distance_mm == snap.road_speed_limit.distance_mm && candidate.way_id < snap.road_speed_limit.way_id)) {
                    snap.road_speed_limit = candidate;
                }
            }
        }
        // Road ahead for the cinematic camera: re-traced every 12th tick
        // (~20 Hz), the cached polyline copied into the ticks in between.
        if (!want_road_ahead) {
            road_ahead_cache_valid_ = false;
            road_ahead_cache_.clear();
        } else {
            if (!road_ahead_cache_valid_ || snap.tick >= road_ahead_tick_ + 12 || snap.tick < road_ahead_tick_) {
                std::vector<std::span<const g2m::RoadSegment>> spans;
                spans.reserve(road_tiles.size());
                for (const auto& t : road_tiles) spans.emplace_back(t->data(), t->size());
                RoadAheadParams params;
                if (!trace_road_ahead(spans, frame.grid_easting(position.x), frame.grid_northing(position.y), forward.x,
                                      forward.y, position.x, position.y, params, road_ahead_cache_)) {
                    road_ahead_cache_.clear();
                }
                road_ahead_tick_ = snap.tick;
                road_ahead_cache_valid_ = true;
            }
            snap.road_ahead = road_ahead_cache_;
        }
    } else {
        road_ahead_cache_valid_ = false;
    }

    const std::size_t wheel_count = world_->vehicle_wheel_count(vehicle_id_);
    snap.wheels.reserve(wheel_count);
    for (std::size_t i = 0; i < wheel_count; ++i) {
        WheelSnapshot ws;
        ws.name = world_->wheel_name(vehicle_id_, i);
        ws.state = world_->wheel_state(vehicle_id_, i);
        ws.telemetry = world_->wheel_telemetry(vehicle_id_, i);
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

namespace rg {
void Session::request_npc_truck(bool enabled,double speed_kph) {
 if(std::isfinite(speed_kph))truck_target_.store(std::clamp(speed_kph,10.,90.)/3.6);
 truck_request_.store(enabled?1:-1);
}
void Session::update_npc_truck() {
 const int request=truck_request_.exchange(0);
 if(request) {
  if(truck_state_.active){world_->destroy_body(truck_body_);truck_state_.active=false;}
  world_->set_aero_wakes({});truck_cancel_.store(true);truck_state_.message="Truck removed";
  if(request>0) {
   if(truck_worker_.joinable()&&!truck_done_.load()){truck_request_.store(request);return;}
   if(truck_worker_.joinable())truck_worker_.join();
   truck_cancel_.store(false);truck_done_.store(false);truck_state_.loading=true;truck_state_.message="Truck: preparing lane route";
   const auto car=world_->get_pose(chassis_body_);auto terrain=config_.terrain?config_.terrain->world_terrain:nullptr;const double speed=truck_target_.load();
   truck_worker_=std::thread([this,terrain,car,speed]{
    TruckRoute route;
    try{route=build_truck_route(terrain,car,speed,truck_cancel_);}catch(const std::exception& e){route.message=std::string("Truck route error: ")+e.what();}
    {std::lock_guard<std::mutex> lock(truck_mutex_);truck_pending_=std::move(route);}truck_done_.store(true);
   });
  } else truck_state_.loading=false;
 }
 if(truck_state_.loading&&truck_done_.load()) {
  if(truck_worker_.joinable())truck_worker_.join();
  {std::lock_guard<std::mutex> lock(truck_mutex_);truck_route_=std::move(truck_pending_);}
  truck_state_.loading=false;truck_state_.message=truck_route_.message;
  if(!truck_route_.points.empty()&&!truck_cancel_.load()) {
   const auto car=world_->get_pose(chassis_body_);const auto heading=car.orientation.rotate(ps::Vec3::unit_x());
   auto closest=truck_route_.points.begin();double distance=1e30;
   for(auto it=truck_route_.points.begin();it!=truck_route_.points.end();++it) {
    if(std::cos(it->yaw)*heading.x+std::sin(it->yaw)*heading.y<.5)continue;
    const auto delta=it->ground-car.position;const double score=delta.x*delta.x+delta.y*delta.y;
    if(score<distance){distance=score;closest=it;}
   }
   auto first=std::lower_bound(closest,truck_route_.points.end(),closest->station+45,[](const auto& p,double s){return p.station<s;});
   if(distance>900||first==truck_route_.points.end()||truck_route_.points.back().station-first->station<20) {
    truck_state_.message="Truck: route no longer ahead; press T again";return;
   }
   truck_route_.points.erase(truck_route_.points.begin(),first);
   const double start=truck_route_.points.front().station;for(auto& point:truck_route_.points)point.station-=start;
   const auto& p=truck_route_.points.front();ps::BodyDesc body;body.motion=ps::BodyMotionType::Kinematic;body.shape=ps::BoxShape{{6.5,1.25,1.8}};
   body.gravity_enabled=false;body.pose.position=p.ground+ps::Vec3{0,0,2.2};body.pose.orientation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),p.yaw)*ps::Quat::from_axis_angle(ps::Vec3::unit_y(),-std::atan(p.grade));
   truck_body_=world_->create_body(body);truck_state_.active=true;truck_station_=0;truck_speed_=std::min(truck_target_.load(),p.speed_m_s);
  }
 }
 if(!truck_state_.active)return;
 const auto car=world_->get_pose(chassis_body_);auto pose=world_->get_pose(truck_body_);
 const auto forward=pose.orientation.rotate(ps::Vec3::unit_x());const auto relative=car.position-pose.position;
 const double dt=1/config_.tick_rate_hz,end=truck_route_.points.back().station;
 auto upper=std::upper_bound(truck_route_.points.begin(),truck_route_.points.end(),truck_station_,[](double s,const auto& p){return s<p.station;});
 const auto& current=*(upper==truck_route_.points.end()?upper-1:upper);
 double target=std::min(truck_target_.load(),current.speed_m_s);
 target=std::min(target,std::sqrt(std::max(0.,2*2.5*(end-truck_station_-5))));
 // Remain within the player's terrain residency - unless the camera follows the truck (then its own interest point keeps its terrain resident).
 // On foot (R9c) the walker's own terrain counts as the player's residency too.
 const double nearest=walker_&&walker_->active()?std::min(relative.length(),(walker_->state().feet-pose.position).length()):relative.length();
 if(nearest>200&&followed_id_.load(std::memory_order_relaxed)!=kNpcTruckVehicleId)target=0;
 const double gap=ps::dot(relative,forward),lateral=std::abs(relative.x*forward.y-relative.y*forward.x);
 if(gap>0&&gap<35&&lateral<2.8)target=std::min(target,std::max(0.,(gap-12)*.7));
 truck_speed_+=std::clamp(target-truck_speed_,-4*dt,1.5*dt);
 const double next_station=std::min(end,truck_station_+truck_speed_*dt);
 auto next=std::lower_bound(truck_route_.points.begin(),truck_route_.points.end(),next_station,[](const auto& p,double s){return p.station<s;});
 if(next==truck_route_.points.end())next=truck_route_.points.end()-1;
 auto prev=next==truck_route_.points.begin()?next:next-1;
 const double span=next->station-prev->station,t=span>1e-9?(next_station-prev->station)/span:0;
 const auto ground=prev->ground+(next->ground-prev->ground)*t;
 const double yaw=prev->yaw+std::remainder(next->yaw-prev->yaw,2*3.141592653589793)*t,grade=prev->grade+(next->grade-prev->grade)*t;
 ps::Motion motion;motion.linear=(ground+ps::Vec3{0,0,2.2}-pose.position)/dt;
 const auto target_rotation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),yaw)*ps::Quat::from_axis_angle(ps::Vec3::unit_y(),-std::atan(grade));
 auto delta_rotation=(target_rotation*pose.orientation.inverse()).normalized();
 ps::Vec3 axis{delta_rotation.x,delta_rotation.y,delta_rotation.z};
 if(delta_rotation.w<0){axis=-axis;delta_rotation.w=-delta_rotation.w;}
 const double sine=axis.length();
 if(sine>1e-12)motion.angular=axis*(2*std::atan2(sine,delta_rotation.w)/(sine*dt));
 world_->backend().set_motion(truck_body_,motion);
 world_->set_aero_wakes({ps::aero::WakeSource{truck_body_,pose.position,motion.linear,{60,2,.15,.45,.2}}});
 truck_station_=next_station;truck_state_.speed_m_s=truck_speed_;
}
}

namespace rg {

// ---------------------------------------------------------------------------
// On foot (R9c)
// ---------------------------------------------------------------------------

namespace {
// Chassis-local driver's door position along the car (the spawn spot's x).
constexpr double kDriverDoorX = 0.3;
// Clearance between the car footprint and a spawned walker's capsule.
constexpr double kSpawnClearanceM = 0.25;
// How far above the chassis origin the car's roof is taken to be, and how far
// below its origin the footprint prism starts (the walker is blocked while its
// capsule overlaps the prism vertically).
constexpr double kRoofAboveChassisM = 0.85;
constexpr double kPrismBelowChassisM = 1.0;
constexpr double kFootprintMarginM = 0.05;
} // namespace

OrientedRect Session::own_car_footprint() const {
    const ps::Pose pose = world_->get_pose(chassis_body_);
    const ps::Vec3 forward = pose.orientation.rotate(ps::Vec3::unit_x());
    return vehicle_footprint(pose.position, forward, config_.chassis_half_extents.x, config_.chassis_half_extents.y,
                             walker_wheels_, kFootprintMarginM, pose.position.z - kPrismBelowChassisM,
                             pose.position.z + kRoofAboveChassisM);
}

ps::Vec3 Session::traffic_anchor() const {
    if (walker_ && walker_->active()) return walker_->state().feet;
    return world_->get_pose(chassis_body_).position;
}

void Session::request_walker_spawn() { walker_request_.store(1, std::memory_order_release); }
void Session::request_walker_enter() { walker_request_.store(2, std::memory_order_release); }
void Session::request_walker_despawn() { walker_request_.store(3, std::memory_order_release); }

void Session::set_walker_input(const WalkerInput& input) {
    std::lock_guard<std::mutex> lock(walker_input_mutex_);
    walker_input_ = input;
    walker_input_.jump = false; // jumps travel through request_walker_jump
}

Session::WalkerCounters Session::walker_counters() const {
    WalkerCounters c;
    c.spawned = walker_spawned_.load(std::memory_order_acquire);
    c.entered = walker_entered_.load(std::memory_order_acquire);
    c.enter_refused = walker_enter_refused_.load(std::memory_order_acquire);
    c.despawned = walker_despawned_.load(std::memory_order_acquire);
    c.spawn_failed = walker_spawn_failed_.load(std::memory_order_acquire);
    return c;
}

void Session::spawn_walker() {
    if (!have_vehicle_) {
        walker_spawn_failed_.fetch_add(1, std::memory_order_release);
        return;
    }
    if (!walker_) walker_ = std::make_unique<WalkerController>(*world_, config_.walker);
    const ps::Pose chassis = world_->get_pose(chassis_body_);
    walker_->set_ignored_bodies({chassis_body_});
    walker_footprint_ = own_car_footprint();
    walker_->set_obstacles(std::span<const OrientedRect>(&walker_footprint_, 1));

    const ps::Vec3 forward = chassis.orientation.rotate(ps::Vec3::unit_x());
    const double yaw = ps::math::atan2(forward.y, forward.x);
    const WalkerController& probe_walker = *walker_;
    const SpawnProbe probe = [&](double x, double y) {
        return probe_walker.probe_standing(x, y, chassis.position.z, 2.0, 4.0);
    };
    const SpawnSpot spot = select_spawn_spot(walker_footprint_, chassis.position, yaw, config_.walker.radius_m,
                                             kDriverDoorX, kSpawnClearanceM, walker_footprint_.z_max + 0.1, probe);
    walker_->spawn(spot.feet, spot.yaw_rad);
    walker_enter_distance_m_ = distance_to_rect(walker_footprint_, spot.feet.x, spot.feet.y);
    walker_can_enter_ = within_enter_range(walker_footprint_, spot.feet, config_.walker.enter_range_m,
                                           config_.walker.enter_max_dz_m);
    walker_active_.store(true, std::memory_order_release);
    walker_spawned_.fetch_add(1, std::memory_order_release);
}

void Session::remove_walker() {
    if (!walker_ || !walker_->active()) return;
    walker_->despawn();
    walker_active_.store(false, std::memory_order_release);
    walker_can_enter_ = false;
}

void Session::process_walker_request() {
    const int request = walker_request_.exchange(0, std::memory_order_acq_rel);
    if (request == 0) return;
    const bool active = walker_ && walker_->active();
    switch (request) {
        case 1:
            if (!active) spawn_walker();
            break;
        case 2:
            if (!active) break;
            if (within_enter_range(walker_footprint_, walker_->state().feet, config_.walker.enter_range_m,
                                   config_.walker.enter_max_dz_m)) {
                remove_walker();
                walker_entered_.fetch_add(1, std::memory_order_release);
            } else {
                walker_enter_refused_.fetch_add(1, std::memory_order_release);
            }
            break;
        case 3:
            if (active) {
                remove_walker();
                walker_despawned_.fetch_add(1, std::memory_order_release);
            }
            break;
        default: break;
    }
}

void Session::update_walker() {
    if (!walker_ || !walker_->active()) return;
    WalkerInput input;
    {
        std::lock_guard<std::mutex> lock(walker_input_mutex_);
        input = walker_input_;
    }
    input.jump = walker_jump_.exchange(0, std::memory_order_relaxed) > 0;
    // The car may still be rolling to a stop (or be relocated): refresh its footprint.
    walker_footprint_ = own_car_footprint();
    walker_->set_obstacles(std::span<const OrientedRect>(&walker_footprint_, 1));
    const WalkerState& st = walker_->step(input, 1.0 / config_.tick_rate_hz);
    walker_enter_distance_m_ = distance_to_rect(walker_footprint_, st.feet.x, st.feet.y);
    walker_can_enter_ = within_enter_range(walker_footprint_, st.feet, config_.walker.enter_range_m,
                                           config_.walker.enter_max_dz_m);
}

}

namespace rg {
std::uint64_t Session::add_test_traffic_actor(const ps::Pose& pose,double speed_mps,double route_length_m,bool stop_at_end){
 if(loop_.running())throw std::logic_error("Session::add_test_traffic_actor: not while running");
 const ps::Vec3 forward=pose.orientation.rotate(ps::Vec3::unit_x());
 const double yaw=ps::math::atan2(forward.y,forward.x);
 TrafficTrip trip;
 const auto add_point=[&](double station,double speed){TruckRoutePoint point;point.ground=pose.position+forward*station;point.yaw=yaw;point.speed_m_s=speed;point.station=station;trip.route.points.push_back(point);};
 add_point(0,speed_mps);
 if(stop_at_end){add_point(route_length_m-5.,speed_mps);add_point(route_length_m-.7,1.);add_point(route_length_m,0.);}else add_point(route_length_m,speed_mps);
 ps::BodyDesc body;body.motion=ps::BodyMotionType::Kinematic;body.gravity_enabled=false;body.shape=ps::BoxShape{{2.3,1.,.65}};
 body.pose=pose;
 const auto id=world_->create_body(body);
 traffic_actors_.push_back({traffic_next_id_++,id,std::move(trip),0,speed_mps,0});
 return traffic_actors_.back().id;
}
void Session::configure_traffic(TrafficConfig c){std::lock_guard<std::mutex> lock(traffic_mutex_);traffic_requested_=sanitize_traffic_config(c);traffic_config_changed_=true;}
void Session::set_visible_traffic(std::vector<std::uint64_t> ids){std::sort(ids.begin(),ids.end());std::lock_guard<std::mutex> lock(traffic_mutex_);traffic_visible_=std::move(ids);}
void Session::update_traffic(bool clear){
 if(clear){traffic_cancel_.store(true);for(auto& a:traffic_actors_)world_->destroy_body(a.body);traffic_actors_.clear();traffic_ready_.trips.clear();traffic_neighbor_grid_.clear();traffic_stuck_.set_active(0);traffic_loading_=false;traffic_scan_needed_=true;traffic_scan_time_=world_->sim_time()+2;return;}
 const auto car=world_->get_pose(chassis_body_);const double now=world_->sim_time(),dt=1/config_.tick_rate_hz;
 const ps::Vec3 anchor=traffic_anchor(); // the walker while on foot (R9c), else the car
 std::vector<std::uint64_t> visible;bool changed=false;
 {std::lock_guard<std::mutex> lock(traffic_mutex_);visible=traffic_visible_;changed=traffic_config_changed_;if(changed){traffic_config_=traffic_requested_;traffic_config_changed_=false;}}
 if(changed){
  traffic_cancel_.store(true);traffic_ready_.trips.clear();traffic_loading_=false;traffic_population_target_=std::min(traffic_population_target_,traffic_config_.max_vehicles);traffic_scan_needed_=true;traffic_scan_time_=now;

 }
 if(traffic_worker_.joinable()&&traffic_done_.load()){
  traffic_worker_.join();TrafficPlan plan;{std::lock_guard<std::mutex> lock(traffic_mutex_);plan=std::move(traffic_pending_);}
  traffic_loading_=false;
  if(!traffic_cancel_.load()){
   traffic_message_=plan.message;
   // A busy/unloaded geometry cache is not an instruction to erase the population.
   if(plan.road_length_m>0)traffic_population_target_=std::min(traffic_config_.max_vehicles,static_cast<int>(std::ceil(plan.road_length_m*traffic_config_.density_per_km/1000)));
   if(plan.road_length_m==0)traffic_scan_needed_=true;
   traffic_ready_=std::move(plan);
  }
 }
 for(int attempts=0,born=0;attempts<8&&born<4&&!traffic_ready_.trips.empty()&&static_cast<int>(traffic_actors_.size())<traffic_population_target_;++attempts){
    auto trip=std::move(traffic_ready_.trips.back());traffic_ready_.trips.pop_back();
    auto ground=trip.route.points.front().ground;const double distance=std::hypot(ground.x-anchor.x,ground.y-anchor.y);if(distance<traffic_config_.min_spawn_m||distance>traffic_config_.radius_m)continue;
    const double half=trip.truck?6.5:2.3,height=trip.truck?2.2:.8;const auto yaw=trip.route.points.front().yaw;
    bool free=true; // Native footprint sweeps below use the backend spatial broadphase.
    if(truck_state_.active&&(world_->get_pose(truck_body_).position-ground).length()<half+15)free=false;
    // Several footprint-height sweeps reject current physical obstructions, not just cached routes.
    const ps::Vec3 forward{std::cos(yaw),std::sin(yaw),0},left{-forward.y,forward.x,0};
    for(double lateral:{-1.,0.,1.})for(double z:{.4,1.,trip.truck?3.:1.})if(world_->backend().ray_cast(ground-forward*(half+2)+left*lateral+ps::Vec3{0,0,z},forward,2*half+4).hit)free=false;
    if(!free)continue;
    ps::BodyDesc body;body.motion=ps::BodyMotionType::Kinematic;body.gravity_enabled=false;body.shape=ps::BoxShape{{half,trip.truck?1.25:1.,trip.truck?1.8:.65}};
    body.pose.position=ground+ps::Vec3{0,0,height};body.pose.orientation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),yaw)*ps::Quat::from_axis_angle(ps::Vec3::unit_y(),-std::atan(trip.route.points.front().grade));
    auto id=world_->create_body(body);const double speed=trip.route.points.front().speed_m_s;
    traffic_actors_.push_back({traffic_next_id_++,id,std::move(trip),0,speed,0});++born;
   }
 if(traffic_config_.density_per_km==0){traffic_population_target_=0;traffic_ready_.trips.clear();}
 if((traffic_scan_needed_||static_cast<int>(traffic_actors_.size())<traffic_population_target_)&&traffic_ready_.trips.empty()&&!traffic_worker_.joinable()&&now>=traffic_scan_time_&&traffic_config_.density_per_km>0){
  traffic_cancel_.store(false);traffic_done_.store(false);traffic_loading_=true;traffic_scan_needed_=false;traffic_scan_time_=now+8;traffic_scan_origin_=anchor;
  auto terrain=config_.terrain?config_.terrain->world_terrain:nullptr;auto config=traffic_config_;config.grip_multiplier=1;const auto seed=traffic_seed_++;
  traffic_worker_=std::thread([this,terrain,player=anchor,config,seed]{
   const auto started=std::chrono::steady_clock::now();
   std::fprintf(stderr,"RG_TRAFFIC_SCAN begin seed=%llu cached_only=yes\n",static_cast<unsigned long long>(seed));
   TrafficPlan plan;try{plan=plan_traffic(terrain,player,config,seed,traffic_cancel_,true);}catch(const std::exception& e){plan.message=e.what();}
   std::fprintf(stderr,"RG_TRAFFIC_SCAN end seed=%llu ms=%.1f trips=%zu destinations=%zu cancelled=%s\n",static_cast<unsigned long long>(seed),std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count(),plan.trips.size(),plan.destinations,traffic_cancel_.load()?"yes":"no");
   {std::lock_guard<std::mutex> lock(traffic_mutex_);traffic_pending_=std::move(plan);}traffic_done_.store(true);
  });
 }
 // Immutable neighbor samples refreshed at 20 Hz; queries never visit the entire population.
 const auto traffic_tick=static_cast<std::uint64_t>(std::llround(now*config_.tick_rate_hz));
 const auto decision_period=std::max<std::uint64_t>(1,static_cast<std::uint64_t>(config_.tick_rate_hz/20));
 if(traffic_tick%decision_period==0||traffic_neighbor_grid_.empty()){
  traffic_neighbor_grid_.clear();traffic_body_keys_.clear();
  for(const auto& actor:traffic_actors_){const auto pose=world_->get_pose(actor.body);const auto p=pose.position;const auto f=pose.orientation.rotate(ps::Vec3::unit_x());
   traffic_body_keys_.insert(body_key(actor.body));
   traffic_neighbor_grid_[{static_cast<int>(std::floor(p.x/32)),static_cast<int>(std::floor(p.y/32))}].push_back({actor.id,p,actor.speed,actor.trip.truck?6.5:2.3,f.x,f.y});}
 }
 if(!std::is_sorted(visible.begin(),visible.end()))std::sort(visible.begin(),visible.end());
 std::vector<ps::aero::WakeSource> wakes;
 if(truck_state_.active){auto pose=world_->get_pose(truck_body_);wakes.push_back({truck_body_,pose.position,world_->get_motion(truck_body_).linear,{60,2,.15,.45,.2}});}
 std::vector<std::uint64_t> stuck_declared;std::uint64_t stuck_active=0;
 for(auto it=traffic_actors_.begin();it!=traffic_actors_.end();){
  auto& a=*it;auto pose=world_->get_pose(a.body);a.unseen=std::binary_search(visible.begin(),visible.end(),a.id)?0:a.unseen+dt;
  const double end=a.trip.route.points.back().station;
  const bool surplus=static_cast<int>(traffic_actors_.size())>traffic_population_target_;
  const bool followed=followed_id_.load(std::memory_order_relaxed)==a.id; // R9b: kept alive (and resident) while the drone camera trails it
  // The route's last point has speed 0 and the point before it is up to one spacing earlier: once the actor passes that
  // point the cap is 0, so it stops there. A fixed 0.5 m arrival margin left every actor that stopped 0.5-2 m short of
  // the end (short trips) parked on the road for ever, blocking the lane (measured: the dominant route_cap queue head).
  const auto& route_points=a.trip.route.points;
  const double arrive_margin=std::clamp(route_points.size()>1?end-route_points[route_points.size()-2].station+.1:.5,.5,4.);
  if((a.station>=end-arrive_margin&&a.speed<.5)||(!followed&&(surplus||(pose.position-anchor).length()>traffic_config_.radius_m+50)&&a.unseen>3)){
   world_->destroy_body(a.body);const auto index=static_cast<std::size_t>(it-traffic_actors_.begin());
   if(index+1<traffic_actors_.size())*it=std::move(traffic_actors_.back());
   traffic_actors_.pop_back();it=index<traffic_actors_.size()?traffic_actors_.begin()+index:traffic_actors_.end();continue;
  }
  auto upper=std::upper_bound(a.trip.route.points.begin(),a.trip.route.points.end(),a.station,[](double s,const auto& p){return s<p.station;});
  const auto& current=*(upper==a.trip.route.points.end()?upper-1:upper);
  double target=std::min(current.speed_m_s*std::sqrt(traffic_config_.grip_multiplier),std::sqrt(std::max(0.,5*traffic_config_.grip_multiplier*(end-a.station))));
  const auto forward=pose.orientation.rotate(ps::Vec3::unit_x());
  if((traffic_tick+a.id)%decision_period==0){
  double follow=1e30;
  std::uint64_t follow_id=0;
  // other_cos = cos of the heading difference to the other actor (2 = not an NPC: the player, the truck: always obeyed).
  // Between NPCs: ONCOMING traffic (cos < -0.5) is never followed - it has its own lane half and kinematic bodies do
  // not collide (obeying it made every head-on pair on a narrow road a permanent mutual stop); CROSSING traffic
  // (|cos| <= 0.5) is yielded to only when the other actor has the lower id (a total order, so a pure crossing
  // conflict can never be a mutual yield); same-direction traffic is followed as before.
  const auto avoid=[&](ps::Vec3 other,double speed,double length,std::uint64_t other_id,double other_cos){const auto relative=other-pose.position;const double gap=ps::dot(relative,forward),side=std::abs(relative.x*forward.y-relative.y*forward.x);
   if(other_cos<-0.5)return;
   if(other_cos<=0.5&&other_id>a.id)return;
   if(gap>0&&side<2.7&&std::abs(relative.z)<3){const double clearance=gap-(a.trip.truck?6.5:2.3)-length;
    // Patience: a wait cycle can mix same-lane following with crossing yields (A behind B, B yields to C, C yields to A
    // by id), which no static priority order breaks. After kCrossingPatienceS below the stuck speed the waiter stops
    // yielding to crossing NPCs (counted in the stats; kinematic bodies simply pass).
    if(other_cos<=0.5&&other_cos<2&&a.stuck.below_s>kCrossingPatienceS){if(clearance<6)traffic_stuck_.note_crossing_override();return;}
    // Two NPCs already overlapping longitudinally and offset sideways (merging lanes, a junction mouth) each see the
    // other "ahead" by a few centimetres: a mutual yield that never resolves (measured: the persistent 2-cycles with
    // 100+ queued behind them). Only the higher id yields there.
    if(clearance<=0&&side>1.2&&other_cos<2&&other_id>a.id)return;
    const double cap=std::max(0.,std::min(speed+(clearance-5)*.5,(clearance-3)/1.5));if(cap<follow){follow=cap;follow_id=other_id;}}
  };
  avoid(car.position,world_->get_motion(chassis_body_).linear.length(),2.5,kFollowPlayerId,2.0);
  const int cx=static_cast<int>(std::floor(pose.position.x/32)),cy=static_cast<int>(std::floor(pose.position.y/32));
  for(int y=cy-3;y<=cy+3;++y)for(int x=cx-3;x<=cx+3;++x){auto cell=traffic_neighbor_grid_.find({x,y});if(cell==traffic_neighbor_grid_.end())continue;
   for(const auto& other:cell->second)if(other.id!=a.id)avoid(other.position,other.speed,other.half_length,other.id,other.fx*forward.x+other.fy*forward.y);}

  if(truck_state_.active)avoid(world_->get_pose(truck_body_).position,truck_speed_,6.5,kNpcTruckVehicleId,2.0);
  a.follow_cap=follow;a.follow_id=follow_id;
  // Static obstacle probe excludes the moving NPC's own collider.
  const double nose=a.trip.truck?6.5:2.3,look=std::max(12.,a.speed*a.speed/8+8);
  auto obstruction=world_->backend().ray_cast_excluding(pose.position+forward*(nose+.2),forward,look,a.body);
  // Other NPCs are the follow rule's business (with its heading/priority filter): a probe hit on one would stop an
  // oncoming or lower-priority actor that the follow rule deliberately ignores.
  if(obstruction.hit&&traffic_body_keys_.contains(body_key(obstruction.body)))obstruction=ps::RayCastHit{};
  a.obstacle_cap=obstruction.hit?std::sqrt(std::max(0.,8*(obstruction.fraction*look-3))):1e30;
  a.obstacle_hit=obstruction.hit;a.obstacle_body=obstruction.body;a.obstacle_normal_z=obstruction.normal.z;
  }
  a.route_cap=target;
  target=std::min({target,a.obstacle_cap,a.follow_cap});
  a.speed+=std::clamp(target-a.speed,-4*dt,(a.trip.truck?1.2:2.5)*dt);
  a.station=std::min(end,a.station+a.speed*dt);
  {const auto ev=advance_stuck_track(a.stuck,a.speed,end-a.station<kStuckArrivingM,dt);
   if(ev==StuckTrackEvent::Declared){a.stuck_since=now-kStuckTimeS;stuck_declared.push_back(a.id);}
   else if(ev==StuckTrackEvent::Recovered)traffic_stuck_.note_recovered(now-a.stuck_since);
   if(a.stuck.declared)++stuck_active;}
  auto next=std::lower_bound(a.trip.route.points.begin(),a.trip.route.points.end(),a.station,[](const auto& p,double s){return p.station<s;});if(next==a.trip.route.points.end())--next;
  auto previous=next==a.trip.route.points.begin()?next:next-1;const double span=next->station-previous->station,t=span>1e-9?(a.station-previous->station)/span:0;
  auto ground=previous->ground+(next->ground-previous->ground)*t;const double yaw=previous->yaw+std::remainder(next->yaw-previous->yaw,6.283185307179586)*t;
  const double grade=previous->grade+(next->grade-previous->grade)*t,height=a.trip.truck?2.2:.8;
  ps::Motion motion;motion.linear=(ground+ps::Vec3{0,0,height}-pose.position)/dt;
  auto rotation=ps::Quat::from_axis_angle(ps::Vec3::unit_z(),yaw)*ps::Quat::from_axis_angle(ps::Vec3::unit_y(),-std::atan(grade));
  auto delta=(rotation*pose.orientation.inverse()).normalized();ps::Vec3 axis{delta.x,delta.y,delta.z};if(delta.w<0){axis=-axis;delta.w=-delta.w;}double sine=axis.length();if(sine>1e-12)motion.angular=axis*(2*std::atan2(sine,delta.w)/(sine*dt));
  world_->backend().set_motion(a.body,motion);if((pose.position-anchor).length()<200)wakes.push_back({a.body,pose.position,motion.linear,a.trip.truck?ps::aero::WakeDesc{60,2,.15,.45,.2}:ps::aero::WakeDesc{25,1.2,.12,.3,.12}});
  ++it;
 }
 world_->set_aero_wakes(std::move(wakes));
 traffic_stuck_.set_active(stuck_active);
 note_stuck_events(stuck_declared,now);
}
// Classifies each newly declared stuck actor by what the controller is obeying (traffic_stuck.h) and logs it.
void Session::note_stuck_events(const std::vector<std::uint64_t>& declared,double now){
 if(!declared.empty()){
  std::unordered_map<std::uint64_t,std::size_t> index;index.reserve(traffic_actors_.size());
  for(std::size_t i=0;i<traffic_actors_.size();++i)index.emplace(traffic_actors_[i].id,i);
  const auto cause_of=[&](const TrafficActor& a,std::uint64_t& other){
   other=0;
   if(a.follow_cap<=a.obstacle_cap&&a.follow_cap<a.route_cap){other=a.follow_id;return a.follow_id==kFollowPlayerId?StuckCause::FollowPlayer:a.follow_id==kNpcTruckVehicleId?StuckCause::FollowTruck:StuckCause::FollowNpc;}
   if(a.obstacle_cap<a.route_cap&&a.obstacle_hit){
    if(a.obstacle_body==chassis_body_)return StuckCause::ProbePlayer;
    if(truck_state_.active&&a.obstacle_body==truck_body_)return StuckCause::ProbeTruck;
    for(const auto& o:traffic_actors_)if(o.body==a.obstacle_body){other=o.id;return StuckCause::ProbeNpc;}
    if(deck_installer_)for(const auto& d:deck_installer_->bodies())if(d==a.obstacle_body)return StuckCause::ProbeDeck;
    if(world_->backend().is_static(a.obstacle_body))return config_.terrain?StuckCause::ProbeTerrain:StuckCause::ProbeStatic;
    return StuckCause::ProbeOther;
   }
   if(a.route_cap<kStuckSpeedMps)return StuckCause::RouteCap;
   return StuckCause::Other;
  };
  const StuckLookup lookup=[&](std::uint64_t id)->std::optional<StuckNode>{
   const auto it=index.find(id);if(it==index.end())return std::nullopt;
   const auto& o=traffic_actors_[it->second];StuckNode n;n.speed_mps=o.speed;n.cause=cause_of(o,n.other_id);
   if(stuck_cause_waits_for_npc(n.cause)){
    const auto oi=index.find(n.other_id);
    if(oi!=index.end()){
     const auto fa=world_->get_pose(o.body).orientation.rotate(ps::Vec3::unit_x()),fo=world_->get_pose(traffic_actors_[oi->second].body).orientation.rotate(ps::Vec3::unit_x());
     n.heading_cos=fa.x*fo.x+fa.y*fo.y;n.has_heading=true;
    }
   }
   return n;
  };
  for(const auto id:declared){
   const auto it=index.find(id);if(it==index.end())continue; // despawned within the same tick
   const auto& a=traffic_actors_[it->second];
   StuckEvent e;e.id=id;e.time_s=now;e.cause=cause_of(a,e.other_id);e.chain=walk_stuck_chain(id,lookup);
   const auto p=world_->get_pose(a.body).position;e.x=p.x;e.y=p.y;e.station_m=a.station;e.route_length_m=a.trip.route.points.back().station;
   {auto upper=std::upper_bound(a.trip.route.points.begin(),a.trip.route.points.end(),a.station,[](double s,const auto& q){return s<q.station;});
    e.way_id=(upper==a.trip.route.points.end()?upper-1:upper)->way_id;}
   if(stuck_cause_waits_for_npc(e.cause)){
    const auto oi=index.find(e.other_id);
    if(oi!=index.end()){
     const auto pa=world_->get_pose(a.body),po=world_->get_pose(traffic_actors_[oi->second].body);
     const auto fa=pa.orientation.rotate(ps::Vec3::unit_x()),fo=po.orientation.rotate(ps::Vec3::unit_x());
     const auto rel=po.position-pa.position;
     e.has_other=true;e.other_heading_cos=fa.x*fo.x+fa.y*fo.y;e.other_gap_m=ps::dot(rel,fa);e.other_lateral_m=rel.x*fa.y-rel.y*fa.x;
    }
   }
   e.route_cap=a.route_cap;e.obstacle_cap=a.obstacle_cap;e.follow_cap=a.follow_cap;e.truck=a.trip.truck;
   traffic_stuck_.record(e);
   if(traffic_stuck_.summary().events<=200)std::fprintf(stderr,"%s\n",TrafficStuckStats::format_event(e).c_str());
  }
 }
 if(now>=traffic_stuck_summary_time_+30){
  traffic_stuck_summary_time_=now;
  if(traffic_stuck_.summary().events>0)std::fprintf(stderr,"%s\n",traffic_stuck_.format_summary(now).c_str());
 }
}
}
