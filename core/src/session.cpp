#include "rg/session.h"

#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/math/transcendental.h"

// ps_godot::FallDetector, reused BY PATH from physics_sim's adapter (see
// session.h's top comment) - Godot-free.
#include "fall_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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
          loader(cfg.fetch, loader_config(cfg)),
          streamer(grid, *resident, loader, g2m::phys::StreamerConfig{}),
          source(std::make_shared<g2m::ps_bridge::G2mTerrainSource>(grid, resident, surface)) {
        interest_points.reserve(4);
    }

    static g2m::phys::LoaderConfig loader_config(const TerrainModeConfig& cfg) {
        g2m::phys::LoaderConfig lc;
        lc.workers = cfg.physics.loader_workers;
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

double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// A FallDetector height sample that touches a hole (NoData or Absent: the
// source writes ps::kHeightfieldNoCollision = FLT_MAX there) is not a ground
// height - a "fall" measured against it is the detector's artefact, not a
// fall through the ground.
constexpr double kHoleHeightThreshold = 1.0e6;

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

    const ps::SurfaceId surface = surface_table_->id_for(phys.terrain_surface);
    if (surface == ps::kInvalidSurfaceId) {
        throw std::invalid_argument("Session: physics.terrain_surface '" + phys.terrain_surface +
                                    "' is not in the surface table");
    }

    terrain_ = std::make_unique<Terrain>(tm, surface);
    world_->set_terrain_source(terrain_->source,
                               g2m::ps_bridge::make_terrain_config(phys.radius_m, phys.max_tile_fills_per_tick));

    // 1. Start-up: block until the gate around the spawn point is ready.
    // Failed keys are retried here (a transient 5xx at start-up must not be
    // fatal); the timeout is the one hard error.
    const Clock::time_point t0 = Clock::now();
    while (!gate_check()) {
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
    for (int k = 0; k < prime_ticks; ++k) step_blocking("terrain priming");
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

    // 3. Spawn: five rays straight down (centre + the four yaw-rotated
    // chassis-footprint corners), from z = +3000 over 6000 m; the chassis
    // centre goes chassis_z_m + spawn_clearance_m above the highest hit.
    double s_yaw = 0.0, c_yaw = 1.0;
    ps::math::sincos(tm.spawn_yaw_rad, s_yaw, c_yaw);
    const double hx = config_.chassis_half_extents.x;
    const double hy = config_.chassis_half_extents.y;
    const double corners[5][2] = {{0.0, 0.0}, {hx, hy}, {hx, -hy}, {-hx, hy}, {-hx, -hy}};
    double z_max = -1.0e300;
    for (const auto& lc : corners) {
        const double wx = tm.spawn_x + c_yaw * lc[0] - s_yaw * lc[1];
        const double wy = tm.spawn_y + s_yaw * lc[0] + c_yaw * lc[1];
        const ps::RayCastHit hit =
            world_->backend().ray_cast(ps::Vec3{wx, wy, 3000.0}, ps::Vec3{0.0, 0.0, -1.0}, 6000.0);
        if (!hit.hit) {
            char msg[160];
            std::snprintf(msg, sizeof msg, "Session: spawn over NoData (no ground under (%.2f, %.2f))", wx, wy);
            throw std::runtime_error(msg);
        }
        z_max = std::max(z_max, static_cast<double>(hit.point.z));
    }

    double s_half = 0.0, c_half = 1.0;
    ps::math::sincos(0.5 * tm.spawn_yaw_rad, s_half, c_half);
    ps::Pose pose;
    pose.position = ps::Vec3{tm.spawn_x, tm.spawn_y, z_max + config_.chassis_z_m + phys.spawn_clearance_m};
    pose.orientation = ps::Quat{0.0, 0.0, s_half, c_half};
    return pose;
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
    if (have_vehicle_) {
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
    const g2m::phys::GateStatus gs = t.streamer.update(points);
    // TileManager keeps one interest point per id; the ids are the streamer's
    // own (removing a point that disappears from the list is not needed while
    // the list is fixed at the chassis).
    for (const g2m::phys::InterestPoint& ip : points) {
        world_->set_terrain_interest_point(ip.id, ps::Vec3{ip.x, ip.y, 0.0}, ip.radius_m);
    }

    status_.ready.store(gs.ready, std::memory_order_relaxed);
    status_.missing_required.store(gs.missing_required, std::memory_order_relaxed);
    status_.inflight.store(gs.inflight, std::memory_order_relaxed);
    status_.resident_l0.store(gs.resident, std::memory_order_relaxed);
    status_.failed.store(gs.failed, std::memory_order_relaxed);
    return gs.ready;
}

bool Session::step_once(bool from_loop) {
    if (terrain_) {
        if (!gate_check()) {
            status_.frozen.store(true, std::memory_order_relaxed);
            if (have_vehicle_) {
                status_.frozen_attempts.fetch_add(1, std::memory_order_relaxed);
                if (!in_freeze_) {
                    in_freeze_ = true;
                    status_.freeze_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
            return false; // frozen: the World is untouched
        }
        status_.frozen.store(false, std::memory_order_relaxed);
        in_freeze_ = false;
    }

    if (have_vehicle_ && drive_script_) {
        drive_script_->apply(DriveTickContext{drive_tick(), chassis_body_, vehicle_id_}, *world_);
    } else if (from_loop) {
        // Copies this session's own control_channels_ into ps::World right
        // before the tick (session.h's set_control doc comment explains why
        // this hand-off has to happen on the stepping thread).
        for (auto& [name, value] : control_channels_) {
            world_->set_control(name, value.load(std::memory_order_relaxed));
        }
    }

    world_->step();
    post_step(from_loop);
    return true;
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
    return s;
}

const g2m::ps_bridge::G2mTerrainSource* Session::terrain_source() const {
    return terrain_ ? terrain_->source.get() : nullptr;
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

} // namespace rg
