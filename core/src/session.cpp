#include "rg/session.h"

#include "ps/backend/shape_desc.h"
#include "ps/io/vehicle_io.h"

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

std::unique_ptr<ps::World> Session::make_world(const SessionConfig& config) {
    ps::WorldConfig world_config;
    world_config.tick_rate_hz = config.tick_rate_hz;
    world_config.substep_rate_hz = config.substep_rate_hz;
    world_config.gravity = config.gravity;
    world_config.job_workers = static_cast<int>(config.job_workers);
    return std::make_unique<ps::World>(world_config);
}

Session::Session(const SessionConfig& config)
    : config_(config),
      world_(make_world(config)),
      sim_thread_(*world_) {
    build_world_contents(config);

    control_channels_.reserve(kControlChannelCount);
    for (std::size_t i = 0; i < kControlChannelCount; ++i) {
        control_channels_[kControlChannelNames[i]].store(0.0, std::memory_order_relaxed);
    }

    // Copies this session's own control_channels_ into ps::World right
    // before each tick (session.h's set_control doc comment explains why
    // this hand-off has to happen on the sim thread rather than the caller
    // writing World::set_control directly).
    sim_thread_.set_pre_step([this] {
        for (auto& [name, value] : control_channels_) {
            world_->set_control(name, value.load(std::memory_order_relaxed));
        }
    });
    sim_thread_.set_post_step([this] {
        snapshot_buffer_.write_slot() = capture_frame_snapshot();
        snapshot_buffer_.publish();
    });
}

Session::~Session() { stop(); }

void Session::build_world_contents(const SessionConfig& config) {
    surface_table_ = std::make_shared<ps::io::SurfaceTable>(config.surface_table_path);
    world_->set_surface_table(surface_table_);

    // Ground: one large flat static box, top face at world z=0 - mirrors
    // external/physics_sim/data/scenarios/vehicle_step_steer.json's own
    // "ground" body exactly (see session.h's SessionConfig comment).
    {
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
        desc.pose.position = ps::Vec3{0.0, 0.0, config.chassis_z_m};
        desc.velocity.linear = config.chassis_initial_velocity;
        desc.friction = config.chassis_friction;
        desc.allow_sleep = false;
        chassis_body_ = world_->create_body(desc);
    }

    vehicle_desc_ = ps::io::load_vehicle_json(config.vehicle_json_path);
    vehicle_id_ = world_->create_vehicle(vehicle_desc_, chassis_body_);
}

void Session::step() { world_->step(); }

void Session::start() { sim_thread_.start(); }

void Session::stop() { sim_thread_.stop(); }

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
