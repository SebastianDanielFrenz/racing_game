// rg/garage_set.h - the garage/showroom set and its camera (PLAN.md R6).
// Engine-neutral. data/garage/garage_set.json ("rg.garage_set/1") describes the
// set (room, turntable, lights, softboxes, camera areas) as data; rg::GarageCamera
// is the state machine that moves a camera between the areas (overview, wheels &
// brakes, engine bay, rear) and turns the turntable. The Godot layer builds the
// 3D set from the description (garage_set.gd), asks the camera for its pose every
// frame and draws; it decides nothing about where the camera goes.
//
// Frame: the car frame is ISO 8855 (x forward, y left, z up) with its origin on
// the floor of the turntable, below the wheel centres. The room frame is the same
// axes with the turntable centre as origin; the car sits at the origin turned by
// the turntable angle. Everything here is in metres/degrees; the Godot layer
// converts to its own axes in one place.
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace rg {

struct GVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct GarageRoom {
    double radius_m = 14.0;
    double height_m = 7.0;
    std::string floor_colour = "#15171b";
    std::string wall_colour = "#1b1f26";
    std::string accent_colour = "#e8742a";
};

struct GarageTurntable {
    double radius_m = 3.6;
    double height_m = 0.12;
    double spin_deg_s = 12.0; // idle spin in a "spin" camera area
    std::string colour = "#2a2d33";
    std::string rim_colour = "#e8742a";
};

struct GarageEnvironment {
    double exposure = 1.0;
    double ambient_energy = 0.35;
    std::string ambient_colour = "#3a4250";
    double reflection_energy = 1.0;
    double floor_reflectivity = 0.35; // 0..1: how much of the car shows mirrored in the floor
};

struct GarageLight {
    std::string id;
    std::string kind = "spot"; // "spot" | "omni"
    GVec3 position;
    GVec3 target;
    std::string colour = "#ffffff";
    double energy = 1.0;
    double range_m = 20.0;
    double angle_deg = 40.0; // spot cone half angle
    bool shadow = false;
};

// A light panel: drawn as an emissive quad in the room and painted into the
// environment panorama the car's paint reflects.
struct GarageSoftbox {
    std::string id;
    GVec3 position;
    GVec3 target; // the point it faces
    double width_m = 4.0;
    double height_m = 1.0;
    std::string colour = "#ffffff";
    double energy = 4.0;
};

struct GarageAreaDef {
    std::string id;     // "overview", "wheels", "engine", "rear"
    std::string label;
    std::string anchor = "centre"; // centre | front_wheel | rear_wheel | engine_bay | rear
    GVec3 anchor_offset;           // metres, car frame, added to the anchor
    double azimuth_deg = 35.0;     // camera direction around the anchor, 0 = from the front, +90 = from the left
    double elevation_deg = 10.0;
    double distance_m = 0.0;       // 0: distance_scale * car length
    double distance_scale = 1.0;
    double fov_deg = 40.0;
    bool spin = false;             // the turntable turns freely while this area is shown
    double car_yaw_deg = 0.0;      // otherwise the turntable settles here
};

struct GarageSetDesc {
    GarageRoom room;
    GarageTurntable turntable;
    GarageEnvironment environment;
    std::vector<GarageLight> lights;
    std::vector<GarageSoftbox> softboxes;
    double transition_s = 0.9;
    std::vector<GarageAreaDef> areas;
    [[nodiscard]] const GarageAreaDef* find_area(const std::string& id) const;
};

std::optional<GarageSetDesc> load_garage_set(const std::string& path, std::string* err);
std::optional<GarageSetDesc> parse_garage_set(const std::string& json_text, const std::string& origin, std::string* err);

// What the camera frames: the car in its own frame.
struct GarageSubject {
    GVec3 bounds_min{-2.3, -0.95, 0.0};
    GVec3 bounds_max{2.3, 0.95, 1.3};
    double front_axle_x = 1.3;
    double rear_axle_x = -1.4;
    double half_track = 0.78;
    double wheel_radius = 0.34;
    std::string engine_bay = "front"; // "front" | "mid" | "rear"
};

struct GarageCameraPose {
    GVec3 position; // room frame
    GVec3 look_at;
    double fov_deg = 40.0;
};

// The anchor point of `area` on `subject`, in the car frame (offset included).
GVec3 garage_anchor(const GarageAreaDef& area, const GarageSubject& subject);
// The pose a camera settles in for `area`, room frame, car turned by `car_yaw_deg`.
GarageCameraPose garage_area_pose(const GarageAreaDef& area, const GarageSubject& subject, double car_yaw_deg);

class GarageCamera {
public:
    GarageCamera(const GarageSetDesc& set, const GarageSubject& subject);

    // The camera area to move to. `instant` snaps (first frame, tests).
    // false for an unknown area id, state unchanged.
    bool go_to(const std::string& area_id, bool instant = false);
    // Same car, new subject (vehicle select cycling): keeps the area, recomputes the pose.
    void set_subject(const GarageSubject& subject);
    void update(double dt_s);

    [[nodiscard]] const std::string& area() const { return area_; }
    [[nodiscard]] GarageCameraPose pose() const { return pose_; }
    [[nodiscard]] double car_yaw_deg() const { return yaw_deg_; }
    [[nodiscard]] bool in_transition() const { return t_ < 1.0; }
    // 0..1 progress of the current camera move (1 = settled).
    [[nodiscard]] double progress() const { return t_; }
    // true once the car stands at the area's yaw (or the area spins) and the camera has settled.
    [[nodiscard]] bool settled() const;

private:
    [[nodiscard]] GarageCameraPose target_pose() const;

    GarageSetDesc set_;
    GarageSubject subject_;
    std::string area_;
    GarageCameraPose from_;
    GarageCameraPose pose_;
    double t_ = 1.0;
    double yaw_deg_ = 0.0;
    double yaw_target_deg_ = 0.0;
};

} // namespace rg
