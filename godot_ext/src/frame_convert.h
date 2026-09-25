// godot_ext/src/frame_convert.h — godot-cpp-facing wrapper around
// external/physics_sim/adapters/godot/src/frame_convert_core.h (the
// Godot-free ISO<->Godot basis-change math, built into rg_core - see
// core/CMakeLists.txt's comment). Deliberately NOT reused by path from
// physics_sim's own frame_convert.h: that file includes godot-cpp headers
// directly, so it is not engine-neutral, and this whole directory (unlike
// rg_core) is EXPECTED to be Godot-specific - re-authoring this ~35-line
// wrapper here (rather than by-path #include of physics_sim's own copy)
// keeps rg_core's "no Godot type" boundary a physical directory boundary
// too, not just a convention. The math itself is not duplicated - both
// this file and physics_sim's own frame_convert.h call the SAME
// frame_convert_core.h functions.

#pragma once

#include "frame_convert_core.h"

#include "ps/math/pose.h"

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace rg_godot {

inline godot::Vector3 to_godot(const ps_godot::Vec3f& v) { return godot::Vector3(v.x, v.y, v.z); }

inline godot::Vector3 iso_to_godot(const ps::Vec3& sim_pos_absolute, const ps::Vec3& origin_m) {
    return to_godot(ps_godot::iso_to_godot_position(sim_pos_absolute, origin_m));
}

inline ps::Vec3 godot_to_iso(const godot::Vector3& godot_pos, const ps::Vec3& origin_m) {
    return ps_godot::godot_to_iso_position(ps_godot::Vec3f{godot_pos.x, godot_pos.y, godot_pos.z}, origin_m);
}

inline godot::Basis to_godot_basis(const ps::Quat& sim_rot) {
    const ps_godot::Basis3f b = ps_godot::iso_to_godot_basis(sim_rot);
    return godot::Basis(to_godot(b.col_x), to_godot(b.col_y), to_godot(b.col_z));
}

inline godot::Transform3D iso_to_godot_transform(const ps::Pose& sim_pose, const ps::Vec3& origin_m) {
    return godot::Transform3D(to_godot_basis(sim_pose.orientation), iso_to_godot(sim_pose.position, origin_m));
}

} // namespace rg_godot
