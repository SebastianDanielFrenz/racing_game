#include "rg_cockpit.h"
#include "rg_driver_actions.h"
#include <godot_cpp/core/class_db.hpp>

namespace rg_godot {
namespace {
godot::String text(std::string_view value) { return godot::String::utf8(std::string(value).c_str()); }
ps::Vec3 vec(const godot::Vector3& v) { return ps::Vec3{v.x, v.y, v.z}; }
godot::Vector3 gvec(const ps::Vec3& v) { return godot::Vector3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)); }
// Read-only channel view: the cockpit never writes channels, DriverActions does that after the events are submitted.
class ReadSink final : public rg::ChannelSink {
public:
    explicit ReadSink(const godot::Dictionary& values) : values_(values) {}
    double get(std::string_view channel) const override { return static_cast<double>(values_.get(text(channel), 0.0)); }
    void set(std::string_view, double) override {}
private:
    godot::Dictionary values_;
};
godot::Dictionary event_dict(const rg::CockpitEvent& e) {
    godot::Dictionary d;
    d["control"] = e.control;
    d["control_id"] = text(e.control_id);
    d["action_id"] = text(e.action.id);
    d["kind"] = static_cast<int>(e.action.kind);
    d["value"] = e.action.value;
    d["source"] = static_cast<int>(e.action.source);
    d["shift"] = e.shift;
    return d;
}
rg::CockpitButton button_of(int b) { return b == 1 ? rg::CockpitButton::Secondary : rg::CockpitButton::Primary; }
}

godot::String RgCockpit::load(const godot::String& layout_path, const godot::String& car_rig_path) {
    std::string error;
    if (hit_.load(layout_path.utf8().get_data(), car_rig_path.utf8().get_data(), &error)) return godot::String();
    return text(error.empty() ? "cockpit hit: load failed" : error);
}
int RgCockpit::get_control_count() const { return static_cast<int>(hit_.control_count()); }
godot::Dictionary RgCockpit::control_info(int index) const {
    godot::Dictionary d;
    if (index < 0 || static_cast<std::size_t>(index) >= hit_.control_count()) return d;
    const rg::CockpitControl& c = hit_.control(static_cast<std::size_t>(index));
    d["id"] = text(c.id);
    d["part"] = text(c.part);
    d["wheel_mounted"] = c.wheel_mounted;
    d["click"] = c.mode == rg::CockpitControlMode::Click;
    d["joint_node"] = text(c.joint_node);
    d["has_joint"] = c.has_joint;
    d["joint_axis"] = gvec(c.joint_axis);
    d["joint_revolute"] = c.joint_revolute;
    d["joint_max"] = c.joint_max;
    d["blank_when_unavailable"] = c.blank_when_unavailable;
    d["hit_half"] = gvec(c.hit_half);
    d["secondary"] = c.secondary.valid();
    return d;
}
godot::Dictionary RgCockpit::control_state(int index) const {
    godot::Dictionary d;
    if (index < 0 || static_cast<std::size_t>(index) >= hit_.control_count()) return d;
    const auto i = static_cast<std::size_t>(index);
    d["available"] = hit_.available(i);
    d["travel"] = hit_.travel(i);
    d["pressed"] = hit_.pressed(i);
    d["hover"] = hit_.hover() == index;
    return d;
}
int RgCockpit::find(const godot::String& id) const { return hit_.find(id.utf8().get_data()); }
godot::Dictionary RgCockpit::mouse_config() const {
    godot::Dictionary d;
    d["max_eye_distance_m"] = hit_.mouse_config().max_eye_distance_m;
    d["pick_margin_m"] = hit_.mouse_config().pick_margin_m;
    d["travel_per_s"] = hit_.mouse_config().travel_per_s;
    d["debounce_s"] = hit_.debounce_s();
    return d;
}
void RgCockpit::set_wheel_angle(double rad) { hit_.set_wheel_angle(rad); }
void RgCockpit::set_enabled(bool enabled) { hit_.set_enabled(enabled); }
bool RgCockpit::is_enabled() const { return hit_.enabled(); }
int RgCockpit::pick(const godot::Vector3& o, const godot::Vector3& d) const { return hit_.pick(vec(o), vec(d)); }
void RgCockpit::pointer_move(const godot::Vector3& o, const godot::Vector3& d) { hit_.pointer_move(vec(o), vec(d)); }
int RgCockpit::pointer_down(const godot::Vector3& o, const godot::Vector3& d, int button) { return hit_.pointer_down(vec(o), vec(d), button_of(button)); }
void RgCockpit::pointer_up(int button) { hit_.pointer_up(button_of(button)); }
int RgCockpit::get_hover() const { return hit_.hover(); }
godot::Array RgCockpit::update(double dt, const godot::Dictionary& snapshot, const godot::Dictionary& channels) {
    hit_.update(dt, driver_snapshot_from(snapshot), ReadSink(channels));
    godot::Array out;
    for (const auto& e : hit_.take_events()) out.push_back(event_dict(e));
    return out;
}
godot::Array RgCockpit::held_actions() const {
    godot::Array out;
    for (const auto& a : hit_.held_actions()) {
        godot::Dictionary d;
        d["action_id"] = text(a.id);
        d["kind"] = static_cast<int>(a.kind);
        d["value"] = a.value;
        d["source"] = static_cast<int>(a.source);
        out.push_back(d);
    }
    return out;
}
godot::Vector3 RgCockpit::hit_centre_model(int index) const {
    if (index < 0 || static_cast<std::size_t>(index) >= hit_.control_count()) return godot::Vector3();
    return gvec(hit_.hit_centre_model(static_cast<std::size_t>(index)));
}
void RgCockpit::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("load", "layout_path", "car_rig_path"), &RgCockpit::load);
    ClassDB::bind_method(D_METHOD("get_control_count"), &RgCockpit::get_control_count);
    ClassDB::bind_method(D_METHOD("control_info", "index"), &RgCockpit::control_info);
    ClassDB::bind_method(D_METHOD("control_state", "index"), &RgCockpit::control_state);
    ClassDB::bind_method(D_METHOD("find", "id"), &RgCockpit::find);
    ClassDB::bind_method(D_METHOD("mouse_config"), &RgCockpit::mouse_config);
    ClassDB::bind_method(D_METHOD("set_wheel_angle", "rad"), &RgCockpit::set_wheel_angle);
    ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &RgCockpit::set_enabled);
    ClassDB::bind_method(D_METHOD("is_enabled"), &RgCockpit::is_enabled);
    ClassDB::bind_method(D_METHOD("pick", "origin", "direction"), &RgCockpit::pick);
    ClassDB::bind_method(D_METHOD("pointer_move", "origin", "direction"), &RgCockpit::pointer_move);
    ClassDB::bind_method(D_METHOD("pointer_down", "origin", "direction", "button"), &RgCockpit::pointer_down);
    ClassDB::bind_method(D_METHOD("pointer_up", "button"), &RgCockpit::pointer_up);
    ClassDB::bind_method(D_METHOD("get_hover"), &RgCockpit::get_hover);
    ClassDB::bind_method(D_METHOD("update", "dt", "snapshot", "channels"), &RgCockpit::update);
    ClassDB::bind_method(D_METHOD("held_actions"), &RgCockpit::held_actions);
    ClassDB::bind_method(D_METHOD("hit_centre_model", "index"), &RgCockpit::hit_centre_model);
}
}
