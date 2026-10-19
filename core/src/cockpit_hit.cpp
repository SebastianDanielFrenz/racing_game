// rg/cockpit_hit.cpp - see cockpit_hit.h.
#include "rg/cockpit_hit.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace rg {
namespace {

using detail::json;

bool vec3_of(const json& j, ps::Vec3& out) {
    if (!j.is_array() || j.size() != 3) return false;
    for (const auto& v : j)
        if (!v.is_number()) return false;
    out = ps::Vec3{j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
    return true;
}

bool finite(const ps::Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Part frame -> parent frame, the same construction as cockpit_common.gd's mount_basis: Z = normal, X = up made
// perpendicular to it, Y = Z x X (right handed).
bool mount_rotation(const ps::Vec3& normal, const ps::Vec3& up, ps::Mat3& out) {
    const ps::Vec3 z = normal.normalized();
    if (z.length_squared() < 0.5) return false;
    ps::Vec3 x = up - z * ps::dot(up, z);
    if (x.length_squared() < 1e-12) return false;
    x = x.normalized();
    const ps::Vec3 y = ps::cross(z, x).normalized();
    out = ps::Mat3{ps::Vec3{x.x, y.x, z.x}, ps::Vec3{x.y, y.y, z.y}, ps::Vec3{x.z, y.z, z.z}};
    return true;
}

// Rotation about a unit axis (Rodrigues), right-hand rule: the rig's joint convention.
ps::Mat3 axis_rotation(const ps::Vec3& axis, double angle) {
    const ps::Vec3 a = axis.normalized();
    const double c = std::cos(angle), s = std::sin(angle), t = 1 - c;
    return ps::Mat3{
        ps::Vec3{t * a.x * a.x + c, t * a.x * a.y - s * a.z, t * a.x * a.z + s * a.y},
        ps::Vec3{t * a.x * a.y + s * a.z, t * a.y * a.y + c, t * a.y * a.z - s * a.x},
        ps::Vec3{t * a.x * a.z - s * a.y, t * a.y * a.z + s * a.x, t * a.z * a.z + c}};
}

bool parse_binding(const json& j, CockpitBinding& out, std::string& err) {
    if (!j.is_object()) { err = "binding is not an object"; return false; }
    double shift = 0;
    if (detail::get_number(j, "shift", shift)) {
        if (shift != 1 && shift != -1) { err = "binding shift must be 1 or -1"; return false; }
        out.shift = static_cast<int>(shift);
        return true;
    }
    if (!detail::get_string(j, "action", out.id) || out.id.empty()) { err = "binding has neither action nor shift"; return false; }
    std::string event = "press";
    detail::get_string(j, "event", event);
    out.value = 1;
    detail::get_number(j, "value", out.value);
    if (event == "press") out.kind = DriverActionKind::Press;
    else if (event == "set") out.kind = DriverActionKind::Set;
    else if (event == "hold") { out.kind = DriverActionKind::Set; out.held = true; out.value = 1; }
    else { err = "binding event '" + event + "' is not press, set or hold"; return false; }
    return true;
}

struct RigNode {
    ps::Vec3 pivot;
    bool has_hit = false;
    ps::Vec3 hit_half, press_axis;
    bool has_joint = false;
    bool revolute = false;
    ps::Vec3 axis;
    double jmin = 0, jmax = 0;
    double press_at = std::numeric_limits<double>::quiet_NaN();
    double release_at = std::numeric_limits<double>::quiet_NaN();
};

bool read_nodes(const json& root, std::map<std::string, RigNode>& out) {
    const auto nodes = root.find("nodes");
    if (nodes == root.end() || !nodes->is_array()) return false;
    for (const auto& n : *nodes) {
        std::string name;
        if (!n.is_object() || !detail::get_string(n, "name", name)) continue;
        RigNode r;
        const auto pivot = n.find("pivot");
        if (pivot != n.end()) vec3_of(*pivot, r.pivot);
        const auto inter = n.find("interaction");
        if (inter != n.end() && inter->is_object()) {
            const auto half = inter->find("half_extents_m");
            if (half != inter->end() && vec3_of(*half, r.hit_half)) {
                r.has_hit = true;
                const auto axis = inter->find("press_axis");
                if (axis != inter->end()) vec3_of(*axis, r.press_axis);
            }
        }
        const auto joint = n.find("joint");
        if (joint != n.end() && joint->is_object()) {
            std::string type;
            if (detail::get_string(*joint, "type", type) && joint->contains("axis") && vec3_of((*joint)["axis"], r.axis)) {
                r.has_joint = true;
                r.revolute = type == "revolute";
                detail::get_number(*joint, "min", r.jmin);
                detail::get_number(*joint, "max", r.jmax);
            }
        }
        const auto behaviour = n.find("behaviour");
        if (behaviour != n.end() && behaviour->is_object()) {
            double v = 0;
            for (const char* key : {"press_at_m", "click_at_rad", "apply_at_rad"})
                if (detail::get_number(*behaviour, key, v)) { r.press_at = v; break; }
            if (detail::get_number(*behaviour, "release_at_m", v)) r.release_at = v;
        }
        out[name] = r;
    }
    return true;
}

std::string parent_dir(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
}

} // namespace

bool CockpitHit::load(const std::string& layout_path, const std::string& car_rig_path, std::string* error) {
    auto fail = [&](const std::string& m) {
        if (error) *error = m;
        controls_.clear();
        state_.clear();
        available_.clear();
        return false;
    };
    controls_.clear();
    const auto text = detail::read_text_file(layout_path);
    if (!text) return fail("cannot read " + layout_path);
    const auto layout = detail::parse_json(*text);
    if (!layout || !layout->is_object()) return fail(layout_path + ": not a JSON object");

    const auto mouse = layout->find("mouse");
    if (mouse == layout->end() || !mouse->is_object() ||
        !detail::get_number(*mouse, "max_eye_distance_m", config_.max_eye_distance_m) ||
        !detail::get_number(*mouse, "pick_margin_m", config_.pick_margin_m) ||
        !detail::get_number(*mouse, "travel_per_s", config_.travel_per_s) ||
        !(config_.max_eye_distance_m > 0) || !(config_.pick_margin_m >= 0) || !(config_.travel_per_s > 0))
        return fail(layout_path + ": \"mouse\" {max_eye_distance_m > 0, pick_margin_m >= 0, travel_per_s > 0} missing or invalid");

    std::string parts_rig_rel;
    if (!detail::get_string(*layout, "rig", parts_rig_rel)) return fail(layout_path + ": no \"rig\"");
    const auto parts_text = detail::read_text_file(parent_dir(layout_path) + parts_rig_rel);
    if (!parts_text) return fail("cannot read parts rig " + parent_dir(layout_path) + parts_rig_rel);
    const auto parts_rig = detail::parse_json(*parts_text);
    std::map<std::string, RigNode> parts;
    if (!parts_rig || !parts_rig->is_object() || !read_nodes(*parts_rig, parts)) return fail("parts rig: no nodes");
    double press_default = 0, release_default = 0;
    {
        const auto cp = parts_rig->find("cockpit_parts");
        const auto defaults = cp != parts_rig->end() && cp->is_object() ? cp->find("interaction_defaults") : json::const_iterator();
        if (cp == parts_rig->end() || defaults == cp->end() || !defaults->is_object() ||
            !detail::get_number(*defaults, "press_fraction", press_default) ||
            !detail::get_number(*defaults, "release_fraction", release_default) ||
            !detail::get_number(*defaults, "debounce_s", debounce_s_) || !(press_default > 0 && press_default <= 1) ||
            !(release_default >= 0 && release_default < press_default) || !(debounce_s_ >= 0))
            return fail("parts rig: cockpit_parts.interaction_defaults {press_fraction, release_fraction, debounce_s} missing or invalid");
    }

    // The car rig: socket frames and the steering wheel (all optional except the wheel for wheel-mounted controls).
    struct Socket { ps::Vec3 position, normal, up; bool frame = false; };
    std::map<std::string, Socket> sockets;
    bool have_wheel = false;
    if (!car_rig_path.empty()) {
        const auto car_text = detail::read_text_file(car_rig_path);
        if (!car_text) return fail("cannot read car rig " + car_rig_path);
        const auto car = detail::parse_json(*car_text);
        if (!car || !car->is_object()) return fail(car_rig_path + ": not a JSON object");
        std::map<std::string, RigNode> car_nodes;
        read_nodes(*car, car_nodes);
        const auto wheel = car_nodes.find("steering_wheel");
        if (wheel != car_nodes.end() && wheel->second.has_joint && finite(wheel->second.axis) && wheel->second.axis.length() > 0.5) {
            wheel_pivot_ = wheel->second.pivot;
            wheel_axis_ = wheel->second.axis.normalized();
            have_wheel = true;
        }
        const auto list = car->find("sockets");
        if (list != car->end() && list->is_array()) {
            for (const auto& s : *list) {
                std::string name;
                Socket socket;
                if (!s.is_object() || !detail::get_string(s, "name", name) || !s.contains("position") || !vec3_of(s["position"], socket.position)) continue;
                socket.frame = s.contains("normal") && s.contains("up") && vec3_of(s["normal"], socket.normal) && vec3_of(s["up"], socket.up);
                sockets[name] = socket;
            }
        }
    }

    const auto mounts = layout->find("mounts");
    if (mounts == layout->end() || !mounts->is_array()) return fail(layout_path + ": no \"mounts\"");
    for (const auto& m : *mounts) {
        if (!m.is_object() || !m.contains("control")) continue;
        const json& c = m["control"];
        CockpitControl ctl;
        std::string parent, socket_name, mode = "push";
        if (!detail::get_string(m, "id", ctl.id) || !detail::get_string(m, "part", ctl.part) || !detail::get_string(m, "parent", parent))
            return fail("a mount with a control lacks id/part/parent");
        const std::string where = "mount '" + ctl.id + "': ";
        ctl.wheel_mounted = parent == "steering_wheel";
        if (ctl.wheel_mounted && !have_wheel) return fail(where + "wheel-mounted but the car rig has no steering_wheel joint");
        detail::get_string(m, "socket", socket_name);
        ps::Vec3 position, normal, up;
        if (!m.contains("position") || !vec3_of(m["position"], position) || !m.contains("normal") || !vec3_of(m["normal"], normal) ||
            !m.contains("up") || !vec3_of(m["up"], up))
            return fail(where + "position/normal/up missing");
        const auto socket = sockets.find(socket_name);
        if (socket != sockets.end()) {
            position = ctl.wheel_mounted ? socket->second.position - wheel_pivot_ : socket->second.position;
            if (socket->second.frame) { normal = socket->second.normal; up = socket->second.up; }
        }
        ctl.mount_origin = position;
        if (!finite(position) || !mount_rotation(normal, up, ctl.mount_rot)) return fail(where + "normal/up are degenerate");

        const auto hit = parts.find(c.contains("hit") && c["hit"].is_string() ? c["hit"].get<std::string>() : "hit_" + ctl.part);
        if (hit == parts.end() || !hit->second.has_hit) return fail(where + "no hit volume (hit_" + ctl.part + ") in the parts rig");
        ctl.hit_centre = hit->second.pivot;
        ctl.hit_half = hit->second.hit_half;
        if (!(ctl.hit_half.x > 0 && ctl.hit_half.y > 0 && ctl.hit_half.z > 0)) return fail(where + "hit volume has no extent");

        detail::get_string(c, "mode", mode);
        if (mode != "push" && mode != "click") return fail(where + "mode must be push or click");
        ctl.mode = mode == "push" ? CockpitControlMode::Push : CockpitControlMode::Click;
        if (ctl.mode == CockpitControlMode::Push) {
            ctl.joint_node = "ctl_" + ctl.part;
            detail::get_string(c, "joint", ctl.joint_node);
            const auto joint = parts.find(ctl.joint_node);
            if (joint == parts.end() || !joint->second.has_joint) return fail(where + "no joint " + ctl.joint_node + " in the parts rig");
            const RigNode& jn = joint->second;
            ctl.has_joint = true;
            ctl.joint_revolute = jn.revolute;
            ctl.joint_axis = jn.axis;
            ctl.joint_max = jn.jmax;
            if (!(ctl.joint_max > 0)) return fail(where + "joint " + ctl.joint_node + " has no positive travel");
            ctl.press_at = std::isnan(jn.press_at) ? press_default : jn.press_at / ctl.joint_max;
            ctl.release_at = std::isnan(jn.release_at) ? release_default : jn.release_at / ctl.joint_max;
            if (!(ctl.press_at > 0 && ctl.press_at <= 1 && ctl.release_at >= 0 && ctl.release_at < ctl.press_at))
                return fail(where + "press/release thresholds are not 0 <= release < press <= 1");
        }
        std::string err;
        if (c.contains("primary") && !parse_binding(c["primary"], ctl.primary, err)) return fail(where + "primary: " + err);
        if (c.contains("secondary") && !parse_binding(c["secondary"], ctl.secondary, err)) return fail(where + "secondary: " + err);
        detail::get_string(c, "state", ctl.state_id);
        double bit = -1;
        if (detail::get_number(c, "mode_bit", bit)) {
            if (bit < 0 || bit > 31) return fail(where + "mode_bit out of range");
            ctl.mode_bit = static_cast<int>(bit);
        }
        detail::get_bool(c, "blank_when_unavailable", ctl.blank_when_unavailable);
        controls_.push_back(ctl);
    }
    if (controls_.empty()) return fail(layout_path + ": no mount carries a \"control\"");
    state_.assign(controls_.size(), State{});
    available_.assign(controls_.size(), false);
    events_.clear();
    hover_ = -1;
    clock_s_ = 0;
    debounced_ = 0;
    return true;
}

int CockpitHit::find(std::string_view id) const {
    for (std::size_t i = 0; i < controls_.size(); ++i)
        if (controls_[i].id == id) return static_cast<int>(i);
    return -1;
}

void CockpitHit::part_pose(std::size_t index, ps::Vec3& origin, ps::Mat3& rot) const {
    const CockpitControl& c = controls_[index];
    if (!c.wheel_mounted) {
        origin = c.mount_origin;
        rot = c.mount_rot;
        return;
    }
    const ps::Mat3 wheel = axis_rotation(wheel_axis_, wheel_angle_);
    origin = wheel_pivot_ + wheel * c.mount_origin;
    rot = wheel * c.mount_rot;
}

ps::Vec3 CockpitHit::hit_centre_model(std::size_t index) const {
    ps::Vec3 origin;
    ps::Mat3 rot;
    part_pose(index, origin, rot);
    return origin + rot * controls_[index].hit_centre;
}

bool CockpitHit::ray_box(std::size_t index, const ps::Vec3& o, const ps::Vec3& d, double& t, double& volume) const {
    ps::Vec3 origin;
    ps::Mat3 rot;
    part_pose(index, origin, rot);
    const ps::Mat3 inv = rot.transposed();
    const ps::Vec3 lo = inv * (o - origin) - controls_[index].hit_centre;
    const ps::Vec3 ld = inv * d;
    const ps::Vec3 half = controls_[index].hit_half + ps::Vec3{config_.pick_margin_m, config_.pick_margin_m, config_.pick_margin_m};
    double tmin = -std::numeric_limits<double>::infinity(), tmax = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ld[i]) < 1e-12) {
            if (std::abs(lo[i]) > half[i]) return false;
            continue;
        }
        const double t1 = (-half[i] - lo[i]) / ld[i], t2 = (half[i] - lo[i]) / ld[i];
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));
    }
    if (tmax < std::max(tmin, 0.0)) return false;
    t = std::max(tmin, 0.0);
    volume = half.x * half.y * half.z;
    return true;
}

int CockpitHit::pick(const ps::Vec3& origin, const ps::Vec3& direction) const {
    if (!finite(origin) || !finite(direction) || direction.length_squared() < 1e-12) return -1;
    const ps::Vec3 d = direction.normalized();
    int best = -1;
    double best_t = 0, best_volume = 0;
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        double t = 0, volume = 0;
        if (!ray_box(i, origin, d, t, volume)) continue;
        // Nearest wins; a clearly smaller box entered within the bigger one's depth wins over it (a button on a pad).
        if (best < 0 || t < best_t - 1e-9 || (std::abs(t - best_t) <= 1e-9 && volume < best_volume)) {
            best = static_cast<int>(i);
            best_t = t;
            best_volume = volume;
        }
    }
    return best;
}

void CockpitHit::set_enabled(bool enabled) {
    if (enabled == enabled_) return;
    enabled_ = enabled;
    if (!enabled) {
        hover_ = -1;
        release_all();
    }
}

void CockpitHit::release_all() {
    for (auto& s : state_) {
        s.pointer = false;
        s.committed = false;
    }
}

void CockpitHit::pointer_move(const ps::Vec3& origin, const ps::Vec3& direction) {
    hover_ = enabled_ ? pick(origin, direction) : -1;
}

bool CockpitHit::fire(std::size_t index, const CockpitBinding& binding, bool release) {
    if (!binding.valid()) return false;
    State& s = state_[index];
    CockpitEvent e;
    e.control = static_cast<int>(index);
    e.control_id = controls_[index].id;
    e.shift = binding.shift;
    e.action.source = ActionSource::CockpitMouse;
    if (!release) {
        if (clock_s_ - s.last_fire_s < debounce_s_) { ++debounced_; return false; }
        s.last_fire_s = clock_s_;
    }
    if (!binding.id.empty()) {
        e.action.id = binding.id;
        e.action.kind = release ? DriverActionKind::Release : binding.kind;
        e.action.value = release ? 0 : binding.value;
    }
    events_.push_back(std::move(e));
    return true;
}

int CockpitHit::pointer_down(const ps::Vec3& origin, const ps::Vec3& direction, CockpitButton button) {
    if (!enabled_) return -1;
    const int hit = pick(origin, direction);
    hover_ = hit;
    if (hit < 0) return -1;
    const std::size_t i = static_cast<std::size_t>(hit);
    State& s = state_[i];
    if (s.pointer || !s.armed) return hit; // a duplicate down for a press that is still held or still returning
    if (!available_[i]) return hit; // inert: hit, swallowed, nothing submitted
    const CockpitControl& c = controls_[i];
    const CockpitBinding& binding = button == CockpitButton::Secondary ? c.secondary : c.primary;
    if (!binding.valid()) return hit;
    s.button = button;
    s.pointer = true;
    if (c.mode == CockpitControlMode::Click) {
        s.fired = fire(i, binding, false);
    } else {
        s.committed = true;
    }
    return hit;
}

void CockpitHit::pointer_up(CockpitButton button) {
    for (auto& s : state_)
        if (s.pointer && s.button == button) s.pointer = false;
}

void CockpitHit::update(double dt, const DriverSnapshot& snapshot, const ChannelSink& channels) {
    if (!std::isfinite(dt) || dt < 0) dt = 0;
    clock_s_ += dt;
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        const CockpitControl& c = controls_[i];
        bool ok = c.primary.valid() || c.secondary.valid();
        if (ok && !c.state_id.empty()) {
            const ControlState st = DriverActions::state(c.state_id, snapshot, channels);
            ok = st.fitted && st.enabled;
        }
        if (ok && c.mode_bit >= 0) ok = (snapshot.drive_modes >> c.mode_bit) & 1u;
        available_[i] = ok;

        State& s = state_[i];
        if (c.mode != CockpitControlMode::Push) continue;
        const bool want = s.pointer || s.committed;
        const double step = config_.travel_per_s * dt;
        if (want) s.travel = std::min(1.0, s.travel + step);
        else s.travel = std::max(0.0, s.travel - step);
        const CockpitBinding& binding = s.button == CockpitButton::Secondary ? c.secondary : c.primary;
        if (s.armed && s.travel >= c.press_at && (want || s.travel > 0)) {
            if (s.committed || s.pointer) {
                s.armed = false;
                s.pressed = true;
                s.committed = false;
                s.fired = available_[i] && fire(i, binding, false);
            }
        }
        if (s.pressed && !want && s.travel <= c.release_at) {
            s.pressed = false;
            s.armed = true;
            if (s.fired && binding.held) fire(i, binding, true);
            s.fired = false;
        }
    }
    if (!enabled_) {
        // Disabled mid-press: let a held level go at once rather than waiting for the travel to fall.
        for (std::size_t i = 0; i < controls_.size(); ++i) {
            State& s = state_[i];
            const CockpitBinding& binding = s.button == CockpitButton::Secondary ? controls_[i].secondary : controls_[i].primary;
            if (s.pressed && s.fired && binding.held) {
                fire(i, binding, true);
                s.fired = false;
            }
        }
    }
}

std::vector<CockpitEvent> CockpitHit::take_events() {
    std::vector<CockpitEvent> out;
    out.swap(events_);
    return out;
}

std::vector<ActionEvent> CockpitHit::held_actions() const {
    std::vector<ActionEvent> out;
    if (!enabled_) return out;
    for (std::size_t i = 0; i < controls_.size(); ++i) {
        const State& s = state_[i];
        const CockpitBinding& binding = s.button == CockpitButton::Secondary ? controls_[i].secondary : controls_[i].primary;
        if (s.pressed && s.fired && binding.held && !binding.id.empty())
            out.push_back(ActionEvent{binding.id, DriverActionKind::Set, binding.value, ActionSource::CockpitMouse});
    }
    return out;
}

} // namespace rg
