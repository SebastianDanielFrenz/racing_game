#include "rg_assists.h"

#include "rg/assist_info.h"

#include <godot_cpp/core/class_db.hpp>

namespace rg_godot {

int RgAssists::next_traction_mode(int available_bits, int current) {
    return rg::next_traction_mode(static_cast<std::uint32_t>(available_bits), current);
}

godot::String RgAssists::traction_mode_name(int code) {
    return godot::String(rg::traction_mode_name(code));
}

double RgAssists::toggled_aero_control(int fitted_bits, double current_value) {
    return rg::toggled_aero_control(static_cast<std::uint32_t>(fitted_bits), current_value);
}

int RgAssists::next_drive_mode(int available_bits, int current) {
    return rg::next_drive_mode(static_cast<std::uint32_t>(available_bits), current);
}

godot::String RgAssists::drive_mode_name(int code) {
    return godot::String(rg::drive_mode_name(code));
}

void RgAssists::_bind_methods() {
    godot::ClassDB::bind_static_method("RgAssists", godot::D_METHOD("next_drive_mode", "available_bits", "current"), &RgAssists::next_drive_mode);
    godot::ClassDB::bind_static_method("RgAssists", godot::D_METHOD("drive_mode_name", "code"), &RgAssists::drive_mode_name);
    godot::ClassDB::bind_static_method("RgAssists", godot::D_METHOD("next_traction_mode", "available_bits", "current"), &RgAssists::next_traction_mode);
    godot::ClassDB::bind_static_method("RgAssists", godot::D_METHOD("traction_mode_name", "code"), &RgAssists::traction_mode_name);
    godot::ClassDB::bind_static_method("RgAssists", godot::D_METHOD("toggled_aero_control", "fitted_bits", "current_value"), &RgAssists::toggled_aero_control);
}

} // namespace rg_godot
