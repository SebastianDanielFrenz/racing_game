#include "rg_electric_motor_sound.h"
#include <godot_cpp/core/class_db.hpp>
#include <cstring>
namespace rg_godot {
godot::PackedFloat32Array RgElectricMotorSound::render(int frames,double gain) {
    const auto pcm=voice_.render(frames,gain);
    godot::PackedFloat32Array out;out.resize(static_cast<int64_t>(pcm.size()));
    if(!pcm.empty())std::memcpy(out.ptrw(),pcm.data(),pcm.size()*sizeof(float));
    return out;
}
void RgElectricMotorSound::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("set_state","rpm","torque_nm","power_w","ready"),&RgElectricMotorSound::set_state);
    godot::ClassDB::bind_method(godot::D_METHOD("render","frames","gain"),&RgElectricMotorSound::render);
}
}
