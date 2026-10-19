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
void RgElectricMotorSound::configure(const godot::Dictionary& d) {
    rg::ElectricMotorSoundConfig c;
    c.base_db=d.get("base_db",c.base_db);
    c.gear_whine_db=d.get("gear_whine_db",c.gear_whine_db);
    c.inverter_db=d.get("inverter_db",c.inverter_db);
    voice_.configure(c);
}
void RgElectricMotorSound::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("configure","config"),&RgElectricMotorSound::configure);
    godot::ClassDB::bind_method(godot::D_METHOD("set_state","rpm","torque_nm","power_w","ready"),&RgElectricMotorSound::set_state);
    godot::ClassDB::bind_method(godot::D_METHOD("render","frames","gain"),&RgElectricMotorSound::render);
}
}
