#pragma once
#include "rg/electric_motor_sound.h"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
namespace rg_godot {
class RgElectricMotorSound : public godot::RefCounted {
    GDCLASS(RgElectricMotorSound,godot::RefCounted)
public:
    void set_state(double rpm,double torque_nm,double power_w,bool ready) { voice_.set_state(rpm,torque_nm,power_w,ready); }
    godot::PackedFloat32Array render(int frames,double gain);
protected:
    static void _bind_methods();
private:
    rg::ElectricMotorSound voice_;
};
}
