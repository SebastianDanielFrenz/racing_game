#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace rg {
// Authored motor/inverter timbre, driven by measured shaft speed and torque.
// Mono PCM at 48 kHz; no playback, physics or Godot dependency.
// Per-layer trim in dB (data/controls/electric_motor_sound.json). Owner 2026-10-18: the high-frequency motor/inverter noise
// was "extremely annoying" and even the base motor tone too loud (an EV at 40 km/h is dominated by wind and tyre noise).
//   base      the electrical fundamental and its 2nd/3rd harmonic (the motor tone proper)
//   gear_whine the gearset mesh whine (17 teeth per shaft turn, up to 6.5 kHz)
//   inverter  the switching whistle (about 5.7 kHz, grows with load)
// A value <= kSilentDb switches the layer off.
struct ElectricMotorSoundConfig {
    static constexpr double kSilentDb = -90.0;
    double base_db = -8.0;
    double gear_whine_db = -30.0;
    double inverter_db = -30.0;
};

class ElectricMotorSound {
public:
    static constexpr int rate = 48000;
    void configure(const ElectricMotorSoundConfig& c) {
        auto gain=[](double db){return (std::isfinite(db)&&db>ElectricMotorSoundConfig::kSilentDb)?std::pow(10.0,std::min(db,12.0)/20.0):0.0;};
        base_gain_=gain(c.base_db);whine_gain_=gain(c.gear_whine_db);inverter_gain_=gain(c.inverter_db);
    }
    void set_state(double rpm, double torque_nm, double power_w, bool ready) {
        target_rpm_ = std::isfinite(rpm) ? std::clamp(std::abs(rpm),0.0,100000.0) : 0.0;
        const double torque = std::isfinite(torque_nm) ? std::abs(torque_nm) : 0.0;
        target_load_ = torque/(torque+150.0);
        regen_ = std::isfinite(power_w) && power_w < -1.0;
        target_level_ = ready ? std::clamp(target_rpm_/180.0,0.0,1.0)*(0.10+0.35*target_load_) : 0.0;
    }
    std::vector<float> render(int frames, double gain) {
        frames=std::clamp(frames,0,16384);
        std::vector<float> out(static_cast<std::size_t>(frames));
        gain=std::isfinite(gain)?std::clamp(gain,0.0,1.0):0.0;
        constexpr double tau=6.283185307179586;
        // Smooth block-rate physics updates per sample; retain oscillator phase.
        constexpr double slew=1.0/1440.0; // about 30 ms
        for(auto& sample:out) {
            rpm_+=(target_rpm_-rpm_)*slew;
            load_+=(target_load_-load_)*slew;
            level_+=(target_level_-level_)*slew;
            regen_mix_+=((regen_?1.0:0.0)-regen_mix_)*slew;
            const double electrical=std::clamp(rpm_/60.0*4.0,0.0,3000.0);
            phase_=std::fmod(phase_+tau*electrical/rate,tau);
            gear_phase_=std::fmod(gear_phase_+tau*std::min(6500.0,rpm_/60.0*17.0)/rate,tau);
            inverter_phase_=std::fmod(inverter_phase_+tau*(5700.0+350.0*load_)/rate,tau);
            const double tone=base_gain_*(0.56*std::sin(phase_)+0.20*std::sin(2*phase_)
                +(0.10+0.08*regen_mix_)*std::sin(3*phase_))
                +whine_gain_*0.13*std::sin(gear_phase_)+inverter_gain_*0.025*load_*std::sin(inverter_phase_);
            sample=static_cast<float>(gain*level_*tone);
        }
        return out;
    }
private:
    double target_rpm_=0,target_load_=0,target_level_=0;
    double rpm_=0,load_=0,level_=0,regen_mix_=0;
    double phase_=0,gear_phase_=0,inverter_phase_=0;
    bool regen_=false;
    double base_gain_=std::pow(10.0,ElectricMotorSoundConfig{}.base_db/20.0);
    double whine_gain_=std::pow(10.0,ElectricMotorSoundConfig{}.gear_whine_db/20.0);
    double inverter_gain_=std::pow(10.0,ElectricMotorSoundConfig{}.inverter_db/20.0);
};
}
