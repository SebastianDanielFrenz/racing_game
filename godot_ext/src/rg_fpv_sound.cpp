#include "rg_fpv_sound.h"

#include <godot_cpp/core/class_db.hpp>

#include <cstring>
#include <string>

namespace rg_godot {

godot::String RgFpvSound::load_config(const godot::String& path) {
    std::string err;
    const godot::CharString utf8 = path.utf8();
    const auto c = rg::load_fpv_audio_config(utf8.get_data(), &err);
    if (!c) return godot::String(err.c_str());
    config_ = *c;
    voice_ = rg::FpvRotorVoice(config_, 1);
    return godot::String();
}

void RgFpvSound::set_activity(double activity) { voice_.set_activity(activity); }

godot::PackedFloat32Array RgFpvSound::render(int frames, double gain) {
    godot::PackedFloat32Array out;
    if (frames <= 0) return out;
    const std::vector<float> pcm = voice_.render(frames, static_cast<float>(gain));
    out.resize(static_cast<int64_t>(pcm.size()));
    if (!pcm.empty()) std::memcpy(out.ptrw(), pcm.data(), pcm.size() * sizeof(float));
    return out;
}

double RgFpvSound::get_frequency_hz() const { return voice_.frequency_hz(); }

void RgFpvSound::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("load_config", "path"), &RgFpvSound::load_config);
    godot::ClassDB::bind_method(godot::D_METHOD("set_activity", "activity"), &RgFpvSound::set_activity);
    godot::ClassDB::bind_method(godot::D_METHOD("render", "frames", "gain"), &RgFpvSound::render);
    godot::ClassDB::bind_method(godot::D_METHOD("get_frequency_hz"), &RgFpvSound::get_frequency_hz);
}

} // namespace rg_godot
