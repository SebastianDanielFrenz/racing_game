#include "rg_simulation.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "spatial_audio.h"
#include <algorithm>
#include <vector>

namespace rg_godot {
godot::Dictionary RgSimulation::start_engine_audio() {
    stop_engine_audio();
    godot::Dictionary result;
    std::string reason = "no running session";
    if (session_ && session_->running())
        engine_voice_ = ps_godot::VehicleVoice::try_create(session_->vehicle_desc(), reason);
    result["reason"] = godot::String(reason.c_str());
    result["active"] = bool(engine_voice_);
    if (engine_voice_) {
        result["rate"] = engine_voice_->sample_rate_hz();
        result["channels"] = engine_voice_->channel_count();
        const auto& frame = session_->snapshot();
        if(!frame.powertrain.engines.empty())engine_voice_->push_tick(frame.powertrain.engines.front(),frame.sim_time);
        engine_voice_->start();
        auto* voice=engine_voice_.get();
        session_->set_engine_audio_publisher([voice](const ps::drivetrain::EngineSoundState& state,double time){
            voice->push_tick(state,time);
        });
    }
    return result;
}
void RgSimulation::stop_engine_audio() {
    if(session_)session_->set_engine_audio_publisher({}); // waits for physics publisher
    engine_voice_.reset(); // invalidates native output and joins synthesis
    audio_tick_ = ~std::uint64_t{0};
}
void RgSimulation::update_engine_audio() {
    if (!engine_voice_ || !session_) return;
    engine_voice_->set_paused(!session_->running() || session_->streaming_status().frozen);

}
bool RgSimulation::connect_engine_audio_spatial(godot::Object* object,int latency_ms) {
    auto* spatial=godot::Object::cast_to<ps_godot::PsSpatialAudio>(object);
    return spatial && engine_voice_ && spatial->attach_native_source(engine_voice_->native_output()->acquire(),latency_ms,1.0f/15.0f);
}

godot::PackedFloat32Array RgSimulation::read_engine_audio(int frames) {
    godot::PackedFloat32Array result;
    if (!engine_voice_ || frames <= 0) return result;
    frames = std::min(frames, 16384);
    const int channels = engine_voice_->channel_count();
    std::vector<float> samples(static_cast<std::size_t>(frames * channels));
    const auto got = engine_voice_->runner().read(samples.data(), static_cast<std::size_t>(frames));
    result.resize(static_cast<int64_t>(got * static_cast<std::size_t>(channels)));
    for (int64_t i = 0; i < result.size(); ++i) result[i] = samples[static_cast<std::size_t>(i)] / 15.0f;
    return result;
}
godot::PackedVector3Array RgSimulation::get_engine_audio_positions() const {
    godot::PackedVector3Array result;
    if (engine_voice_) for (int c = 0; c < engine_voice_->channel_count(); ++c) {
        const auto p = engine_voice_->source(c).position_m;
        result.push_back(godot::Vector3(p.x, p.y, p.z));
    }
    return result;
}
} // namespace rg_godot
