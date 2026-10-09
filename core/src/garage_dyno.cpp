#include "rg/garage_dyno.h"
#include "drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
namespace rg {
namespace {
std::vector<DynoPoint> measure_dyno(const ps::drivetrain::SimulatedEngineDesc* simulated,
    const ps::drivetrain::TorqueMapEngineDesc* mapped,int count,const std::atomic<bool>* cancel,
    const std::function<void(const std::vector<DynoPoint>&,int)>& publish) {
    const double idle=simulated ? simulated->idle_rpm:mapped->idle_rpm;
    const double limit=simulated ? simulated->limiter.rpm:mapped->limiter.rpm;
    std::unique_ptr<ps::drivetrain::SimulatedEngine> engine;
    if(simulated)engine=std::make_unique<ps::drivetrain::SimulatedEngine>(*simulated);
    constexpr double pi=3.14159265358979323846, h=1.0/960.0;
    std::vector<int> order{0,32};
    for(int stride=16;stride>=1;stride/=2)for(int i=stride;i<32;i+=2*stride)order.push_back(i);
    std::vector<DynoPoint> points;
    for(int index=0;index<count;++index) {
        const int point=order[index];
        if(cancel->load()) throw ps::io::GenerationCancelled();
        const double rpm=idle+(limit-1.0-idle)*point/32.0;
        double torque=0, compressor_samples=0, turbine_samples=0, fallback_samples=0;
        const bool turbo_simulation=simulated && simulated->turbo_pair && simulated->turbo_pair->maps && simulated->turbo_pair->maps->stages;
        if(engine) {
            engine->reset(ps::drivetrain::EngineState::Running);
            ps::drivetrain::Sensors sensors;
            sensors.omega=rpm*pi/30.0; sensors.ignition=1;
            sensors.clutch_engaged_fraction=1; sensors.gearbox_in_neutral=false;
            ps::drivetrain::ThrottleCommand throttle; throttle.command=1;
            for(int step=0;step<15360;++step) {
                if((step&255)==0 && cancel->load()) throw ps::io::GenerationCancelled();
                auto pre=engine->pre_solve(sensors,throttle,ps::drivetrain::TcuRequest{},h);
                engine->post_solve(sensors.omega,h);
                if(step>=14400) {
                    torque+=pre.tau0-pre.friction_cap_kinetic;
                    compressor_samples+=pre.turbo_compressor_stage_used;
                    turbine_samples+=pre.turbo_turbine_stage_used;
                    fallback_samples+=pre.turbo_stage_map_fallback;
                }
            }
            torque/=960.0;
        } else {
            // These authored WOT curves already specify net crank torque.
            torque=mapped->wot_torque_nm_vs_rpm.evaluate(rpm);
        }
        if(!std::isfinite(torque)) throw std::runtime_error("Non-finite dyno torque");
        points.push_back({rpm,torque,torque*rpm*pi/30000.0,turbo_simulation,
            compressor_samples/960.0,turbine_samples/960.0,fallback_samples/960.0});
        std::sort(points.begin(),points.end(),[](const auto& a,const auto& b){return a.rpm<b.rpm;});
        publish(points,index+1);
    }
    return points;
}
}

GarageDyno::GarageDyno():thread_([this]{run();}) {}
GarageDyno::~GarageDyno() {
    { std::lock_guard lock(mutex_); stopping_=true; if(cancel_) cancel_->store(true); }
    wake_.notify_one(); thread_.join();
}
void GarageDyno::request(DynoRequest request) {
    std::lock_guard lock(mutex_);
    if(requested_key_==request.key) return;
    if(cancel_) cancel_->store(true);
    if(result_.vehicle_id!=request.entry.id) result_=DynoResult{};
    requested_key_=request.key;
    ++revision_;
    result_.vehicle_id=request.entry.id;
    result_.busy=true; result_.error.clear(); result_.progress=0; result_.provisional_points.clear();
    pending_=std::move(request);
    wake_.notify_one();
}
void GarageDyno::cancel() {
    std::lock_guard lock(mutex_);
    ++revision_; requested_key_.clear(); pending_.reset();
    if(cancel_) cancel_->store(true);
    result_.busy=false; result_.provisional_points.clear();
}
DynoResult GarageDyno::result() const { std::lock_guard lock(mutex_); return result_; }
void GarageDyno::run() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
#endif
    ps::jobs::JobSystemOptions pool_options; pool_options.background_priority=true;
    ps::jobs::JobSystem pool(2,pool_options);
    for(;;) {
        DynoRequest request;
        std::uint64_t revision;
        std::shared_ptr<std::atomic<bool>> cancel;
        { std::unique_lock lock(mutex_);
          wake_.wait(lock,[this]{return stopping_ || pending_.has_value();});
          if(stopping_) return;
          // Wait for slider edits to settle before starting another cold map.
          auto observed=revision_;
          while(wake_.wait_for(lock,std::chrono::milliseconds(350),[&]{return stopping_ || revision_!=observed;})) {
              if(stopping_)return;
              observed=revision_;
          }
          if(!pending_)continue;
          request=std::move(*pending_); pending_.reset(); revision=revision_;
          cancel=std::make_shared<std::atomic<bool>>(false); cancel_=cancel;
        }
        auto progress=[&](int value) {
            std::lock_guard lock(mutex_);
            if(revision==revision_) result_.progress=value;
        };
        std::vector<DynoPoint> points;
        std::string error;
        try {
            ScopedWorkDir work(request.context.work_root+"_dyno","pull");
            const auto material=materialise_setup(request.entry,request.table,request.context,request.setup,work.path());
            if(!material.ok) throw std::runtime_error(material.error);
            ps::io::EngineMapOptions maps;
            maps.cache_dir=request.context.engine_map_cache_dir;
            maps.cancel=cancel.get(); maps.pool=&pool;
            maps.on_progress=[&](std::size_t done,std::size_t total){progress(int(60*done/std::max(std::size_t(1),total)));};
            int coarse_stage=0;
            maps.on_partial_engine=[&](const auto& coarse) {
                const bool first=(coarse_stage++==0);
                measure_dyno(&coarse,nullptr,9,cancel.get(),[&](const auto& partial,int done) {
                    if(!first && done<9)return;
                    std::lock_guard lock(mutex_);
                    if(revision==revision_ && !cancel->load())result_.provisional_points=partial;
                });
            };
            const auto desc=ps::io::load_vehicle_json(material.vehicle_path,maps);
            const ps::drivetrain::SimulatedEngineDesc* simulated=nullptr;
            const ps::drivetrain::TorqueMapEngineDesc* mapped=nullptr;
            for(const auto& component:desc.powertrain.components) {
                if(auto e=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&component.params)) simulated=e;
                if(auto e=std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&component.params)) mapped=e;
            }
            if(!simulated && !mapped) throw std::runtime_error("Engine type has no supported dyno harness");
            points=measure_dyno(simulated,mapped,33,cancel.get(),[&](const auto& partial,int done) {
                std::lock_guard lock(mutex_);
                if(revision!=revision_ || cancel->load())return;
                if(partial.size()>=result_.provisional_points.size())result_.provisional_points=partial;
                result_.progress=60+done*40/33;
            });
        } catch(const std::exception& e) { error=e.what(); }
        { std::lock_guard lock(mutex_);
          if(revision!=revision_ || cancel->load()) continue;
          result_.busy=false; result_.error=error;result_.provisional_points.clear();
          if(error.empty()) {
              result_.points=std::move(points); result_.revision=revision;
              result_.key=request.key; result_.progress=100;
          }
        }
    }
}
} // namespace rg
