#include "rg/garage_dyno.h"
#include "../../external/physics_sim/core/src/drivetrain/engine_simulated.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include <iostream>
#include <map>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <cmath>
#include <thread>
namespace {
rg::DynoRequest request(const std::string& key) {
    const std::string root=RG_SOURCE_DIR;
    std::string error;
    auto catalog=rg::load_vehicle_catalog(root+"/data/vehicles/catalog.json",root,&error);
    REQUIRE(catalog.has_value());
    auto table=rg::load_setup_options(root+"/data/vehicles/setup_options.json",&error);
    REQUIRE(table.has_value());
    rg::DynoRequest r;
    r.entry=*catalog->find("car_hyper"); r.table=*table;
    r.context.work_root=root+"/out/dyno_unit_work";
    r.context.engine_map_cache_dir=root+"/out/godot_engine_cache";
    r.setup.vehicle_id=r.entry.id; r.key=key;
    return r;
}
rg::DynoResult finished(rg::GarageDyno& dyno) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(120);
    bool saw_provisional=false;
    while(dyno.result().busy && std::chrono::steady_clock::now()<deadline) {
        const auto partial=dyno.result();
        if(partial.provisional_points.size()>=2) {
            saw_provisional=true;
            CHECK(partial.provisional_points.front().rpm<partial.provisional_points.back().rpm);
            for(std::size_t i=1;i<partial.provisional_points.size();++i)
                CHECK(partial.provisional_points[i-1].rpm<partial.provisional_points[i].rpm);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(saw_provisional);
    auto result=dyno.result();
    INFO(result.error);
    REQUIRE_FALSE(result.busy);
    REQUIRE(result.error.empty());
    REQUIRE(result.points.size()==33);
    CHECK(result.provisional_points.empty());
    return result;
}
}
TEST_CASE("garage dyno: actual WOT output, power units, newest setup wins", "[garage_dyno]") {
    rg::GarageDyno dyno;
    dyno.request(request("stock"));
    const auto stock=finished(dyno);
    double peak=0;
    for(const auto& p:stock.points) {
        CHECK(std::isfinite(p.torque_nm));
        CHECK(p.power_kw==Catch::Approx(p.torque_nm*p.rpm*3.14159265358979323846/30000.0));
        peak=std::max(peak,p.power_kw);
    }
    CHECK(peak>900);
    CHECK(peak<1100);
    dyno.request(request("stock"));
    CHECK_FALSE(dyno.result().busy);
    CHECK(dyno.result().revision==stock.revision);
    auto stale=request("stale"); stale.setup.values["turbo_install"]=std::string("hyper_single");
    dyno.request(stale);
    CHECK(dyno.result().points.size()==stock.points.size());
    CHECK(dyno.result().key=="stock");
    auto newest=request("NA"); newest.setup.values["turbo_install"]=std::string("hyper_na");
    dyno.request(newest);
    const auto na=finished(dyno);
    CHECK(na.key=="NA");
    CHECK(na.revision>stock.revision);
    double na_peak=0;
    for(const auto& p:na.points) na_peak=std::max(na_peak,p.power_kw);
    CHECK(na_peak<peak*0.75);
    dyno.request(request("cancelled"));
    dyno.cancel();
    CHECK_FALSE(dyno.result().busy);
    CHECK(dyno.result().provisional_points.empty());
    CHECK(dyno.result().key=="NA");
    CHECK(dyno.result().revision==na.revision);
    auto invalid=request("invalid"); invalid.setup.values["turbo_install"]=std::string("not_a_part");
    dyno.request(invalid);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(dyno.result().busy && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    CHECK_FALSE(dyno.result().error.empty());
    CHECK(dyno.result().key=="NA");
    CHECK(dyno.result().revision==na.revision);
}

TEST_CASE("diagnose hyper boost target against actual pressure and losses", "[.boost_diagnostic]") {
    auto r=request("boost_probe");
    std::string error;
    auto catalog=rg::load_vehicle_catalog(std::string(RG_SOURCE_DIR)+"/data/vehicles/catalog.json",RG_SOURCE_DIR,&error);
    r.entry=*catalog->find("car_hyper_n2o");r.setup.vehicle_id=r.entry.id;
    r.setup.values["turbo_install_n2o"]=std::string("hyper_twin_large_n2o");
    r.setup.values["turbo_boost_target"]=4.3/1.8;
    rg::ScopedWorkDir work(r.context.work_root,"boost_diagnostic");
    auto material=rg::materialise_setup(r.entry,r.table,r.context,r.setup,work.path());
    INFO(material.error);REQUIRE(material.ok);
    ps::jobs::JobSystem workers(4);
    ps::io::EngineMapOptions maps;maps.pool=&workers;maps.cache_dir=r.context.engine_map_cache_dir;
    auto vehicle=ps::io::load_vehicle_json(material.vehicle_path,maps);
    for(const auto& component:vehicle.powertrain.components)if(auto original=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&component.params)) {
        REQUIRE(original->turbo_pair.has_value());
        std::map<std::pair<double,double>,double> reference_power;
        for(double target:{2.86,4.19,4.3})for(double rpm:{4500.0,6500.0,7500.0,8249.0})for(int hz:{960,1920}) {
            auto desc=*original;desc.turbo_pair->target_boost_pa=target*100000;
            ps::drivetrain::SimulatedEngine engine(desc);engine.reset(ps::drivetrain::EngineState::Running);
            ps::drivetrain::Sensors sensors;sensors.omega=rpm*3.141592653589793/30;sensors.ignition=1;sensors.clutch_engaged_fraction=1;sensors.gearbox_in_neutral=false;
            ps::drivetrain::ThrottleCommand throttle;throttle.command=1;
            double torque=0,boost=0,exhaust=0,charge=0,turbo=0,wastegate=0;
            for(int step=0;step<60*hz;++step) {
                auto pre=engine.pre_solve(sensors,throttle,ps::drivetrain::TcuRequest{},1.0/hz);(void)engine.post_solve(sensors.omega,1.0/hz);
                if((step>=15*hz && step<16*hz) || step>=59*hz) {
                    torque+=pre.tau0-pre.friction_cap_kinetic;boost+=pre.boost_bar;exhaust+=pre.p_exhaust_pa;charge+=pre.t_charge_k;turbo+=pre.turbo_rpm;wastegate+=pre.wastegate_opening;
                }
                if(step==16*hz-1 || step==60*hz-1) {
                    std::cout<<"hz="<<hz<<" target="<<target<<" rpm="<<rpm<<" seconds="<<((step+1)/hz)<<" power_kw="<<(torque/hz*rpm*3.141592653589793/30000)<<" boost="<<boost/hz<<" exhaust_bar_abs="<<exhaust/hz/100000<<" charge_k="<<charge/hz<<" turbo_rpm="<<turbo/hz<<" wastegate="<<wastegate/hz<<std::endl;
                    CHECK(std::isfinite(torque));CHECK(torque>0);
                    CHECK(boost/hz>target*0.8);CHECK(boost/hz<target*1.2);
                    CHECK(std::isfinite(turbo));
                    if(step==60*hz-1) {
                        const double power=torque/hz*rpm*3.141592653589793/30000;
                        if(hz==960)reference_power[{target,rpm}]=power;
                        else CHECK(power==Catch::Approx(reference_power.at({target,rpm})).epsilon(0.02));
                    }
                    torque=boost=exhaust=charge=turbo=wastegate=0;
                }
            }
        }
    }
}

TEST_CASE("diagnose extreme boost target power collapse", "[.extreme_boost_diagnostic]") {
    auto r=request("boost_probe");
    std::string error;
    auto catalog=rg::load_vehicle_catalog(std::string(RG_SOURCE_DIR)+"/data/vehicles/catalog.json",RG_SOURCE_DIR,&error);
    r.entry=*catalog->find("car_hyper_n2o");r.setup.vehicle_id=r.entry.id;
    r.setup.values["turbo_install_n2o"]=std::string("hyper_twin_large_n2o");
    r.setup.values["turbo_boost_target"]=29.49/1.8;
    rg::ScopedWorkDir work(r.context.work_root,"boost_diagnostic");
    auto material=rg::materialise_setup(r.entry,r.table,r.context,r.setup,work.path());
    INFO(material.error);REQUIRE(material.ok);
    ps::jobs::JobSystem workers(4);
    ps::io::EngineMapOptions maps;maps.pool=&workers;maps.cache_dir=r.context.engine_map_cache_dir;
    auto vehicle=ps::io::load_vehicle_json(material.vehicle_path,maps);
    for(const auto& component:vehicle.powertrain.components)if(auto original=std::get_if<ps::drivetrain::SimulatedEngineDesc>(&component.params)) {
        REQUIRE(original->turbo_pair.has_value());
        std::map<std::pair<double,double>,double> reference_power;
        for(bool compressible:{false,true})for(double target:{29.49})for(double rpm:{3000.0,4000.0,5000.0,6000.0,7000.0,8249.0})for(int hz:{960,1920}) {
            auto desc=*original;desc.turbo_pair->target_boost_pa=target*100000;
            desc.turbo_pair->compressible_exhaust=compressible;
            ps::drivetrain::SimulatedEngine engine(desc);engine.reset(ps::drivetrain::EngineState::Running);
            ps::drivetrain::Sensors sensors;sensors.omega=rpm*3.141592653589793/30;sensors.ignition=1;sensors.clutch_engaged_fraction=1;sensors.gearbox_in_neutral=false;
            ps::drivetrain::ThrottleCommand throttle;throttle.command=1;
            double torque=0,boost=0,exhaust=0,charge=0,turbo=0,wastegate=0;
            for(int step=0;step<60*hz;++step) {
                auto pre=engine.pre_solve(sensors,throttle,ps::drivetrain::TcuRequest{},1.0/hz);(void)engine.post_solve(sensors.omega,1.0/hz);
                if((step>=15*hz && step<16*hz) || step>=59*hz) {
                    torque+=pre.tau0-pre.friction_cap_kinetic;boost+=pre.boost_bar;exhaust+=pre.p_exhaust_pa;charge+=pre.t_charge_k;turbo+=pre.turbo_rpm;wastegate+=pre.wastegate_opening;
                }
                if(step==16*hz-1 || step==60*hz-1) {
                    std::cout<<"hz="<<hz<<" target="<<target<<" rpm="<<rpm<<" seconds="<<((step+1)/hz)<<" power_kw="<<(torque/hz*rpm*3.141592653589793/30000)<<" boost="<<boost/hz<<" exhaust_bar_abs="<<exhaust/hz/100000<<" charge_k="<<charge/hz<<" turbo_rpm="<<turbo/hz<<" wastegate="<<wastegate/hz<<std::endl;
                    CHECK(std::isfinite(torque));
                    CHECK(std::isfinite(boost));
                    if(compressible) CHECK(torque>0);
                    CHECK(std::isfinite(turbo));
                    if(step==60*hz-1) {
                        const double power=torque/hz*rpm*3.141592653589793/30000;
                        if(hz==960)reference_power[{target,rpm}]=power;
                        else CHECK(power==Catch::Approx(reference_power.at({target,rpm})).margin(0.1));
                    }
                    torque=boost=exhaust=charge=turbo=wastegate=0;
                }
            }
        }
    }
}
