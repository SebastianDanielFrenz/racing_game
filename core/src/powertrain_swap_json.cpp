#include "powertrain_swap_json.h"
#include <filesystem>
#include <set>
namespace rg::detail {
namespace {
using json=nlohmann::json;namespace fs=std::filesystem;
json read(const fs::path& path){const auto text=read_text_file(path.generic_string());if(!text)return {};const auto j=parse_json(*text);return j?*j:json{};}
void refs(json& node,const fs::path& dir){
 if(node.is_object()){for(auto it=node.begin();it!=node.end();++it)if(it.key()!="source")refs(it.value(),dir);}
 else if(node.is_array()){for(auto& v:node)refs(v,dir);}
 else if(node.is_string()){const auto str=node.get<std::string>();if(str.size()<400&&str.ends_with(".json")){auto path=fs::path(str);if(!path.is_absolute())path=dir/path;std::error_code ec;if(fs::is_regular_file(path,ec))node=path.lexically_normal().generic_string();}}
}
bool has(const json& v,const std::string& type){if(!v.contains("powertrain"))return false;const auto& p=v["powertrain"];if(!p.contains("components")||!p["components"].is_array())return false;for(const auto& c:p["components"])if(c.is_object()&&c.value("type",std::string{})==type)return true;return false;}
}
std::string powertrain_kind(const json& vehicle){return has(vehicle,"engine")?(has(vehicle,"electric_motor")?"hybrid_battery":"ice"):(has(vehicle,"fuel_cell")?"fuel_cell_ev":"battery_ev");}
std::string swap_powertrain(json& v,const std::string& choice,const std::string& config_path,const std::string& vehicle_path){
 const auto cfg=read(config_path);if(!cfg.is_object()||cfg.value("format",std::string{})!="rg.powertrain_swaps/1")return "powertrain conversion catalogue is unavailable";
 bool known=false;for(const auto& c:cfg.at("choices"))if(c.at("id")==choice)known=true;if(!known)return "unknown powertrain conversion: "+choice;
 std::set<std::string> wheels;for(const auto& w:v.value("wheels",json::array()))if(w.is_object())wheels.insert(w.value("name",std::string{}));
 if(wheels!=std::set<std::string>{"FL","FR","RL","RR"})return "powertrain conversions currently require four named road wheels";
 const fs::path dir=fs::path(config_path).parent_path();
 auto donor=[&](const char* key){const auto path=(dir/cfg.at(key).get<std::string>()).lexically_normal();auto d=read(path);refs(d,path.parent_path());return d;};
 const auto parking_brake=[&](const json& controller) {
  if(controller.value("type",std::string{})!="electronics")return false;
  auto path=fs::path(controller.value("ref",std::string{}));if(path.is_relative())path=fs::path(vehicle_path).parent_path()/path;
  const auto part=read(path.lexically_normal());return part.is_object()&&part.value("kind",std::string{})=="epb";
 };
 json retained_parking=json::array();
 for(auto controller:v.value("controllers",json::array()))if(parking_brake(controller)){refs(controller,fs::path(vehicle_path).parent_path());retained_parking.push_back(controller);}
 const bool electric=choice=="battery_ev"||choice=="fuel_cell_ev";
 if(electric){
  auto d=donor("electric_donor");if(!d.is_object()||!d.contains("powertrain"))return "electric conversion donor is unavailable";
  v["powertrain"]=d["powertrain"];v["controllers"]=d.value("controllers",json::array());
  bool has_parking=false;for(const auto& c:v["controllers"])has_parking|=parking_brake(c);if(!has_parking)for(const auto& c:retained_parking)v["controllers"].push_back(c);
  v.erase("cooling_system");v.erase("engine_thermal_start");
 } else if(!has(v,"engine")){
  auto d=donor("ice_donor");if(!d.is_object()||!d.contains("powertrain"))return "combustion conversion donor is unavailable";
  v["powertrain"]=d["powertrain"];v["controllers"]=d.value("controllers",json::array());
  bool has_parking=false;for(const auto& c:v["controllers"])has_parking|=parking_brake(c);if(!has_parking)for(const auto& c:retained_parking)v["controllers"].push_back(c);
  for(const char* key:{"cooling_system","engine_thermal_start"})if(d.contains(key))v[key]=d[key];
 }
 auto& pt=v["powertrain"];auto& comps=pt["components"];
 if(!electric&&choice!="ice"){
  std::string engine_id;for(const auto& c:comps)if(c.value("type",std::string{})=="engine")engine_id=c.at("id").get<std::string>();
  if(engine_id.empty())return "hybrid conversion requires a combustion engine";
  auto battery=cfg.at("hybrid_battery");battery["id"]="conversion_battery";battery["type"]="battery";comps.push_back(battery);
  auto motor=cfg.at("hybrid_motor");motor["id"]="conversion_motor";motor["type"]="electric_motor";motor["battery"]="conversion_battery";comps.push_back(motor);
  auto generator=cfg.at("hybrid_generator");generator["id"]="conversion_generator";generator["type"]="electric_motor";generator["battery"]="conversion_battery";comps.push_back(generator);
  auto control=cfg.at("hybrid_control");control["id"]="conversion_control";control["type"]="hybrid_control";comps.push_back(control);
  comps.push_back({{"id","conversion_reduction"},{"type","gear_pair"},{"ratio",2.2},{"efficiency",.98},{"inertia_in_kgm2",.02},{"inertia_out_kgm2",.02}});
  bool generator_attached=false,traction_attached=false;
  std::string gearbox;for(const auto& c:comps)if(c.value("type",std::string{})=="gearbox")gearbox=c.at("id").get<std::string>();
  for(auto& shaft:pt["shafts"]){bool crank=false,output=false;for(const auto& port:shaft["ports"]){if(port==engine_id+".crank")crank=true;if(port==gearbox+".out")output=true;}
   if(crank){shaft["ports"].push_back("conversion_generator.shaft");generator_attached=true;}
   if(output){shaft["ports"].push_back("conversion_reduction.out");traction_attached=true;}
  }
  if(!generator_attached||!traction_attached)return "hybrid conversion needs crank and gearbox output connections";
  pt["shafts"].push_back({{"ports",{"conversion_motor.shaft","conversion_reduction.in"}}});
  // One hybrid-aware electronics unit; the mechanical TCU remains fitted.
  auto controllers=v.value("controllers",json::array());json kept=json::array();
  for(const auto& controller:controllers)if(controller.value("type",std::string{})!="electronics"||parking_brake(controller))kept.push_back(controller);
  kept.push_back({{"id","hybrid_assists"},{"type","electronics"},{"ref",(dir/"../electronics/assist_unit_hybrid.json").lexically_normal().generic_string()}});kept.push_back({{"id","hybrid_modes"},{"type","electronics"},{"ref",(dir/"../electronics/drive_modes_hybrid.json").lexically_normal().generic_string()}});v["controllers"]=kept;

 }
 if(choice=="fuel_cell_ev"||choice=="hybrid_dual_hydrogen"){
  const std::string battery=choice=="fuel_cell_ev"?"battery":"conversion_battery";
  if(choice=="fuel_cell_ev")for(auto& c:comps)if(c.value("type",std::string{})=="battery"){auto b=cfg.at("hybrid_battery");b["id"]=battery;b["type"]="battery";b["initial_soc"]=.6;c=b;}
  auto stack=cfg.at("fuel_cell");stack["id"]="conversion_fuel_cell";stack["type"]="fuel_cell";stack["battery"]=battery;comps.push_back(stack);
 }
 if(choice=="hybrid_dual_hydrogen"){
  for(auto& c:comps){
   if(c.value("type",std::string{})=="engine") { c["ref"]=(dir/"../engines/hydrogen_i4_2l0.json").lexically_normal().generic_string();c.erase("forced_induction");c.erase("nitrous"); }
   if(c.value("type",std::string{})=="fuel_tank") {c["fuel"]=(dir/"../../external/physics_sim/data/fuels/hydrogen.json").lexically_normal().generic_string();c["capacity_kg"]=6;c["initial_kg"]=5;}
  }
  v.erase("cooling_system");v.erase("engine_thermal_start");
 }
 v["contents_mass"]={{"enabled",true},{"update_threshold_kg",.1}};
 return {};
}
}
