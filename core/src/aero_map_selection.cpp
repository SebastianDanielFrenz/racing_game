#include "rg/aero_map_selection.h"
#include "ps/io/aero_coefficient_map_io.h"
#include "g2m/core/hash.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <stdexcept>
namespace rg {
namespace {
std::string bytes(const std::filesystem::path& path,std::uintmax_t limit) {
 if(std::filesystem::file_size(path)>limit)throw std::invalid_argument("aero map file exceeds size limit: "+path.string());
 std::ifstream file(path,std::ios::binary);if(!file)throw std::invalid_argument("cannot read aero map file: "+path.string());
 return {std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>()};
}
std::string sha(const std::string& content){return g2m::to_hex(g2m::Sha256::of(std::string_view(content)));}
}
void apply_aero_map_selection(const std::string& selection_path,ps::vehicle::VehicleDesc& vehicle) {
 if(selection_path.empty())return;
 const auto selection=nlohmann::json::parse(bytes(selection_path,1024*1024));
 if(selection.at("enabled").get<bool>()) {
  const auto map_path=std::filesystem::path(selection_path).parent_path()/selection.at("coefficient_map").at("file").get<std::string>();
  const auto proof=nlohmann::json::parse(bytes(map_path.string()+".provenance.json",2*1024*1024));
  if(!proof.at("validated").get<bool>() || proof.value("kind",std::string{})!="cfd")throw std::invalid_argument("aero map requires reviewed CFD provenance; no authored/synthetic table may be enabled");
  if(proof.at("map_sha256").get<std::string>()!=sha(bytes(map_path,256*1024*1024)))throw std::invalid_argument("aero coefficient bytes differ from reviewed provenance");
  if(!proof.at("geometry").is_array()||proof.at("geometry").empty())throw std::invalid_argument("aero geometry manifest must not be empty");
  std::string geometry_digest,combined;
  for(const auto& geometry:proof.at("geometry")) {
   const auto geometry_path=map_path.parent_path()/geometry.at("path").get<std::string>();
   const auto digest=sha(bytes(geometry_path,512*1024*1024));
   if(!combined.empty())combined+="\n";combined+=digest;geometry_digest=digest;
   if(geometry.at("sha256").get<std::string>()!=digest)throw std::invalid_argument("aero geometry changed; rebake/review coefficients first");
  }
  if(proof.at("geometry").size()>1)geometry_digest=sha(combined);
  if(geometry_digest!=proof.at("geometry_sha256").get<std::string>())throw std::invalid_argument("aero geometry manifest identity mismatch");
  const auto raw_map=nlohmann::json::parse(bytes(map_path,256*1024*1024));
  if(raw_map.at("provenance").at("geometry_sha256")!=proof.at("geometry_sha256"))throw std::invalid_argument("aero map and review geometry identities differ");
 }
 auto candidate=vehicle.aero;
 ps::io::load_aero_coefficient_map_reference(selection_path,candidate);
 if(candidate.coefficient_map.table && candidate.coefficient_map.scope==ps::aero::MapScope::ReplacePassiveAero) {
  const auto& axes=candidate.coefficient_map.table->data().axes;
  for(const auto& surface:candidate.surfaces) {
   if(surface.min_offset_rad!=surface.max_offset_rad && axes[4].empty())
    throw std::invalid_argument("full passive aero map for an adjustable wing requires wing_offset_deg samples and wing_surface binding");
   if(surface.max_lift_m>0 && axes[5].empty())
    throw std::invalid_argument("full passive aero map for a lifting wing requires wing_lift_m samples and wing_surface binding");
  }
 }
 vehicle.aero=std::move(candidate);
}
}
