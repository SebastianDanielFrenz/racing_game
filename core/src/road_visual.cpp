#include "rg/road_visual.h"
#include "rg/road_structures.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <array>
#include <span>
#include <map>
#include <optional>
#include <set>
#include "rg/npc_traffic.h"
#include <tuple>
#include <vector>
namespace rg {
namespace {
std::string tag(const g2m::RoadGraphWay& way,std::string_view key) {for(const auto& [k,v]:way.tags)if(k==key)return v;return {};}
int integer(const std::string& s) {int n=0;auto r=std::from_chars(s.data(),s.data()+s.size(),n);return r.ec==std::errc{}&&r.ptr==s.data()+s.size()&&n>=0&&n<=30?n:0;}

constexpr double kJunctionLiftM=0.004; // above the ribbons (lift) and the turn arrows (+3 mm)
// ---- junction surfaces (geo2map roads.geom encoding 7, design D44 kerb returns) ----
struct P2 { double x,y; };
double cross(P2 o,P2 a,P2 b) {return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x);}
// Ear clipping of a simple counter-clockwise ring. Collinear and repeated vertices are tolerated; a ring that cannot be
// clipped completely (numerically degenerate) returns the triangles found so far.
std::vector<std::array<P2,3>> triangulate(std::vector<P2> ring) {
 std::vector<std::array<P2,3>> out;
 const auto inside=[](P2 p,P2 a,P2 b,P2 c){return cross(a,b,p)>=0&&cross(b,c,p)>=0&&cross(c,a,p)>=0;};
 const std::size_t limit=ring.size()+16;std::size_t guard=0,start=0; // every pass removes one vertex or stops
 while(ring.size()>3&&guard++<limit) {
  bool clipped=false;
  for(std::size_t step=0;step<ring.size();++step) {
   const std::size_t i=(start+step)%ring.size();
   const std::size_t ia=(i+ring.size()-1)%ring.size(),ic=(i+1)%ring.size();
   const P2 a=ring[ia],b=ring[i],c=ring[ic];const double turn=cross(a,b,c);
   if(turn<0)continue;
   if(turn==0){ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;} // collinear: drop the vertex
   bool ear=true;
   for(std::size_t k=0;k<ring.size()&&ear;++k)if(k!=ia&&k!=i&&k!=ic&&inside(ring[k],a,b,c)&&!(ring[k].x==a.x&&ring[k].y==a.y)&&!(ring[k].x==c.x&&ring[k].y==c.y))ear=false;
   if(!ear)continue;
   // Resume next to the clipped ear: one pass per ear on typical rings instead of a rescan from vertex 0.
   out.push_back({a,b,c});ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;
  }
  if(!clipped)break;
 }
 if(ring.size()==3&&cross(ring[0],ring[1],ring[2])>0)out.push_back({ring[0],ring[1],ring[2]});
 return out;
}
// Convex polygon clipped to x >= v (keep = +1) or x <= v (keep = -1) on axis 0 (x) or 1 (y).
std::vector<P2> clip(const std::vector<P2>& poly,int axis,double v,int keep) {
 std::vector<P2> out;const auto d=[&](P2 p){return keep*((axis?p.y:p.x)-v);};
 for(std::size_t i=0;i<poly.size();++i) {
  const P2 a=poly[i],b=poly[(i+1)%poly.size()];const double da=d(a),db=d(b);
  if(da>=0)out.push_back(a);
  if((da<0)!=(db<0)){const double t=da/(da-db);P2 q{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};if(axis)q.y=v;else q.x=v;out.push_back(q);}
 }
 return out;
}
}
// Every junction this tile owns, as an asphalt patch over the road bed ring: ear clipping, then each triangle cut by the
// terrain lattice lines (DGM1 cell centres, x.5 m) so the patch bends exactly where the terrain does and neighbouring
// pieces share their vertices (no T-junctions). Heights come from `ground`; patches sit `lift` above it, a little above
// the ribbons, so the square ribbon ends under the junction disappear. No markings (flags 0, fade 0).
void add_junction_surfaces(RoadVisualMesh& mesh,const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadVisualGround& ground,double lift) {
 const double ox=key.min_easting(),oy=key.min_northing();
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  // The road bed ring: carriageway plus shoulders, the band the ribbons draw as asphalt (the paved ring adds sidewalks).
  const auto& v=j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices;if(v.size()<3)continue;
  std::vector<P2> ring;ring.reserve(v.size());
  for(const auto& q:v)ring.push_back({static_cast<double>(q.x)/1000.0-ox,static_cast<double>(q.y)/1000.0-oy});
  const auto first=mesh.vertices.size();const auto first_index=mesh.indices.size();bool valid=true;
  std::map<std::pair<std::int64_t,std::int64_t>,std::int32_t> shared; // vertex dedup on a 0.1 mm key
  const auto vertex=[&](P2 p)->std::optional<std::int32_t>{
   const std::pair<std::int64_t,std::int64_t> k{std::llround(p.x*1e4),std::llround(p.y*1e4)};
   if(auto it=shared.find(k);it!=shared.end())return it->second;
   const auto h=ground(p.x+ox,p.y+oy);if(!h)return {};
   const auto index=static_cast<std::int32_t>(mesh.vertices.size());shared.emplace(k,index);
   mesh.vertices.push_back({static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(*h+lift+kJunctionLiftM),0,0,1000.0f,0,0,0,0});
   return index;
  };
  for(const auto& t:triangulate(ring)) {
   const double x0=std::min({t[0].x,t[1].x,t[2].x}),x1=std::max({t[0].x,t[1].x,t[2].x});
   const double y0=std::min({t[0].y,t[1].y,t[2].y}),y1=std::max({t[0].y,t[1].y,t[2].y});
   for(double gx=std::floor(x0-.5)+.5;gx<x1&&valid;gx+=1)
    for(double gy=std::floor(y0-.5)+.5;gy<y1&&valid;gy+=1) {
     std::vector<P2> cell(t.begin(),t.end());
     cell=clip(cell,0,gx,1);cell=clip(cell,0,gx+1,-1);cell=clip(cell,1,gy,1);cell=clip(cell,1,gy+1,-1);
     if(cell.size()<3)continue;
     std::vector<std::int32_t> ids;
     for(const auto& p:cell){auto id=vertex(p);if(!id){valid=false;break;}if(ids.empty()||ids.back()!=*id)ids.push_back(*id);}
     if(!valid)break;
     if(ids.size()>1&&ids.front()==ids.back())ids.pop_back();
     for(std::size_t i=1;i+1<ids.size();++i)mesh.indices.insert(mesh.indices.end(),{ids[0],ids[i],ids[i+1]});
    }
   if(!valid)break;
  }
  if(!valid){mesh.vertices.resize(first);mesh.indices.resize(first_index);}
 }
}
namespace {
// Lane layout of one paved stretch as the ribbon draws it (the fragment shader reads lanes, forward and the flags).
struct LaneLayout { std::string highway,direction; bool one=false,fast=false; double width=0; int lanes=1,forward=1,flags=0; };
enum MarkFlag : int { kMarking=1, kEdgeLines=2, kSolidSeparator=4, kFast=8, kPaint=16, kOverlay=32, kLeftEdgeBroken=64, kRightEdgeBroken=128,
                     kLeftEdgeHidden=512, kRightEdgeHidden=1024, kStones=2048, kConcrete=4096 };
// Paving surface of a way (OSM surface=*) as the ribbon shader draws it: stones (paving stones, sett, cobblestone),
// concrete, else asphalt.
int surface_flags(const std::string& surface) {
 if(surface.starts_with("paving_stones")||surface=="sett"||surface=="cobblestone"||surface=="unhewn_cobblestone"||surface=="grass_paver")return kStones;
 if(surface.starts_with("concrete"))return kConcrete;
 return 0;
}
// Whether a side road opens the edge line of the road it joins. Not when it is closed to the public (access no,
// private, permissive, permit, ... - a works crossover between two ramps, an emergency gate). On a motorway or a
// motorway/trunk link a service road opens it only when it is a plain one (no driveway, parking aisle, ...) leaving or
// joining at a shallow angle (a rest-area ramp); one meeting it at a steep angle does not, the edge line runs on solid.
bool opens_edge_line(const g2m::RoadGraphWay& side,const std::string& through_highway,double angle) {
 // permissive (the owner tolerates traffic) counts as closed here: on the A 66 ramps it marks a works crossover.
 const auto restricted=[](const std::string& v){
  return v=="no"||v=="private"||v=="permissive"||v=="permit"||v=="agricultural"||v=="forestry"||v=="emergency"||v=="military"||v=="delivery";};
 for(const char* k:{"access","vehicle","motor_vehicle","motorcar"})if(restricted(tag(side,k)))return false;
 const bool fast=through_highway=="motorway"||through_highway=="motorway_link"||through_highway=="trunk_link";
 if(!fast||tag(side,"highway")!="service")return true;
 return tag(side,"service").empty()&&angle<35*3.14159265358979323846/180;
}
std::optional<LaneLayout> lane_layout(const g2m::RoadGeomEntry& entry,const g2m::RoadGraphWay& w) {
 if(!entry.profile)return {};const auto& p=*entry.profile;if(p.attributes.land_class!=g2m::LandClass::PavedRoad)return {};
 LaneLayout l;l.highway=tag(w,"highway");const auto& highway=l.highway;
 if(highway=="footway"||highway=="path"||highway=="cycleway"||highway=="steps"||highway=="pedestrian")return {};
 l.direction=tag(w,"oneway");const auto& direction=l.direction;
 l.one=direction=="yes"||direction=="1"||direction=="true"||direction=="-1"||(direction.empty()&&(highway=="motorway"||tag(w,"junction")=="roundabout"));
 l.fast=highway=="motorway"||highway=="trunk"||highway=="motorway_link"||highway=="trunk_link";
 const bool edge_lines=l.fast||highway=="primary"||highway=="secondary"||highway=="tertiary";
 l.width=p.attributes.width_mm/1000.0;
 int lanes=p.attributes.lanes;const int f=integer(tag(w,"lanes:forward")),b=integer(tag(w,"lanes:backward"));
 if(lanes==0&&f+b>0)lanes=f+b;
 if(lanes==0)lanes=l.one?std::clamp(static_cast<int>(std::round(l.width/3.25)),1,l.fast?4:2):(l.width>=5.5?2:1);
 l.lanes=std::clamp(lanes,1,30);l.forward=std::clamp(l.one?(direction=="-1"?0:l.lanes):(f>0?f:(b>0?l.lanes-b:(l.lanes+1)/2)),0,l.lanes);
 // The served cross-section (geo2map P1) knows the counted lanes, their directions and widths: it wins over the tags.
 if(entry.cross_section) {
  int counted=0,forward=0;double width=0;
  for(const auto& band:entry.cross_section->bands)if(band.counted){++counted;forward+=band.direction==g2m::LaneDirection::Forward;width+=band.width_mm/1000.0;}
  if(counted>0){l.lanes=std::min(counted,30);l.forward=std::clamp(forward,0,l.lanes);l.width=width;}
 }
 const bool marking=tag(w,"lane_markings")!="no"&&l.width/l.lanes>=2.4&&highway!="living_street";
 const int paving=surface_flags(tag(w,"surface"));
 // A paving-stone or cobbled road carries no painted lines.
 l.flags=paving==kStones?paving:((marking?kMarking:0)|(edge_lines?kEdgeLines:0)|(tag(w,"overtaking")=="no"?kSolidSeparator:0)|(l.fast?kFast:0)|paving);
 return l;
}
const g2m::RoadGraphWay* find_way(const g2m::RoadGeomTile& geometry,std::int64_t id) {
 auto it=std::lower_bound(geometry.source.graph.ways.begin(),geometry.source.graph.ways.end(),id,[](const auto& w,auto v){return w.osm_id<v;});
 return it==geometry.source.graph.ways.end()||it->osm_id!=id?nullptr:&*it;
}
struct Pose { double x,y,heading; };
// A point of the reference line, extended straight past either end (furniture can sit a little beyond its stretch).
std::optional<Pose> reference_pose(const g2m::RoadProfile& p,double station) {
 const double length=p.reference.length_m,clamped=std::clamp(station,0.0,length);
 auto xy=p.reference.at(clamped);if(!xy)return {};
 const double extra=station-clamped;return Pose{xy->x+std::cos(xy->heading)*extra,xy->y+std::sin(xy->heading)*extra,xy->heading};
}
// Lateral extents of a stretch at a station (reference frame, t positive left): the asphalt (the road bed: carriageway
// plus shoulders, the same band the junction road-bed ring and the grip raster use) and the counted lanes the markings
// divide. From the served cross-section; without one, the attribute width centred on the reference.
struct Span { double bed_left,bed_right,mark_left,mark_right; double centre() const {return (mark_left+mark_right)/2;} double width() const {return mark_left-mark_right;} };
Span span_at(const g2m::RoadGeomEntry& entry,const LaneLayout& layout,double station) {
 const double half=entry.profile->attributes.width_mm/2000.0;Span fallback{half,-half,half,-half};
 if(!entry.cross_section)return fallback;
 const auto& xs=*entry.cross_section;if(xs.bands.empty()||xs.bands.size()>32)return fallback;
 const double s=std::clamp(station,0.0,entry.profile->reference.length_m);
 const auto fp=g2m::carve_footprint(xs,s);
 std::array<double,33> edges{};if(!xs.boundaries_at(s,std::span<double>(edges.data(),xs.bands.size()+1)))return fallback;
 double left=-1e9,right=1e9;
 for(std::size_t i=0;i<xs.bands.size();++i)if(xs.bands[i].counted){left=std::max(left,edges[i]);right=std::min(right,edges[i+1]);}
 if(left<=right){left=fp.carriageway_left;right=fp.carriageway_right;}
 if(!(fp.roadbed_left>fp.roadbed_right)||!(left>right))return fallback;
 (void)layout;return {std::max(fp.roadbed_left,left),std::min(fp.roadbed_right,right),left,right};
}
// One approach of a served junction, seen from the junction: where its trim line cuts the road and how it is laid out.
struct Approach {
 std::size_t entry=0;bool at_end=false;double cut=0; // station of the trim line on the entry
 Pose pose{};double inward=0,width=0;                // the cut point (lane centre), heading INTO the junction, lanes' width
 Pose lead{};                                        // the lane centre 1 m outward of the cut: a through overlay starts here
 LaneLayout layout;int rank=0;bool roundabout=false;const g2m::RoadGraphWay* way=nullptr;
 int in_lanes=0;double in_from=0,in_to=0;            // lanes driving into the junction and their lateral span (reference frame)
};
double wrap(double a) {return std::remainder(a,2*3.14159265358979323846);}
// Whether (x, y) lies inside a junction ring (mm vertices) or within `tolerance` metres of its boundary.
bool near_ring(const std::vector<g2m::geom::PointMm>& ring,double x,double y,double tolerance) {
 bool inside=false;double best=1e18;
 for(std::size_t i=0,k=ring.size()-1;i<ring.size();k=i++) {
  const double ax=ring[k].x/1000.0,ay=ring[k].y/1000.0,bx=ring[i].x/1000.0,by=ring[i].y/1000.0;
  if((ay>y)!=(by>y)&&x<ax+(bx-ax)*(y-ay)/(by-ay))inside=!inside;
  const double dx=bx-ax,dy=by-ay,l2=dx*dx+dy*dy,t=l2>0?std::clamp(((x-ax)*dx+(y-ay)*dy)/l2,0.0,1.0):0;
  best=std::min(best,std::hypot(ax+t*dx-x,ay+t*dy-y));
 }
 return inside||best<=tolerance;
}
}
RoadVisualMesh build_road_visual(const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadVisualGround& ground,double lift) {
 RoadVisualMesh mesh;const auto decks=build_road_decks(geometry);
 // Deck height at a station of a separated stretch. The station is clamped into the deck so a ribbon row that lands
 // a rounding step past the deck end still finds it; a bridge without a deck falls back to its own profile.
 const auto deck_height=[&](const g2m::RoadGeomEntry& entry,double station,double lateral)->std::optional<double>{
  for(const auto& d:decks)if(d->way_id==entry.way_id&&station>=d->start_station-.05&&station<=d->end_station+.05)
   if(auto z=road_deck_height(*d,std::clamp(station,d->start_station,d->end_station),lateral))return z;
  const auto& p=*entry.profile;if(auto z=p.at(std::clamp(station,0.0,p.reference.length_m)))return z->height_m;
  return {};
 };
 const auto separated_at=[&](const g2m::RoadGeomEntry& entry,double station){
  return entry.profile->attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&station>=d->start_station&&station<=d->end_station;});
 };
 // Surface height of a stretch at (station, lateral): the deck on a structure, else the ground.
 const auto surface=[&](const g2m::RoadGeomEntry& entry,double station,double lateral,double e,double n)->std::optional<double>{
  return separated_at(entry,station)?deck_height(entry,station,lateral):ground(e,n);
 };
 const double ox=key.min_easting(),oy=key.min_northing();
 const auto owned=[&](double x,double y){return std::floor(x/1024)==key.x&&std::floor(y/1024)==key.y;};
 // Painted rectangle on a stretch (station s0..s1, lateral t0..t1, reference frame), split every metre across so it
 // follows the ground; `raise` above the ribbon surface.
 const auto paint_rect=[&](const g2m::RoadGeomEntry& entry,double s0,double s1,double t0,double t1,double raise) {
  const auto& p=*entry.profile;const int across=std::max(1,static_cast<int>(std::ceil(std::abs(t1-t0))));
  const auto first=static_cast<std::int32_t>(mesh.vertices.size());
  for(double station:{s0,s1})for(int j=0;j<=across;++j) {
   const double t=t0+(t1-t0)*j/across;auto pose=reference_pose(p,station);if(!pose){mesh.vertices.resize(first);return;}
   const double e=pose->x-std::sin(pose->heading)*t,n=pose->y+std::cos(pose->heading)*t;
   auto h=surface(entry,station,t,e,n);if(!h){mesh.vertices.resize(first);return;}
   mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*h+lift+raise),0,0,1000.0f,1,0,0,kPaint});
  }
  for(int j=0;j<across;++j){const auto a=first+j,b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
 };
 // ---- junction analysis: through roads, side roads, lines (owner tile only: geometry.junctions) ----
 struct LineKey { std::size_t entry;bool forward; auto operator<=>(const LineKey&) const = default; };
 std::set<LineKey> lined;                               // approaches that already carry a stop or give-way line
 std::set<std::size_t> signal_entries;                  // entries with a placed signal (a signalled junction)
 for(const auto& fu:geometry.furniture)if(fu.decline==g2m::FurnitureDecline::None){
  if(fu.kind==g2m::FurnitureKind::StopLine||fu.kind==g2m::FurnitureKind::GiveWayLine)lined.insert({fu.entry_index,fu.travel==g2m::LaneDirection::Forward});
  if(fu.kind==g2m::FurnitureKind::Signal)signal_entries.insert(fu.entry_index);
 }
 std::set<std::tuple<std::int64_t,std::uint32_t,bool>> continued; // approach ends whose markings run on over the junction
 struct Overlay { Approach a,b;int broken=0;const std::vector<g2m::geom::PointMm>* ring=nullptr; };
 std::vector<Overlay> overlays;
 struct GiveWay { std::size_t entry;double station,t0,t1; };
 std::vector<GiveWay> give_ways;
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  std::vector<Approach> list;bool signalled=false;
  for(const auto& ap:j.approaches) {
   if(ap.status!=g2m::JunctionApproachStatus::Used||ap.entry_index>=geometry.entries.size())continue;
   const auto& entry=geometry.entries[ap.entry_index];const auto* w=find_way(geometry,entry.way_id);if(!w||!entry.profile)continue;
   auto layout=lane_layout(entry,*w);if(!layout)continue;
   signalled=signalled||signal_entries.contains(ap.entry_index);
   Approach a;a.entry=ap.entry_index;a.at_end=ap.at_end;a.layout=*layout;a.rank=traffic_road_rank(*w);a.roundabout=tag(*w,"junction")=="roundabout";a.way=w;
   const double length=entry.profile->reference.length_m;a.cut=std::clamp(ap.at_end?length-ap.trim_m:ap.trim_m,0.0,length);
   auto pose=reference_pose(*entry.profile,a.cut);if(!pose)continue;
   // The cut point on the centre of the counted lanes (the marking frame), and the lanes toward the junction: the
   // way-direction lanes (right of the reference) at the stretch end, else the others.
   const auto span=span_at(entry,*layout,a.cut);const double lane=span.width()/layout->lanes;
   a.pose=*pose;a.pose.x-=std::sin(pose->heading)*span.centre();a.pose.y+=std::cos(pose->heading)*span.centre();
   a.width=span.width();a.inward=ap.at_end?pose->heading:pose->heading+3.14159265358979323846;
   {const double st=std::clamp(ap.at_end?a.cut-1:a.cut+1,0.0,length);auto lp=reference_pose(*entry.profile,st);if(!lp)continue;
    const auto ls=span_at(entry,*layout,st);a.lead={lp->x-std::sin(lp->heading)*ls.centre(),lp->y+std::cos(lp->heading)*ls.centre(),lp->heading};}
   a.in_lanes=ap.at_end?layout->forward:layout->lanes-layout->forward;
   if(ap.at_end){a.in_from=span.mark_right;a.in_to=span.mark_right+a.in_lanes*lane;}else{a.in_from=span.mark_left-a.in_lanes*lane;a.in_to=span.mark_left;}
   list.push_back(a);
  }
  if(list.size()<2)continue;
  const bool roundabout=std::any_of(list.begin(),list.end(),[](const auto& a){return a.roundabout;});
  // The through road: the pair of approaches that continue each other within 35 degrees, highest rank first.
  constexpr double kThrough=35*3.14159265358979323846/180;
  int best_i=-1,best_j=-1,best_rank=-1;double best_dev=1e9;
  for(std::size_t i=0;i<list.size();++i)for(std::size_t k=i+1;k<list.size();++k) {
   const double dev=std::abs(wrap(list[k].inward+3.14159265358979323846-list[i].inward));if(dev>kThrough)continue;
   const int rank=std::min(list[i].rank,list[k].rank);
   if(rank>best_rank||(rank==best_rank&&dev<best_dev)){best_rank=rank;best_dev=dev;best_i=static_cast<int>(i);best_j=static_cast<int>(k);}
  }
  bool through=best_i>=0&&!roundabout;
  if(through)for(std::size_t i=0;i<list.size();++i)for(std::size_t k=i+1;k<list.size();++k) {
   // A crossing road of the same rank (another straight pair sharing no approach): markings stop at the box.
   if(static_cast<int>(i)==best_i||static_cast<int>(i)==best_j||static_cast<int>(k)==best_i||static_cast<int>(k)==best_j)continue;
   if(std::abs(wrap(list[k].inward+3.14159265358979323846-list[i].inward))<=kThrough&&std::min(list[i].rank,list[k].rank)>=best_rank)through=false;
  }
  if(signalled)through=false;
  if(through) {
   // Orient the overlay with the traffic: from the approach that can be driven into the junction.
   Approach a=list[static_cast<std::size_t>(best_i)],b=list[static_cast<std::size_t>(best_j)];
   if(a.in_lanes==0&&b.in_lanes>0)std::swap(a,b);
   Overlay o{a,b,0,&j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices};
   const double dx=b.pose.x-a.pose.x,dy=b.pose.y-a.pose.y;
   for(std::size_t i=0;i<list.size();++i){if(static_cast<int>(i)==best_i||static_cast<int>(i)==best_j)continue;
    // The side road's angle to the through road's axis (0 = it leaves or joins parallel to it; a and b point in
    // opposite directions, so the smaller of the two deviations is the angle to the axis).
    const double angle=std::min(std::abs(wrap(list[i].inward-a.inward)),std::abs(wrap(list[i].inward-b.inward)));
    if(!opens_edge_line(*list[i].way,a.layout.highway,angle))continue;
    o.broken|=(dx*(list[i].pose.y-a.pose.y)-dy*(list[i].pose.x-a.pose.x))>0?kLeftEdgeBroken:kRightEdgeBroken;}
   overlays.push_back(o);
   for(const auto* x:{&a,&b}){const auto& e=geometry.entries[x->entry];continued.insert({e.way_id,e.stretch.start_ref,x->at_end});}
  }
  if(signalled)continue;
  // Give-way lines nobody tagged (StVO 8 with sign 205/215): at a roundabout entry, and on a lower-ranked side road
  // of a through road unless it merges at a shallow angle (a slip lane or an on-ramp: no line, it merges).
  for(std::size_t i=0;i<list.size();++i) {
   const auto& a=list[i];if(a.in_lanes==0||a.roundabout||a.rank==0)continue;
   const auto& entry=geometry.entries[a.entry];if(lined.contains({a.entry,a.at_end}))continue;
   bool yields=roundabout;
   if(!yields&&best_i>=0&&static_cast<int>(i)!=best_i&&static_cast<int>(i)!=best_j&&a.rank<best_rank) {
    yields=true;
    for(int t:{best_i,best_j})if(std::abs(wrap(list[static_cast<std::size_t>(t)].inward+3.14159265358979323846-a.inward))<25*3.14159265358979323846/180)yields=false;
   }
   if(!yields)continue;
   const double set_back=.25,station=a.at_end?a.cut-set_back:a.cut+set_back;
   give_ways.push_back({a.entry,station,a.in_from,a.in_to});(void)entry;
  }
 }
 // Trims of the junctions this tile owns, per (way, stretch start_ref, at_end): lane markings end at the trim line.
 std::map<std::tuple<std::int64_t,std::uint32_t,bool>,double> trims;
 for(const auto& j:geometry.junctions)if(j.decline==g2m::JunctionDecline::None)
  for(const auto& a:j.approaches)if(a.status==g2m::JunctionApproachStatus::Used&&a.entry_index<geometry.entries.size()){
   const auto& e=geometry.entries[a.entry_index];trims[{e.way_id,e.stretch.start_ref,a.at_end}]=a.trim_m;}
 std::map<std::int64_t,int> degree;for(const auto& edge:geometry.source.graph.edges){++degree[edge.from_node];++degree[edge.to_node];}
 for(const auto& entry:geometry.entries) {
  if(!entry.profile)continue;const auto& p=*entry.profile;
  const auto* way_ptr=find_way(geometry,entry.way_id);if(!way_ptr)continue;const auto& w=*way_ptr;
  const auto layout=lane_layout(entry,w);if(!layout)continue;
  const auto& direction=layout->direction;const bool one=layout->one;
  const int lanes=layout->lanes,forward=layout->forward,flags=layout->flags;
  const double length=p.reference.length_m;if(length<=.01)continue;
  const bool start_junction=p.stretch.start_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.start_ref]]>2;
  const bool end_junction=p.stretch.end_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.end_ref]]>2;
  // A junction served for this stretch end (owner tile only) gives the exact trim; elsewhere the 3-11 m ramp stays. A
  // through road's markings do not fade: the junction overlay continues them.
  const auto trim_of=[&](bool at_end)->std::optional<double>{auto t=trims.find({entry.way_id,entry.stretch.start_ref,at_end});if(t==trims.end())return {};return t->second;};
  const bool start_on=continued.contains({entry.way_id,entry.stretch.start_ref,false}),end_on=continued.contains({entry.way_id,entry.stretch.start_ref,true});
  const auto start_trim=trim_of(false),end_trim=trim_of(true);
  const int count=std::max(1,static_cast<int>(std::ceil(length))); // metre spacing matches the terrain detail.
  const double shoulder=.30;
  struct Row { double station,x,y,nx,ny,fade;Span span;bool separated; };
  for(int i=0;i<count;++i) {
   // The last row is exactly `length`: length*count/count can land one ulp past the profile end, which the profile
   // rejects - the whole last metre of the stretch was dropped and left a gap at every such stretch end.
   const double stations[2]={length*i/count,i+1==count?length:length*(i+1)/count};auto mid=p.reference.at((stations[0]+stations[1])*.5);if(!mid)continue;
   if(!owned(mid->x,mid->y))continue;
   std::array<Row,2> rows{};bool valid=true;
   for(int r=0;r<2;++r) {
    const double station=stations[r];auto xy=p.reference.at(station);if(!xy){valid=false;break;}
    double fade=1;
    if(start_on&&start_trim)fade=std::min(fade,std::clamp(station-*start_trim,0.0,1.0)); // the overlay covers this metre
    else if(start_trim)fade=std::min(fade,std::clamp((station-*start_trim)/2.0,0.0,1.0));
    else if(start_junction)fade=std::min(fade,std::clamp((station-3)/8.0,0.0,1.0));
    if(end_on&&end_trim)fade=std::min(fade,std::clamp(length-station-*end_trim,0.0,1.0));
    else if(end_trim)fade=std::min(fade,std::clamp((length-station-*end_trim)/2.0,0.0,1.0));
    else if(end_junction)fade=std::min(fade,std::clamp((length-station-3)/8.0,0.0,1.0));
    rows[static_cast<std::size_t>(r)]={station,xy->x,xy->y,-std::sin(xy->heading),std::cos(xy->heading),fade,span_at(entry,*layout,station),separated_at(entry,station)};
   }
   if(!valid)continue;
   // Three strips across: the right verge (gravel, ground only), the road bed (asphalt, the markings divide its
   // counted lanes) and the left verge. u is measured from the centre of the counted lanes.
   const bool on_ground=!rows[0].separated&&!rows[1].separated;
   for(int strip=on_ground?0:1;strip<(on_ground?3:2);++strip) {
    const auto lo=[&](const Row& r){return strip==0?r.span.bed_right-shoulder:strip==1?r.span.bed_right:r.span.bed_left;};
    const auto hi=[&](const Row& r){return strip==0?r.span.bed_right:strip==1?r.span.bed_left:r.span.bed_left+shoulder;};
    const int across=strip==1?std::max(2,static_cast<int>(std::ceil(hi(rows[0])-lo(rows[0])))):1;
    const auto first=mesh.vertices.size();bool ok=true;
    for(const auto& r:rows) {
     for(int j=0;j<=across&&ok;++j) {
      const double lateral=lo(r)+(hi(r)-lo(r))*j/across,e=r.x+r.nx*lateral,n=r.y+r.ny*lateral;
      const auto height=r.separated?deck_height(entry,r.station,lateral):ground(e,n);
      if(!height){ok=false;break;}
      mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift),static_cast<float>(lateral-r.span.centre()),static_cast<float>(r.station),static_cast<float>(r.span.width()),static_cast<float>(r.fade),static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags),strip==1?0.0f:1.0f});
     }
    }
    if(!ok){mesh.vertices.resize(first);continue;}
    for(int j=0;j<across;++j){const auto a=static_cast<std::int32_t>(first+j),b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
   }
  }
  // Direction arrows only where OSM explicitly records turn:lanes.
  const auto arrows=[&](std::string turns,int direction,int lane_start,int lane_count) {
   if(turns.empty()||lane_count<=0||length<20)return;
   std::vector<std::string> tokens;std::size_t begin=0;
   while(true){auto end=turns.find('|',begin);tokens.push_back(turns.substr(begin,end-begin));if(end==std::string::npos)break;begin=end+1;}
   if(static_cast<int>(tokens.size())!=lane_count)return;
   // The arrow (tail 2 m behind the station, tip 1.7 m ahead) stands in front of the first marking the traffic meets at
   // the end it drives toward: the junction trim, a stop or give-way line of this direction, a crossing. 15 m before
   // the end unless one of those reaches further back; then its tip ends kArrowGapM before it.
   constexpr double kArrowGapM=3.0,kArrowTipM=1.7,kArrowTailM=2.0,kArrowReachM=60.0;
   const std::size_t index=static_cast<std::size_t>(&entry-geometry.entries.data());
   double barrier=direction>0?length:0.0;
   const auto bar=[&](double s0,double s1){
    if(direction>0){if(s0>length-kArrowReachM)barrier=std::min(barrier,s0);}
    else if(s1<kArrowReachM)barrier=std::max(barrier,s1);
   };
   if(auto it=trims.find({entry.way_id,entry.stretch.start_ref,direction>0});it!=trims.end())
    bar(direction>0?length-it->second:0.0,direction>0?length:it->second);
   const bool travel_forward=direction>0;
   for(const auto& fu:geometry.furniture) {
    if(fu.decline!=g2m::FurnitureDecline::None||fu.entry_index!=index||fu.kind==g2m::FurnitureKind::Signal)continue;
    const bool line=fu.kind==g2m::FurnitureKind::StopLine||fu.kind==g2m::FurnitureKind::GiveWayLine;
    if(line&&(fu.travel==g2m::LaneDirection::Forward)!=travel_forward)continue;
    bar(fu.station_m-fu.length_m/2,fu.station_m+fu.length_m/2);
   }
   for(const auto& g:give_ways)if(g.entry==index)bar(g.station-.25,g.station+.25);
   const double station=direction>0?std::min(length-15,barrier-kArrowGapM-kArrowTipM):std::max(15.0,barrier+kArrowGapM+kArrowTipM);
   if(direction>0?station-kArrowTailM<0:station+kArrowTailM>length)return;
   auto center=p.reference.at(station);if(!center||!owned(center->x,center->y))return;
   const auto span=span_at(entry,*layout,station);const double width=span.width(),half=width/2,centre=span.centre();
   const double lane_width=width/lanes;if(lane_width<2.4)return;
   for(int lane=0;lane<lane_count;++lane){
    const double offset=direction>0?-half+(lane_start+lane_count-lane-.5)*lane_width:-half+(lane_start+lane+.5)*lane_width;
    const auto vertex=[&](double side,double along)->std::optional<RoadVisualVertex>{
     const double st=station+direction*along;auto xy=p.reference.at(st);if(!xy)return {};
     const double lateral=centre+offset+direction*side,e=xy->x-std::sin(xy->heading)*lateral,n=xy->y+std::cos(xy->heading)*lateral;
     auto height=surface(entry,st,lateral,e,n);if(!height)return {};
     return RoadVisualVertex{static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+.003),0,0,static_cast<float>(width),1,0,0,kPaint};
    };
    const auto triangle=[&](double ax,double ay,double bx,double by,double cx,double cy){
     auto a=vertex(ax,ay),b=vertex(bx,by),c=vertex(cx,cy);if(!a||!b||!c)return;const auto first=static_cast<std::int32_t>(mesh.vertices.size());
     mesh.vertices.insert(mesh.vertices.end(),{*a,*b,*c});mesh.indices.insert(mesh.indices.end(),{first,first+1,first+2});
    };
    const auto stroke=[&](double ax,double ay,double bx,double by){const double dx=bx-ax,dy=by-ay,len=std::hypot(dx,dy);if(len<.01)return;const double nx=-dy/len*.11,ny=dx/len*.11;triangle(ax+nx,ay+ny,ax-nx,ay-ny,bx+nx,by+ny);triangle(ax-nx,ay-ny,bx-nx,by-ny,bx+nx,by+ny);};
    const auto& token=tokens[lane];
    const bool through=token.find("through")!=std::string::npos;
    const bool left=token.find("left")!=std::string::npos,right=token.find("right")!=std::string::npos;
    if(!through&&!left&&!right)continue;
    stroke(0,-2,0,0);
    if(through){stroke(0,0,0,1.1);triangle(-.45,.85,.45,.85,0,1.7);}
    if(left){stroke(0,0,.7,.5);triangle(.65,.15,.65,.85,1.15,.5);}
    if(right){stroke(0,0,-.7,.5);triangle(-.65,.15,-.65,.85,-1.15,.5);}
   }
  };
  if(one)arrows(tag(w,"turn:lanes"),direction=="-1"?-1:1,0,lanes);
  else {arrows(tag(w,"turn:lanes:forward"),1,0,forward);arrows(tag(w,"turn:lanes:backward"),-1,forward,lanes-forward);}
 }
 add_junction_surfaces(mesh,geometry,key,ground,lift);
 // ---- markings over the junction patches ----
 constexpr double kOverlayLiftM=kJunctionLiftM+.003,kLineLiftM=kJunctionLiftM+.004;
 // The through road's lane, centre and edge lines continue across the junction along a cubic between the two trim
 // cuts; the edge line is broken across each side road's mouth (a broad dash on a motorway or trunk: the exit or the
 // merge). Only the paint is drawn (kOverlay: the shader discards the rest), so the junction patch shows between.
 for(const auto& o:overlays) {
  const auto& a=o.a;const auto& b=o.b;
  const double ax=std::cos(a.inward),ay=std::sin(a.inward),bx=-std::cos(b.inward),by=-std::sin(b.inward);
  const double chord=std::hypot(b.lead.x-a.lead.x,b.lead.y-a.lead.y);if(chord<.5||chord>200)continue;
  if(!owned((a.pose.x+b.pose.x)/2,(a.pose.y+b.pose.y)/2))continue;
  const double h=chord/3;
  const double p0x=a.lead.x,p0y=a.lead.y,p1x=p0x+ax*h,p1y=p0y+ay*h,p3x=b.lead.x,p3y=b.lead.y,p2x=p3x-bx*h,p2y=p3y-by*h;
  // Lane layout in the a -> b direction: lanes on the right are the ones driving into the junction from a.
  int lanes=a.layout.lanes,forward=a.in_lanes;
  const int b_lanes=b.layout.lanes,b_right=b.layout.lanes-b.in_lanes;
  double wa=a.width,wb=b.width,oa=0,ob=0; // strip width and its centre offset (left +) at either end
  if(lanes!=b_lanes||forward!=b_right) {
   const bool two_way=forward>0&&forward<lanes&&b_right>0&&b_right<b_lanes;
   if(two_way){lanes=2;forward=1;}
   else if(forward==lanes&&b_right==b_lanes) {
    // A one-way road gaining or losing a lane at an exit or a merge: the through lanes run on, flush with the side
    // away from the ramp; their broken edge line is the line between them and the exit or merge lane.
    const int keep=std::min(lanes,b_lanes);const double lane=keep==lanes?a.width/lanes:b.width/b_lanes,w=keep*lane;
    const int side=o.broken==kRightEdgeBroken?1:o.broken==kLeftEdgeBroken?-1:0;
    oa=side*(a.width-w)/2;ob=side*(b.width-w)/2;wa=wb=w;lanes=forward=keep;
   }
   else {lanes=1;forward=1;}
  }
  int flags=(a.layout.flags&b.layout.flags&(kMarking|kEdgeLines))|(a.layout.flags&(kSolidSeparator|kFast))|kOverlay|o.broken;
  if(!(flags&kMarking))continue;
  const int count=std::max(2,static_cast<int>(std::ceil(chord)));
  struct Row { double x,y,nx,ny,arc,width,offset; };
  std::vector<Row> rows;double arc=0,px=p0x,py=p0y;
  for(int i=0;i<=count;++i) {
   const double t=static_cast<double>(i)/count,u=1-t;
   const double x=u*u*u*p0x+3*u*u*t*p1x+3*u*t*t*p2x+t*t*t*p3x,y=u*u*u*p0y+3*u*u*t*p1y+3*u*t*t*p2y+t*t*t*p3y;
   const double dx=3*u*u*(p1x-p0x)+6*u*t*(p2x-p1x)+3*t*t*(p3x-p2x),dy=3*u*u*(p1y-p0y)+6*u*t*(p2y-p1y)+3*t*t*(p3y-p2y);
   const double len=std::max(std::hypot(dx,dy),1e-9);
   arc+=std::hypot(x-px,y-py);px=x;py=y;
   rows.push_back({x,y,-dy/len,dx/len,arc,wa+(wb-wa)*t,oa+(ob-oa)*t});
  }
  // An edge line that would leave the junction's road bed (the ring is narrower there, or a kerb return cuts the
  // corner) is not drawn: a line over the verge reads as wrong as a missing one.
  for(const auto& r:rows) {
   if(!o.ring||o.ring->size()<3||r.arc<1.5||r.arc>arc-1.5)continue; // the lead metre at either end lies on the ribbon
   for(int side:{1,-1}) {
    const double l=r.offset+side*(r.width/2-.25);
    if(!near_ring(*o.ring,r.x+r.nx*l,r.y+r.ny*l,.3))flags|=side>0?kLeftEdgeHidden:kRightEdgeHidden;
   }
  }
  const auto first=static_cast<std::int32_t>(mesh.vertices.size());bool valid=true;int across=0;
  for(const auto& r:rows) {
   if(!valid)break;
   const double x=r.x,y=r.y,nx=r.nx,ny=r.ny,width=r.width,half=width/2,offset=r.offset;
   across=std::max(2,static_cast<int>(std::ceil(std::max(wa,wb))));
   for(int j=0;j<=across;++j) {
    const double lateral=-half+width*j/across,e=x+nx*(offset+lateral),n=y+ny*(offset+lateral);
    const auto height=ground(e,n);if(!height){valid=false;break;}
    mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+kOverlayLiftM),static_cast<float>(lateral),static_cast<float>(r.arc),static_cast<float>(width),1,static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags)});
   }
  }
  if(!valid){mesh.vertices.resize(static_cast<std::size_t>(first));continue;}
  for(int i=0;i<count;++i)for(int j=0;j<across;++j){const auto c=first+i*(across+1)+j,d=c+across+1;mesh.indices.insert(mesh.indices.end(),{c,d,c+1,c+1,d,d+1});}
 }
 // Stop, give-way and crossing markings: OSM-placed furniture (geo2map) and the give-way lines implied above.
 const auto in_tile=[&](const g2m::RoadGeomEntry& entry,double station){auto pose=reference_pose(*entry.profile,station);return pose&&owned(pose->x,pose->y);};
 const auto give_way_line=[&](const g2m::RoadGeomEntry& entry,double station,double depth,double t0,double t1) {
  // Wartelinie (sign 341): 0.50 m dashes with 0.25 m gaps across the lanes, starting at the centre-line end.
  const double lo=std::min(t0,t1),hi=std::max(t0,t1);
  for(double t=lo;t<hi-.05;t+=.75)paint_rect(entry,station-depth/2,station+depth/2,t,std::min(t+.5,hi),kLineLiftM);
 };
 for(const auto& g:give_ways){const auto& entry=geometry.entries[g.entry];if(in_tile(entry,g.station))give_way_line(entry,g.station,.5,g.t0,g.t1);}
 for(const auto& fu:geometry.furniture) {
  if(fu.decline!=g2m::FurnitureDecline::None||fu.kind==g2m::FurnitureKind::Signal||fu.entry_index>=geometry.entries.size())continue;
  const auto& entry=geometry.entries[fu.entry_index];if(!entry.profile||!in_tile(entry,fu.station_m))continue;
  const double s0=fu.station_m-fu.length_m/2,s1=fu.station_m+fu.length_m/2,lo=std::min(fu.t_from,fu.t_to),hi=std::max(fu.t_from,fu.t_to);
  if(fu.kind==g2m::FurnitureKind::StopLine){paint_rect(entry,s0,s1,lo,hi,kLineLiftM);continue;}
  if(fu.kind==g2m::FurnitureKind::GiveWayLine){give_way_line(entry,fu.station_m,fu.length_m,lo,hi);continue;}
  // Crossings. Zebra (sign 293): 0.50 m bars along the road with 0.50 m gaps; a ladder adds the two side lines.
  // A signalled crossing without a recorded pattern, or "dashes": the Furt, two broken 0.12 m lines (0.50 / 0.20 m).
  auto pattern=fu.pattern;
  if(pattern==g2m::CrossingPattern::None&&(fu.flags&g2m::kFurnitureSignalised))pattern=g2m::CrossingPattern::Dashes;
  if(pattern==g2m::CrossingPattern::Zebra||pattern==g2m::CrossingPattern::Ladder) {
   const double span=hi-lo;const int bars=std::max(1,static_cast<int>(std::floor((span+.5)/1.0)));
   const double start=lo+(span-(bars*1.0-.5))/2;
   for(int k=0;k<bars;++k)paint_rect(entry,s0,s1,start+k,start+k+.5,kLineLiftM);
  }
  if(pattern==g2m::CrossingPattern::Ladder||pattern==g2m::CrossingPattern::Lines)
   for(double s:{s0,s1-.12})paint_rect(entry,s,s+.12,lo,hi,kLineLiftM);
  if(pattern==g2m::CrossingPattern::Dashes)
   for(double s:{s0,s1-.12})for(double t=lo;t<hi-.05;t+=.7)paint_rect(entry,s,s+.12,t,std::min(t+.5,hi),kLineLiftM);
 }
 return mesh;
}
}
