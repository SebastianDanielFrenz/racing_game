// tools/rg_route_gen - generates the G3/R3 acceptance route (docs/g3_consumer_design.md 6.1, slice S3)
// from the real road graph and writes it as "rg.route/1" JSON.
//
//   rg_route_gen <gen-config.json> [--out route.json] [--world-config PATH]   generate (default out: stdout)
//   rg_route_gen --explore <gen-config.json>                                    network summary + step diagnostics
//   rg_route_gen --bridges x0 x1 y0 y1 [--world-config PATH]                    list bridge runs in a box
//
// The generator config ("rg.route_gen/1", data/routes/*.gen.json) names the spawn, the world box to load,
// the smoothing, and an ordered list of steps. A step either TRAVERSES an OSM way in one direction
// (every arc of it, in travel order - used for the bridges) or goes VIA an OSM node (the cheapest arc
// arriving there). Between steps the route follows the shortest path over directed arcs (oneway honoured,
// no U-turns). Everything is deterministic: ordered containers, fixed-format output - the same config and
// data pin regenerate the same bytes. See core/include/rg/road_network.h for the network model.
//
// Junctions are CLUSTERS of OSM nodes with >= 3 drivable neighbours (the roads.graph layer of the pinned
// geo2map has no junction records); the output stores each cluster's OSM node ids so a later pin bump can
// re-resolve them (docs/g3_consumer_design.md, "As built (S3)").
#include "rg/road_network.h"
#include "rg/route_check.h"
#include "rg/world_config.h"
#include "rg/world_terrain.h"

#include "g2m/core/tile_key.h"
#include "ps/math/transcendental.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using json = nlohmann::json;

struct Step {
    bool traverse = true;
    std::int64_t way = 0;
    int direction = 1;
    std::string group;
    std::int64_t node = 0;
};

struct GenConfig {
    std::string name;
    std::string description;
    double spawn_x = 0, spawn_y = 0, spawn_yaw_deg = 0;
    double box[4] = {0, 0, 0, 0}; // x0 x1 y0 y1, session metres
    std::string geo2map_pin;
    double smooth_radius_m = 10.0, max_spacing_m = 5.0;
    int min_junctions = 20;
    json criteria = json::object();
    std::vector<Step> steps;
};

bool read_config(const std::string& path, GenConfig* c, std::string* err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        *err = path + ": cannot open";
        return false;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    const json j = json::parse(buf.str(), nullptr, false);
    if (j.is_discarded() || !j.is_object() || j.value("format", "") != "rg.route_gen/1") {
        *err = path + ": not an rg.route_gen/1 object";
        return false;
    }
    c->name = j.value("name", "");
    c->description = j.value("description", "");
    c->geo2map_pin = j.value("geo2map_pin", "");
    const json& sp = j.at("spawn");
    c->spawn_x = sp.at("x").get<double>();
    c->spawn_y = sp.at("y").get<double>();
    c->spawn_yaw_deg = sp.at("yaw_deg").get<double>();
    const json& bx = j.at("load_box_m");
    for (int i = 0; i < 4; ++i) c->box[i] = bx.at(static_cast<std::size_t>(i)).get<double>();
    if (j.contains("smoothing")) {
        c->smooth_radius_m = j.at("smoothing").value("radius_m", 10.0);
        c->max_spacing_m = j.at("smoothing").value("max_spacing_m", 5.0);
    }
    c->min_junctions = j.value("min_junctions", 20);
    if (j.contains("criteria")) c->criteria = j.at("criteria");
    for (const json& s : j.at("steps")) {
        Step st;
        if (s.contains("traverse")) {
            const json& t = s.at("traverse");
            st.traverse = true;
            st.way = t.at("way").get<std::int64_t>();
            st.direction = t.at("direction").get<int>();
            st.group = t.value("group", "");
        } else {
            st.traverse = false;
            st.node = s.at("via_node").get<std::int64_t>();
        }
        c->steps.push_back(st);
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::string world_path = std::string(RG_SOURCE_DIR) + "/data/world/world_config.json";
    std::string config_path, out_path;
    bool explore = false, bridges_mode = false;
    double bbox[4] = {0, 0, 0, 0};
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (a == "--world-config" && i + 1 < argc) {
            world_path = argv[++i];
        } else if (a == "--explore") {
            explore = true;
        } else if (a == "--bridges" && i + 4 < argc) {
            bridges_mode = true;
            for (int k = 0; k < 4; ++k) bbox[k] = std::atof(argv[++i]);
        } else if (a[0] != '-') {
            config_path = a;
        } else {
            std::fprintf(stderr, "unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    GenConfig cfg;
    std::string err;
    if (!bridges_mode) {
        if (config_path.empty() || !read_config(config_path, &cfg, &err)) {
            std::fprintf(stderr, "%s\n",
                         err.empty() ? "usage: rg_route_gen <gen-config.json> [--out F] | --explore <cfg> | --bridges x0 x1 y0 y1"
                                     : err.c_str());
            return 2;
        }
    } else {
        for (int k = 0; k < 4; ++k) cfg.box[k] = bbox[k];
    }
    auto world = rg::load_world_config(world_path, &err);
    if (!world) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 2;
    }
    auto terrain = rg::WorldTerrain::open(*world, &err);
    if (!terrain) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 2;
    }
    const double e0 = world->session_origin_utm.e0, n0 = world->session_origin_utm.n0;

    rg::RoadNetwork net;
    int tiles = 0;
    for (double y = cfg.box[2]; y <= cfg.box[3] + 1024; y += 1024) {
        for (double x = cfg.box[0]; x <= cfg.box[1] + 1024; x += 1024) {
            auto key = g2m::tile_key_at(terrain->frame().zone(), 2, e0 + x, n0 + y);
            if (!key) continue;
            auto t = terrain->road_graph_tile(*key);
            if (!t) {
                std::fprintf(stderr, "road graph tile %s failed to load\n", g2m::to_string(*key).c_str());
                return 2;
            }
            net.add_road_graph_tile(*t, e0, n0);
            ++tiles;
        }
    }
    net.finish();
    const auto st = net.stats();
    std::fprintf(stderr, "network: %d tiles, %zu nodes, %zu drivable ways, %zu arcs, %zu junction nodes in %zu clusters\n",
                 tiles, st.nodes, st.ways, st.arcs, st.junction_nodes, st.junctions);

    auto pos = [&](std::int64_t id) {
        const auto& n = net.nodes()[static_cast<std::size_t>(net.node_index(id))];
        return std::make_pair(n.x, n.y);
    };

    if (bridges_mode) {
        for (const auto& r : net.bridge_runs()) {
            const auto a = pos(r.enter_node), b = pos(r.exit_node);
            const auto& w = net.ways()[static_cast<std::size_t>(net.way_index(r.way_id))];
            std::string ref, nm, hw;
            for (auto& [k, v] : w.tags) {
                if (k == "ref") ref = v;
                if (k == "name") nm = v;
                if (k == "highway") hw = v;
            }
            std::printf("bridge way=%lld dir=%d hw=%s ref=%s name=%s len=%.1f enter=%lld(%.1f,%.1f) exit=%lld(%.1f,%.1f) jn_in=%d jn_out=%d\n",
                        (long long)r.way_id, r.direction, hw.c_str(), ref.c_str(), nm.c_str(), r.length_m,
                        (long long)r.enter_node, a.first, a.second, (long long)r.exit_node, b.first, b.second,
                        net.junction_of_node(net.node_index(r.enter_node)), net.junction_of_node(net.node_index(r.exit_node)));
        }
        return 0;
    }

    // ---- plan ----
    double hy = 0.0, hx = 1.0;
    ps::math::sincos(cfg.spawn_yaw_deg * 3.14159265358979323846 / 180.0, hy, hx);
    std::vector<rg::RouteStep> rsteps;
    std::map<std::int64_t, std::string> group_of_way;
    for (const Step& s : cfg.steps) {
        rsteps.push_back({s.traverse ? rg::RouteStep::Kind::TraverseWay : rg::RouteStep::Kind::ViaNode,
                          s.traverse ? s.way : s.node, s.direction});
        if (s.traverse) group_of_way[s.way] = s.group;
    }
    const auto planned = rg::plan_path(net, cfg.spawn_x, cfg.spawn_y, hx, hy, rsteps, &err);
    if (!planned) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    for (std::size_t i = 0; i < planned->step_approach_m.size(); ++i) {
        std::fprintf(stderr, "step %zu: approach %.0f m\n", i, planned->step_approach_m[i]);
    }
    const std::vector<int>& path = planned->arcs;
    const double t0 = planned->start_t, d0 = planned->start_dist_m;

    // ---- polyline ----
    std::vector<std::pair<double, double>> raw;
    std::vector<double> raw_cum; // cumulative raw length at each raw point
    auto push = [&](double x, double y) {
        if (!raw.empty() && std::hypot(x - raw.back().first, y - raw.back().second) < 1e-6) return;
        raw_cum.push_back(raw.empty() ? 0.0 : raw_cum.back() + std::hypot(x - raw.back().first, y - raw.back().second));
        raw.push_back({x, y});
    };
    push(cfg.spawn_x, cfg.spawn_y);
    {
        const auto& a = net.arcs()[static_cast<std::size_t>(path.front())];
        const auto& p = net.nodes()[static_cast<std::size_t>(a.from)];
        const auto& q = net.nodes()[static_cast<std::size_t>(a.to)];
        if (d0 > 0.3) push(p.x + t0 * (q.x - p.x), p.y + t0 * (q.y - p.y));
    }
    std::vector<std::size_t> arc_end_raw(path.size()); // raw index after pushing each arc's `to` node
    for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& q = net.nodes()[static_cast<std::size_t>(net.arcs()[static_cast<std::size_t>(path[i])].to)];
        push(q.x, q.y);
        arc_end_raw[i] = raw.size() - 1;
    }
    const auto wps = rg::smooth_polyline(raw, cfg.smooth_radius_m, cfg.max_spacing_m);
    std::vector<double> cum(wps.size(), 0.0);
    for (std::size_t i = 1; i < wps.size(); ++i) {
        cum[i] = cum[i - 1] + std::hypot(wps[i].first - wps[i - 1].first, wps[i].second - wps[i - 1].second);
    }
    const double total = cum.back(), raw_total = raw_cum.back();
    const double ratio = raw_total > 0 ? total / raw_total : 1.0;
    // s of a route location: the waypoint nearest to (x, y) within +-100 m of the expected arc length.
    auto s_of = [&](double raw_s, double x, double y) {
        const double exp_s = raw_s * ratio;
        double best = 1e30, best_s = exp_s;
        for (std::size_t i = 0; i < wps.size(); ++i) {
            if (std::fabs(cum[i] - exp_s) > 100.0) continue;
            const double d = std::hypot(wps[i].first - x, wps[i].second - y);
            if (d < best) {
                best = d;
                best_s = cum[i];
            }
        }
        return best_s;
    };

    // ---- junctions in visit order (first approach) ----
    struct JRec {
        int idx;
        double s;
    };
    std::vector<JRec> jrecs;
    {
        std::vector<char> seen(net.junctions().size(), 0);
        auto visit = [&](int node, std::size_t raw_index) {
            const int j = net.junction_of_node(node);
            if (j < 0 || seen[static_cast<std::size_t>(j)]) return;
            seen[static_cast<std::size_t>(j)] = 1;
            const auto& jn = net.junctions()[static_cast<std::size_t>(j)];
            jrecs.push_back({j, s_of(raw_cum[raw_index], jn.x, jn.y)});
        };
        for (std::size_t i = 0; i < path.size(); ++i) visit(net.arcs()[static_cast<std::size_t>(path[i])].to, arc_end_raw[i]);
    }

    // ---- bridge crossings: every maximal run of bridge arcs of one way and direction on the path ----
    std::vector<rg::RouteBridge> bridges;
    for (std::size_t i = 0; i < path.size();) {
        const auto& a0 = net.arcs()[static_cast<std::size_t>(path[i])];
        if (!a0.bridge) {
            ++i;
            continue;
        }
        std::size_t j = i;
        double len = 0.0;
        while (j < path.size()) {
            const auto& aj = net.arcs()[static_cast<std::size_t>(path[j])];
            if (!aj.bridge || aj.way_id != a0.way_id || aj.direction != a0.direction) break;
            if (j > i && std::abs(aj.segment - net.arcs()[static_cast<std::size_t>(path[j - 1])].segment) != 1) break;
            len += aj.length;
            ++j;
        }
        const auto& a1 = net.arcs()[static_cast<std::size_t>(path[j - 1])];
        rg::RouteBridge b;
        const auto g = group_of_way.find(a0.way_id);
        b.group = g != group_of_way.end() ? g->second : "way " + std::to_string(a0.way_id);
        b.way_id = a0.way_id;
        b.direction = a0.direction;
        b.enter_node = net.nodes()[static_cast<std::size_t>(a0.from)].osm_id;
        b.exit_node = net.nodes()[static_cast<std::size_t>(a1.to)].osm_id;
        const auto pa = pos(b.enter_node), pb = pos(b.exit_node);
        b.enter_x = pa.first;
        b.enter_y = pa.second;
        b.exit_x = pb.first;
        b.exit_y = pb.second;
        b.length_m = len;
        const std::size_t enter_raw = i == 0 ? 0 : arc_end_raw[i - 1];
        b.s_enter_m = s_of(raw_cum[enter_raw], pa.first, pa.second);
        b.s_exit_m = s_of(raw_cum[arc_end_raw[j - 1]], pb.first, pb.second);
        bridges.push_back(b);
        i = j;
    }

    // ---- verification by waypoint recount ----
    std::vector<rg::RoutePoint> rp;
    for (const auto& p : wps) rp.push_back({p.first, p.second});
    std::vector<rg::RouteJunction> js;
    for (const JRec& r : jrecs) {
        const auto& jn = net.junctions()[static_cast<std::size_t>(r.idx)];
        rg::RouteJunction j;
        j.id = "n" + std::to_string(jn.node_ids.front());
        j.node_ids = jn.node_ids;
        j.x = jn.x;
        j.y = jn.y;
        j.radius_m = jn.radius_m + 8.0;
        j.s_m = r.s;
        js.push_back(std::move(j));
    }
    const int crossed = rg::count_junctions_crossed(rp, js);
    int bridges_ok = 0;
    for (const auto& b : bridges) bridges_ok += rg::route_crosses_bridge(rp, b) ? 1 : 0;
    std::fprintf(stderr,
                 "route: %zu waypoints, %.1f m (raw %.1f m), %zu junction clusters on the arc path, %d confirmed by waypoint recount, "
                 "%zu bridge crossings (%d confirmed)\n",
                 wps.size(), total, raw_total, js.size(), crossed, bridges.size(), bridges_ok);
    if (explore) {
        for (const auto& j : js) {
            std::fprintf(stderr, "  junction %s at (%.1f, %.1f) r=%.1f s=%.0f nodes=%zu\n", j.id.c_str(), j.x, j.y, j.radius_m, j.s_m, j.node_ids.size());
        }
        for (const auto& b : bridges) {
            std::fprintf(stderr, "  bridge %s way %lld dir %d len %.1f s %.0f..%.0f\n", b.group.c_str(), (long long)b.way_id, b.direction, b.length_m, b.s_enter_m, b.s_exit_m);
        }
        return 0;
    }
    if (crossed != static_cast<int>(js.size()) || bridges_ok != static_cast<int>(bridges.size())) {
        std::fprintf(stderr, "internal check failed: the waypoint recount disagrees with the arc path\n");
        return 1;
    }
    if (static_cast<int>(js.size()) < cfg.min_junctions) {
        std::fprintf(stderr, "only %zu junction clusters, need %d: add steps to the config\n", js.size(), cfg.min_junctions);
        return 1;
    }

    // ---- write (fixed key order and number formats: byte-stable) ----
    std::ostringstream o;
    char b[512];
    auto f = [&](const char* fmt, auto... args) {
        std::snprintf(b, sizeof b, fmt, args...);
        o << b;
    };
    o << "{\n  \"format\": \"rg.route/1\",\n";
    o << "  \"name\": " << json(cfg.name).dump() << ",\n";
    o << "  \"source\": " << json(cfg.description).dump() << ",\n";
    f("  \"session_origin_utm\": {\"zone\": %d, \"e0\": %.0f, \"n0\": %.0f},\n", world->session_origin_utm.zone, e0, n0);
    f("  \"spawn\": {\"x\": %.2f, \"y\": %.2f, \"yaw_deg\": %.1f},\n", cfg.spawn_x, cfg.spawn_y, cfg.spawn_yaw_deg);
    o << "  \"criteria\": {";
    {
        bool first = true;
        json crit = cfg.criteria;
        crit["min_junctions"] = cfg.min_junctions;
        for (auto it = crit.begin(); it != crit.end(); ++it) {
            o << (first ? "" : ", ") << json(it.key()).dump() << ": " << it.value().dump();
            first = false;
        }
    }
    o << "},\n";
    const std::size_t slash = config_path.find_last_of("/\\");
    const std::string config_name = slash == std::string::npos ? config_path : config_path.substr(slash + 1);
    o << "  \"generator\": {\"tool\": \"rg_route_gen\", \"version\": 1, \"config\": " << json(config_name).dump()
      << ", \"geo2map_pin\": " << json(cfg.geo2map_pin).dump();
    f(", \"drivable_classes_version\": %d, \"junction_cluster_m\": %.1f},\n", rg::kDrivableClassesVersion, rg::kJunctionClusterM);
    o << "  \"junctions\": [\n";
    for (std::size_t i = 0; i < js.size(); ++i) {
        const auto& j = js[i];
        f("    {\"id\": \"%s\", \"x\": %.2f, \"y\": %.2f, \"radius_m\": %.2f, \"s_m\": %.1f, \"node_ids\": [", j.id.c_str(), j.x, j.y, j.radius_m, j.s_m);
        for (std::size_t k = 0; k < j.node_ids.size(); ++k) f("%s%lld", k ? ", " : "", (long long)j.node_ids[k]);
        o << "]}" << (i + 1 < js.size() ? "," : "") << "\n";
    }
    o << "  ],\n  \"bridges\": [\n";
    for (std::size_t i = 0; i < bridges.size(); ++i) {
        const auto& r = bridges[i];
        o << "    {\"group\": " << json(r.group).dump();
        f(", \"way_id\": %lld, \"direction\": %d, \"enter_node\": %lld, \"exit_node\": %lld, \"enter_x\": %.2f, \"enter_y\": %.2f, "
          "\"exit_x\": %.2f, \"exit_y\": %.2f, \"length_m\": %.1f, \"s_enter_m\": %.1f, \"s_exit_m\": %.1f}%s\n",
          (long long)r.way_id, r.direction, (long long)r.enter_node, (long long)r.exit_node, r.enter_x, r.enter_y, r.exit_x, r.exit_y,
          r.length_m, r.s_enter_m, r.s_exit_m, i + 1 < bridges.size() ? "," : "");
    }
    o << "  ],\n  \"waypoints\": [\n";
    for (std::size_t i = 0; i < wps.size(); ++i) {
        // waypoints[0] is the spawn to the cm, exactly as written in "spawn"
        const double x = i == 0 ? cfg.spawn_x : wps[i].first, y = i == 0 ? cfg.spawn_y : wps[i].second;
        f("    [%.2f, %.2f]%s\n", x, y, i + 1 < wps.size() ? "," : "");
    }
    o << "  ]\n}\n";
    if (out_path.empty()) {
        std::fputs(o.str().c_str(), stdout);
    } else {
        std::ofstream out(out_path, std::ios::binary);
        out << o.str();
        std::fprintf(stderr, "wrote %s\n", out_path.c_str());
    }
    return 0;
}
