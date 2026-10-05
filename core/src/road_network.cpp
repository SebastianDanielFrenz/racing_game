// rg/road_network.cpp - see road_network.h for the contract.
#include "rg/road_network.h"

#include "g2m/layer/road_graph_tile.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>

namespace rg {

namespace {

std::string tag_of(const RoadNetWay& w, const char* key) {
    for (const auto& [k, v] : w.tags) {
        if (k == key) return v;
    }
    return {};
}

bool drivable_class(const std::string& h) {
    static const std::set<std::string> ok = {"motorway",       "trunk",          "primary",      "secondary",
                                             "tertiary",       "unclassified",   "residential",  "motorway_link",
                                             "trunk_link",     "primary_link",   "secondary_link", "tertiary_link"};
    return ok.count(h) != 0;
}

bool public_access(const std::string& v) {
    return v.empty() || v == "yes" || v == "permissive" || v == "designated" || v == "destination" ||
           v == "customers" || v == "delivery";
}

// True if the way is open to cars at all (class + access).
bool way_drivable(const RoadNetWay& w) {
    if (!drivable_class(tag_of(w, "highway"))) return false;
    for (const char* k : {"access", "vehicle", "motor_vehicle", "motorcar"}) {
        const std::string v = tag_of(w, k);
        if (!v.empty() && !public_access(v)) return false;
    }
    return true;
}

// +1: forward only, -1: backward only, 0: both.
int oneway_of(const RoadNetWay& w) {
    const std::string o = tag_of(w, "oneway");
    if (o.empty()) {
        if (tag_of(w, "highway") == "motorway" || tag_of(w, "junction") == "roundabout") return 1;
        return 0;
    }
    if (o == "yes" || o == "1" || o == "true") return 1;
    if (o == "-1" || o == "reverse") return -1;
    return 0;
}

bool is_bridge(const RoadNetWay& w) {
    const std::string b = tag_of(w, "bridge");
    return !b.empty() && b != "no";
}

double hypot2(double dx, double dy) { return std::sqrt(dx * dx + dy * dy); }

} // namespace

void RoadNetwork::add_node(const RoadNetNode& node) { pending_nodes_.emplace(node.osm_id, node); }

void RoadNetwork::add_way(const RoadNetWay& way) { pending_ways_.emplace(way.osm_id, way); }

void RoadNetwork::add_road_graph_tile(const g2m::RoadGraphTile& tile, double e0, double n0) {
    for (const auto& n : tile.graph.nodes) {
        add_node({n.osm_id, static_cast<double>(n.position.x) / 1000.0 - e0,
                  static_cast<double>(n.position.y) / 1000.0 - n0});
    }
    for (const auto& w : tile.graph.ways) {
        add_way({w.osm_id, w.node_ids, w.tags});
    }
}

void RoadNetwork::finish() {
    if (finished_) return;
    finished_ = true;
    for (const auto& [id, n] : pending_nodes_) {
        node_by_id_[id] = nodes_.size();
        nodes_.push_back(n);
    }
    for (const auto& [id, w] : pending_ways_) {
        if (!way_drivable(w) || w.node_ids.size() < 2) continue;
        way_by_id_[id] = ways_.size();
        ways_.push_back(w);
    }
    out_.assign(nodes_.size(), {});

    for (std::size_t wi = 0; wi < ways_.size(); ++wi) {
        const RoadNetWay& w = ways_[wi];
        const int ow = oneway_of(w);
        const bool bridge = is_bridge(w);
        for (std::size_t s = 0; s + 1 < w.node_ids.size(); ++s) {
            const auto a = node_by_id_.find(w.node_ids[s]);
            const auto b = node_by_id_.find(w.node_ids[s + 1]);
            if (a == node_by_id_.end() || b == node_by_id_.end() || a->second == b->second) continue;
            const RoadNetNode& na = nodes_[a->second];
            const RoadNetNode& nb = nodes_[b->second];
            const double len = hypot2(nb.x - na.x, nb.y - na.y);
            if (ow >= 0) {
                arcs_.push_back({static_cast<int>(a->second), static_cast<int>(b->second), w.osm_id,
                                 static_cast<int>(wi), static_cast<int>(s), 1, len, bridge});
            }
            if (ow <= 0) {
                arcs_.push_back({static_cast<int>(b->second), static_cast<int>(a->second), w.osm_id,
                                 static_cast<int>(wi), static_cast<int>(s), -1, len, bridge});
            }
        }
    }
    for (std::size_t i = 0; i < arcs_.size(); ++i) {
        out_[static_cast<std::size_t>(arcs_[i].from)].push_back(static_cast<int>(i));
    }

    // Junction nodes: >= 3 distinct drivable neighbours (either direction) and >= 2 distinct ways.
    std::vector<std::set<int>> neigh(nodes_.size());
    std::vector<std::set<int>> nways(nodes_.size());
    for (const RoadArc& a : arcs_) {
        neigh[static_cast<std::size_t>(a.from)].insert(a.to);
        neigh[static_cast<std::size_t>(a.to)].insert(a.from);
        nways[static_cast<std::size_t>(a.from)].insert(a.way);
        nways[static_cast<std::size_t>(a.to)].insert(a.way);
    }
    std::vector<int> jn;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (neigh[i].size() >= 3 && nways[i].size() >= 2) jn.push_back(static_cast<int>(i));
    }
    junction_nodes_ = jn.size();

    // Cluster by proximity: union-find over a grid of kJunctionClusterM cells.
    std::vector<int> parent(jn.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
    auto find = [&](int v) {
        while (parent[static_cast<std::size_t>(v)] != v) {
            parent[static_cast<std::size_t>(v)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(v)])];
            v = parent[static_cast<std::size_t>(v)];
        }
        return v;
    };
    std::map<std::pair<long, long>, std::vector<int>> grid;
    auto cell = [&](double v) { return static_cast<long>(std::floor(v / kJunctionClusterM)); };
    for (std::size_t k = 0; k < jn.size(); ++k) {
        const RoadNetNode& n = nodes_[static_cast<std::size_t>(jn[k])];
        grid[{cell(n.x), cell(n.y)}].push_back(static_cast<int>(k));
    }
    for (std::size_t k = 0; k < jn.size(); ++k) {
        const RoadNetNode& n = nodes_[static_cast<std::size_t>(jn[k])];
        for (long dx = -1; dx <= 1; ++dx) {
            for (long dy = -1; dy <= 1; ++dy) {
                const auto it = grid.find({cell(n.x) + dx, cell(n.y) + dy});
                if (it == grid.end()) continue;
                for (int o : it->second) {
                    if (o <= static_cast<int>(k)) continue;
                    const RoadNetNode& m = nodes_[static_cast<std::size_t>(jn[static_cast<std::size_t>(o)])];
                    if (hypot2(n.x - m.x, n.y - m.y) <= kJunctionClusterM) {
                        const int ra = find(static_cast<int>(k)), rb = find(o);
                        if (ra != rb) parent[static_cast<std::size_t>(std::max(ra, rb))] = std::min(ra, rb);
                    }
                }
            }
        }
    }
    std::map<int, std::vector<int>> groups; // root -> member node indices
    for (std::size_t k = 0; k < jn.size(); ++k) groups[find(static_cast<int>(k))].push_back(jn[k]);
    std::vector<RoadJunction> js;
    for (const auto& [root, members] : groups) {
        RoadJunction j;
        for (int m : members) j.node_ids.push_back(nodes_[static_cast<std::size_t>(m)].osm_id);
        std::sort(j.node_ids.begin(), j.node_ids.end());
        for (int m : members) {
            j.x += nodes_[static_cast<std::size_t>(m)].x;
            j.y += nodes_[static_cast<std::size_t>(m)].y;
        }
        j.x /= static_cast<double>(members.size());
        j.y /= static_cast<double>(members.size());
        for (int m : members) {
            j.radius_m = std::max(j.radius_m, hypot2(nodes_[static_cast<std::size_t>(m)].x - j.x,
                                                     nodes_[static_cast<std::size_t>(m)].y - j.y));
        }
        js.push_back(std::move(j));
    }
    std::sort(js.begin(), js.end(),
              [](const RoadJunction& a, const RoadJunction& b) { return a.node_ids.front() < b.node_ids.front(); });
    junctions_ = std::move(js);
    node_junction_.assign(nodes_.size(), -1);
    for (std::size_t j = 0; j < junctions_.size(); ++j) {
        for (std::int64_t id : junctions_[j].node_ids) {
            node_junction_[node_by_id_.at(id)] = static_cast<int>(j);
        }
    }
    pending_nodes_.clear();
    pending_ways_.clear();
}

RoadNetworkStats RoadNetwork::stats() const {
    return {nodes_.size(), ways_.size(), arcs_.size(), junction_nodes_, junctions_.size()};
}

int RoadNetwork::node_index(std::int64_t id) const {
    const auto it = node_by_id_.find(id);
    return it == node_by_id_.end() ? -1 : static_cast<int>(it->second);
}

int RoadNetwork::way_index(std::int64_t id) const {
    const auto it = way_by_id_.find(id);
    return it == way_by_id_.end() ? -1 : static_cast<int>(it->second);
}

int RoadNetwork::junction_of_node(int node) const {
    if (node < 0 || static_cast<std::size_t>(node) >= node_junction_.size()) return -1;
    return node_junction_[static_cast<std::size_t>(node)];
}

std::vector<int> RoadNetwork::way_arcs(std::int64_t way_id, int direction) const {
    std::vector<int> r;
    const int wi = way_index(way_id);
    if (wi < 0) return r;
    for (std::size_t i = 0; i < arcs_.size(); ++i) {
        if (arcs_[i].way == wi && arcs_[i].direction == direction) r.push_back(static_cast<int>(i));
    }
    std::sort(r.begin(), r.end(), [&](int a, int b) {
        const int sa = arcs_[static_cast<std::size_t>(a)].segment, sb = arcs_[static_cast<std::size_t>(b)].segment;
        return direction > 0 ? sa < sb : sa > sb;
    });
    return r;
}

std::vector<BridgeRun> RoadNetwork::bridge_runs() const {
    std::vector<BridgeRun> runs;
    for (std::size_t wi = 0; wi < ways_.size(); ++wi) {
        for (int dir : {1, -1}) {
            const auto arcs = way_arcs(ways_[wi].osm_id, dir);
            BridgeRun cur;
            auto flush = [&] {
                if (!cur.arcs.empty()) {
                    cur.way_id = ways_[wi].osm_id;
                    cur.direction = dir;
                    cur.enter_node =
                        nodes_[static_cast<std::size_t>(arcs_[static_cast<std::size_t>(cur.arcs.front())].from)].osm_id;
                    cur.exit_node =
                        nodes_[static_cast<std::size_t>(arcs_[static_cast<std::size_t>(cur.arcs.back())].to)].osm_id;
                    runs.push_back(cur);
                }
                cur = BridgeRun{};
            };
            int prev_seg = -2;
            for (int a : arcs) {
                const RoadArc& arc = arcs_[static_cast<std::size_t>(a)];
                const bool consecutive = std::abs(arc.segment - prev_seg) == 1;
                if (!arc.bridge || (!cur.arcs.empty() && !consecutive)) flush();
                if (arc.bridge) {
                    cur.arcs.push_back(a);
                    cur.length_m += arc.length;
                }
                prev_seg = arc.segment;
            }
            flush();
        }
    }
    return runs;
}

int RoadNetwork::nearest_arc(double x, double y, double hx, double hy, double max_dist_m, double min_cos,
                             double* t_out, double* dist_out) const {
    const double hl = hypot2(hx, hy);
    int best = -1;
    double best_d = max_dist_m;
    double best_t = 0.0;
    for (std::size_t i = 0; i < arcs_.size(); ++i) {
        const RoadArc& a = arcs_[i];
        const RoadNetNode& p = nodes_[static_cast<std::size_t>(a.from)];
        const RoadNetNode& q = nodes_[static_cast<std::size_t>(a.to)];
        const double dx = q.x - p.x, dy = q.y - p.y;
        const double l2 = dx * dx + dy * dy;
        if (l2 <= 0.0) continue;
        if (hl > 0.0 && (dx * hx + dy * hy) / (std::sqrt(l2) * hl) < min_cos) continue;
        double t = ((x - p.x) * dx + (y - p.y) * dy) / l2;
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        const double d = hypot2(x - (p.x + t * dx), y - (p.y + t * dy));
        if (d < best_d - 1.0e-12) {
            best_d = d;
            best = static_cast<int>(i);
            best_t = t;
        }
    }
    if (best >= 0) {
        if (t_out) *t_out = best_t;
        if (dist_out) *dist_out = best_d;
    }
    return best;
}

std::optional<std::vector<int>> RoadNetwork::shortest_arc_path(int from_arc, int to_arc, double max_length_m) const {
    if (from_arc < 0 || to_arc < 0 || static_cast<std::size_t>(from_arc) >= arcs_.size() ||
        static_cast<std::size_t>(to_arc) >= arcs_.size()) {
        return std::nullopt;
    }
    constexpr double kInf = 1.0e300;
    std::vector<double> dist(arcs_.size(), kInf);
    std::vector<int> prev(arcs_.size(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    dist[static_cast<std::size_t>(from_arc)] = 0.0;
    q.push({0.0, from_arc});
    while (!q.empty()) {
        const auto [d, a] = q.top();
        q.pop();
        if (d > dist[static_cast<std::size_t>(a)]) continue;
        if (a == to_arc) break;
        const RoadArc& arc = arcs_[static_cast<std::size_t>(a)];
        for (int b : out_[static_cast<std::size_t>(arc.to)]) {
            const RoadArc& nb = arcs_[static_cast<std::size_t>(b)];
            if (nb.to == arc.from) continue; // U-turn
            const double nd = d + (b == to_arc ? 0.0 : nb.length);
            if (nd > max_length_m) continue;
            if (nd < dist[static_cast<std::size_t>(b)]) {
                dist[static_cast<std::size_t>(b)] = nd;
                prev[static_cast<std::size_t>(b)] = a;
                q.push({nd, b});
            }
        }
    }
    if (dist[static_cast<std::size_t>(to_arc)] >= kInf) return std::nullopt;
    std::vector<int> path;
    for (int a = prev[static_cast<std::size_t>(to_arc)]; a >= 0 && a != from_arc;
         a = prev[static_cast<std::size_t>(a)]) {
        path.push_back(a);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> RoadNetwork::junctions_on_arcs(const std::vector<int>& arcs) const {
    std::vector<int> r;
    std::set<int> seen;
    auto visit = [&](int node) {
        const int j = junction_of_node(node);
        if (j >= 0 && seen.insert(j).second) r.push_back(j);
    };
    for (std::size_t i = 0; i < arcs.size(); ++i) {
        const RoadArc& a = arcs_[static_cast<std::size_t>(arcs[i])];
        if (i == 0) visit(a.from);
        visit(a.to);
    }
    return r;
}

std::optional<PlannedPath> plan_path(const RoadNetwork& net, double sx, double sy, double hx, double hy,
                                     const std::vector<RouteStep>& steps, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err != nullptr) *err = m;
        return std::nullopt;
    };
    PlannedPath out;
    const int start = net.nearest_arc(sx, sy, hx, hy, 15.0, 0.5, &out.start_t, &out.start_dist_m);
    if (start < 0) return fail("no drivable arc within 15 m of the start in the start direction");
    out.arcs.push_back(start);
    int current = start;
    for (std::size_t si = 0; si < steps.size(); ++si) {
        const RouteStep& s = steps[si];
        const std::string where = "step " + std::to_string(si) + ": ";
        auto length_of = [&](const std::vector<int>& arcs) {
            double l = 0.0;
            for (int a : arcs) l += net.arcs()[static_cast<std::size_t>(a)].length;
            return l;
        };
        if (s.kind == RouteStep::Kind::TraverseWay) {
            const auto way = net.way_arcs(s.id, s.direction);
            if (way.empty()) {
                return fail(where + "way " + std::to_string(s.id) + " has no arcs in direction " +
                            std::to_string(s.direction));
            }
            if (way.front() == current) return fail(where + "the route is already on way " + std::to_string(s.id));
            const auto p = net.shortest_arc_path(current, way.front());
            if (!p) {
                return fail(where + "way " + std::to_string(s.id) + " direction " + std::to_string(s.direction) +
                            " is unreachable");
            }
            out.step_approach_m.push_back(length_of(*p));
            out.arcs.insert(out.arcs.end(), p->begin(), p->end());
            out.arcs.insert(out.arcs.end(), way.begin(), way.end());
            current = way.back();
        } else {
            const int ni = net.node_index(s.id);
            if (ni < 0) return fail(where + "unknown node " + std::to_string(s.id));
            std::optional<std::vector<int>> best;
            double best_len = 1.0e300;
            int best_arc = -1;
            for (std::size_t a = 0; a < net.arcs().size(); ++a) {
                if (net.arcs()[a].to != ni || static_cast<int>(a) == current) continue;
                const auto p = net.shortest_arc_path(current, static_cast<int>(a));
                if (!p) continue;
                const double l = length_of(*p) + net.arcs()[a].length;
                if (l < best_len) {
                    best_len = l;
                    best = p;
                    best_arc = static_cast<int>(a);
                }
            }
            if (!best) return fail(where + "node " + std::to_string(s.id) + " is unreachable");
            out.step_approach_m.push_back(best_len);
            out.arcs.insert(out.arcs.end(), best->begin(), best->end());
            out.arcs.push_back(best_arc);
            current = best_arc;
        }
    }
    return out;
}

std::vector<std::pair<double, double>> smooth_polyline(const std::vector<std::pair<double, double>>& pts,
                                                       double radius_m, double max_spacing_m) {
    if (pts.size() < 2) return pts;
    constexpr double kMinTurnCos = 0.99863; // ~3 degrees
    std::vector<std::pair<double, double>> out;
    out.push_back(pts.front());
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        const auto& a = pts[i - 1];
        const auto& b = pts[i];
        const auto& c = pts[i + 1];
        const double ux = b.first - a.first, uy = b.second - a.second;
        const double vx = c.first - b.first, vy = c.second - b.second;
        const double lu = hypot2(ux, uy), lv = hypot2(vx, vy);
        if (lu <= 1.0e-9 || lv <= 1.0e-9) continue;
        const double cs = (ux * vx + uy * vy) / (lu * lv);
        if (cs > kMinTurnCos) {
            out.push_back(b);
            continue;
        }
        const double cl = cs < -0.999 ? -0.999 : cs;
        const double tan_half = std::sqrt((1.0 - cl) / (1.0 + cl));
        const double t = std::min(radius_m * tan_half, 0.45 * std::min(lu, lv));
        const std::pair<double, double> p0{b.first - ux / lu * t, b.second - uy / lu * t};
        const std::pair<double, double> p2{b.first + vx / lv * t, b.second + vy / lv * t};
        const int n = std::max(3, static_cast<int>(std::ceil(2.0 * t)));
        for (int k = 0; k <= n; ++k) {
            const double s = static_cast<double>(k) / n;
            const double w0 = (1.0 - s) * (1.0 - s), w1 = 2.0 * s * (1.0 - s), w2 = s * s;
            out.push_back({w0 * p0.first + w1 * b.first + w2 * p2.first,
                           w0 * p0.second + w1 * b.second + w2 * p2.second});
        }
    }
    out.push_back(pts.back());
    // Resample to uniform spacing <= max_spacing_m.
    std::vector<double> cum(out.size(), 0.0);
    for (std::size_t i = 1; i < out.size(); ++i) {
        cum[i] = cum[i - 1] + hypot2(out[i].first - out[i - 1].first, out[i].second - out[i - 1].second);
    }
    const double total = cum.back();
    if (!(total > 1.0e-9)) return out;
    const std::size_t n = std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(total / max_spacing_m)) + 1);
    std::vector<std::pair<double, double>> res;
    res.reserve(n);
    std::size_t seg = 1;
    for (std::size_t k = 0; k < n; ++k) {
        const double s = total * static_cast<double>(k) / static_cast<double>(n - 1);
        while (seg + 1 < out.size() && cum[seg] < s) ++seg;
        const double l = cum[seg] - cum[seg - 1];
        const double f = l > 0.0 ? (s - cum[seg - 1]) / l : 0.0;
        res.push_back({out[seg - 1].first + f * (out[seg].first - out[seg - 1].first),
                       out[seg - 1].second + f * (out[seg].second - out[seg - 1].second)});
    }
    res.front() = pts.front();
    res.back() = pts.back();
    return res;
}

} // namespace rg
