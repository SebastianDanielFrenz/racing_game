#include "rg/crops.h"

#include "json_util.h"

#include <algorithm>
#include <cmath>

namespace rg {
namespace {
using detail::json;

std::uint64_t mix(std::uint64_t x) { // splitmix64 finaliser
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}
std::uint64_t field_hash(const CropConfig& c, std::int64_t way_id, std::uint64_t salt) {
    return mix(mix(static_cast<std::uint64_t>(way_id) ^ mix(c.world_seed)) ^ salt);
}
bool leap(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }
// Day of year in non-leap numbering (29 Feb counts as 28 Feb).
int doy_365(const CivilDate& d) {
    int doy = day_of_year(d);
    if (leap(d.year) && (d.month > 2 || (d.month == 2 && d.day == 29))) doy -= 1;
    return doy;
}
const CropStage& stage_at(const CropDef& crop, int doy) {
    const CropStage* out = &crop.stages.back(); // before the first start: the last stage of the previous year
    for (const auto& s : crop.stages)
        if (s.start_doy <= doy) out = &s;
    return *out;
}
bool read_rgb(const json& o, std::array<double, 3>& out) {
    const auto it = o.find("rgb");
    if (it == o.end() || !it->is_array() || it->size() != 3) return false;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return false;
        out[i] = (*it)[i].get<double>();
    }
    return true;
}
} // namespace

const CropDef* CropConfig::find(const std::string& id) const {
    for (const auto& c : crops) {
        if (c.id == id) return &c;
        if (std::find(c.aliases.begin(), c.aliases.end(), id) != c.aliases.end()) return &c;
    }
    return nullptr;
}

std::optional<CropConfig> load_crop_config(const std::string& path, std::string* error) {
    auto bad = [&](const std::string& what) {
        if (error) *error = path + ": " + what;
        return std::nullopt;
    };
    const auto text = detail::read_text_file(path);
    if (!text) return bad("cannot read");
    const auto parsed = detail::parse_json(*text);
    if (!parsed || !parsed->is_object()) return bad("not a JSON object");
    const json& j = *parsed;
    std::string format;
    if (!detail::get_string(j, "format", format) || format != "rg.crops/1") return bad("format must be \"rg.crops/1\"");
    CropConfig c;
    double seed = 0, jitter = 0;
    if (!detail::get_number(j, "world_seed", seed) || seed < 0 || !detail::get_number(j, "date_jitter_days", jitter) || jitter < 0 || jitter > 30)
        return bad("world_seed >= 0 and date_jitter_days in [0, 30] required");
    c.world_seed = static_cast<std::uint64_t>(seed);
    c.date_jitter_days = static_cast<int>(jitter);
    const auto crops = j.find("crops");
    if (crops == j.end() || !crops->is_array() || crops->empty()) return bad("crops must be a non-empty array");
    for (const auto& e : *crops) {
        CropDef d;
        double end = 0;
        if (!e.is_object() || !detail::get_string(e, "id", d.id) || !detail::get_string(e, "label", d.label) ||
            !detail::get_number(e, "row_spacing_m", d.row_spacing_m) || d.row_spacing_m <= 0 ||
            !detail::get_number(e, "season_end_doy", end) || end < 1 || end > 365)
            return bad("each crop needs id, label, row_spacing_m > 0, season_end_doy in [1, 365]");
        d.season_end_doy = static_cast<int>(end);
        if (const auto al = e.find("aliases"); al != e.end() && al->is_array())
            for (const auto& a : *al)
                if (a.is_string()) d.aliases.push_back(a.get<std::string>());
        const auto stages = e.find("stages");
        if (stages == e.end() || !stages->is_array() || stages->empty()) return bad("crop " + d.id + ": stages must be a non-empty array");
        for (const auto& s : *stages) {
            CropStage st;
            double start = 0;
            if (!s.is_object() || !detail::get_string(s, "id", st.id) || !detail::get_number(s, "start_doy", start) || start < 1 || start > 365 ||
                !detail::get_number(s, "height_m", st.height_m) || st.height_m < 0 || !detail::get_number(s, "ground_cover", st.ground_cover) ||
                st.ground_cover < 0 || st.ground_cover > 1 || !read_rgb(s, st.rgb))
                return bad("crop " + d.id + ": each stage needs id, start_doy in [1, 365], height_m >= 0, ground_cover in [0, 1], rgb");
            st.start_doy = static_cast<int>(start);
            if (!d.stages.empty() && st.start_doy <= d.stages.back().start_doy) return bad("crop " + d.id + ": stages must ascend in start_doy");
            d.stages.push_back(st);
        }
        if (c.find(d.id)) return bad("duplicate crop id " + d.id);
        c.crops.push_back(std::move(d));
    }
    const auto rots = j.find("rotations");
    if (rots == j.end() || !rots->is_array() || rots->empty()) return bad("rotations must be a non-empty array");
    for (const auto& r : *rots) {
        Rotation rot;
        const auto seq = r.find("sequence");
        if (!r.is_object() || !detail::get_number(r, "weight", rot.weight) || rot.weight <= 0 || seq == r.end() || !seq->is_array() || seq->empty())
            return bad("each rotation needs weight > 0 and a non-empty sequence");
        for (const auto& id : *seq) {
            if (!id.is_string() || !c.find(id.get<std::string>())) return bad("rotation names an unknown crop");
            rot.sequence.push_back(id.get<std::string>());
        }
        c.rotations.push_back(std::move(rot));
    }
    if (const auto fixed = j.find("fixed_landuse"); fixed != j.end() && fixed->is_object())
        for (const auto& [key, value] : fixed->items()) {
            if (!value.is_string() || !c.find(value.get<std::string>())) return bad("fixed_landuse " + key + " names an unknown crop");
            c.fixed_landuse.emplace_back(key, value.get<std::string>());
        }
    return c;
}

const CropDef* rotation_crop(const CropConfig& c, std::int64_t way_id, int year) {
    if (c.rotations.empty()) return nullptr;
    double total = 0;
    for (const auto& r : c.rotations) total += r.weight;
    const double pick = static_cast<double>(field_hash(c, way_id, 1) >> 11) / 9007199254740992.0 * total;
    const Rotation* rot = &c.rotations.back();
    double acc = 0;
    for (const auto& r : c.rotations) {
        acc += r.weight;
        if (pick < acc) { rot = &r; break; }
    }
    const std::uint64_t n = rot->sequence.size();
    const std::uint64_t phase = field_hash(c, way_id, 2) % n;
    const std::uint64_t index = (phase + static_cast<std::uint64_t>(static_cast<std::int64_t>(year) + 1000000) % n) % n;
    return c.find(rot->sequence[index]);
}

std::optional<FieldLook> field_look(const CropConfig& c, std::int64_t way_id, const std::string& landuse, const std::string& crop_tag,
                                    const CivilDate& date) {
    int doy = doy_365(date);
    if (c.date_jitter_days > 0) {
        const int span = 2 * c.date_jitter_days + 1;
        doy += static_cast<int>(field_hash(c, way_id, 3) % static_cast<std::uint64_t>(span)) - c.date_jitter_days;
    }
    int year = date.year;
    if (doy < 1) { doy += 365; --year; }
    if (doy > 365) { doy -= 365; ++year; }
    const CropDef* crop = nullptr;
    int harvest_year = year;
    if (!crop_tag.empty()) crop = c.find(crop_tag);
    if (!crop) {
        for (const auto& [key, id] : c.fixed_landuse)
            if (key == landuse) crop = c.find(id);
    }
    if (!crop && landuse == "farmland") {
        crop = rotation_crop(c, way_id, year);
        if (crop && doy >= crop->season_end_doy) {
            harvest_year = year + 1;
            crop = rotation_crop(c, way_id, harvest_year);
        }
    } else if (crop && !crop_tag.empty() && doy >= crop->season_end_doy) {
        harvest_year = year + 1;
    }
    if (!crop) return std::nullopt;
    const auto& stage = stage_at(*crop, doy);
    FieldLook out;
    out.crop_id = crop->id;
    out.crop_label = crop->label;
    out.stage_id = stage.id;
    out.harvest_year = harvest_year;
    out.height_m = stage.height_m;
    out.ground_cover = stage.ground_cover;
    out.row_spacing_m = crop->row_spacing_m;
    out.rgb = stage.rgb;
    return out;
}

double field_row_angle_deg(const std::vector<std::array<double, 2>>& p) {
    if (p.size() < 2) return 0;
    double best = -1, angle = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& a = p[i];
        const auto& b = p[(i + 1) % p.size()];
        const double dx = b[0] - a[0], dy = b[1] - a[1];
        const double len = dx * dx + dy * dy;
        if (len > best) {
            best = len;
            angle = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
        }
    }
    angle = std::fmod(angle, 180.0);
    if (angle < 0) angle += 180.0;
    return angle;
}

} // namespace rg
