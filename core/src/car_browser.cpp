// rg/car_browser.cpp - see car_browser.h.
#include "rg/car_browser.h"

#include "rg/settings.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>

namespace rg {

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// Case-insensitive "a before b".
bool name_less(const std::string& a, const std::string& b) {
    const std::string la = lower(a), lb = lower(b);
    return la < lb;
}

constexpr double kBandEdgesKw[kPowerBandCount - 1] = {100.0, 200.0, 400.0, 700.0};

std::string join_csv(const std::set<std::string>& values) {
    std::string out;
    for (const std::string& v : values) {
        if (!out.empty()) out += ',';
        out += v;
    }
    return out;
}

std::vector<std::string> split_csv(const std::string& text) {
    std::vector<std::string> out;
    std::string token;
    for (const char c : text) {
        if (c == ',') {
            if (!token.empty()) out.push_back(token);
            token.clear();
        } else if (c != ' ') {
            token += c;
        }
    }
    if (!token.empty()) out.push_back(token);
    return out;
}

bool valid_layout(const std::string& s) { return s == "FWD" || s == "RWD" || s == "AWD" || s == "other"; }

bool valid_band(const std::string& s) {
    for (int i = 0; i < kPowerBandCount; ++i) {
        if (power_band_id(i) == s) return true;
    }
    return false;
}

bool valid_body_type(const std::string& s) {
    if (s.empty() || s.size() > 48) return false;
    return std::all_of(s.begin(), s.end(), [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

std::string layout_title(const std::string& layout) {
    if (layout == "FWD") return "Front-wheel drive";
    if (layout == "RWD") return "Rear-wheel drive";
    if (layout == "AWD") return "All-wheel drive";
    return "Other layout";
}

int layout_order(const std::string& layout) {
    if (layout == "FWD") return 0;
    if (layout == "RWD") return 1;
    if (layout == "AWD") return 2;
    return 3;
}

} // namespace

// ---- names ------------------------------------------------------------------

const char* to_string(GroupBy g) {
    switch (g) {
        case GroupBy::BodyType: return "body_type";
        case GroupBy::DriveLayout: return "drive_layout";
        case GroupBy::PowerBand: return "power_band";
        case GroupBy::Manufacturer: return "manufacturer";
        case GroupBy::None: return "none";
    }
    return "none";
}

const char* to_string(SortKey k) {
    switch (k) {
        case SortKey::Name: return "name";
        case SortKey::Power: return "power";
        case SortKey::Torque: return "torque";
        case SortKey::Mass: return "mass";
        case SortKey::PowerToWeight: return "power_to_weight";
        case SortKey::Displacement: return "displacement";
    }
    return "name";
}

std::optional<GroupBy> group_by_from_string(const std::string& s) {
    for (const GroupBy g : {GroupBy::BodyType, GroupBy::DriveLayout, GroupBy::PowerBand, GroupBy::Manufacturer, GroupBy::None}) {
        if (s == to_string(g)) return g;
    }
    return std::nullopt;
}

std::optional<SortKey> sort_key_from_string(const std::string& s) {
    for (const SortKey k : {SortKey::Name, SortKey::Power, SortKey::Torque, SortKey::Mass, SortKey::PowerToWeight, SortKey::Displacement}) {
        if (s == to_string(k)) return k;
    }
    return std::nullopt;
}

const std::vector<std::string>& group_by_names() {
    static const std::vector<std::string> names = {"body_type", "drive_layout", "power_band", "manufacturer", "none"};
    return names;
}

const std::vector<std::string>& sort_key_names() {
    static const std::vector<std::string> names = {"name", "power", "torque", "mass", "power_to_weight", "displacement"};
    return names;
}

std::string group_by_label(GroupBy g) {
    switch (g) {
        case GroupBy::BodyType: return "Body type";
        case GroupBy::DriveLayout: return "Drive layout";
        case GroupBy::PowerBand: return "Power";
        case GroupBy::Manufacturer: return "Manufacturer";
        case GroupBy::None: return "None";
    }
    return "None";
}

std::string sort_key_label(SortKey k) {
    switch (k) {
        case SortKey::Name: return "Name";
        case SortKey::Power: return "Power";
        case SortKey::Torque: return "Torque";
        case SortKey::Mass: return "Mass";
        case SortKey::PowerToWeight: return "Power-to-weight";
        case SortKey::Displacement: return "Displacement";
    }
    return "Name";
}

int power_band_index(double power_kw) {
    int band = 0;
    for (int i = 0; i < kPowerBandCount - 1; ++i) {
        if (power_kw >= kBandEdgesKw[i]) band = i + 1;
    }
    return band;
}

std::string power_band_id(int index) {
    static const char* ids[kPowerBandCount] = {"under_100", "100_199", "200_399", "400_699", "700_plus"};
    return ids[std::clamp(index, 0, kPowerBandCount - 1)];
}

std::string power_band_label(int index) {
    static const char* labels[kPowerBandCount] = {"Under 100 kW", "100 - 199 kW", "200 - 399 kW", "400 - 699 kW", "700 kW and more"};
    return labels[std::clamp(index, 0, kPowerBandCount - 1)];
}

std::string body_type_label(const std::string& body_type) {
    std::string out = body_type;
    for (char& c : out) {
        if (c == '_') c = ' ';
    }
    if (!out.empty()) out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

double power_to_weight_kw_per_t(const BrowserCar& c) {
    if (!(c.power_kw > 0.0) || !(c.mass_kg > 0.0)) return 0.0;
    return c.power_kw / c.mass_kg * 1000.0;
}

bool BrowserFilter::accepts(const BrowserCar& c) const {
    if (!layouts.empty() && layouts.count(c.layout) == 0) return false;
    if (!body_types.empty() && body_types.count(c.body_type) == 0) return false;
    if (!power_bands.empty() && power_bands.count(power_band_id(power_band_index(c.power_kw))) == 0) return false;
    return true;
}

// ---- model --------------------------------------------------------------------

CarBrowser::CarBrowser(std::vector<BrowserCar> cars) : cars_(std::move(cars)) { rebuild(""); }

void CarBrowser::set_cars(std::vector<BrowserCar> cars) {
    const std::string keep = focus_id();
    cars_ = std::move(cars);
    rebuild(keep);
}

const BrowserCar* CarBrowser::find(const std::string& id) const {
    for (const BrowserCar& c : cars_) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

void CarBrowser::set_group_by(GroupBy g) {
    if (state_.group_by == g) return;
    const std::string keep = focus_id();
    state_.group_by = g;
    rebuild(keep);
}

void CarBrowser::set_sort(SortKey key, bool descending) {
    if (state_.sort_key == key && state_.sort_descending == descending) return;
    const std::string keep = focus_id();
    state_.sort_key = key;
    state_.sort_descending = descending;
    rebuild(keep);
}

void CarBrowser::set_filter(const BrowserFilter& f) {
    const std::string keep = focus_id();
    state_.filter = f;
    rebuild(keep);
}

void CarBrowser::toggle_filter_value(const std::string& facet, const std::string& value) {
    BrowserFilter f = state_.filter;
    std::set<std::string>* set = nullptr;
    if (facet == "layout") set = &f.layouts;
    else if (facet == "body_type") set = &f.body_types;
    else if (facet == "power_band") set = &f.power_bands;
    if (set == nullptr) return;
    if (set->erase(value) == 0) set->insert(value);
    set_filter(f);
}

void CarBrowser::clear_filter() { set_filter(BrowserFilter{}); }

std::vector<FilterOption> CarBrowser::filter_options(const std::string& facet) const {
    std::vector<FilterOption> out;
    const auto add = [&](const std::string& value, const std::string& label) {
        FilterOption o;
        o.value = value;
        o.label = label;
        out.push_back(std::move(o));
    };
    const auto bump = [&](const std::string& value) {
        for (FilterOption& o : out) {
            if (o.value == value) ++o.count;
        }
    };
    if (facet == "layout") {
        for (const char* l : {"FWD", "RWD", "AWD", "other"}) add(l, std::string(l) == "other" ? "Other" : l);
        for (const BrowserCar& c : cars_) bump(c.layout);
    } else if (facet == "power_band") {
        for (int i = 0; i < kPowerBandCount; ++i) add(power_band_id(i), power_band_label(i));
        for (const BrowserCar& c : cars_) bump(power_band_id(power_band_index(c.power_kw)));
    } else if (facet == "body_type") {
        std::set<std::string> types;
        for (const BrowserCar& c : cars_) types.insert(c.body_type);
        std::vector<std::string> sorted(types.begin(), types.end());
        std::sort(sorted.begin(), sorted.end(), [](const std::string& a, const std::string& b) { return name_less(body_type_label(a), body_type_label(b)); });
        for (const std::string& t : sorted) add(t, body_type_label(t));
        for (const BrowserCar& c : cars_) bump(c.body_type);
    } else {
        return out;
    }
    const std::set<std::string>& selected = facet == "layout" ? state_.filter.layouts
                                            : facet == "power_band" ? state_.filter.power_bands
                                                                    : state_.filter.body_types;
    for (FilterOption& o : out) o.selected = selected.count(o.value) != 0;
    // Drop values no car has (a layout nobody uses), except a selected one - the player can switch it off.
    out.erase(std::remove_if(out.begin(), out.end(), [](const FilterOption& o) { return o.count == 0 && !o.selected; }), out.end());
    return out;
}

void CarBrowser::apply_settings(const Settings& settings) {
    BrowserViewState next = state_;
    if (const auto g = group_by_from_string(settings.get_string("browser.group_by"))) next.group_by = *g;
    if (const auto k = sort_key_from_string(settings.get_string("browser.sort_key"))) next.sort_key = *k;
    next.sort_descending = settings.get_bool("browser.sort_desc");
    next.filter = BrowserFilter{};
    for (const std::string& t : split_csv(settings.get_string("browser.filter_layouts"))) {
        if (valid_layout(t)) next.filter.layouts.insert(t);
    }
    for (const std::string& t : split_csv(settings.get_string("browser.filter_body_types"))) {
        if (valid_body_type(t)) next.filter.body_types.insert(t);
    }
    for (const std::string& t : split_csv(settings.get_string("browser.filter_power_bands"))) {
        if (valid_band(t)) next.filter.power_bands.insert(t);
    }
    const std::string keep = focus_id();
    state_ = std::move(next);
    rebuild(keep);
}

void CarBrowser::store_settings(Settings& settings) const {
    settings.set("browser.group_by", std::string(to_string(state_.group_by)));
    settings.set("browser.sort_key", std::string(to_string(state_.sort_key)));
    settings.set("browser.sort_desc", state_.sort_descending);
    settings.set("browser.filter_layouts", join_csv(state_.filter.layouts));
    settings.set("browser.filter_body_types", join_csv(state_.filter.body_types));
    settings.set("browser.filter_power_bands", join_csv(state_.filter.power_bands));
}

// ---- grid ---------------------------------------------------------------------

void CarBrowser::set_rows(int rows) {
    rows = std::clamp(rows, kMinRows, kMaxRows);
    if (rows == rows_) return;
    const std::string keep = focus_id();
    rows_ = rows;
    rebuild(keep);
}

int CarBrowser::rows_for_height(double available_px, double tile_px, double gap_px, double header_px) {
    if (!(tile_px > 0.0)) return kMinRows;
    const double usable = available_px - header_px;
    const int fit = static_cast<int>(std::floor((usable + gap_px) / (tile_px + gap_px)));
    return std::clamp(fit, kMinRows, kMaxRows);
}

void CarBrowser::rebuild(const std::string& keep_focus_id) {
    order_.clear();
    categories_.clear();
    car_category_.assign(cars_.size(), -1);
    car_slot_.assign(cars_.size(), 0);

    // 1. the cars that pass the filter, with the group each belongs to
    struct Grouped {
        int car;
        std::string key;
        std::string title;
        int rank; // group order for the modes with a fixed order
    };
    std::vector<Grouped> grouped;
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const BrowserCar& c = cars_[i];
        if (!state_.filter.accepts(c)) continue;
        Grouped g;
        g.car = static_cast<int>(i);
        g.rank = 0;
        switch (state_.group_by) {
            case GroupBy::BodyType:
                g.key = c.body_type;
                g.title = body_type_label(c.body_type);
                break;
            case GroupBy::DriveLayout:
                g.key = c.layout;
                g.title = layout_title(c.layout);
                g.rank = layout_order(c.layout);
                break;
            case GroupBy::PowerBand: {
                const int band = power_band_index(c.power_kw);
                g.key = power_band_id(band);
                g.title = power_band_label(band);
                g.rank = band;
                break;
            }
            case GroupBy::Manufacturer:
                g.key = c.manufacturer;
                g.title = c.manufacturer.empty() ? "Other" : c.manufacturer;
                g.rank = c.manufacturer.empty() ? 1 : 0; // cars without one come last
                break;
            case GroupBy::None:
                break;
        }
        grouped.push_back(std::move(g));
    }

    // 2. the sort inside a group: unknown values last, then the player's direction, ties by name then id
    const auto value_of = [&](const BrowserCar& c, bool* known) {
        *known = true;
        switch (state_.sort_key) {
            case SortKey::Power: *known = c.power_kw > 0.0; return c.power_kw;
            case SortKey::Torque: *known = c.torque_nm > 0.0; return c.torque_nm;
            case SortKey::Mass: *known = c.mass_kg > 0.0; return c.mass_kg;
            case SortKey::PowerToWeight: {
                const double v = power_to_weight_kw_per_t(c);
                *known = v > 0.0;
                return v;
            }
            case SortKey::Displacement: *known = c.displacement_known; return c.displacement_l;
            case SortKey::Name: break;
        }
        return 0.0;
    };
    const auto car_less = [&](int a, int b) {
        const BrowserCar& ca = cars_[static_cast<std::size_t>(a)];
        const BrowserCar& cb = cars_[static_cast<std::size_t>(b)];
        if (state_.sort_key != SortKey::Name) {
            bool ka = false, kb = false;
            const double va = value_of(ca, &ka), vb = value_of(cb, &kb);
            if (ka != kb) return ka; // known before unknown, whatever the direction
            if (ka && va != vb) return state_.sort_descending ? va > vb : va < vb;
            if (!(ca.title == cb.title)) return name_less(ca.title, cb.title); // ties: by name, ascending
        } else if (!(ca.title == cb.title)) {
            return state_.sort_descending ? name_less(cb.title, ca.title) : name_less(ca.title, cb.title);
        }
        return ca.id < cb.id;
    };

    // 3. the groups in their order
    std::vector<std::string> keys;
    for (const Grouped& g : grouped) {
        if (std::find(keys.begin(), keys.end(), g.key) == keys.end()) keys.push_back(g.key);
    }
    const auto rank_of = [&](const std::string& key) {
        for (const Grouped& g : grouped) {
            if (g.key == key) return g.rank;
        }
        return 0;
    };
    const auto title_of = [&](const std::string& key) {
        for (const Grouped& g : grouped) {
            if (g.key == key) return g.title;
        }
        return std::string();
    };
    std::sort(keys.begin(), keys.end(), [&](const std::string& a, const std::string& b) {
        const int ra = rank_of(a), rb = rank_of(b);
        if (ra != rb) return ra < rb;
        return name_less(title_of(a), title_of(b));
    });

    int column = 0;
    for (const std::string& key : keys) {
        BrowserCategory cat;
        cat.key = key;
        cat.title = title_of(key);
        for (const Grouped& g : grouped) {
            if (g.key == key) cat.cars.push_back(g.car);
        }
        std::sort(cat.cars.begin(), cat.cars.end(), car_less);
        cat.first_column = column;
        cat.column_count = static_cast<int>((cat.cars.size() + static_cast<std::size_t>(rows_) - 1) / static_cast<std::size_t>(rows_));
        column += cat.column_count;
        const int cat_index = static_cast<int>(categories_.size());
        for (std::size_t k = 0; k < cat.cars.size(); ++k) {
            const std::size_t car = static_cast<std::size_t>(cat.cars[k]);
            car_category_[car] = cat_index;
            car_slot_[car] = static_cast<int>(k);
            order_.push_back(cat.cars[k]);
        }
        categories_.push_back(std::move(cat));
    }
    total_columns_ = column;

    // 4. the focus survives while its car is still listed
    focus_ = -1;
    if (!keep_focus_id.empty()) {
        for (std::size_t i = 0; i < cars_.size(); ++i) {
            if (cars_[i].id == keep_focus_id && car_category_[i] >= 0) focus_ = static_cast<int>(i);
        }
    }
    if (focus_ < 0 && !order_.empty()) focus_ = order_.front();
    if (focus_ >= 0) {
        // keep the aimed-at row when the grid keeps the same focus, else follow the focus
        sticky_row_ = cell_of(focus_).row; // the grid changed shape: aim at the focus's own row
    } else {
        sticky_row_ = 0;
    }
    first_column_ = std::clamp(first_column_, 0, std::max(0, total_columns_ - 1));
}

GridCell CarBrowser::cell_of(int car_index) const {
    GridCell cell;
    if (car_index < 0 || static_cast<std::size_t>(car_index) >= car_category_.size()) return cell;
    const int cat = car_category_[static_cast<std::size_t>(car_index)];
    if (cat < 0) return cell;
    const int slot = car_slot_[static_cast<std::size_t>(car_index)];
    cell.column = categories_[static_cast<std::size_t>(cat)].first_column + slot / rows_;
    cell.row = slot % rows_;
    return cell;
}

int CarBrowser::category_of(int car_index) const {
    if (car_index < 0 || static_cast<std::size_t>(car_index) >= car_category_.size()) return -1;
    return car_category_[static_cast<std::size_t>(car_index)];
}

int CarBrowser::car_at(int column, int row) const {
    if (column < 0 || column >= total_columns_ || row < 0 || row >= rows_) return -1;
    for (const BrowserCategory& cat : categories_) {
        if (column < cat.first_column + cat.column_count) {
            const std::size_t slot = static_cast<std::size_t>((column - cat.first_column) * rows_ + row);
            return slot < cat.cars.size() ? cat.cars[slot] : -1;
        }
    }
    return -1;
}

std::string CarBrowser::focus_id() const {
    return focus_ >= 0 ? cars_[static_cast<std::size_t>(focus_)].id : std::string();
}

GridCell CarBrowser::focus_cell() const { return cell_of(focus_); }

bool CarBrowser::set_focus(const std::string& id) {
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        if (cars_[i].id == id && car_category_[i] >= 0) {
            focus_ = static_cast<int>(i);
            sticky_row_ = cell_of(focus_).row;
            return true;
        }
    }
    return false;
}

bool CarBrowser::move_focus(NavDir dir) {
    if (order_.empty()) {
        focus_ = -1;
        return false;
    }
    if (focus_ < 0) focus_ = order_.front();
    const GridCell cell = cell_of(focus_);
    const int cat_index = category_of(focus_);
    const BrowserCategory& cat = categories_[static_cast<std::size_t>(cat_index)];

    // The number of cars in a grid column.
    const auto column_height = [&](int column) {
        for (const BrowserCategory& c : categories_) {
            if (column < c.first_column + c.column_count) {
                const int before = (column - c.first_column) * rows_;
                return std::min(rows_, static_cast<int>(c.cars.size()) - before);
            }
        }
        return 0;
    };

    switch (dir) {
        case NavDir::Left:
        case NavDir::Right: {
            const int step = dir == NavDir::Right ? 1 : -1;
            const int column = ((cell.column + step) % total_columns_ + total_columns_) % total_columns_;
            const int row = std::min(sticky_row_, column_height(column) - 1);
            focus_ = car_at(column, row);
            break;
        }
        case NavDir::Up:
        case NavDir::Down: {
            const int h = column_height(cell.column);
            const int row = ((cell.row + (dir == NavDir::Down ? 1 : -1)) % h + h) % h;
            focus_ = car_at(cell.column, row);
            sticky_row_ = row;
            break;
        }
        case NavDir::PrevCategory:
        case NavDir::NextCategory: {
            const int n = static_cast<int>(categories_.size());
            const int target = ((cat_index + (dir == NavDir::NextCategory ? 1 : -1)) % n + n) % n;
            focus_ = categories_[static_cast<std::size_t>(target)].cars.front();
            sticky_row_ = 0;
            break;
        }
    }
    (void)cat;
    return true;
}

// ---- scrolling ------------------------------------------------------------------

void CarBrowser::scroll_to_focus(int visible_columns) {
    visible_columns = std::max(visible_columns, 1);
    const GridCell cell = focus_cell();
    if (!cell.valid()) return;
    const int margin = visible_columns >= 4 ? 1 : 0;
    int first = first_column_;
    if (cell.column < first + margin) first = cell.column - margin;
    if (cell.column > first + visible_columns - 1 - margin) first = cell.column - (visible_columns - 1 - margin);
    set_first_column(first, visible_columns);
}

void CarBrowser::scroll_by(int delta, int visible_columns) { set_first_column(first_column_ + delta, visible_columns); }

void CarBrowser::set_first_column(int column, int visible_columns) {
    visible_columns = std::max(visible_columns, 1);
    first_column_ = std::clamp(column, 0, std::max(0, total_columns_ - visible_columns));
}

BrowserView CarBrowser::view(int visible_columns, int overscan) const {
    BrowserView v;
    visible_columns = std::max(visible_columns, 1);
    v.first_column = std::clamp(first_column_, 0, std::max(0, total_columns_ - visible_columns));
    v.column_count = visible_columns;
    if (total_columns_ == 0) return v;
    const int lo = std::max(0, v.first_column - std::max(overscan, 0));
    const int hi = std::min(total_columns_ - 1, v.first_column + visible_columns - 1 + std::max(overscan, 0));
    for (std::size_t ci = 0; ci < categories_.size(); ++ci) {
        const BrowserCategory& cat = categories_[ci];
        const int cat_lo = cat.first_column;
        const int cat_hi = cat.first_column + cat.column_count - 1;
        if (cat_hi < lo || cat_lo > hi) continue;
        v.headers.push_back(VisibleHeader{static_cast<int>(ci), cat.first_column, cat.column_count});
        for (int column = std::max(lo, cat_lo); column <= std::min(hi, cat_hi); ++column) {
            for (int row = 0; row < rows_; ++row) {
                const std::size_t slot = static_cast<std::size_t>((column - cat.first_column) * rows_ + row);
                if (slot >= cat.cars.size()) break;
                v.tiles.push_back(VisibleTile{cat.cars[slot], column, row});
            }
        }
    }
    return v;
}

// ---- thumbnails -------------------------------------------------------------------

std::string thumbnail_key(const std::string& model_path, const std::string& paint, const std::string& rim, int render_version) {
    std::string stem = std::filesystem::path(model_path).stem().string();
    for (char& c : stem) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    const auto hex = [](const std::string& colour) {
        std::string out;
        for (const char c : colour) {
            if (c != '#') out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return out;
    };
    return stem + "_" + hex(paint) + "_" + hex(rim) + "_v" + std::to_string(render_version);
}

} // namespace rg
