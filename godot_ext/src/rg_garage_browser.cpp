// godot_ext/src/rg_garage_browser.cpp - the car browser half of RgGarage (PLAN.md R6c).
// Every decision lives in rg::CarBrowser (core/include/rg/car_browser.h); this file converts
// its view-models to Godot Variants and forwards the player's choices.
#include "rg_garage.h"

#include "rg/settings.h"

#include <godot_cpp/core/class_db.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <utility>

using godot::D_METHOD;
using godot::String;

namespace rg_godot {

namespace {

std::string to_std(const String& s) { return std::string(s.utf8().get_data()); }
String from_std(const std::string& s) { return String::utf8(s.c_str()); }

std::optional<rg::NavDir> nav_from_string(const std::string& s) {
    if (s == "left") return rg::NavDir::Left;
    if (s == "right") return rg::NavDir::Right;
    if (s == "up") return rg::NavDir::Up;
    if (s == "down") return rg::NavDir::Down;
    if (s == "prev_category") return rg::NavDir::PrevCategory;
    if (s == "next_category") return rg::NavDir::NextCategory;
    return std::nullopt;
}

std::string join_csv(const std::set<std::string>& values) {
    std::string out;
    for (const std::string& v : values) {
        if (!out.empty()) out += ',';
        out += v;
    }
    return out;
}

} // namespace

void RgGarage::browser_load_state(const godot::Dictionary& state) {
    if (garage_ == nullptr) return;
    rg::Settings s; // defaults for every key the dictionary leaves out
    const auto put = [&](const char* dict_key, const char* setting_key) {
        if (!state.has(dict_key)) return;
        const godot::Variant v = state[dict_key];
        switch (v.get_type()) {
            case godot::Variant::BOOL: s.set(setting_key, static_cast<bool>(v)); break;
            case godot::Variant::STRING:
            case godot::Variant::STRING_NAME: s.set(setting_key, to_std(String(v))); break;
            default: break;
        }
    };
    put("group_by", "browser.group_by");
    put("sort_key", "browser.sort_key");
    put("sort_desc", "browser.sort_desc");
    put("filter_layouts", "browser.filter_layouts");
    put("filter_body_types", "browser.filter_body_types");
    put("filter_power_bands", "browser.filter_power_bands");
    garage_->browser().apply_settings(s);
}

godot::Dictionary RgGarage::browser_get_state() const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::BrowserViewState& st = garage_->browser().state();
    d["group_by"] = String(rg::to_string(st.group_by));
    d["sort_key"] = String(rg::to_string(st.sort_key));
    d["sort_desc"] = st.sort_descending;
    d["filter_layouts"] = from_std(join_csv(st.filter.layouts));
    d["filter_body_types"] = from_std(join_csv(st.filter.body_types));
    d["filter_power_bands"] = from_std(join_csv(st.filter.power_bands));
    d["filter_active"] = st.filter.active();
    d["group_label"] = from_std(rg::group_by_label(st.group_by));
    d["sort_label"] = from_std(rg::sort_key_label(st.sort_key));
    return d;
}

godot::Array RgGarage::browser_get_group_choices() const {
    godot::Array out;
    for (const std::string& id : rg::group_by_names()) {
        godot::Dictionary d;
        d["id"] = from_std(id);
        d["label"] = from_std(rg::group_by_label(*rg::group_by_from_string(id)));
        out.push_back(d);
    }
    return out;
}

godot::Array RgGarage::browser_get_sort_choices() const {
    godot::Array out;
    for (const std::string& id : rg::sort_key_names()) {
        godot::Dictionary d;
        d["id"] = from_std(id);
        d["label"] = from_std(rg::sort_key_label(*rg::sort_key_from_string(id)));
        out.push_back(d);
    }
    return out;
}

bool RgGarage::browser_set_group_by(const String& id) {
    if (garage_ == nullptr) return false;
    const auto g = rg::group_by_from_string(to_std(id));
    if (!g) return false;
    garage_->browser().set_group_by(*g);
    return true;
}

void RgGarage::browser_set_sort(const String& key, bool descending) {
    if (garage_ == nullptr) return;
    if (const auto k = rg::sort_key_from_string(to_std(key))) garage_->browser().set_sort(*k, descending);
}

void RgGarage::browser_toggle_filter(const String& facet, const String& value) {
    if (garage_ != nullptr) garage_->browser().toggle_filter_value(to_std(facet), to_std(value));
}

void RgGarage::browser_clear_filter() {
    if (garage_ != nullptr) garage_->browser().clear_filter();
}

godot::Array RgGarage::browser_get_filter_options(const String& facet) const {
    godot::Array out;
    if (garage_ == nullptr) return out;
    for (const rg::FilterOption& o : garage_->browser().filter_options(to_std(facet))) {
        godot::Dictionary d;
        d["value"] = from_std(o.value);
        d["label"] = from_std(o.label);
        d["count"] = static_cast<int64_t>(o.count);
        d["selected"] = o.selected;
        out.push_back(d);
    }
    return out;
}

void RgGarage::browser_set_rows(int rows) {
    if (garage_ != nullptr) garage_->browser().set_rows(rows);
}

int RgGarage::browser_rows_for_height(double available_px, double tile_px, double gap_px, double header_px) const {
    return rg::CarBrowser::rows_for_height(available_px, tile_px, gap_px, header_px);
}

godot::Dictionary RgGarage::browser_get_layout() const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::CarBrowser& b = garage_->browser();
    d["rows"] = static_cast<int64_t>(b.rows());
    d["total_columns"] = static_cast<int64_t>(b.total_columns());
    d["listed"] = static_cast<int64_t>(b.listed_count());
    d["total"] = static_cast<int64_t>(b.cars().size());
    d["first_column"] = static_cast<int64_t>(b.first_column());
    d["filter_active"] = b.state().filter.active();
    godot::Array cats;
    for (const rg::BrowserCategory& c : b.categories()) {
        godot::Dictionary cd;
        cd["key"] = from_std(c.key);
        cd["title"] = from_std(c.title);
        cd["first_column"] = static_cast<int64_t>(c.first_column);
        cd["column_count"] = static_cast<int64_t>(c.column_count);
        cd["count"] = static_cast<int64_t>(c.cars.size());
        cats.push_back(cd);
    }
    d["categories"] = cats;
    return d;
}

godot::Dictionary RgGarage::browser_get_view(int visible_columns, int overscan) const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::CarBrowser& b = garage_->browser();
    const rg::BrowserView v = b.view(visible_columns, overscan);
    d["first_column"] = static_cast<int64_t>(v.first_column);
    godot::Array tiles;
    for (const rg::VisibleTile& t : v.tiles) {
        godot::Dictionary td;
        td["id"] = from_std(b.cars()[static_cast<std::size_t>(t.car)].id);
        td["column"] = static_cast<int64_t>(t.column);
        td["row"] = static_cast<int64_t>(t.row);
        tiles.push_back(td);
    }
    d["tiles"] = tiles;
    godot::Array headers;
    for (const rg::VisibleHeader& h : v.headers) {
        const rg::BrowserCategory& c = b.categories()[static_cast<std::size_t>(h.category)];
        godot::Dictionary hd;
        hd["category"] = static_cast<int64_t>(h.category);
        hd["title"] = from_std(c.title);
        hd["first_column"] = static_cast<int64_t>(h.first_column);
        hd["column_count"] = static_cast<int64_t>(h.column_count);
        hd["count"] = static_cast<int64_t>(c.cars.size());
        headers.push_back(hd);
    }
    d["headers"] = headers;
    return d;
}

godot::PackedStringArray RgGarage::browser_get_listed_ids() const {
    godot::PackedStringArray out;
    if (garage_ == nullptr) return out;
    const rg::CarBrowser& b = garage_->browser();
    for (const rg::BrowserCategory& c : b.categories()) {
        for (const int i : c.cars) out.push_back(from_std(b.cars()[static_cast<std::size_t>(i)].id));
    }
    return out;
}

bool RgGarage::browser_move_focus(const String& dir) {
    if (garage_ == nullptr) return false;
    const auto d = nav_from_string(to_std(dir));
    return d && garage_->browser().move_focus(*d);
}

bool RgGarage::browser_set_focus(const String& id) {
    return garage_ != nullptr && garage_->browser().set_focus(to_std(id));
}

String RgGarage::browser_get_focus_id() const { return garage_ ? from_std(garage_->browser().focus_id()) : String(); }

godot::Dictionary RgGarage::browser_get_focus_cell() const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::GridCell c = garage_->browser().focus_cell();
    if (!c.valid()) return d;
    d["column"] = static_cast<int64_t>(c.column);
    d["row"] = static_cast<int64_t>(c.row);
    return d;
}

void RgGarage::browser_scroll_to_focus(int visible_columns) {
    if (garage_ != nullptr) garage_->browser().scroll_to_focus(visible_columns);
}

void RgGarage::browser_scroll_by(int delta, int visible_columns) {
    if (garage_ != nullptr) garage_->browser().scroll_by(delta, visible_columns);
}

void RgGarage::browser_set_first_column(int column, int visible_columns) {
    if (garage_ != nullptr) garage_->browser().set_first_column(column, visible_columns);
}

int RgGarage::browser_get_first_column() const { return garage_ ? garage_->browser().first_column() : 0; }

godot::Dictionary RgGarage::browser_get_car(const String& id) const {
    godot::Dictionary d;
    if (garage_ == nullptr) return d;
    const rg::BrowserCar* c = garage_->browser().find(to_std(id));
    if (c == nullptr) return d;
    d["id"] = from_std(c->id);
    d["title"] = from_std(c->title);
    d["subtitle"] = from_std(c->subtitle);
    d["body_type"] = from_std(c->body_type);
    d["body_type_label"] = from_std(rg::body_type_label(c->body_type));
    d["manufacturer"] = from_std(c->manufacturer);
    d["layout"] = from_std(c->layout);
    d["preset"] = c->preset;
    d["paint"] = from_std(c->paint);
    d["rim"] = from_std(c->rim);
    d["model_path"] = from_std(c->model_path);
    d["power_kw"] = c->power_kw;
    d["torque_nm"] = c->torque_nm;
    d["mass_kg"] = c->mass_kg;
    d["thumbnail_key"] = from_std(rg::thumbnail_key(c->model_path, c->paint, c->rim));
    return d;
}

godot::Dictionary RgGarage::browser_get_maxima() const {
    godot::Dictionary d;
    double kw = 1.0, nm = 1.0;
    if (garage_ != nullptr) {
        for (const rg::BrowserCar& c : garage_->browser().cars()) {
            kw = std::max(kw, c.power_kw);
            nm = std::max(nm, c.torque_nm);
        }
    }
    d["power_kw"] = kw;
    d["torque_nm"] = nm;
    return d;
}

void RgGarage::browser_refresh() {
    if (garage_ != nullptr) garage_->refresh_browser();
}

void RgGarage::bind_browser_methods() {
    using godot::ClassDB;
    ClassDB::bind_method(D_METHOD("browser_load_state", "state"), &RgGarage::browser_load_state);
    ClassDB::bind_method(D_METHOD("browser_get_state"), &RgGarage::browser_get_state);
    ClassDB::bind_method(D_METHOD("browser_get_group_choices"), &RgGarage::browser_get_group_choices);
    ClassDB::bind_method(D_METHOD("browser_get_sort_choices"), &RgGarage::browser_get_sort_choices);
    ClassDB::bind_method(D_METHOD("browser_set_group_by", "id"), &RgGarage::browser_set_group_by);
    ClassDB::bind_method(D_METHOD("browser_set_sort", "key", "descending"), &RgGarage::browser_set_sort);
    ClassDB::bind_method(D_METHOD("browser_toggle_filter", "facet", "value"), &RgGarage::browser_toggle_filter);
    ClassDB::bind_method(D_METHOD("browser_clear_filter"), &RgGarage::browser_clear_filter);
    ClassDB::bind_method(D_METHOD("browser_get_filter_options", "facet"), &RgGarage::browser_get_filter_options);
    ClassDB::bind_method(D_METHOD("browser_set_rows", "rows"), &RgGarage::browser_set_rows);
    ClassDB::bind_method(D_METHOD("browser_rows_for_height", "available_px", "tile_px", "gap_px", "header_px"),
                         &RgGarage::browser_rows_for_height);
    ClassDB::bind_method(D_METHOD("browser_get_layout"), &RgGarage::browser_get_layout);
    ClassDB::bind_method(D_METHOD("browser_get_view", "visible_columns", "overscan"), &RgGarage::browser_get_view);
    ClassDB::bind_method(D_METHOD("browser_get_listed_ids"), &RgGarage::browser_get_listed_ids);
    ClassDB::bind_method(D_METHOD("browser_move_focus", "dir"), &RgGarage::browser_move_focus);
    ClassDB::bind_method(D_METHOD("browser_set_focus", "id"), &RgGarage::browser_set_focus);
    ClassDB::bind_method(D_METHOD("browser_get_focus_id"), &RgGarage::browser_get_focus_id);
    ClassDB::bind_method(D_METHOD("browser_get_focus_cell"), &RgGarage::browser_get_focus_cell);
    ClassDB::bind_method(D_METHOD("browser_scroll_to_focus", "visible_columns"), &RgGarage::browser_scroll_to_focus);
    ClassDB::bind_method(D_METHOD("browser_scroll_by", "delta", "visible_columns"), &RgGarage::browser_scroll_by);
    ClassDB::bind_method(D_METHOD("browser_set_first_column", "column", "visible_columns"), &RgGarage::browser_set_first_column);
    ClassDB::bind_method(D_METHOD("browser_get_first_column"), &RgGarage::browser_get_first_column);
    ClassDB::bind_method(D_METHOD("browser_get_car", "id"), &RgGarage::browser_get_car);
    ClassDB::bind_method(D_METHOD("browser_get_maxima"), &RgGarage::browser_get_maxima);
    ClassDB::bind_method(D_METHOD("browser_refresh"), &RgGarage::browser_refresh);
}

} // namespace rg_godot
