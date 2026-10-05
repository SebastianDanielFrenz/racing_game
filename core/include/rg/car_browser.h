// rg/car_browser.h - rg::CarBrowser: the model of the car browser (PLAN.md R6c).
// Engine-neutral. The browser is the one screen the garage (main menu) and the pause
// menu's "Change car" share: a grid of car tiles that scrolls horizontally, grouped
// under category headers, with the stats of the focused car beside it.
//
// This class decides everything the screen shows; the Godot layer only draws it:
//   - the cars (one BrowserCar per catalog entry - a preset is a car of its own),
//   - the categories (COMPUTED from the cars by a group-by mode, never authored),
//   - the filter and the sort inside a category,
//   - the grid: cars fill a category column by column, `rows` cars tall (2..4,
//     chosen from the height the screen has), headers sit above their columns,
//   - the focus and how arrow keys / the D-pad / the stick move it across tiles and
//     category boundaries,
//   - the horizontal scroll position and which tiles are visible (the UI instantiates
//     only those, so a catalog of hundreds of cars stays smooth),
//   - the persisted view state (group-by, sort, filters) as rg::Settings keys,
//   - the thumbnail cache key of a car's preview image.
//
// Grid and navigation rules (all tested, tests/unit/test_car_browser.cpp):
//   - A category with n cars takes ceil(n / rows) columns; car k sits in column
//     k / rows, row k % rows of its category. The last column may be short.
//   - Left/Right move one column, keeping the row the player is aiming at ("sticky
//     row": a short column clamps it but remembers it) and cross category boundaries
//     seamlessly; past either end they wrap to the other end.
//   - Up/Down move one row within the column and wrap inside it.
//   - PrevCategory/NextCategory (bumpers) jump to the first car of the previous/next
//     category, wrapping.
//   - After any change of the filter, the focus stays on its car when that car is
//     still listed, else it moves to the first listed car.
#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace rg {

class Settings;

// What the browser knows about one catalog entry (built from the catalog + stats +
// the entry's effective paint by rg::Garage::browser_cars).
struct BrowserCar {
    std::string id;
    std::string title;
    std::string subtitle;
    std::string body_type = "other"; // lower_snake
    std::string manufacturer;        // "" = none
    std::string layout = "other";    // "FWD" | "RWD" | "AWD" | "other"
    std::string model_path;          // the 3D model (thumbnail source)
    std::string paint;               // "#rrggbb" of the preview
    std::string rim;
    bool preset = false;             // a preset of another car
    double power_kw = 0.0;           // peak, 0 = unknown
    double power_rpm = 0.0;
    double torque_nm = 0.0;
    double torque_rpm = 0.0;
    double mass_kg = 0.0;
    bool displacement_known = false;
    double displacement_l = 0.0;
};

// kW per tonne, 0 when power or mass is unknown.
double power_to_weight_kw_per_t(const BrowserCar& c);

enum class GroupBy { BodyType, DriveLayout, PowerBand, Manufacturer, None };
enum class SortKey { Name, Power, Torque, Mass, PowerToWeight, Displacement };
enum class NavDir { Left, Right, Up, Down, PrevCategory, NextCategory };

const char* to_string(GroupBy g); // "body_type" | "drive_layout" | "power_band" | "manufacturer" | "none"
const char* to_string(SortKey k); // "name" | "power" | "torque" | "mass" | "power_to_weight" | "displacement"
std::optional<GroupBy> group_by_from_string(const std::string& s);
std::optional<SortKey> sort_key_from_string(const std::string& s);
const std::vector<std::string>& group_by_names();
const std::vector<std::string>& sort_key_names();
std::string group_by_label(GroupBy g); // "Body type", "Drive layout", ...
std::string sort_key_label(SortKey k);

// Power bands (kW): under 100 | 100-199 | 200-399 | 400-699 | 700+. Ids are stable
// ("under_100", "100_199", "200_399", "400_699", "700_plus") - they are persisted.
inline constexpr int kPowerBandCount = 5;
int power_band_index(double power_kw); // 0..4; an unknown (0) power is band 0
std::string power_band_id(int index);
std::string power_band_label(int index);

// A "body_type" id as a title: "hypercar" -> "Hypercar", "pickup_truck" -> "Pickup truck".
std::string body_type_label(const std::string& body_type);

struct BrowserFilter {
    std::set<std::string> layouts;     // "FWD"/"RWD"/"AWD"/"other"; empty = every layout
    std::set<std::string> body_types;  // empty = every body type
    std::set<std::string> power_bands; // band ids; empty = every band
    [[nodiscard]] bool active() const { return !layouts.empty() || !body_types.empty() || !power_bands.empty(); }
    [[nodiscard]] bool accepts(const BrowserCar& c) const;
};

struct BrowserViewState {
    GroupBy group_by = GroupBy::BodyType;
    SortKey sort_key = SortKey::Name;
    bool sort_descending = false;
    BrowserFilter filter;
};

// One value of a filter facet, for the filter panel: how many cars (of ALL cars) have it.
struct FilterOption {
    std::string value; // what BrowserFilter holds
    std::string label; // what the panel shows
    int count = 0;
    bool selected = false;
};

struct BrowserCategory {
    std::string key;           // the group value ("sedan", "AWD", "200_399", "Other", "")
    std::string title;         // header text ("" with GroupBy::None)
    std::vector<int> cars;     // indices into cars(), in display order
    int first_column = 0;      // grid column of its first car
    int column_count = 0;
};

struct GridCell {
    int column = -1;
    int row = -1;
    [[nodiscard]] bool valid() const { return column >= 0; }
};

struct VisibleTile {
    int car = -1; // index into cars()
    int column = 0;
    int row = 0;
};

struct VisibleHeader {
    int category = 0;     // index into categories()
    int first_column = 0; // the category's first column (may lie left of the view: pin it)
    int column_count = 0;
};

struct BrowserView {
    int first_column = 0; // the scroll position the lists were built for
    int column_count = 0; // columns asked for
    std::vector<VisibleTile> tiles;
    std::vector<VisibleHeader> headers;
};

class CarBrowser {
public:
    CarBrowser() = default;
    explicit CarBrowser(std::vector<BrowserCar> cars);

    // Replaces the cars (e.g. a car's paint changed in the configurator). The view
    // state, the focus (when its car still exists) and the scroll position are kept.
    void set_cars(std::vector<BrowserCar> cars);
    [[nodiscard]] const std::vector<BrowserCar>& cars() const { return cars_; }
    [[nodiscard]] const BrowserCar* find(const std::string& id) const;

    // ---- view state ----
    [[nodiscard]] const BrowserViewState& state() const { return state_; }
    void set_group_by(GroupBy g);
    void set_sort(SortKey key, bool descending);
    void set_filter(const BrowserFilter& f);
    void toggle_filter_value(const std::string& facet, const std::string& value); // facet: "layout" | "body_type" | "power_band"
    void clear_filter();
    // Panel content: every value of a facet present among ALL cars, selection marked.
    [[nodiscard]] std::vector<FilterOption> filter_options(const std::string& facet) const;

    // The persisted state as rg::Settings keys: browser.group_by, browser.sort_key,
    // browser.sort_desc, browser.filter_layouts, browser.filter_body_types,
    // browser.filter_power_bands (filters as comma-separated lists). Unknown or invalid
    // values keep the current state of that field.
    void apply_settings(const Settings& settings);
    void store_settings(Settings& settings) const;

    // ---- the grid ----
    void set_rows(int rows); // clamped to [kMinRows, kMaxRows]
    [[nodiscard]] int rows() const { return rows_; }
    static constexpr int kMinRows = 2;
    static constexpr int kMaxRows = 4;
    // Rows that fit `available_px` (tiles of `tile_px` height with `gap_px` between, minus the header
    // band), clamped to [kMinRows, kMaxRows].
    static int rows_for_height(double available_px, double tile_px, double gap_px, double header_px);

    [[nodiscard]] const std::vector<BrowserCategory>& categories() const { return categories_; }
    [[nodiscard]] std::size_t listed_count() const { return order_.size(); } // cars passing the filter
    [[nodiscard]] int total_columns() const { return total_columns_; }
    // The car index at a grid cell, or -1.
    [[nodiscard]] int car_at(int column, int row) const;
    [[nodiscard]] GridCell cell_of(int car_index) const;
    [[nodiscard]] int category_of(int car_index) const;

    // ---- focus ----
    [[nodiscard]] int focus_index() const { return focus_; } // index into cars(), -1 when nothing is listed
    [[nodiscard]] std::string focus_id() const;
    [[nodiscard]] GridCell focus_cell() const;
    bool set_focus(const std::string& id); // false when the car is not listed
    // Moves the focus; false when nothing is listed (the focus stays unset).
    bool move_focus(NavDir dir);

    // ---- scrolling and virtualisation ----
    [[nodiscard]] int first_column() const { return first_column_; }
    // Keeps the focused column inside [first_column, first_column + visible_columns) with one column of margin.
    void scroll_to_focus(int visible_columns);
    // Moves the view by `delta` columns, clamped to the grid.
    void scroll_by(int delta, int visible_columns);
    void set_first_column(int column, int visible_columns);
    // The tiles and headers visible with `visible_columns` columns from the current scroll position
    // (plus `overscan` extra columns on both sides, for smooth scrolling).
    [[nodiscard]] BrowserView view(int visible_columns, int overscan = 0) const;

private:
    void rebuild(const std::string& keep_focus_id);

    std::vector<BrowserCar> cars_;
    BrowserViewState state_;
    int rows_ = 3;
    std::vector<int> order_; // listed cars in category order
    std::vector<BrowserCategory> categories_;
    std::vector<int> car_category_; // per cars_ index, -1 when filtered out
    std::vector<int> car_slot_;     // per cars_ index: position inside its category
    int total_columns_ = 0;
    int focus_ = -1;
    int sticky_row_ = 0;
    int first_column_ = 0;
};

// The cache key of a car's preview image: model file stem, paint and rim colours and the render
// version, e.g. "car_sedan_7a8a99_c4c8cf_v1" - a file name safe string. Bump kThumbnailRenderVersion
// whenever the preview's camera, lights or paint shader change, so old PNGs are not reused.
inline constexpr int kThumbnailRenderVersion = 1;
std::string thumbnail_key(const std::string& model_path, const std::string& paint, const std::string& rim,
                          int render_version = kThumbnailRenderVersion);

} // namespace rg
