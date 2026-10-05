// test_car_browser.cpp - rg::CarBrowser (PLAN.md R6c): categorisation per group mode,
// filtering, sorting, grid navigation, scrolling/virtualisation, persisted view state,
// thumbnail keys. Each test names the sabotage that makes it fail ("sabotage:").
#include "rg/car_browser.h"

#include "rg/settings.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

namespace {

rg::BrowserCar car(const std::string& id, const std::string& body, const std::string& layout, double kw, double nm,
                   double kg, double litres = 0.0, const std::string& maker = "") {
    rg::BrowserCar c;
    c.id = id;
    c.title = id; // titles equal ids in the fixtures; "Z" prefix ordering is by lower-case name
    c.body_type = body;
    c.layout = layout;
    c.power_kw = kw;
    c.torque_nm = nm;
    c.mass_kg = kg;
    c.displacement_known = litres > 0.0;
    c.displacement_l = litres;
    c.manufacturer = maker;
    c.paint = "#7a8a99";
    c.rim = "#c4c8cf";
    c.model_path = "x/" + id + ".glb";
    return c;
}

// Eight cars, mixed everything.
std::vector<rg::BrowserCar> small_set() {
    return {
        car("alpha", "sedan", "FWD", 90.0, 200.0, 1300.0, 1.6, "Acme"),
        car("bravo", "sedan", "RWD", 150.0, 300.0, 1400.0, 2.0, "Acme"),
        car("charlie", "sedan", "AWD", 250.0, 400.0, 1600.0, 3.0),
        car("delta", "sports", "RWD", 300.0, 450.0, 1300.0, 3.5, "Bolt"),
        car("echo", "sports", "AWD", 450.0, 600.0, 1500.0, 4.0, "Bolt"),
        car("foxtrot", "hypercar", "RWD", 1000.0, 1371.0, 1500.0, 3.9),
        car("golf", "race", "RWD", 0.0, 0.0, 900.0), // unknown power/torque/displacement
        car("hotel", "race", "AWD", 800.0, 900.0, 1100.0, 5.0, "Bolt"),
    };
}

// A synthetic catalog of n cars with spread-out attributes.
std::vector<rg::BrowserCar> synthetic(int n) {
    static const char* bodies[] = {"sedan", "hatchback", "estate", "pickup", "suv", "sports", "hypercar", "race", "utility"};
    static const char* layouts[] = {"FWD", "RWD", "AWD"};
    static const char* makers[] = {"", "Acme", "Bolt", "Crest", "Dyne"};
    std::vector<rg::BrowserCar> out;
    for (int i = 0; i < n; ++i) {
        char id[32];
        std::snprintf(id, sizeof id, "car_%03d", i);
        out.push_back(car(id, bodies[(i * 7) % 9], layouts[(i * 5) % 3], 60.0 + (i * 37) % 900, 150.0 + (i * 53) % 1200,
                          900.0 + (i * 29) % 1200, 1.0 + (i % 40) * 0.15, makers[i % 5]));
    }
    return out;
}

std::vector<std::string> ids_in_order(const rg::CarBrowser& b) {
    std::vector<std::string> out;
    for (const rg::BrowserCategory& cat : b.categories()) {
        for (const int i : cat.cars) out.push_back(b.cars()[static_cast<std::size_t>(i)].id);
    }
    return out;
}

std::vector<std::string> category_titles(const rg::CarBrowser& b) {
    std::vector<std::string> out;
    for (const rg::BrowserCategory& cat : b.categories()) out.push_back(cat.title);
    return out;
}

std::vector<std::string> ids_of(const rg::CarBrowser& b, const std::string& category_key) {
    for (const rg::BrowserCategory& cat : b.categories()) {
        if (cat.key != category_key) continue;
        std::vector<std::string> out;
        for (const int i : cat.cars) out.push_back(b.cars()[static_cast<std::size_t>(i)].id);
        return out;
    }
    return {};
}

} // namespace

using Ids = std::vector<std::string>;

TEST_CASE("browser: power bands and body type labels", "[car_browser]") {
    // sabotage: moving an edge (>= 200 -> > 200) fails the 200.0 check
    CHECK(rg::power_band_index(0.0) == 0);
    CHECK(rg::power_band_index(99.9) == 0);
    CHECK(rg::power_band_index(100.0) == 1);
    CHECK(rg::power_band_index(199.9) == 1);
    CHECK(rg::power_band_index(200.0) == 2);
    CHECK(rg::power_band_index(400.0) == 3);
    CHECK(rg::power_band_index(699.0) == 3);
    CHECK(rg::power_band_index(700.0) == 4);
    CHECK(rg::power_band_index(1000.2) == 4);
    CHECK(rg::body_type_label("hypercar") == "Hypercar");
    CHECK(rg::body_type_label("pickup_truck") == "Pickup truck");
    CHECK(rg::group_by_from_string("power_band") == rg::GroupBy::PowerBand);
    CHECK_FALSE(rg::group_by_from_string("nope").has_value());
    CHECK(rg::sort_key_from_string("power_to_weight") == rg::SortKey::PowerToWeight);
}

TEST_CASE("browser: grouping by body type", "[car_browser]") {
    // sabotage: grouping by manufacturer instead of body_type fails the titles
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::BodyType);
    CHECK(category_titles(b) == Ids{"Hypercar", "Race", "Sedan", "Sports"}); // alphabetical
    CHECK(ids_of(b, "sedan") == Ids{"alpha", "bravo", "charlie"});
    CHECK(ids_of(b, "race") == Ids{"golf", "hotel"});
    CHECK(b.listed_count() == 8);
}

TEST_CASE("browser: grouping by drive layout has a fixed order", "[car_browser]") {
    // sabotage: alphabetical order of the keys (AWD first) fails
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::DriveLayout);
    CHECK(category_titles(b) == Ids{"Front-wheel drive", "Rear-wheel drive", "All-wheel drive"});
    CHECK(ids_of(b, "RWD") == Ids{"bravo", "delta", "foxtrot", "golf"});
    CHECK(ids_of(b, "AWD") == Ids{"charlie", "echo", "hotel"});
}

TEST_CASE("browser: grouping by power band is ascending and skips empty bands", "[car_browser]") {
    // sabotage: a car with unknown power (0) landing in a band other than "Under 100 kW" fails the first check
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::PowerBand);
    CHECK(category_titles(b) == Ids{"Under 100 kW", "100 - 199 kW", "200 - 399 kW", "400 - 699 kW", "700 kW and more"});
    CHECK(ids_of(b, "under_100") == Ids{"alpha", "golf"});
    CHECK(ids_of(b, "700_plus") == Ids{"foxtrot", "hotel"});
    CHECK(ids_of(b, "200_399") == Ids{"charlie", "delta"});

    rg::BrowserFilter f;
    f.layouts = {"AWD"};
    b.set_filter(f); // charlie 250, echo 450, hotel 800
    CHECK(category_titles(b) == Ids{"200 - 399 kW", "400 - 699 kW", "700 kW and more"}); // the empty bands vanish
}

TEST_CASE("browser: grouping by manufacturer puts cars without one last under Other", "[car_browser]") {
    // sabotage: "Other" ordered by its title among the real manufacturers (it would precede "Zeta") fails
    auto cars = small_set();
    cars[5].manufacturer = "Zeta"; // foxtrot
    rg::CarBrowser b(cars);
    b.set_group_by(rg::GroupBy::Manufacturer);
    CHECK(category_titles(b) == Ids{"Acme", "Bolt", "Zeta", "Other"});
    CHECK(ids_of(b, "").size() == 2); // charlie, golf
    CHECK(ids_of(b, "Bolt") == Ids{"delta", "echo", "hotel"});
}

TEST_CASE("browser: no grouping is one headerless category", "[car_browser]") {
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::None);
    REQUIRE(b.categories().size() == 1);
    CHECK(b.categories()[0].title.empty());
    CHECK(b.categories()[0].cars.size() == 8);
}

TEST_CASE("browser: filters are ANDed across facets and ORed inside one", "[car_browser]") {
    // sabotage: ORing across facets (AWD or sports) lists more than the 1 car below
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::None);
    b.toggle_filter_value("layout", "AWD");
    CHECK(b.listed_count() == 3);
    CHECK(b.state().filter.active());
    b.toggle_filter_value("layout", "RWD"); // AWD or RWD
    CHECK(b.listed_count() == 7);           // everything but alpha (FWD)
    b.toggle_filter_value("layout", "RWD"); // off again
    b.toggle_filter_value("body_type", "sports");
    CHECK(b.listed_count() == 1); // echo only
    CHECK(ids_in_order(b) == Ids{"echo"});
    b.toggle_filter_value("power_band", "under_100");
    CHECK(b.listed_count() == 0);
    CHECK(b.focus_index() == -1);
    CHECK_FALSE(b.move_focus(rg::NavDir::Right)); // an empty listing never crashes
    b.clear_filter();
    CHECK(b.listed_count() == 8);
    CHECK_FALSE(b.state().filter.active());
}

TEST_CASE("browser: filter options count all cars and mark the selection", "[car_browser]") {
    rg::CarBrowser b(small_set());
    b.toggle_filter_value("layout", "AWD");
    const auto layouts = b.filter_options("layout");
    REQUIRE(layouts.size() == 3); // "other" has no car and is dropped
    CHECK(layouts[0].value == "FWD");
    CHECK(layouts[0].count == 1);
    CHECK_FALSE(layouts[0].selected);
    CHECK(layouts[2].value == "AWD");
    CHECK(layouts[2].count == 3); // counts ignore the active filter, so the panel never shrinks
    CHECK(layouts[2].selected);
    const auto types = b.filter_options("body_type");
    CHECK(types.size() == 4);
    CHECK(types[0].label == "Hypercar");
    CHECK(b.filter_options("power_band").size() == 5);
    CHECK(b.filter_options("nope").empty());
}

TEST_CASE("browser: sorting inside a category, both directions, unknown values last", "[car_browser]") {
    // sabotage: putting unknown (0) values first in ascending order fails the power check
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::None);

    b.set_sort(rg::SortKey::Name, false);
    CHECK(ids_in_order(b) == Ids{"alpha", "bravo", "charlie", "delta", "echo", "foxtrot", "golf", "hotel"});
    b.set_sort(rg::SortKey::Name, true);
    CHECK(ids_in_order(b) == Ids{"hotel", "golf", "foxtrot", "echo", "delta", "charlie", "bravo", "alpha"});

    b.set_sort(rg::SortKey::Power, false);
    CHECK(ids_in_order(b) == Ids{"alpha", "bravo", "charlie", "delta", "echo", "hotel", "foxtrot", "golf"}); // golf: unknown, last
    b.set_sort(rg::SortKey::Power, true);
    CHECK(ids_in_order(b) == Ids{"foxtrot", "hotel", "echo", "delta", "charlie", "bravo", "alpha", "golf"}); // still last

    b.set_sort(rg::SortKey::Mass, false);
    CHECK(ids_in_order(b).front() == "golf"); // 900 kg is a known value
    CHECK(ids_in_order(b).back() == "charlie");

    b.set_sort(rg::SortKey::Torque, true);
    CHECK(ids_in_order(b).front() == "foxtrot");

    // power to weight: foxtrot 1000/1500 = 667 kW/t, hotel 800/1100 = 727
    b.set_sort(rg::SortKey::PowerToWeight, true);
    CHECK(ids_in_order(b)[0] == "hotel");
    CHECK(ids_in_order(b)[1] == "foxtrot");
    CHECK(ids_in_order(b).back() == "golf");

    b.set_sort(rg::SortKey::Displacement, false);
    CHECK(ids_in_order(b).front() == "alpha"); // 1.6 L
    CHECK(ids_in_order(b).back() == "golf");   // unknown
    b.set_sort(rg::SortKey::Displacement, true);
    CHECK(ids_in_order(b).front() == "hotel"); // 5.0 L
    CHECK(ids_in_order(b).back() == "golf");
}

TEST_CASE("browser: equal sort values tie-break by name then id, deterministically", "[car_browser]") {
    // sabotage: an unstable tie-break (no title/id comparison) makes the order depend on input order
    std::vector<rg::BrowserCar> cars = {car("b", "sedan", "FWD", 100, 1, 1000), car("a", "sedan", "FWD", 100, 1, 1000),
                                        car("c", "sedan", "FWD", 100, 1, 1000)};
    rg::CarBrowser fwd(cars);
    std::reverse(cars.begin(), cars.end());
    rg::CarBrowser rev(cars);
    for (rg::CarBrowser* b : {&fwd, &rev}) {
        b->set_group_by(rg::GroupBy::None);
        b->set_sort(rg::SortKey::Power, true);
    }
    CHECK(ids_in_order(fwd) == Ids{"a", "b", "c"});
    CHECK(ids_in_order(rev) == ids_in_order(fwd));
}

TEST_CASE("browser: grid columns fill category by category, rows tall", "[car_browser]") {
    // sabotage: row-major fill instead of column-major fails the car_at checks
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_rows(2);
    // Hypercar: 1 car (1 col), Race: 2 cars (1 col), Sedan: 3 cars (2 cols), Sports: 2 cars (1 col)
    REQUIRE(b.categories().size() == 4);
    CHECK(b.categories()[0].first_column == 0);
    CHECK(b.categories()[1].first_column == 1);
    CHECK(b.categories()[2].first_column == 2);
    CHECK(b.categories()[2].column_count == 2);
    CHECK(b.categories()[3].first_column == 4);
    CHECK(b.total_columns() == 5);
    const auto id_at = [&](int col, int row) {
        const int i = b.car_at(col, row);
        return i < 0 ? std::string("-") : b.cars()[static_cast<std::size_t>(i)].id;
    };
    CHECK(id_at(0, 0) == "foxtrot");
    CHECK(id_at(0, 1) == "-"); // a short column
    CHECK(id_at(1, 0) == "golf");
    CHECK(id_at(1, 1) == "hotel");
    CHECK(id_at(2, 0) == "alpha");
    CHECK(id_at(2, 1) == "bravo");
    CHECK(id_at(3, 0) == "charlie");
    CHECK(id_at(3, 1) == "-");
    CHECK(id_at(4, 0) == "delta");
    CHECK(id_at(5, 0) == "-");
    // with 3 rows the sedans fit one column
    b.set_rows(3);
    CHECK(b.categories()[2].column_count == 1);
    b.set_rows(9);
    CHECK(b.rows() == rg::CarBrowser::kMaxRows); // clamped
    b.set_rows(0);
    CHECK(b.rows() == rg::CarBrowser::kMinRows);
}

TEST_CASE("browser: rows follow the available height", "[car_browser]") {
    // tile 150, gap 10, header 40
    CHECK(rg::CarBrowser::rows_for_height(300.0, 150.0, 10.0, 40.0) == 2);  // fits 1.6 -> clamped up to 2
    CHECK(rg::CarBrowser::rows_for_height(540.0, 150.0, 10.0, 40.0) == 3);  // 500 -> 3 (3*150+2*10 = 470; 4 needs 630)
    CHECK(rg::CarBrowser::rows_for_height(700.0, 150.0, 10.0, 40.0) == 4);
    CHECK(rg::CarBrowser::rows_for_height(5000.0, 150.0, 10.0, 40.0) == 4); // clamped down
    CHECK(rg::CarBrowser::rows_for_height(100.0, 0.0, 10.0, 40.0) == 2);    // degenerate tile height
}

TEST_CASE("browser: focus moves across rows, columns and category boundaries", "[car_browser]") {
    // sabotage: Left/Right clamping at the ends instead of wrapping fails the wrap checks;
    //           resetting the row to 0 on every column move fails the sticky-row check
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_rows(2);
    REQUIRE(b.focus_id() == "foxtrot"); // first listed

    CHECK(b.move_focus(rg::NavDir::Right));
    CHECK(b.focus_id() == "golf"); // Race column, row 0 (the Hypercar column is one car tall: row clamps)
    b.move_focus(rg::NavDir::Down);
    CHECK(b.focus_id() == "hotel");
    b.move_focus(rg::NavDir::Right); // into Sedan, keeps row 1
    CHECK(b.focus_id() == "bravo");
    b.move_focus(rg::NavDir::Right); // Sedan's second column is one car tall: the aimed-at row 1 clamps to 0 ...
    CHECK(b.focus_id() == "charlie");
    b.move_focus(rg::NavDir::Right); // ... but is remembered: Sports' column has two cars, so row 1 again
    CHECK(b.focus_id() == "echo");

    // sticky row: aim at row 1, cross a short column, come back to row 1
    b.set_focus("bravo");
    b.move_focus(rg::NavDir::Right); // charlie (row clamped to 0)
    CHECK(b.focus_id() == "charlie");
    b.move_focus(rg::NavDir::Left);
    CHECK(b.focus_id() == "bravo");
    // vertical wrap inside a column
    b.move_focus(rg::NavDir::Down);
    CHECK(b.focus_id() == "alpha");
    b.move_focus(rg::NavDir::Up);
    CHECK(b.focus_id() == "bravo");
    // horizontal wrap
    b.set_focus("delta");
    b.move_focus(rg::NavDir::Right);
    CHECK(b.focus_id() == "foxtrot");
    b.move_focus(rg::NavDir::Left);
    CHECK(b.focus_id() == "delta");
}

TEST_CASE("browser: the aimed-at row survives a short column", "[car_browser]") {
    // 3 columns of 3: ..., a column with one car, ...
    std::vector<rg::BrowserCar> cars;
    for (const char* id : {"a1", "a2", "a3", "a4", "a5", "a6"}) cars.push_back(car(id, "aaa", "FWD", 100, 1, 1000));
    cars.push_back(car("b1", "bbb", "FWD", 100, 1, 1000)); // a one-car category: one short column
    for (const char* id : {"c1", "c2", "c3"}) cars.push_back(car(id, "ccc", "FWD", 100, 1, 1000));
    rg::CarBrowser b(cars);
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_rows(3);
    b.set_focus("a2");               // row 1 of column 0
    b.move_focus(rg::NavDir::Right); // a5, column 1 row 1
    CHECK(b.focus_id() == "a5");
    b.move_focus(rg::NavDir::Right); // b1: column 2 has one car, row clamps to 0
    CHECK(b.focus_id() == "b1");
    b.move_focus(rg::NavDir::Right); // c1..c3 column: the remembered row 1 applies again
    CHECK(b.focus_id() == "c2");
    b.set_focus("a5");
    b.move_focus(rg::NavDir::Right);
    CHECK(b.focus_id() == "b1");
    // a vertical move in the short column keeps wrapping inside it (height 1)
    b.move_focus(rg::NavDir::Down);
    CHECK(b.focus_id() == "b1");
}

TEST_CASE("browser: category jumps go to the first car and wrap", "[car_browser]") {
    // sabotage: not wrapping at the last category fails the third check
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_rows(2);
    b.set_focus("bravo"); // Sedan
    b.move_focus(rg::NavDir::NextCategory);
    CHECK(b.focus_id() == "delta"); // Sports, first car
    b.move_focus(rg::NavDir::NextCategory);
    CHECK(b.focus_id() == "foxtrot"); // wrapped to Hypercar
    b.move_focus(rg::NavDir::PrevCategory);
    CHECK(b.focus_id() == "delta"); // wrapped back
    b.move_focus(rg::NavDir::PrevCategory);
    CHECK(b.focus_id() == "alpha"); // Sedan
}

TEST_CASE("browser: the focus survives a filter or sort change while its car is listed", "[car_browser]") {
    // sabotage: always resetting the focus to the first car fails the first check
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::None);
    b.set_focus("echo");
    b.set_sort(rg::SortKey::Power, true);
    CHECK(b.focus_id() == "echo");
    b.toggle_filter_value("layout", "AWD");
    CHECK(b.focus_id() == "echo");
    b.toggle_filter_value("layout", "AWD");
    b.toggle_filter_value("layout", "FWD"); // echo is AWD: no longer listed
    CHECK(b.focus_id() == "alpha");         // the first (only) listed car
    CHECK_FALSE(b.set_focus("echo"));       // an unlisted car cannot be focused
    CHECK(b.focus_id() == "alpha");
}

TEST_CASE("browser: scrolling follows the focus and clamps", "[car_browser]") {
    rg::CarBrowser b(synthetic(60));
    b.set_group_by(rg::GroupBy::None);
    b.set_rows(3);
    CHECK(b.total_columns() == 20);
    b.scroll_to_focus(6);
    CHECK(b.first_column() == 0);
    b.set_focus("car_059"); // last column (19)
    b.scroll_to_focus(6);
    CHECK(b.first_column() == 14); // the end of the grid: 20 - 6
    const rg::GridCell cell = b.focus_cell();
    CHECK(cell.column >= b.first_column());
    CHECK(cell.column < b.first_column() + 6);
    b.set_first_column(100, 6);
    CHECK(b.first_column() == 14);
    b.scroll_by(-100, 6);
    CHECK(b.first_column() == 0);
    b.scroll_by(3, 6);
    CHECK(b.first_column() == 3);
    // focus moving right past the window drags the window along with a margin
    b.set_first_column(0, 6);
    b.set_focus("car_000");
    for (int i = 0; i < 8; ++i) b.move_focus(rg::NavDir::Right);
    b.scroll_to_focus(6);
    const rg::GridCell c2 = b.focus_cell();
    CHECK(c2.column == 8);
    CHECK(c2.column <= b.first_column() + 5);
    CHECK(c2.column >= b.first_column());
}

TEST_CASE("browser: only the visible tiles are listed", "[car_browser]") {
    // sabotage: view() returning every tile fails the size bound
    rg::CarBrowser b(synthetic(200));
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_rows(3);
    b.set_first_column(5, 6);
    const rg::BrowserView v = b.view(6, 1);
    CHECK_FALSE(v.tiles.empty());
    CHECK(v.tiles.size() <= static_cast<std::size_t>(8 * 3)); // 6 + 2 overscan columns, 3 rows
    for (const rg::VisibleTile& t : v.tiles) {
        CHECK(t.column >= 4);
        CHECK(t.column <= 5 + 6);
        CHECK(b.cell_of(t.car).column == t.column);
        CHECK(b.cell_of(t.car).row == t.row);
    }
    CHECK_FALSE(v.headers.empty());
    for (const rg::VisibleHeader& h : v.headers) {
        CHECK(h.first_column + h.column_count > 4);
        CHECK(h.first_column <= 5 + 6);
    }
    // the union of all windows is every listed car, exactly once
    std::set<int> seen;
    for (int col = 0; col < b.total_columns(); col += 6) {
        b.set_first_column(col, 6);
        for (const rg::VisibleTile& t : b.view(6, 0).tiles) seen.insert(t.car); // the last window is clamped and overlaps
    }
    CHECK(seen.size() == 200);
}

TEST_CASE("browser: a 200-entry catalog groups, sorts and navigates quickly", "[car_browser]") {
    const auto cars = synthetic(200);
    rg::CarBrowser b(cars);
    const auto t0 = std::chrono::steady_clock::now();
    for (const rg::GroupBy g : {rg::GroupBy::BodyType, rg::GroupBy::DriveLayout, rg::GroupBy::PowerBand,
                                rg::GroupBy::Manufacturer, rg::GroupBy::None}) {
        b.set_group_by(g);
        for (const rg::SortKey k : {rg::SortKey::Name, rg::SortKey::Power, rg::SortKey::PowerToWeight}) {
            b.set_sort(k, true);
            CHECK(b.listed_count() == 200);
            std::size_t sum = 0;
            for (const auto& cat : b.categories()) sum += cat.cars.size();
            CHECK(sum == 200); // every car in exactly one category
        }
    }
    for (int i = 0; i < 2000; ++i) b.move_focus(i % 3 == 0 ? rg::NavDir::Down : rg::NavDir::Right);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    CHECK(ms < 500.0); // generous: it measures in single milliseconds
    // sorted power really is monotone inside every category
    b.set_group_by(rg::GroupBy::BodyType);
    b.set_sort(rg::SortKey::Power, false);
    for (const auto& cat : b.categories()) {
        for (std::size_t i = 1; i < cat.cars.size(); ++i) {
            CHECK(cars[static_cast<std::size_t>(cat.cars[i - 1])].power_kw <= cars[static_cast<std::size_t>(cat.cars[i])].power_kw);
        }
    }
}

TEST_CASE("browser: view state round-trips through the settings", "[car_browser][settings]") {
    // sabotage: not storing sort_desc, or storing the filter sets unsorted, fails the round trip
    rg::CarBrowser a(small_set());
    a.set_group_by(rg::GroupBy::PowerBand);
    a.set_sort(rg::SortKey::PowerToWeight, true);
    a.toggle_filter_value("layout", "AWD");
    a.toggle_filter_value("layout", "RWD");
    a.toggle_filter_value("body_type", "sports");
    rg::Settings s;
    a.store_settings(s);
    CHECK(s.get_string("browser.group_by") == "power_band");
    CHECK(s.get_string("browser.filter_layouts") == "AWD,RWD");
    CHECK(s.dirty());

    const std::string json = s.to_json();
    rg::Settings loaded;
    const rg::SettingsLoadReport rep = loaded.load_text(json);
    CHECK(rep.invalid_keys.empty());
    CHECK(rep.unknown_keys.empty());

    rg::CarBrowser b(small_set());
    b.apply_settings(loaded);
    CHECK(b.state().group_by == rg::GroupBy::PowerBand);
    CHECK(b.state().sort_key == rg::SortKey::PowerToWeight);
    CHECK(b.state().sort_descending);
    CHECK(b.state().filter.layouts == std::set<std::string>{"AWD", "RWD"});
    CHECK(b.state().filter.body_types == std::set<std::string>{"sports"});
    CHECK(ids_in_order(b) == ids_in_order(a));
}

TEST_CASE("browser: hostile persisted values are ignored", "[car_browser][settings]") {
    rg::Settings s;
    s.set("browser.filter_layouts", std::string("AWD,../../etc,SIDEWAYS, RWD"));
    s.set("browser.filter_power_bands", std::string("900_plus,100_199"));
    s.set("browser.filter_body_types", std::string("Sports Car,sports"));
    rg::CarBrowser b(small_set());
    b.apply_settings(s);
    CHECK(b.state().filter.layouts == std::set<std::string>{"AWD", "RWD"}); // unknown ids dropped
    CHECK(b.state().filter.power_bands == std::set<std::string>{"100_199"});
    CHECK(b.state().filter.body_types == std::set<std::string>{"sports"});
    // a choice the schema rejects cannot even be stored
    CHECK_FALSE(s.set("browser.group_by", std::string("sideways")));
}

TEST_CASE("browser: the hidden browser keys do not appear on the settings screen", "[car_browser][settings]") {
    const auto sections = rg::settings_sections();
    CHECK(std::find(sections.begin(), sections.end(), "browser") == sections.end());
    REQUIRE(rg::find_setting("browser.group_by") != nullptr);
    CHECK_FALSE(rg::find_setting("browser.group_by")->shown);
}

TEST_CASE("browser: set_cars keeps the view state and the focus", "[car_browser]") {
    rg::CarBrowser b(small_set());
    b.set_group_by(rg::GroupBy::DriveLayout);
    b.set_focus("echo");
    auto cars = small_set();
    cars[4].paint = "#ff0000"; // echo's paint changed in the configurator
    b.set_cars(cars);
    CHECK(b.state().group_by == rg::GroupBy::DriveLayout);
    CHECK(b.focus_id() == "echo");
    CHECK(b.find("echo")->paint == "#ff0000");
    cars.erase(cars.begin() + 4); // echo disappears
    b.set_cars(cars);
    CHECK(b.focus_id() != "echo");
    CHECK(b.focus_index() >= 0);
}

TEST_CASE("browser: thumbnail keys depend on model, paint, rim and render version", "[car_browser]") {
    // sabotage: leaving the render version out of the key fails the last check
    const std::string k = rg::thumbnail_key("a/b/car_sedan.glb", "#7a8a99", "#c4c8cf", 1);
    CHECK(k == "car_sedan_7a8a99_c4c8cf_v1");
    CHECK(rg::thumbnail_key("a/b/car_sedan.glb", "#7A8A99", "#c4c8cf", 1) == k); // case-insensitive colours
    CHECK(rg::thumbnail_key("other/car_hyper.glb", "#7a8a99", "#c4c8cf", 1) != k);
    CHECK(rg::thumbnail_key("a/b/car_sedan.glb", "#000000", "#c4c8cf", 1) != k);
    CHECK(rg::thumbnail_key("a/b/car_sedan.glb", "#7a8a99", "#000000", 1) != k);
    CHECK(rg::thumbnail_key("a/b/car_sedan.glb", "#7a8a99", "#c4c8cf", 2) != k);
    CHECK(rg::thumbnail_key("a/b/car_sedan.glb", "#7a8a99", "#c4c8cf") ==
          rg::thumbnail_key("a/b/car_sedan.glb", "#7a8a99", "#c4c8cf", rg::kThumbnailRenderVersion));
    // a hostile model path cannot escape the cache directory
    const std::string evil = rg::thumbnail_key("../../x y/..\\ev:il.glb", "#7a8a99", "#c4c8cf");
    CHECK(evil.find('/') == std::string::npos);
    CHECK(evil.find('\\') == std::string::npos);
    CHECK(evil.find(':') == std::string::npos);
    CHECK(evil.find(' ') == std::string::npos);
}
