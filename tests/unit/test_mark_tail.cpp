// test_mark_tail.cpp - the post-trigger window of owner drive marks (rg/mark_tail.h).
// Sabotage that fails each case: complete only when t - start > window (C1), flush returning the full window (C2).

#include "rg/mark_tail.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("mark tail: a window completes when its time is full", "[mark_tail]") {
    rg::MarkTail tail;
    const auto id = tail.begin(100.0, rg::kDefaultPostMarkWindowS);
    for (double t = 100.1; t < 109.95; t += 0.1) REQUIRE(tail.advance(t).empty());
    const auto done = tail.advance(110.0);
    REQUIRE(done.size() == 1);
    CHECK(done[0].id == id);
    CHECK(done[0].complete);
    CHECK(done[0].covered_s == Catch::Approx(10.0));
    CHECK(tail.empty());
}

TEST_CASE("mark tail: flush closes open windows with the shorter coverage", "[mark_tail]") {
    rg::MarkTail tail;
    const auto a = tail.begin(10.0, 10.0);
    tail.advance(12.0);
    const auto b = tail.begin(12.0, 10.0);
    tail.advance(14.5);
    REQUIRE(tail.open_ids().size() == 2);
    const auto done = tail.flush();
    REQUIRE(done.size() == 2);
    CHECK(done[0].id == a);
    CHECK_FALSE(done[0].complete);
    CHECK(done[0].covered_s == Catch::Approx(4.5));
    CHECK(done[1].id == b);
    CHECK(done[1].covered_s == Catch::Approx(2.5));
    CHECK(tail.empty());
    CHECK(tail.flush().empty());
}

#include <filesystem>
#include <fstream>

TEST_CASE("mark tail: the window comes from the config file and is validated", "[mark_tail]") {
    const auto dir = std::filesystem::temp_directory_path();
    auto write = [&](const char* name, const char* text) {
        const auto p = (dir / name).string();
        std::ofstream(p) << text;
        return p;
    };
    std::string err;
    CHECK(rg::load_post_mark_window_s(write("rg_mt_a.json", "{\"post_window_s\": 3.5}"), &err) == Catch::Approx(3.5));
    CHECK(err.empty());
    CHECK(rg::load_post_mark_window_s(write("rg_mt_b.json", "{\"post_window_s\": 0}"), &err) == 0.0);
    CHECK(err.empty());
    CHECK(rg::load_post_mark_window_s(write("rg_mt_c.json", "{\"post_window_s\": -1}"), &err) == rg::kDefaultPostMarkWindowS);
    CHECK_FALSE(err.empty());
    CHECK(rg::load_post_mark_window_s(write("rg_mt_d.json", "{\"post_window_s\": \"x\"}"), &err) == rg::kDefaultPostMarkWindowS);
    CHECK(rg::load_post_mark_window_s((dir / "rg_mt_missing.json").string(), &err) == rg::kDefaultPostMarkWindowS);
    CHECK_FALSE(err.empty());
    // The shipped file.
    CHECK(rg::load_post_mark_window_s(std::string(RG_SOURCE_DIR) + "/data/world/drive_marks.json", &err) == Catch::Approx(10.0));
}
