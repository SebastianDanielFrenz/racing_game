// test_shell_flow.cpp - rg::ShellFlow (R5): every transition of the screen
// state machine, the refusals, the actions returned, and the menu contents.
#include "rg/shell_flow.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

using rg::Screen;
using rg::ShellActionKind;
using rg::ShellFlow;

bool has_action(const rg::ShellTransition& t, ShellActionKind k) {
    for (const auto& a : t.actions) {
        if (a.kind == k) return true;
    }
    return false;
}

const rg::ShellAction* find_action(const rg::ShellTransition& t, ShellActionKind k) {
    for (const auto& a : t.actions) {
        if (a.kind == k) return &a;
    }
    return nullptr;
}

rg::WorldRequest flat_request() {
    rg::WorldRequest w;
    w.kind = rg::WorldKind::Flat;
    w.label = "Flat test world";
    return w;
}

rg::WorldRequest real_request(double x = 100.0, double y = -200.0, double yaw = 30.0) {
    rg::WorldRequest w;
    w.kind = rg::WorldKind::RealWorld;
    w.has_spawn = true;
    w.x = x;
    w.y = y;
    w.yaw_deg = yaw;
    w.label = "A point";
    return w;
}

// Walks Boot -> MainMenu -> SpawnPicker -> Loading -> Drive.
void to_drive(ShellFlow& f) {
    REQUIRE(f.handle(ShellFlow::boot_finished()).accepted);
    REQUIRE(f.handle(ShellFlow::menu_item("free_roam")).accepted);
    REQUIRE(f.handle(ShellFlow::spawn_picked(flat_request())).accepted);
    REQUIRE(f.handle(ShellFlow::load_ready()).accepted);
    REQUIRE(f.screen() == Screen::Drive);
}

} // namespace

TEST_CASE("shell: menus hold exactly the shipped items", "[shell]") {
    std::vector<std::string> main_ids, pause_ids;
    for (const auto& m : rg::main_menu_items()) main_ids.push_back(m.id);
    for (const auto& m : rg::pause_menu_items()) pause_ids.push_back(m.id);
    CHECK(main_ids == std::vector<std::string>{"free_roam", "garage", "settings", "credits", "quit"});
    CHECK(pause_ids == std::vector<std::string>{"resume", "reset_car", "garage", "settings", "main_menu"});
    for (const auto& m : rg::main_menu_items()) CHECK_FALSE(m.label.empty());
    for (const auto& m : rg::pause_menu_items()) CHECK_FALSE(m.label.empty());
    CHECK(std::string(rg::to_string(Screen::SpawnPicker)) == "spawn_picker");
    CHECK(std::string(rg::to_string(Screen::VehicleSelect)) == "vehicle_select");
    CHECK(std::string(rg::to_string(Screen::Configurator)) == "configurator");
}

TEST_CASE("shell: boot goes to the main menu", "[shell]") {
    ShellFlow f;
    CHECK(f.screen() == Screen::Boot);
    const auto t = f.handle(ShellFlow::boot_finished());
    CHECK(t.accepted);
    CHECK(t.from == Screen::Boot);
    CHECK(t.to == Screen::MainMenu);
    REQUIRE_FALSE(t.actions.empty());
    CHECK(t.actions.back().kind == ShellActionKind::ShowScreen);
    CHECK(t.actions.back().screen == Screen::MainMenu);
    // A key press (Back) also skips the splash.
    ShellFlow g;
    CHECK(g.handle(ShellFlow::back()).accepted);
    CHECK(g.screen() == Screen::MainMenu);
}

TEST_CASE("shell: a start flag skips the menu", "[shell]") {
    ShellFlow f;
    const auto t = f.handle(ShellFlow::direct_start(real_request()));
    CHECK(t.accepted);
    CHECK(f.screen() == Screen::Loading);
    const auto* load = find_action(t, ShellActionKind::LoadWorld);
    REQUIRE(load != nullptr);
    CHECK(load->world.kind == rg::WorldKind::RealWorld);
    CHECK(load->world.has_spawn);
    CHECK(f.world().x == 100.0);
    CHECK(f.handle(ShellFlow::load_ready()).accepted);
    CHECK(f.screen() == Screen::Drive);
    CHECK(f.world_loaded());
    // DirectStart is only valid at boot.
    CHECK_FALSE(f.handle(ShellFlow::direct_start(flat_request())).accepted);
    // A non-finite spawn is refused.
    ShellFlow g;
    CHECK_FALSE(g.handle(ShellFlow::direct_start(real_request(std::nan(""), 0, 0))).accepted);
    CHECK(g.screen() == Screen::Boot);
}

TEST_CASE("shell: main menu items", "[shell]") {
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        CHECK(f.handle(ShellFlow::menu_item("free_roam")).to == Screen::SpawnPicker);
    }
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        CHECK(f.handle(ShellFlow::menu_item("settings")).to == Screen::Settings);
        CHECK(f.settings_return() == Screen::MainMenu);
    }
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        CHECK(f.handle(ShellFlow::menu_item("credits")).to == Screen::Credits);
        const auto back = f.handle(ShellFlow::back());
        CHECK(back.accepted);
        CHECK(f.screen() == Screen::MainMenu);
    }
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        const auto t = f.handle(ShellFlow::menu_item("quit"));
        CHECK(t.accepted);
        CHECK(has_action(t, ShellActionKind::Quit));
        CHECK(f.screen() == Screen::Quit);
        // Nothing moves a quitting shell.
        CHECK_FALSE(f.handle(ShellFlow::back()).accepted);
        CHECK_FALSE(f.handle(ShellFlow::boot_finished()).accepted);
    }
    // An item that is not in the main menu (pause items, garbage) is refused.
    ShellFlow f;
    f.handle(ShellFlow::boot_finished());
    for (const char* bad : {"resume", "reset_car", "main_menu", "events", "map_and_route", ""}) {
        CAPTURE(bad);
        CHECK_FALSE(f.handle(ShellFlow::menu_item(bad)).accepted);
        CHECK(f.screen() == Screen::MainMenu);
    }
    // Back at the main menu does nothing (Quit is an explicit item).
    CHECK_FALSE(f.handle(ShellFlow::back()).accepted);
}

TEST_CASE("shell: settings returns to where it was opened and saves", "[shell]") {
    ShellFlow f;
    f.handle(ShellFlow::boot_finished());
    f.handle(ShellFlow::menu_item("settings"));
    const auto t = f.handle(ShellFlow::back());
    CHECK(t.accepted);
    CHECK(has_action(t, ShellActionKind::SaveSettings));
    CHECK(f.screen() == Screen::MainMenu);

    ShellFlow g;
    to_drive(g);
    g.handle(ShellFlow::pause_toggle());
    CHECK(g.handle(ShellFlow::menu_item("settings")).to == Screen::Settings);
    CHECK(g.settings_return() == Screen::Pause);
    CHECK(g.world_loaded()); // the world stays up behind the settings
    const auto back = g.handle(ShellFlow::back());
    CHECK(has_action(back, ShellActionKind::SaveSettings));
    CHECK(g.screen() == Screen::Pause);
}

TEST_CASE("shell: free roam -> picker -> loading -> drive", "[shell]") {
    ShellFlow f;
    f.handle(ShellFlow::boot_finished());
    f.handle(ShellFlow::menu_item("free_roam"));
    CHECK(f.screen() == Screen::SpawnPicker);
    // Back from the picker returns to the menu without loading anything.
    {
        ShellFlow g;
        g.handle(ShellFlow::boot_finished());
        g.handle(ShellFlow::menu_item("free_roam"));
        const auto t = g.handle(ShellFlow::back());
        CHECK(t.accepted);
        CHECK(g.screen() == Screen::MainMenu);
        CHECK_FALSE(has_action(t, ShellActionKind::LoadWorld));
    }
    // A picked flat world loads.
    const auto picked = f.handle(ShellFlow::spawn_picked(flat_request()));
    CHECK(picked.accepted);
    CHECK(f.screen() == Screen::Loading);
    const auto* load = find_action(picked, ShellActionKind::LoadWorld);
    REQUIRE(load != nullptr);
    CHECK(load->world.kind == rg::WorldKind::Flat);
    CHECK_FALSE(f.world_loaded());
    const auto ready = f.handle(ShellFlow::load_ready());
    CHECK(ready.accepted);
    CHECK(f.screen() == Screen::Drive);
    CHECK(f.world_loaded());
}

TEST_CASE("shell: a failed or cancelled load returns to the menu and unloads", "[shell]") {
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        f.handle(ShellFlow::menu_item("free_roam"));
        f.handle(ShellFlow::spawn_picked(real_request()));
        const auto t = f.handle(ShellFlow::load_failed("WorldTerrain::open: no store"));
        CHECK(t.accepted);
        CHECK(has_action(t, ShellActionKind::UnloadWorld));
        CHECK(f.screen() == Screen::MainMenu);
        CHECK(f.last_error() == "WorldTerrain::open: no store");
        CHECK_FALSE(f.world_loaded());
        // The next load clears the error.
        f.handle(ShellFlow::menu_item("free_roam"));
        f.handle(ShellFlow::spawn_picked(flat_request()));
        CHECK(f.last_error().empty());
    }
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        f.handle(ShellFlow::menu_item("free_roam"));
        f.handle(ShellFlow::spawn_picked(real_request()));
        const auto t = f.handle(ShellFlow::load_cancelled());
        CHECK(has_action(t, ShellActionKind::UnloadWorld));
        CHECK(f.screen() == Screen::MainMenu);
        CHECK(f.last_error().empty());
    }
    {
        ShellFlow f;
        f.handle(ShellFlow::boot_finished());
        f.handle(ShellFlow::menu_item("free_roam"));
        f.handle(ShellFlow::spawn_picked(real_request()));
        const auto t = f.handle(ShellFlow::back()); // Esc on the loading screen cancels
        CHECK(has_action(t, ShellActionKind::UnloadWorld));
        CHECK(f.screen() == Screen::MainMenu);
    }
    {
        ShellFlow f; // an empty failure message still reads as an error
        f.handle(ShellFlow::direct_start(flat_request()));
        f.handle(ShellFlow::load_failed(""));
        CHECK_FALSE(f.last_error().empty());
    }
}

TEST_CASE("shell: pause and resume", "[shell]") {
    ShellFlow f;
    to_drive(f);
    // Esc/P pauses and pauses the simulation.
    const auto paused = f.handle(ShellFlow::pause_toggle());
    CHECK(paused.accepted);
    const auto* sp = find_action(paused, ShellActionKind::SetPaused);
    REQUIRE(sp != nullptr);
    CHECK(sp->flag);
    CHECK(f.screen() == Screen::Pause);
    // The toggle again, Back and "resume" all resume.
    for (int how = 0; how < 3; ++how) {
        ShellFlow g;
        to_drive(g);
        g.handle(ShellFlow::pause_toggle());
        const auto t = how == 0 ? g.handle(ShellFlow::pause_toggle())
                       : how == 1 ? g.handle(ShellFlow::back())
                                  : g.handle(ShellFlow::menu_item("resume"));
        CAPTURE(how);
        CHECK(t.accepted);
        const auto* r = find_action(t, ShellActionKind::SetPaused);
        REQUIRE(r != nullptr);
        CHECK_FALSE(r->flag);
        CHECK(g.screen() == Screen::Drive);
    }
    // Reset car resumes and resets.
    const auto reset = f.handle(ShellFlow::menu_item("reset_car"));
    CHECK(reset.accepted);
    CHECK(has_action(reset, ShellActionKind::ResetCar));
    CHECK_FALSE(find_action(reset, ShellActionKind::SetPaused)->flag);
    CHECK(f.screen() == Screen::Drive);
}

TEST_CASE("shell: pause -> main menu unloads, then free roam works again", "[shell]") {
    ShellFlow f;
    to_drive(f);
    f.handle(ShellFlow::pause_toggle());
    const auto t = f.handle(ShellFlow::menu_item("main_menu"));
    CHECK(t.accepted);
    CHECK(has_action(t, ShellActionKind::UnloadWorld));
    CHECK(f.screen() == Screen::MainMenu);
    CHECK_FALSE(f.world_loaded());
    // Second session in the same shell.
    CHECK(f.handle(ShellFlow::menu_item("free_roam")).accepted);
    CHECK(f.handle(ShellFlow::spawn_picked(flat_request())).accepted);
    CHECK(f.handle(ShellFlow::load_ready()).accepted);
    CHECK(f.screen() == Screen::Drive);
    CHECK(f.world_loaded());
}

TEST_CASE("shell: events outside their screen are refused and change nothing", "[shell]") {
    const std::vector<rg::ShellEvent> all = {
        ShellFlow::boot_finished(),  ShellFlow::direct_start(flat_request()), ShellFlow::menu_item("free_roam"),
        ShellFlow::menu_item("resume"), ShellFlow::spawn_picked(flat_request()), ShellFlow::back(),
        ShellFlow::load_ready(),     ShellFlow::load_failed("x"),             ShellFlow::load_cancelled(),
        ShellFlow::pause_toggle(),   ShellFlow::vehicle_chosen("car_sedan"), ShellFlow::garage_drive(),
        ShellFlow::menu_item("garage"),
    };
    // Which events each screen accepts (everything else must be refused).
    const auto accepts = [](Screen s, const rg::ShellEvent& e) {
        using K = rg::ShellEventKind;
        switch (s) {
            case Screen::Boot: return e.kind == K::BootFinished || e.kind == K::Back || e.kind == K::DirectStart;
            case Screen::MainMenu: return e.kind == K::MenuItem && (e.item == "free_roam" || e.item == "garage");
            case Screen::SpawnPicker: return e.kind == K::Back || e.kind == K::SpawnPicked;
            case Screen::Loading: return e.kind == K::LoadReady || e.kind == K::LoadFailed || e.kind == K::LoadCancelled || e.kind == K::Back;
            case Screen::Drive: return e.kind == K::PauseToggle;
            case Screen::Pause:
                return e.kind == K::PauseToggle || e.kind == K::Back ||
                       (e.kind == K::MenuItem && (e.item == "resume" || e.item == "garage"));
            case Screen::VehicleSelect: return e.kind == K::Back || e.kind == K::VehicleChosen;
            case Screen::Configurator: return e.kind == K::Back || e.kind == K::GarageDrive;
            case Screen::Settings: return e.kind == K::Back;
            case Screen::Credits: return e.kind == K::Back;
            case Screen::Quit: return false;
        }
        return false;
    };
    // Build a flow standing on each screen, then throw every event at it.
    const std::vector<Screen> screens = {Screen::Boot,    Screen::MainMenu, Screen::SpawnPicker, Screen::Loading, Screen::Drive,
                                         Screen::Pause,   Screen::Settings, Screen::Credits,     Screen::VehicleSelect,
                                         Screen::Configurator, Screen::Quit};
    for (const Screen target : screens) {
        for (std::size_t i = 0; i < all.size(); ++i) {
            ShellFlow f;
            switch (target) {
                case Screen::Boot: break;
                case Screen::MainMenu: f.handle(ShellFlow::boot_finished()); break;
                case Screen::SpawnPicker:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("free_roam"));
                    break;
                case Screen::Loading: f.handle(ShellFlow::direct_start(flat_request())); break;
                case Screen::Drive: to_drive(f); break;
                case Screen::Pause:
                    to_drive(f);
                    f.handle(ShellFlow::pause_toggle());
                    break;
                case Screen::Settings:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("settings"));
                    break;
                case Screen::Credits:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("credits"));
                    break;
                case Screen::VehicleSelect:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("garage"));
                    break;
                case Screen::Configurator:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("garage"));
                    f.handle(ShellFlow::vehicle_chosen("car_sedan"));
                    break;
                case Screen::Quit:
                    f.handle(ShellFlow::boot_finished());
                    f.handle(ShellFlow::menu_item("quit"));
                    break;
            }
            REQUIRE(f.screen() == target);
            const bool should = accepts(target, all[i]);
            const auto t = f.handle(all[i]);
            CAPTURE(rg::to_string(target), i);
            CHECK(t.accepted == should);
            if (!should) {
                CHECK(f.screen() == target);
                CHECK(t.actions.empty());
            }
        }
    }
}

TEST_CASE("shell: every accepted transition ends with ShowScreen of its target", "[shell]") {
    ShellFlow f;
    const std::vector<rg::ShellEvent> script = {
        ShellFlow::boot_finished(),
        ShellFlow::menu_item("settings"),
        ShellFlow::back(),
        ShellFlow::menu_item("credits"),
        ShellFlow::back(),
        ShellFlow::menu_item("free_roam"),
        ShellFlow::spawn_picked(real_request()),
        ShellFlow::load_ready(),
        ShellFlow::pause_toggle(),
        ShellFlow::menu_item("settings"),
        ShellFlow::back(),
        ShellFlow::menu_item("main_menu"),
        ShellFlow::menu_item("quit"),
    };
    for (const auto& e : script) {
        const auto t = f.handle(e);
        REQUIRE(t.accepted);
        REQUIRE_FALSE(t.actions.empty());
        CHECK(t.actions.back().kind == ShellActionKind::ShowScreen);
        CHECK(t.actions.back().screen == t.to);
        CHECK(f.screen() == t.to);
    }
    CHECK(f.screen() == Screen::Quit);
}

TEST_CASE("shell: garage from the main menu -> select -> configurator -> spawn picker", "[shell][garage]") {
    ShellFlow f;
    f.handle(ShellFlow::boot_finished());
    const auto open = f.handle(ShellFlow::menu_item("garage"));
    REQUIRE(open.accepted);
    CHECK(f.screen() == Screen::VehicleSelect);
    CHECK(f.garage_return() == Screen::MainMenu);
    CHECK(has_action(open, ShellActionKind::OpenGarage));
    CHECK_FALSE(has_action(open, ShellActionKind::LoadWorld));

    // back closes the garage and returns to the main menu
    const auto back = f.handle(ShellFlow::back());
    CHECK(back.accepted);
    CHECK(f.screen() == Screen::MainMenu);
    CHECK(has_action(back, ShellActionKind::CloseGarage));

    f.handle(ShellFlow::menu_item("garage"));
    CHECK_FALSE(f.handle(ShellFlow::vehicle_chosen("")).accepted); // an empty id is refused
    const auto chosen = f.handle(ShellFlow::vehicle_chosen("car_hyper"));
    REQUIRE(chosen.accepted);
    CHECK(f.screen() == Screen::Configurator);
    CHECK(f.garage_vehicle() == "car_hyper");
    CHECK_FALSE(has_action(chosen, ShellActionKind::CloseGarage)); // the garage scene stays up

    // back to the selection keeps the garage open
    const auto to_select = f.handle(ShellFlow::back());
    CHECK(to_select.accepted);
    CHECK(f.screen() == Screen::VehicleSelect);
    CHECK_FALSE(has_action(to_select, ShellActionKind::CloseGarage));
    f.handle(ShellFlow::vehicle_chosen("car_sedan"));
    CHECK(f.garage_vehicle() == "car_sedan");

    // Drive: the garage closes and the spawn picker follows (as in Free roam)
    const auto drive = f.handle(ShellFlow::garage_drive());
    REQUIRE(drive.accepted);
    CHECK(f.screen() == Screen::SpawnPicker);
    CHECK(has_action(drive, ShellActionKind::CloseGarage));
    CHECK_FALSE(has_action(drive, ShellActionKind::LoadWorld));
    CHECK(f.handle(ShellFlow::spawn_picked(flat_request())).accepted);
    CHECK(f.screen() == Screen::Loading);
    CHECK(f.handle(ShellFlow::load_ready()).accepted);
    CHECK(f.screen() == Screen::Drive);
}

TEST_CASE("shell: garage from the pause menu respawns the chosen car", "[shell][garage]") {
    ShellFlow f;
    to_drive(f);
    f.handle(ShellFlow::pause_toggle());
    const auto open = f.handle(ShellFlow::menu_item("garage"));
    REQUIRE(open.accepted);
    CHECK(f.screen() == Screen::VehicleSelect);
    CHECK(f.garage_return() == Screen::Pause);
    CHECK(has_action(open, ShellActionKind::OpenGarage));
    CHECK_FALSE(has_action(open, ShellActionKind::UnloadWorld)); // the paused world stays up
    CHECK(f.world_loaded());

    // back returns to the pause menu (world still loaded)
    const auto back = f.handle(ShellFlow::back());
    CHECK(back.accepted);
    CHECK(f.screen() == Screen::Pause);
    CHECK(has_action(back, ShellActionKind::CloseGarage));
    CHECK(f.world_loaded());

    f.handle(ShellFlow::menu_item("garage"));
    f.handle(ShellFlow::vehicle_chosen("car_sedan_rwd"));
    const auto drive = f.handle(ShellFlow::garage_drive());
    REQUIRE(drive.accepted);
    CHECK(f.screen() == Screen::Loading);
    CHECK(has_action(drive, ShellActionKind::CloseGarage));
    const auto* load = find_action(drive, ShellActionKind::LoadWorld);
    REQUIRE(load != nullptr);
    CHECK(load->flag); // "respawn where the old car stood"
    CHECK(load->world.kind == rg::WorldKind::Flat);
    CHECK(has_action(drive, ShellActionKind::SetPaused));
    CHECK(f.handle(ShellFlow::load_ready()).accepted);
    CHECK(f.screen() == Screen::Drive);

    // a failed respawn load returns to the main menu with the message, like any load
    f.handle(ShellFlow::pause_toggle());
    f.handle(ShellFlow::menu_item("garage"));
    f.handle(ShellFlow::vehicle_chosen("car_hyper"));
    f.handle(ShellFlow::garage_drive());
    const auto failed = f.handle(ShellFlow::load_failed("boom"));
    CHECK(failed.accepted);
    CHECK(f.screen() == Screen::MainMenu);
    CHECK(f.last_error() == "boom");
    CHECK(has_action(failed, ShellActionKind::UnloadWorld));
}
