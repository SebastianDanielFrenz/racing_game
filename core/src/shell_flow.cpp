// rg/shell_flow.cpp - see shell_flow.h.
#include "rg/shell_flow.h"

#include <cmath>
#include <utility>

namespace rg {

const char* to_string(Screen s) {
    switch (s) {
        case Screen::Boot: return "boot";
        case Screen::MainMenu: return "main_menu";
        case Screen::SpawnPicker: return "spawn_picker";
        case Screen::Loading: return "loading";
        case Screen::Drive: return "drive";
        case Screen::Pause: return "pause";
        case Screen::Settings: return "settings";
        case Screen::Credits: return "credits";
        case Screen::Quit: return "quit";
    }
    return "boot";
}

const std::vector<MenuItem>& main_menu_items() {
    static const std::vector<MenuItem> items = {
        {"free_roam", "Free roam"},
        {"settings", "Settings"},
        {"credits", "Credits"},
        {"quit", "Quit"},
    };
    return items;
}

const std::vector<MenuItem>& pause_menu_items() {
    static const std::vector<MenuItem> items = {
        {"resume", "Resume"},
        {"reset_car", "Reset car"},
        {"settings", "Settings"},
        {"main_menu", "Main menu"},
    };
    return items;
}

namespace {

bool has_item(const std::vector<MenuItem>& items, const std::string& id) {
    for (const MenuItem& m : items) {
        if (m.id == id) return true;
    }
    return false;
}

ShellAction show(Screen s) {
    ShellAction a;
    a.kind = ShellActionKind::ShowScreen;
    a.screen = s;
    return a;
}

ShellAction simple(ShellActionKind k, bool flag = false) {
    ShellAction a;
    a.kind = k;
    a.flag = flag;
    return a;
}

ShellAction load(const WorldRequest& w) {
    ShellAction a;
    a.kind = ShellActionKind::LoadWorld;
    a.world = w;
    return a;
}

bool request_valid(const WorldRequest& w) {
    if (!w.has_spawn) return true;
    return std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.yaw_deg);
}

} // namespace

ShellTransition ShellFlow::refuse() const {
    ShellTransition t;
    t.accepted = false;
    t.from = screen_;
    t.to = screen_;
    return t;
}

ShellTransition ShellFlow::go(Screen to, std::vector<ShellAction> actions) {
    ShellTransition t;
    t.accepted = true;
    t.from = screen_;
    t.to = to;
    t.actions = std::move(actions);
    t.actions.push_back(show(to));
    screen_ = to;
    return t;
}

ShellTransition ShellFlow::handle(const ShellEvent& e) {
    switch (screen_) {
        case Screen::Boot:
            if (e.kind == ShellEventKind::BootFinished || e.kind == ShellEventKind::Back) {
                return go(Screen::MainMenu, {});
            }
            if (e.kind == ShellEventKind::DirectStart && request_valid(e.world)) {
                world_ = e.world;
                world_loaded_ = false;
                last_error_.clear();
                return go(Screen::Loading, {load(e.world)});
            }
            return refuse();

        case Screen::MainMenu:
            if (e.kind == ShellEventKind::MenuItem && has_item(main_menu_items(), e.item)) {
                if (e.item == "free_roam") return go(Screen::SpawnPicker, {});
                if (e.item == "settings") {
                    settings_return_ = Screen::MainMenu;
                    return go(Screen::Settings, {});
                }
                if (e.item == "credits") return go(Screen::Credits, {});
                if (e.item == "quit") return go(Screen::Quit, {simple(ShellActionKind::Quit)});
            }
            return refuse();

        case Screen::SpawnPicker:
            if (e.kind == ShellEventKind::Back) return go(Screen::MainMenu, {});
            if (e.kind == ShellEventKind::SpawnPicked && request_valid(e.world)) {
                world_ = e.world;
                world_loaded_ = false;
                last_error_.clear();
                return go(Screen::Loading, {load(e.world)});
            }
            return refuse();

        case Screen::Loading:
            if (e.kind == ShellEventKind::LoadReady) {
                world_loaded_ = true;
                return go(Screen::Drive, {});
            }
            if (e.kind == ShellEventKind::LoadFailed) {
                last_error_ = e.message.empty() ? "The world did not load." : e.message;
                world_loaded_ = false;
                return go(Screen::MainMenu, {simple(ShellActionKind::UnloadWorld)});
            }
            if (e.kind == ShellEventKind::LoadCancelled || e.kind == ShellEventKind::Back) {
                world_loaded_ = false;
                return go(Screen::MainMenu, {simple(ShellActionKind::UnloadWorld)});
            }
            return refuse();

        case Screen::Drive:
            if (e.kind == ShellEventKind::PauseToggle) {
                return go(Screen::Pause, {simple(ShellActionKind::SetPaused, true)});
            }
            return refuse();

        case Screen::Pause:
            if (e.kind == ShellEventKind::PauseToggle || e.kind == ShellEventKind::Back) {
                return go(Screen::Drive, {simple(ShellActionKind::SetPaused, false)});
            }
            if (e.kind == ShellEventKind::MenuItem && has_item(pause_menu_items(), e.item)) {
                if (e.item == "resume") return go(Screen::Drive, {simple(ShellActionKind::SetPaused, false)});
                if (e.item == "reset_car") {
                    return go(Screen::Drive,
                              {simple(ShellActionKind::ResetCar), simple(ShellActionKind::SetPaused, false)});
                }
                if (e.item == "settings") {
                    settings_return_ = Screen::Pause;
                    return go(Screen::Settings, {});
                }
                if (e.item == "main_menu") {
                    world_loaded_ = false;
                    return go(Screen::MainMenu,
                              {simple(ShellActionKind::SetPaused, false), simple(ShellActionKind::UnloadWorld)});
                }
            }
            return refuse();

        case Screen::Settings:
            if (e.kind == ShellEventKind::Back) {
                return go(settings_return_, {simple(ShellActionKind::SaveSettings)});
            }
            return refuse();

        case Screen::Credits:
            if (e.kind == ShellEventKind::Back) return go(Screen::MainMenu, {});
            return refuse();

        case Screen::Quit:
            return refuse();
    }
    return refuse();
}

ShellEvent ShellFlow::boot_finished() {
    ShellEvent e;
    e.kind = ShellEventKind::BootFinished;
    return e;
}
ShellEvent ShellFlow::direct_start(WorldRequest world) {
    ShellEvent e;
    e.kind = ShellEventKind::DirectStart;
    e.world = std::move(world);
    return e;
}
ShellEvent ShellFlow::menu_item(std::string id) {
    ShellEvent e;
    e.kind = ShellEventKind::MenuItem;
    e.item = std::move(id);
    return e;
}
ShellEvent ShellFlow::spawn_picked(WorldRequest world) {
    ShellEvent e;
    e.kind = ShellEventKind::SpawnPicked;
    e.world = std::move(world);
    return e;
}
ShellEvent ShellFlow::back() {
    ShellEvent e;
    e.kind = ShellEventKind::Back;
    return e;
}
ShellEvent ShellFlow::load_ready() {
    ShellEvent e;
    e.kind = ShellEventKind::LoadReady;
    return e;
}
ShellEvent ShellFlow::load_failed(std::string message) {
    ShellEvent e;
    e.kind = ShellEventKind::LoadFailed;
    e.message = std::move(message);
    return e;
}
ShellEvent ShellFlow::load_cancelled() {
    ShellEvent e;
    e.kind = ShellEventKind::LoadCancelled;
    return e;
}
ShellEvent ShellFlow::pause_toggle() {
    ShellEvent e;
    e.kind = ShellEventKind::PauseToggle;
    return e;
}

} // namespace rg
