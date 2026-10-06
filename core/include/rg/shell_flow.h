// rg/shell_flow.h - rg::ShellFlow: the game shell's screen state machine
// (PLAN.md R5, 11.2). Engine-neutral: the Godot layer reports what happened
// (a menu item chosen, a world finished loading, Esc pressed) and carries out
// the actions this returns (show a screen, load/unload a world, pause the
// simulation, quit). It decides nothing about drawing.
//
//   Boot -> MainMenu
//   MainMenu -> SpawnPicker ("Free roam") | VehicleSelect ("Garage") | Settings | Credits | Quit
//   SpawnPicker -> Loading (a spawn was picked) | MainMenu (back)
//   Loading -> Drive (world ready) | MainMenu (cancelled or failed; the error
//              is kept for the menu to show)
//   Drive <-> Pause
//   Pause -> Controls | Settings | VehicleSelect ("Garage (respawn)") | ChangeCar ("Change car") | MainMenu (unloads the world) | Drive (resume / reset car)
//   ChangeCar -> Loading (a car chosen: the world reloads with it, the new car starting where the old one stood, no road reset) |
//                Pause (back)
//   VehicleSelect -> Configurator (a car chosen) | back to where the garage was opened (MainMenu or Pause)
//   Configurator -> VehicleSelect (back) | SpawnPicker (Drive, garage opened from the main menu) |
//                   Loading (Drive, garage opened from the pause menu: the world reloads with the
//                   chosen car, respawning where the old one stood)
//   Settings -> Controls ("Controls" row) | back to where it was opened (MainMenu or Pause)
//   Controls -> back to where it was opened (Settings or Pause); the player's bindings are saved on the way out
//   Credits -> MainMenu
// Start flags that skip the menu (--drive, --flat, ...) enter through
// DirectStart: Boot -> Loading with the world already chosen. Every other
// event is refused (accepted == false, state unchanged) outside the screens
// listed above - a stray key never moves the shell.
//
// Menu content is data here too (main_menu_items()/pause_menu_items()): no
// "Events" or "Map & route" entries until those features exist. Which car is
// shown, selected and configured is rg::Garage's business, not the flow's: the
// flow only knows the garage screens and tells the Godot layer to open/close the
// garage scene (OpenGarage/CloseGarage).
#pragma once

#include "rg/player_mode.h"

#include <string>
#include <vector>

namespace rg {

enum class Screen { Boot, MainMenu, SpawnPicker, Loading, Drive, Pause, Settings, Credits, VehicleSelect, Configurator, ChangeCar, Controls, Quit };

const char* to_string(Screen s);

struct MenuItem {
    std::string id;    // "free_roam", "garage", "settings", "credits", "quit", "resume", "reset_car", "change_car", "controls", "main_menu"
    std::string label; // shown text
};

// Main menu: Free roam | Garage | Settings | Credits | Quit.
const std::vector<MenuItem>& main_menu_items();
// Pause menu: Resume | Reset car | Change car | Garage (respawn) | Controls | Settings | Main menu.
const std::vector<MenuItem>& pause_menu_items();

enum class ShellEventKind {
    BootFinished,   // the boot splash is done (timer or key)
    DirectStart,    // a start flag skipped the menu: world in `world`
    MenuItem,       // an item of the current menu chosen: id in `item`
    SpawnPicked,    // the picker's choice: world in `world`
    Back,           // Esc / back button on the current screen
    LoadReady,      // Loading: the world is running
    LoadFailed,     // Loading: it did not load; message in `message`
    LoadCancelled,  // Loading: the player gave up
    PauseToggle,    // Esc/P while driving or paused
    VehicleChosen,  // VehicleSelect: the highlighted car was chosen: id in `item`
    GarageDrive,    // Configurator: "Drive" (the chosen car, with its saved setup)
};

// What the player asked to load. WorldKind::Flat has no position.
struct WorldRequest {
    WorldKind kind = WorldKind::Flat;
    bool has_spawn = false; // false: the world's own default spawn
    double x = 0.0;         // session-local
    double y = 0.0;
    double yaw_deg = 0.0;
    std::string label;      // for the loading screen ("Engelsruhe, Unterliederbach")
    bool open_address_search = false; // after Drive starts, open the F10 address search
};

struct ShellEvent {
    ShellEventKind kind = ShellEventKind::Back;
    std::string item;    // MenuItem
    WorldRequest world;  // DirectStart, SpawnPicked
    std::string message; // LoadFailed
};

enum class ShellActionKind {
    ShowScreen,     // `screen` is now current (always the last action of a transition)
    LoadWorld,      // `world` to load (start the loading flow)
    UnloadWorld,    // stop and release the running world (sim thread, streaming, audio)
    SetPaused,      // `flag`: pause / resume the simulation
    ResetCar,       // Pause menu "Reset car"
    SaveSettings,   // leaving the settings screen: persist
    SaveControls,   // leaving the controls screen: persist the bindings (user://controls.json)
    Quit,           // exit the application
    OpenGarage,     // show the garage scene (main menu: nothing behind it; pause: the paused world stays loaded)
    CloseGarage,    // remove the garage scene and every node it created
};

struct ShellAction {
    ShellActionKind kind = ShellActionKind::ShowScreen;
    Screen screen = Screen::Boot;
    WorldRequest world;
    // LoadWorld: true = the garage respawn of the pause menu - the world reloads with the
    // chosen car and the new car starts where the old one stood (the Godot layer swaps
    // the spawn for the current pose). SetPaused: the paused state.
    bool flag = false;
};

struct ShellTransition {
    bool accepted = false;
    Screen from = Screen::Boot;
    Screen to = Screen::Boot;
    std::vector<ShellAction> actions;
};

class ShellFlow {
public:
    ShellFlow() = default;

    [[nodiscard]] Screen screen() const { return screen_; }
    // Where Settings was opened from (MainMenu or Pause); meaningful in Settings.
    [[nodiscard]] Screen settings_return() const { return settings_return_; }
    // Where Controls was opened from (Settings or Pause); meaningful in Controls.
    [[nodiscard]] Screen controls_return() const { return controls_return_; }
    // The world being loaded or running (Loading/Drive/Pause/Settings-from-Pause).
    [[nodiscard]] const WorldRequest& world() const { return world_; }
    // Where the garage was opened from (MainMenu or Pause); meaningful in VehicleSelect/Configurator.
    [[nodiscard]] Screen garage_return() const { return garage_return_; }
    // The car chosen in VehicleSelect (the Configurator's car); empty before one was chosen.
    [[nodiscard]] const std::string& garage_vehicle() const { return garage_vehicle_; }
    [[nodiscard]] bool world_loaded() const { return world_loaded_; }
    // The last LoadFailed message; cleared when a new load starts. The main
    // menu shows it once.
    [[nodiscard]] const std::string& last_error() const { return last_error_; }
    void clear_error() { last_error_.clear(); }

    ShellTransition handle(const ShellEvent& event);

    // Convenience builders.
    static ShellEvent boot_finished();
    static ShellEvent direct_start(WorldRequest world);
    static ShellEvent menu_item(std::string id);
    static ShellEvent spawn_picked(WorldRequest world);
    static ShellEvent back();
    static ShellEvent load_ready();
    static ShellEvent load_failed(std::string message);
    static ShellEvent load_cancelled();
    static ShellEvent pause_toggle();
    static ShellEvent vehicle_chosen(std::string id);
    static ShellEvent garage_drive();

private:
    ShellTransition refuse() const;
    ShellTransition go(Screen to, std::vector<ShellAction> actions);

    Screen screen_ = Screen::Boot;
    Screen settings_return_ = Screen::MainMenu;
    Screen controls_return_ = Screen::Settings;
    Screen garage_return_ = Screen::MainMenu;
    std::string garage_vehicle_;
    WorldRequest world_;
    bool world_loaded_ = false; // a world is up (Drive, Pause, or Settings opened from Pause)
    std::string last_error_;
};

} // namespace rg
