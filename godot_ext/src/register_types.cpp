// godot_ext/src/register_types.cpp — GDExtension entry point. See
// game/rg_godot.gdextension's entry_symbol for how Godot finds
// rg_godot_library_init below (mirrors physics_sim's own
// adapters/godot/src/register_types.cpp).

#include "register_types.h"

#include "rg_camera_math.h"
#include "rg_controls.h"
#include "rg_garage.h"
#include "rg_shell.h"
#include "rg_simulation.h"
#include "rg_terrain_view.h"
#include "spatial_audio.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_rg_godot_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
    ClassDB::register_class<rg_godot::RgSimulation>();
    ClassDB::register_class<rg_godot::RgShell>();
    ClassDB::register_class<rg_godot::RgControls>();
    ClassDB::register_class<rg_godot::RgGarage>();
    ClassDB::register_class<rg_godot::RgCameraMath>();
    ClassDB::register_class<rg_godot::RgTerrainView>();
    ClassDB::register_class<ps_godot::PsSpatialAudio>();
}

void uninitialize_rg_godot_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
}

extern "C" {
GDExtensionBool GDE_EXPORT rg_godot_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                                                  GDExtensionClassLibraryPtr p_library,
                                                  GDExtensionInitialization* r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_rg_godot_module);
    init_obj.register_terminator(uninitialize_rg_godot_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}
}
