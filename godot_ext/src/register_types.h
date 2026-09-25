// godot_ext/src/register_types.h — GDExtension entry point.

#pragma once

#include <godot_cpp/core/class_db.hpp>

void initialize_rg_godot_module(godot::ModuleInitializationLevel p_level);
void uninitialize_rg_godot_module(godot::ModuleInitializationLevel p_level);
