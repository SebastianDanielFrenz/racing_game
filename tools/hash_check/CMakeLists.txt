# hash_check: the R0 state_hash acceptance check (see main.cpp's own header
# comment for exactly what it compares). Links rg_core only - it drives
# rg::Session synchronously, no wall clock, no thread, no Godot.

add_executable(hash_check main.cpp)
target_link_libraries(hash_check PRIVATE rg_core rg_warnings rg_options)
