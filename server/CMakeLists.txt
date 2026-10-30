add_executable(rg_server main.cpp)
target_link_libraries(rg_server PRIVATE rg_core g2m_http g2m_dep_httplib rg_options rg_warnings)
if(WIN32)
  target_compile_definitions(rg_server PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

add_executable(rg_world_client_probe world_client_probe.cpp)
target_link_libraries(rg_world_client_probe PRIVATE rg_core rg_options rg_warnings)
