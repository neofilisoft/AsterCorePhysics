add_library(astercore_godot SHARED
    ${PHYSICS_REPO_ROOT}/Integrations/Godot/AsterCoreGodotBridge.cpp
    ${PHYSICS_REPO_ROOT}/Integrations/Godot/gdextension_minimal.h
)

target_include_directories(astercore_godot PRIVATE ${PHYSICS_REPO_ROOT} ${PHYSICS_REPO_ROOT}/Integrations/Godot)
target_link_libraries(astercore_godot PRIVATE AsterCoreCAPI)
set_target_properties(astercore_godot PROPERTIES PREFIX "" OUTPUT_NAME "astercore_godot")

set(ASTERCORE_GODOT_PACKAGE_DIR ${PHYSICS_REPO_ROOT}/Build/Packaging/Artifacts/AsterCorePhysicsGodot)
add_custom_command(TARGET astercore_godot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ASTERCORE_GODOT_PACKAGE_DIR}/addons/astercore_physics/bin/windows
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PHYSICS_REPO_ROOT}/Integrations/Godot/addon ${ASTERCORE_GODOT_PACKAGE_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:astercore_godot> ${ASTERCORE_GODOT_PACKAGE_DIR}/addons/astercore_physics/bin/windows/astercore_godot.dll
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:AsterCoreCAPI> ${ASTERCORE_GODOT_PACKAGE_DIR}/addons/astercore_physics/bin/windows/AsterCoreCAPI.dll
)
