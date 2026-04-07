add_library(AsterCoreFelissBridge STATIC
    ${PHYSICS_REPO_ROOT}/Integrations/Feliss/FelissParticleFluidBridge.cpp
    ${PHYSICS_REPO_ROOT}/Integrations/Feliss/FelissParticleFluidBridge.h
    ${PHYSICS_REPO_ROOT}/Integrations/Unreal/AsterPhysicsUnrealAdapter.h
    ${PHYSICS_REPO_ROOT}/Integrations/Godot/AsterPhysicsGodotAdapter.h
)

target_include_directories(AsterCoreFelissBridge PUBLIC ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterCoreFelissBridge PUBLIC AsterCore)

if (BUILD_ASTERCORE_GODOT_PLUGIN)
  add_subdirectory(Godot)
endif()
