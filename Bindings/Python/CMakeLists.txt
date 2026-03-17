add_library(AsterCorePythonBridge STATIC
	${PHYSICS_REPO_ROOT}/Bindings/Python/AsterGymEnvironment.cpp
	${PHYSICS_REPO_ROOT}/Bindings/Python/AsterGymEnvironment.h
	${PHYSICS_REPO_ROOT}/Bindings/Python/AsterPhysicsCAPI.cpp
	${PHYSICS_REPO_ROOT}/Bindings/Python/AsterPhysicsCAPI.h
)

target_include_directories(AsterCorePythonBridge PUBLIC ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterCorePythonBridge PUBLIC AsterCore)
