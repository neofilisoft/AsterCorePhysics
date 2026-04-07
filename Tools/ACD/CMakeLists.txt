add_library(AsterCoreACDCore STATIC
	${PHYSICS_REPO_ROOT}/Tools/ACD/ACDCaptureSession.cpp
	${PHYSICS_REPO_ROOT}/Tools/ACD/ACDCaptureSession.h
	${PHYSICS_REPO_ROOT}/Tools/ACD/ACDComputeBackendToggle.cpp
	${PHYSICS_REPO_ROOT}/Tools/ACD/ACDComputeBackendToggle.h
)

target_include_directories(AsterCoreACDCore PUBLIC
	$<BUILD_INTERFACE:${PHYSICS_REPO_ROOT}>
	$<INSTALL_INTERFACE:include/>)
target_link_libraries(AsterCoreACDCore PUBLIC AsterCore)

add_executable(AsterCoreDebuggerCLI ${PHYSICS_REPO_ROOT}/Tools/ACD/AsterCoreDebuggerCLI.cpp)
target_include_directories(AsterCoreDebuggerCLI PRIVATE ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterCoreDebuggerCLI PRIVATE AsterCoreACDCore)
