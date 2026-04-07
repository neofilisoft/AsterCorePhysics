add_library(AsterCoreFractalEditorCore STATIC
	${PHYSICS_REPO_ROOT}/Tools/FractalEditor/FractureAuthoringProfile.cpp
	${PHYSICS_REPO_ROOT}/Tools/FractalEditor/FractureAuthoringProfile.h
)

target_include_directories(AsterCoreFractalEditorCore PUBLIC
	$<BUILD_INTERFACE:${PHYSICS_REPO_ROOT}>
	$<INSTALL_INTERFACE:include/>)
target_link_libraries(AsterCoreFractalEditorCore PUBLIC AsterCore AsterCoreEditorCore)

add_executable(AsterFractalEditorCLI ${PHYSICS_REPO_ROOT}/Tools/FractalEditor/AsterFractalEditorCLI.cpp)
target_include_directories(AsterFractalEditorCLI PRIVATE ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterFractalEditorCLI PRIVATE AsterCoreFractalEditorCore)
