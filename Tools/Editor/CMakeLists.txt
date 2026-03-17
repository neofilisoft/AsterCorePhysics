add_library(AsterCoreEditorCore STATIC
	${PHYSICS_REPO_ROOT}/Tools/Editor/PhysicsAssetDocument.cpp
	${PHYSICS_REPO_ROOT}/Tools/Editor/PhysicsAssetDocument.h
)

target_include_directories(AsterCoreEditorCore PUBLIC ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterCoreEditorCore PUBLIC AsterCore)

add_executable(AsterPhysicsEditorCLI ${PHYSICS_REPO_ROOT}/Tools/Editor/AsterPhysicsEditorCLI.cpp)
target_include_directories(AsterPhysicsEditorCLI PRIVATE ${PHYSICS_REPO_ROOT})
target_link_libraries(AsterPhysicsEditorCLI PRIVATE AsterCoreEditorCore)
