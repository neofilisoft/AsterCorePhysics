# Root
set(JOLT_VIEWER_ROOT ${PHYSICS_REPO_ROOT}/AsterCoreViewer)

# Source files
set(JOLT_VIEWER_SRC_FILES
	${JOLT_VIEWER_ROOT}/AsterCoreViewer.cmake
	${JOLT_VIEWER_ROOT}/AsterCoreViewer.cpp
	${JOLT_VIEWER_ROOT}/AsterCoreViewer.h
)

# Group source files
source_group(TREE ${JOLT_VIEWER_ROOT} FILES ${JOLT_VIEWER_SRC_FILES})

# Create AsterCoreViewer executable
if ("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
	# Icon
	set(ACPH_ICON "${CMAKE_CURRENT_SOURCE_DIR}/macOS/icon.icns")
	set_source_files_properties(${ACPH_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

	add_executable(AsterCoreViewer MACOSX_BUNDLE ${JOLT_VIEWER_SRC_FILES} ${TEST_FRAMEWORK_ASSETS} ${ACPH_ICON})
	set_property(TARGET AsterCoreViewer PROPERTY MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/iOS/AsterCoreViewerInfo.plist")
	set_property(TARGET AsterCoreViewer PROPERTY XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "com.astercorephysics.astercoreviewer")
	set_property(TARGET AsterCoreViewer PROPERTY BUILD_RPATH "/usr/local/lib" INSTALL_RPATH "/usr/local/lib") # to find the Vulkan shared lib
else()
	add_executable(AsterCoreViewer ${JOLT_VIEWER_SRC_FILES})
endif()
target_include_directories(AsterCoreViewer PUBLIC ${JOLT_VIEWER_ROOT})
target_link_libraries(AsterCoreViewer LINK_PUBLIC TestFramework)

# Set the correct working directory
set_property(TARGET AsterCoreViewer PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${PHYSICS_REPO_ROOT}")
