add_library(AsterCoreCAPI SHARED
    ${PHYSICS_REPO_ROOT}/Bindings/CAPI/AsterCoreCAPI.cpp
    ${PHYSICS_REPO_ROOT}/Bindings/CAPI/AsterCoreCAPI.h
)

target_include_directories(AsterCoreCAPI PUBLIC
    $<BUILD_INTERFACE:${PHYSICS_REPO_ROOT}>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(AsterCoreCAPI PUBLIC AsterCore)
target_compile_definitions(AsterCoreCAPI PRIVATE ASTERCORE_CAPI_BUILD)

set_target_properties(AsterCoreCAPI PROPERTIES
    PREFIX ""
    OUTPUT_NAME "AsterCoreCAPI")
