#pragma once

#include <stdint.h>

#ifdef _WIN32
  #if defined(ASTERCORE_CAPI_BUILD)
    #define ASTERCORE_CAPI_EXPORT __declspec(dllexport)
  #else
    #define ASTERCORE_CAPI_EXPORT __declspec(dllimport)
  #endif
#else
  #define ASTERCORE_CAPI_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ACPH_CAPIWorldHandle_t *ACPH_CAPIWorldHandle;

typedef struct ACPH_CAPIVec3
{
    float x;
    float y;
    float z;
} ACPH_CAPIVec3;

typedef struct ACPH_CAPIQuat
{
    float x;
    float y;
    float z;
    float w;
} ACPH_CAPIQuat;

typedef struct ACPH_CAPIWorldSettings
{
    uint32_t max_bodies;
    uint32_t max_body_pairs;
    uint32_t max_contact_constraints;
    uint32_t temp_allocator_size_mb;
    uint32_t worker_threads;
    ACPH_CAPIVec3 gravity;
} ACPH_CAPIWorldSettings;

typedef struct ACPH_CAPIRaycastHit
{
    int hit;
    uint32_t body_id;
    float fraction;
    ACPH_CAPIVec3 position;
    ACPH_CAPIVec3 normal;
} ACPH_CAPIRaycastHit;

ASTERCORE_CAPI_EXPORT int ACPH_CAPIInitialize(void);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIShutdown(void);
ASTERCORE_CAPI_EXPORT const char *ACPH_CAPIGetVersionString(void);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIDefaultWorldSettings(ACPH_CAPIWorldSettings *out_settings);
ASTERCORE_CAPI_EXPORT ACPH_CAPIWorldHandle ACPH_CAPICreateWorld(const ACPH_CAPIWorldSettings *settings);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIDestroyWorld(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIIsWorldValid(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT const char *ACPH_CAPIGetLastError(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIResetWorld(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT void ACPH_CAPISetGravity(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 gravity);
ASTERCORE_CAPI_EXPORT ACPH_CAPIVec3 ACPH_CAPIGetGravity(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIOptimizeBroadPhase(ACPH_CAPIWorldHandle world);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIWorldStep(ACPH_CAPIWorldHandle world, float delta_time, uint32_t collision_steps);
ASTERCORE_CAPI_EXPORT uint32_t ACPH_CAPICreateBoxBody(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 half_extents, ACPH_CAPIVec3 position, int is_dynamic, float mass);
ASTERCORE_CAPI_EXPORT uint32_t ACPH_CAPICreateSphereBody(ACPH_CAPIWorldHandle world, float radius, ACPH_CAPIVec3 position, int is_dynamic, float mass);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIDestroyBody(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIHasBody(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIIsBodyActive(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT ACPH_CAPIVec3 ACPH_CAPIGetBodyPosition(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT void ACPH_CAPISetBodyPosition(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 position);
ASTERCORE_CAPI_EXPORT ACPH_CAPIQuat ACPH_CAPIGetBodyRotation(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT void ACPH_CAPISetBodyRotation(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIQuat rotation);
ASTERCORE_CAPI_EXPORT void ACPH_CAPISetBodyLinearVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 velocity);
ASTERCORE_CAPI_EXPORT ACPH_CAPIVec3 ACPH_CAPIGetBodyLinearVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT void ACPH_CAPISetBodyAngularVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 velocity);
ASTERCORE_CAPI_EXPORT ACPH_CAPIVec3 ACPH_CAPIGetBodyAngularVelocity(ACPH_CAPIWorldHandle world, uint32_t body_id);
ASTERCORE_CAPI_EXPORT void ACPH_CAPIApplyBodyImpulse(ACPH_CAPIWorldHandle world, uint32_t body_id, ACPH_CAPIVec3 impulse);
ASTERCORE_CAPI_EXPORT int ACPH_CAPIRaycast(ACPH_CAPIWorldHandle world, ACPH_CAPIVec3 origin, ACPH_CAPIVec3 direction, float max_distance, ACPH_CAPIRaycastHit *out_hit);

#ifdef __cplusplus
}
#endif
