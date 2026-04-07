#pragma once

#include <AsterCore/AsterCore.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ACPH_GymEnvironmentHandle_t *ACPH_GymEnvironmentHandle;

typedef struct ACPH_GymEnvironmentSettings
{
	ACPH::uint32 mObservationSize;
	ACPH::uint32 mActionSize;
	float mFixedDeltaTime;
	ACPH::uint32 mSubSteps;
	ACPH::uint32 mMaxEpisodeSteps;
} ACPH_GymEnvironmentSettings;

typedef struct ACPH_GymStepResult
{
	float mReward;
	int mTerminated;
	int mTruncated;
	float mSimulationTime;
	ACPH::uint32 mEpisodeStep;
	ACPH::uint32 mUpdateErrorMask;
} ACPH_GymStepResult;

ACPH_EXPORT ACPH_GymEnvironmentHandle ACPH_GymCreateEnvironment(void);
ACPH_EXPORT void ACPH_GymDestroyEnvironment(ACPH_GymEnvironmentHandle inHandle);
ACPH_EXPORT void ACPH_GymConfigureEnvironment(ACPH_GymEnvironmentHandle inHandle, const ACPH_GymEnvironmentSettings *inSettings);
ACPH_EXPORT void ACPH_GymAttachSimulation(ACPH_GymEnvironmentHandle inHandle, void *inPhysicsSystem, void *inJobSystem, void *inTempAllocator);
ACPH_EXPORT void ACPH_GymResetEnvironment(ACPH_GymEnvironmentHandle inHandle);
ACPH_EXPORT ACPH_GymStepResult ACPH_GymStepEnvironment(ACPH_GymEnvironmentHandle inHandle, const float *inActions, ACPH::uint32 inActionCount);
ACPH_EXPORT const float *ACPH_GymGetObservationData(ACPH_GymEnvironmentHandle inHandle);
ACPH_EXPORT ACPH::uint32 ACPH_GymGetObservationSize(ACPH_GymEnvironmentHandle inHandle);

#ifdef __cplusplus
}
#endif
