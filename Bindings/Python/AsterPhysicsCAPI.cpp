#include <AsterCore/AsterCore.h>
#include <Bindings/Python/AsterGymEnvironment.h>
#include <Bindings/Python/AsterPhysicsCAPI.h>
#include <AsterCore/Core/JobSystem.h>
#include <AsterCore/Core/TempAllocator.h>
#include <AsterCore/Physics/PhysicsSystem.h>

using namespace ACPH;

struct ACPH_GymEnvironmentHandle_t
{
	AsterGymEnvironment mEnvironment;
};

ACPH_GymEnvironmentHandle ACPH_GymCreateEnvironment(void)
{
	return new ACPH_GymEnvironmentHandle_t();
}

void ACPH_GymDestroyEnvironment(ACPH_GymEnvironmentHandle inHandle)
{
	delete inHandle;
}

void ACPH_GymConfigureEnvironment(ACPH_GymEnvironmentHandle inHandle, const ACPH_GymEnvironmentSettings *inSettings)
{
	if (inHandle == nullptr || inSettings == nullptr)
		return;

	AsterGymEnvironmentSettings settings;
	settings.mObservationSize = inSettings->mObservationSize;
	settings.mActionSize = inSettings->mActionSize;
	settings.mFixedDeltaTime = inSettings->mFixedDeltaTime;
	settings.mSubSteps = inSettings->mSubSteps;
	settings.mMaxEpisodeSteps = inSettings->mMaxEpisodeSteps;
	inHandle->mEnvironment.Configure(settings);
}

void ACPH_GymAttachSimulation(ACPH_GymEnvironmentHandle inHandle, void *inPhysicsSystem, void *inJobSystem, void *inTempAllocator)
{
	if (inHandle == nullptr)
		return;

	inHandle->mEnvironment.AttachSimulation(reinterpret_cast<PhysicsSystem *>(inPhysicsSystem), reinterpret_cast<JobSystem *>(inJobSystem), reinterpret_cast<TempAllocator *>(inTempAllocator));
}

void ACPH_GymResetEnvironment(ACPH_GymEnvironmentHandle inHandle)
{
	if (inHandle != nullptr)
		inHandle->mEnvironment.Reset();
}

ACPH_GymStepResult ACPH_GymStepEnvironment(ACPH_GymEnvironmentHandle inHandle, const float *inActions, uint32 inActionCount)
{
	ACPH_GymStepResult c_result = {};
	if (inHandle == nullptr)
		return c_result;

	const AsterGymStepResult result = inHandle->mEnvironment.Step(inActions, inActionCount);
	c_result.mReward = result.mReward;
	c_result.mTerminated = result.mTerminated ? 1 : 0;
	c_result.mTruncated = result.mTruncated ? 1 : 0;
	c_result.mSimulationTime = result.mSimulationTime;
	c_result.mEpisodeStep = result.mEpisodeStep;
	c_result.mUpdateErrorMask = uint32(result.mUpdateError);
	return c_result;
}

const float *ACPH_GymGetObservationData(ACPH_GymEnvironmentHandle inHandle)
{
	return inHandle != nullptr? inHandle->mEnvironment.GetObservationData() : nullptr;
}

uint32 ACPH_GymGetObservationSize(ACPH_GymEnvironmentHandle inHandle)
{
	return inHandle != nullptr? inHandle->mEnvironment.GetObservationSize() : 0;
}
