#include <AsterCore/AsterCore.h>
#include <Bindings/Python/AsterGymEnvironment.h>
#include <AsterCore/Core/JobSystem.h>
#include <AsterCore/Core/TempAllocator.h>
#include <AsterCore/Physics/PhysicsSystem.h>
#include <algorithm>

ACPH_NAMESPACE_BEGIN

void AsterGymEnvironment::Configure(const AsterGymEnvironmentSettings &inSettings)
{
	mSettings = inSettings;
	mObservations.resize(mSettings.mObservationSize);
	std::fill(mObservations.begin(), mObservations.end(), 0.0f);
	mActions.resize(mSettings.mActionSize);
	std::fill(mActions.begin(), mActions.end(), 0.0f);
	Reset();
}

void AsterGymEnvironment::AttachSimulation(PhysicsSystem *inPhysicsSystem, JobSystem *inJobSystem, TempAllocator *inTempAllocator)
{
	mPhysicsSystem = inPhysicsSystem;
	mJobSystem = inJobSystem;
	mTempAllocator = inTempAllocator;
}

void AsterGymEnvironment::Reset()
{
	std::fill(mObservations.begin(), mObservations.end(), 0.0f);
	std::fill(mActions.begin(), mActions.end(), 0.0f);
	mSimulationTime = 0.0f;
	mEpisodeStep = 0;
	mPendingReward = 0.0f;
	mTerminated = false;
	mTruncated = false;
}

AsterGymStepResult AsterGymEnvironment::Step(const float *inActions, uint32 inActionCount)
{
	if (inActions != nullptr)
	{
		const uint32 count = min<uint32>(inActionCount, uint32(mActions.size()));
		for (uint32 i = 0; i < count; ++i)
			mActions[i] = inActions[i];
	}

	AsterGymStepResult result;
	const uint32 sub_step_count = max<uint32>(mSettings.mSubSteps, 1);
	const float sub_step_dt = mSettings.mFixedDeltaTime / float(sub_step_count);
	for (uint32 sub_step = 0; sub_step < sub_step_count; ++sub_step)
	{
		if (mPhysicsSystem != nullptr && mJobSystem != nullptr && mTempAllocator != nullptr)
			result.mUpdateError |= mPhysicsSystem->Update(sub_step_dt, 1, mTempAllocator, mJobSystem);
		mSimulationTime += sub_step_dt;
	}

	++mEpisodeStep;
	if (mEpisodeStep >= mSettings.mMaxEpisodeSteps)
		mTruncated = true;

	result.mReward = mPendingReward;
	result.mTerminated = mTerminated;
	result.mTruncated = mTruncated;
	result.mSimulationTime = mSimulationTime;
	result.mEpisodeStep = mEpisodeStep;
	mPendingReward = 0.0f;
	return result;
}

void AsterGymEnvironment::SetObservationScalar(uint32 inIndex, float inValue)
{
	if (inIndex < mObservations.size())
		mObservations[inIndex] = inValue;
}

void AsterGymEnvironment::SetTerminalState(bool inTerminated, bool inTruncated)
{
	mTerminated = inTerminated;
	mTruncated = inTruncated;
}

ACPH_NAMESPACE_END
