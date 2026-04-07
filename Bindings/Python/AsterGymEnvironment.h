#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Core/Array.h>
#include <AsterCore/Core/NonCopyable.h>
#include <AsterCore/Physics/EPhysicsUpdateError.h>

ACPH_NAMESPACE_BEGIN

class PhysicsSystem;
class JobSystem;
class TempAllocator;

struct AsterGymEnvironmentSettings
{
	uint32	mObservationSize = 0;
	uint32	mActionSize = 0;
	float	mFixedDeltaTime = 1.0f / 60.0f;
	uint32	mSubSteps = 1;
	uint32	mMaxEpisodeSteps = 4096;
};

struct AsterGymStepResult
{
	float				mReward = 0.0f;
	bool				mTerminated = false;
	bool				mTruncated = false;
	float				mSimulationTime = 0.0f;
	uint32				mEpisodeStep = 0;
	EPhysicsUpdateError	mUpdateError = EPhysicsUpdateError::None;
};

class ACPH_EXPORT AsterGymEnvironment final : public NonCopyable
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	void Configure(const AsterGymEnvironmentSettings &inSettings);
	const AsterGymEnvironmentSettings &GetSettings() const { return mSettings; }

	void AttachSimulation(PhysicsSystem *inPhysicsSystem, JobSystem *inJobSystem, TempAllocator *inTempAllocator);
	void Reset();
	AsterGymStepResult Step(const float *inActions, uint32 inActionCount);

	void SetObservationScalar(uint32 inIndex, float inValue);
	void AddReward(float inReward) { mPendingReward += inReward; }
	void SetTerminalState(bool inTerminated, bool inTruncated);

	const float *GetObservationData() const { return mObservations.empty()? nullptr : mObservations.data(); }
	uint32 GetObservationSize() const { return uint32(mObservations.size()); }
	const float *GetActionData() const { return mActions.empty()? nullptr : mActions.data(); }
	uint32 GetActionSize() const { return uint32(mActions.size()); }
	float GetSimulationTime() const { return mSimulationTime; }
	uint32 GetEpisodeStep() const { return mEpisodeStep; }

private:
	AsterGymEnvironmentSettings	mSettings;
	PhysicsSystem *mPhysicsSystem = nullptr;
	JobSystem *mJobSystem = nullptr;
	TempAllocator *mTempAllocator = nullptr;
	Array<float> mObservations;
	Array<float> mActions;
	float mSimulationTime = 0.0f;
	uint32 mEpisodeStep = 0;
	float mPendingReward = 0.0f;
	bool mTerminated = false;
	bool mTruncated = false;
};

ACPH_NAMESPACE_END
