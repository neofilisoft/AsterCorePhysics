// AsterCore Physics Library
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/GPU/PhysicsComputeScheduler.h>

ACPH_NAMESPACE_BEGIN

void PhysicsComputeScheduler::SetBackend(PhysicsComputeBackend *inBackend)
{
	unique_lock lock(mMutex);
	mBackend = inBackend;
}

PhysicsComputeBackend *PhysicsComputeScheduler::GetBackend() const
{
	shared_lock lock(mMutex);
	return mBackend;
}

bool PhysicsComputeScheduler::TryDispatchBroadPhase(const PhysicsComputeBroadPhaseRequest &inRequest, PhysicsComputeBroadPhaseResult &ioResult) const
{
	shared_lock lock(mMutex);
	return mBackend != nullptr && mBackend->Supports(EPhysicsComputeStage::BroadPhase) && mBackend->DispatchBroadPhase(inRequest, ioResult);
}

bool PhysicsComputeScheduler::TryDispatchNarrowPhase(const PhysicsComputeNarrowPhaseRequest &inRequest, PhysicsComputeNarrowPhaseResult &ioResult) const
{
	shared_lock lock(mMutex);
	return mBackend != nullptr && mBackend->Supports(EPhysicsComputeStage::NarrowPhase) && mBackend->DispatchNarrowPhase(inRequest, ioResult);
}

ACPH_NAMESPACE_END
