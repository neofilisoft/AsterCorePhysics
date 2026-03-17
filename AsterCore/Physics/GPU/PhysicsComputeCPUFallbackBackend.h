#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/GPU/PhysicsComputeBackend.h>

ACPH_NAMESPACE_BEGIN

class ACPH_EXPORT PhysicsComputeCPUFallbackBackend final : public PhysicsComputeBackend
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	explicit PhysicsComputeCPUFallbackBackend(uint32 inMaxPairsPerDispatch = 1 << 20) : mMaxPairsPerDispatch(inMaxPairsPerDispatch) { }

	uint32 GetSupportedStages() const override { return uint32(EPhysicsComputeStage::BroadPhase); }
	bool DispatchBroadPhase(const PhysicsComputeBroadPhaseRequest &inRequest, PhysicsComputeBroadPhaseResult &ioResult) override;
	bool DispatchNarrowPhase(const PhysicsComputeNarrowPhaseRequest &inRequest, PhysicsComputeNarrowPhaseResult &ioResult) override;

private:
	uint32 mMaxPairsPerDispatch;
};

ACPH_NAMESPACE_END
