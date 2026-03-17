#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/GPU/PhysicsComputeCPUFallbackBackend.h>

ACPH_NAMESPACE_BEGIN

bool PhysicsComputeCPUFallbackBackend::DispatchBroadPhase(const PhysicsComputeBroadPhaseRequest &inRequest, PhysicsComputeBroadPhaseResult &ioResult)
{
	if (inRequest.mBounds.mData == nullptr || inRequest.mBodyIDs.mData == nullptr || ioResult.mPairs == nullptr)
		return false;
	if (inRequest.mBounds.mStride < sizeof(AABox) || inRequest.mBodyIDs.mStride < sizeof(BodyID))
		return false;

	const uint32 body_count = min(inRequest.mBounds.mCount, inRequest.mBodyIDs.mCount);
	const uint8 *bounds_bytes = reinterpret_cast<const uint8 *>(inRequest.mBounds.mData);
	const uint8 *id_bytes = reinterpret_cast<const uint8 *>(inRequest.mBodyIDs.mData);

	ioResult.mPairCount = 0;
	ioResult.mUsedAccelerator = false;
	for (uint32 a = 0; a < body_count; ++a)
	{
		const AABox &bounds_a = *reinterpret_cast<const AABox *>(bounds_bytes + size_t(a) * inRequest.mBounds.mStride);
		const BodyID &body_a = *reinterpret_cast<const BodyID *>(id_bytes + size_t(a) * inRequest.mBodyIDs.mStride);
		if (!inRequest.mWorldBounds.Overlaps(bounds_a))
			continue;

		for (uint32 b = a + 1; b < body_count; ++b)
		{
			if (ioResult.mPairCount >= ioResult.mPairCapacity || ioResult.mPairCount >= mMaxPairsPerDispatch)
				return true;

			const AABox &bounds_b = *reinterpret_cast<const AABox *>(bounds_bytes + size_t(b) * inRequest.mBounds.mStride);
			if (!inRequest.mWorldBounds.Overlaps(bounds_b) || !bounds_a.Overlaps(bounds_b))
				continue;

			const BodyID &body_b = *reinterpret_cast<const BodyID *>(id_bytes + size_t(b) * inRequest.mBodyIDs.mStride);
			ioResult.mPairs[ioResult.mPairCount++] = { body_a, body_b };
		}
	}

	return true;
}

bool PhysicsComputeCPUFallbackBackend::DispatchNarrowPhase(const PhysicsComputeNarrowPhaseRequest &inRequest, PhysicsComputeNarrowPhaseResult &ioResult)
{
	ioResult.mContactCount = 0;
	ioResult.mUsedAccelerator = false;
	return inRequest.mCandidatePairs.mData != nullptr;
}

ACPH_NAMESPACE_END
