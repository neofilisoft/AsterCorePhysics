// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <algorithm>

#include <AsterCore/Physics/Constraints/JiggleConstraint.h>

ACPH_NAMESPACE_BEGIN

void JiggleConstraint::Reset(Vec3Arg inRootPosition, Vec3Arg inRootVelocity)
{
	mPreviousRootAnchor = inRootPosition + mSettings.mLocalAnchor;
	mPreviousRootVelocity = inRootVelocity;
	mPosition = mPreviousRootAnchor;
	mVelocity = Vec3::sZero();
	mInitialized = true;
}

void JiggleConstraint::Update(float inDeltaTime, Vec3Arg inRootPosition, Vec3Arg inRootVelocity, Vec3Arg inGravity)
{
	if (!mInitialized || inDeltaTime <= 0.0f)
	{
		Reset(inRootPosition, inRootVelocity);
		return;
	}

	const Vec3 root_anchor = inRootPosition + mSettings.mLocalAnchor;
	const Vec3 root_delta = root_anchor - mPreviousRootAnchor;
	const Vec3 root_acceleration = (inRootVelocity - mPreviousRootVelocity) / inDeltaTime;

	mPosition += root_delta;

	const Vec3 spring_offset = root_anchor - mPosition;
	const Vec3 effective_acceleration = mSettings.mGravityInfluence * inGravity - mSettings.mInertiaInfluence * root_acceleration;
	const Vec3 spring_force = mSettings.mStiffness * spring_offset - mSettings.mDamping * mVelocity;

	mVelocity += (effective_acceleration + spring_force) * inDeltaTime;
	mPosition += mVelocity * inDeltaTime;

	Vec3 max_offset = mPosition - root_anchor;
	const float max_distance_sq = Square(mSettings.mMaxDistance);
	if (max_offset.LengthSq() > max_distance_sq)
	{
		max_offset = max_offset.Normalized() * mSettings.mMaxDistance;
		mPosition = root_anchor + max_offset;

		const Vec3 radial_direction = max_offset.Normalized();
		mVelocity -= radial_direction * mVelocity.Dot(radial_direction);
	}

	mPreviousRootAnchor = root_anchor;
	mPreviousRootVelocity = inRootVelocity;
}

void JiggleConstraint::sUpdateBatch(JobSystem &inJobSystem, float inDeltaTime, Vec3Arg inGravity, JiggleConstraint *const *inConstraints, const Vec3 *inRootPositions, const Vec3 *inRootVelocities, uint32 inCount, uint32 inMinBatchSize)
{
	if (inCount == 0)
		return;
	const uint32 max_concurrency = max<uint32>(uint32(inJobSystem.GetMaxConcurrency()), 1);
	const uint32 batch_size = max<uint32>(inMinBatchSize, (inCount + max_concurrency - 1) / max_concurrency);
	const uint32 num_jobs = (inCount + batch_size - 1) / batch_size;
	if (num_jobs <= 1)
	{
		for (uint32 i = 0; i < inCount; ++i)
			inConstraints[i]->Update(inDeltaTime, inRootPositions[i], inRootVelocities[i], inGravity);
		return;
	}
	JobSystem::Barrier *barrier = inJobSystem.CreateBarrier();
	for (uint32 job_index = 0; job_index < num_jobs; ++job_index)
	{
		const uint32 start = job_index * batch_size;
		const uint32 end = min(start + batch_size, inCount);
		JobSystem::JobHandle handle = inJobSystem.CreateJob("JiggleConstraintBatch", Color::sOrange, [=]()
		{
			for (uint32 i = start; i < end; ++i)
				inConstraints[i]->Update(inDeltaTime, inRootPositions[i], inRootVelocities[i], inGravity);
		});
		barrier->AddJob(handle);
	}
	inJobSystem.WaitForJobs(barrier);
	inJobSystem.DestroyBarrier(barrier);
}
ACPH_NAMESPACE_END