// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/JobSystem.h>
#include <AsterCore/Core/NonCopyable.h>

ACPH_NAMESPACE_BEGIN

struct JiggleConstraintSettings
{
	Vec3							mLocalAnchor = Vec3::sZero();
	float							mStiffness = 90.0f;
	float							mDamping = 14.0f;
	float							mGravityInfluence = 1.0f;
	float							mInertiaInfluence = 0.45f;
	float							mMaxDistance = 0.6f;
};

class ACPH_EXPORT JiggleConstraint final : public NonCopyable
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	JiggleConstraint() = default;
	explicit						JiggleConstraint(const JiggleConstraintSettings &inSettings) : mSettings(inSettings) { }

	void							SetSettings(const JiggleConstraintSettings &inSettings)		{ mSettings = inSettings; }
	const JiggleConstraintSettings &GetSettings() const						{ return mSettings; }

	void							Reset(Vec3Arg inRootPosition, Vec3Arg inRootVelocity = Vec3::sZero());
	void							Update(float inDeltaTime, Vec3Arg inRootPosition, Vec3Arg inRootVelocity, Vec3Arg inGravity);

	static void					sUpdateBatch(JobSystem &inJobSystem, float inDeltaTime, Vec3Arg inGravity, JiggleConstraint *const *inConstraints, const Vec3 *inRootPositions, const Vec3 *inRootVelocities, uint32 inCount, uint32 inMinBatchSize = 64);

	Vec3							GetPosition() const									{ return mPosition; }
	Vec3							GetVelocity() const									{ return mVelocity; }

private:
	JiggleConstraintSettings		mSettings;
	Vec3							mPosition = Vec3::sZero();
	Vec3							mVelocity = Vec3::sZero();
	Vec3							mPreviousRootAnchor = Vec3::sZero();
	Vec3							mPreviousRootVelocity = Vec3::sZero();
	bool							mInitialized = false;
};

ACPH_NAMESPACE_END