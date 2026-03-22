// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/Physics/Destruction/PreFracturedMesh.h>

ACPH_NAMESPACE_BEGIN

void PreFracturedMesh::AddChunk(const PreFracturedChunk &inChunk)
{
	mChunks.push_back(inChunk);
}

bool PreFracturedMesh::IsValid() const
{
	if (mChunks.empty())
		return false;

	for (const PreFracturedChunk &chunk : mChunks)
		if (chunk.mShape == nullptr)
			return false;

	return true;
}

RVec3 PreFracturedMesh::GetChunkWorldPosition(uint32 inChunkIndex, RVec3Arg inRootPosition, QuatArg inRootRotation) const
{
	ACPH_ASSERT(inChunkIndex < mChunks.size());
	const PreFracturedChunk &chunk = mChunks[inChunkIndex];
	return inRootPosition + RVec3(inRootRotation * chunk.mLocalPosition);
}

Quat PreFracturedMesh::GetChunkWorldRotation(uint32 inChunkIndex, QuatArg inRootRotation) const
{
	ACPH_ASSERT(inChunkIndex < mChunks.size());
	return inRootRotation * mChunks[inChunkIndex].mLocalRotation;
}

BodyCreationSettings PreFracturedMesh::CreateChunkCreationSettings(uint32 inChunkIndex, RVec3Arg inRootPosition, QuatArg inRootRotation, EMotionType inMotionType) const
{
	return CreateChunkCreationSettings(inChunkIndex, mChunkTemplate, inRootPosition, inRootRotation, inMotionType);
}

BodyCreationSettings PreFracturedMesh::CreateChunkCreationSettings(uint32 inChunkIndex, const BodyCreationSettings &inIntactTemplate, RVec3Arg inRootPosition, QuatArg inRootRotation, EMotionType inMotionType) const
{
	ACPH_ASSERT(inChunkIndex < mChunks.size());

	const PreFracturedChunk &chunk = mChunks[inChunkIndex];
	BodyCreationSettings settings = inIntactTemplate;
	ApplyChunkTemplateOverrides(settings);
	settings.SetShape(chunk.mShape);
	settings.mPosition = GetChunkWorldPosition(inChunkIndex, inRootPosition, inRootRotation);
	settings.mRotation = GetChunkWorldRotation(inChunkIndex, inRootRotation);
	settings.mMotionType = inMotionType;
	if (chunk.mOverrideObjectLayer)
		settings.mObjectLayer = chunk.mObjectLayer;
	if (chunk.mOverrideMotionQuality)
		settings.mMotionQuality = chunk.mMotionQuality;
	if (chunk.mOverrideUserData)
		settings.mUserData = chunk.mUserData;
	if (chunk.mMass > 0.0f)
	{
		settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
		settings.mMassPropertiesOverride.mMass = chunk.mMass;
	}
	return settings;
}

void PreFracturedMesh::ApplyChunkTemplateOverrides(BodyCreationSettings &ioSettings) const
{
	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::ObjectLayer))
		ioSettings.mObjectLayer = mChunkTemplate.mObjectLayer;

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::CollisionGroup))
		ioSettings.mCollisionGroup = mChunkTemplate.mCollisionGroup;

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::MaterialProperties))
	{
		ioSettings.mFriction = mChunkTemplate.mFriction;
		ioSettings.mRestitution = mChunkTemplate.mRestitution;
	}

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::Damping))
	{
		ioSettings.mLinearDamping = mChunkTemplate.mLinearDamping;
		ioSettings.mAngularDamping = mChunkTemplate.mAngularDamping;
	}

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::GravityFactor))
		ioSettings.mGravityFactor = mChunkTemplate.mGravityFactor;

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::VelocityLimits))
	{
		ioSettings.mMaxLinearVelocity = mChunkTemplate.mMaxLinearVelocity;
		ioSettings.mMaxAngularVelocity = mChunkTemplate.mMaxAngularVelocity;
	}

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::MotionQuality))
		ioSettings.mMotionQuality = mChunkTemplate.mMotionQuality;

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::MotionProperties))
	{
		ioSettings.mAllowedDOFs = mChunkTemplate.mAllowedDOFs;
		ioSettings.mAllowDynamicOrKinematic = mChunkTemplate.mAllowDynamicOrKinematic;
		ioSettings.mIsSensor = mChunkTemplate.mIsSensor;
		ioSettings.mCollideKinematicVsNonDynamic = mChunkTemplate.mCollideKinematicVsNonDynamic;
		ioSettings.mUseManifoldReduction = mChunkTemplate.mUseManifoldReduction;
		ioSettings.mApplyGyroscopicForce = mChunkTemplate.mApplyGyroscopicForce;
		ioSettings.mEnhancedInternalEdgeRemoval = mChunkTemplate.mEnhancedInternalEdgeRemoval;
		ioSettings.mAllowSleeping = mChunkTemplate.mAllowSleeping;
	}

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::SolverSteps))
	{
		ioSettings.mNumVelocityStepsOverride = mChunkTemplate.mNumVelocityStepsOverride;
		ioSettings.mNumPositionStepsOverride = mChunkTemplate.mNumPositionStepsOverride;
	}

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::UserData))
		ioSettings.mUserData = mChunkTemplate.mUserData;

	if (HasChunkTemplateOverride(mChunkTemplateOverrides, EChunkTemplateOverride::MassProperties))
	{
		ioSettings.mOverrideMassProperties = mChunkTemplate.mOverrideMassProperties;
		ioSettings.mInertiaMultiplier = mChunkTemplate.mInertiaMultiplier;
		ioSettings.mMassPropertiesOverride = mChunkTemplate.mMassPropertiesOverride;
	}
}

ACPH_NAMESPACE_END
