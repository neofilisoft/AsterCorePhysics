// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Array.h>
#include <AsterCore/Core/Reference.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>

ACPH_NAMESPACE_BEGIN

enum class EChunkTemplateOverride : uint32
{
	None					= 0,
	ObjectLayer				= 1 << 0,
	CollisionGroup			= 1 << 1,
	MaterialProperties		= 1 << 2,
	Damping					= 1 << 3,
	GravityFactor			= 1 << 4,
	VelocityLimits			= 1 << 5,
	MotionQuality			= 1 << 6,
	MotionProperties		= 1 << 7,
	SolverSteps				= 1 << 8,
	UserData				= 1 << 9,
	MassProperties			= 1 << 10,
	All						= 0xffffffffu
};

constexpr EChunkTemplateOverride operator | (EChunkTemplateOverride inLHS, EChunkTemplateOverride inRHS)
{
	return EChunkTemplateOverride(uint32(inLHS) | uint32(inRHS));
}

constexpr EChunkTemplateOverride operator & (EChunkTemplateOverride inLHS, EChunkTemplateOverride inRHS)
{
	return EChunkTemplateOverride(uint32(inLHS) & uint32(inRHS));
}

constexpr EChunkTemplateOverride &operator |= (EChunkTemplateOverride &ioLHS, EChunkTemplateOverride inRHS)
{
	ioLHS = ioLHS | inRHS;
	return ioLHS;
}

constexpr bool HasChunkTemplateOverride(EChunkTemplateOverride inValue, EChunkTemplateOverride inFlag)
{
	return (uint32(inValue & inFlag) != 0);
}

struct ACPH_EXPORT PreFracturedChunk
{
	RefConst<Shape>				mShape;
	Vec3						mLocalPosition = Vec3::sZero();
	Quat						mLocalRotation = Quat::sIdentity();
	float						mMass = 10.0f;
	bool						mAnchored = false;
	bool						mOverrideObjectLayer = false;
	ObjectLayer					mObjectLayer = 0;
	bool						mOverrideMotionQuality = false;
	EMotionQuality				mMotionQuality = EMotionQuality::Discrete;
	float						mImpulseScale = 1.0f;
	bool						mOverrideUserData = false;
	uint64						mUserData = 0;
};

class ACPH_EXPORT PreFracturedMesh
{
public:
	void						SetChunkTemplate(const BodyCreationSettings &inChunkTemplate)	{ mChunkTemplate = inChunkTemplate; }
	const BodyCreationSettings &GetChunkTemplate() const								{ return mChunkTemplate; }
	void						SetChunkTemplateOverrides(EChunkTemplateOverride inOverrides)	{ mChunkTemplateOverrides = inOverrides; }
	EChunkTemplateOverride		GetChunkTemplateOverrides() const								{ return mChunkTemplateOverrides; }

	void						AddChunk(const PreFracturedChunk &inChunk);
	bool						IsValid() const;

	uint32						GetChunkCount() const										{ return uint32(mChunks.size()); }
	const PreFracturedChunk &	GetChunk(uint32 inChunkIndex) const						{ return mChunks[inChunkIndex]; }

	RVec3						GetChunkWorldPosition(uint32 inChunkIndex, RVec3Arg inRootPosition, QuatArg inRootRotation) const;
	Quat						GetChunkWorldRotation(uint32 inChunkIndex, QuatArg inRootRotation) const;
	BodyCreationSettings		CreateChunkCreationSettings(uint32 inChunkIndex, RVec3Arg inRootPosition, QuatArg inRootRotation, EMotionType inMotionType) const;
	BodyCreationSettings		CreateChunkCreationSettings(uint32 inChunkIndex, const BodyCreationSettings &inIntactTemplate, RVec3Arg inRootPosition, QuatArg inRootRotation, EMotionType inMotionType) const;

private:
	void						ApplyChunkTemplateOverrides(BodyCreationSettings &ioSettings) const;

	BodyCreationSettings		mChunkTemplate;
	EChunkTemplateOverride		mChunkTemplateOverrides = EChunkTemplateOverride::None;
	Array<PreFracturedChunk>	mChunks;
};

ACPH_NAMESPACE_END
