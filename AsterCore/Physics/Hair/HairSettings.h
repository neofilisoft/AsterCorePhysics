// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

struct HairStrandSettings
{
	bool						mEnableTearing = false;
	float						mStrainThreshold = 1.75f;
	float						mMinimumEdgeLength = 0.02f;
	uint32						mMaxFracturesPerStep = 8;
};

struct HairSelfCollisionSettings
{
	bool						mEnableSelfCollision = true;
	float						mThickness = 0.025f;
	uint32						mHashBucketCount = 1024;
	uint32						mMaxCandidatePairs = 4096;
};

struct HairSelfCollisionPair
{
	uint32						mVertexA = 0;
	uint32						mVertexB = 0;
};

struct HairSettings
{
	HairStrandSettings			mStrandSettings;
	HairSelfCollisionSettings	mSelfCollisionSettings;
};

class ACPH_EXPORT HairSettingsUtility final
{
public:
	static bool					sIsValid(const HairSettings &inSettings);
	static float				sGetCollisionCellSize(const HairSelfCollisionSettings &inSettings);
};

ACPH_NAMESPACE_END
