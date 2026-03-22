// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Hair/HairSettings.h>

ACPH_NAMESPACE_BEGIN

bool HairSettingsUtility::sIsValid(const HairSettings &inSettings)
{
	return inSettings.mStrandSettings.mStrainThreshold >= 1.0f
		&& inSettings.mStrandSettings.mMinimumEdgeLength >= 0.0f
		&& inSettings.mStrandSettings.mMaxFracturesPerStep > 0
		&& inSettings.mSelfCollisionSettings.mThickness >= 0.0f
		&& inSettings.mSelfCollisionSettings.mHashBucketCount > 0
		&& inSettings.mSelfCollisionSettings.mMaxCandidatePairs > 0;
}

float HairSettingsUtility::sGetCollisionCellSize(const HairSelfCollisionSettings &inSettings)
{
	return max(inSettings.mThickness, 1.0e-4f);
}

ACPH_NAMESPACE_END
