// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

struct SoftBodyFractureSettings
{
	bool						mEnableTearing = false;
	float						mStrainThreshold = 1.75f;
	float						mMinimumEdgeLength = 0.02f;
	uint32						mMaxFracturesPerStep = 8;
};

struct SoftBodySelfCollisionSettings
{
	bool						mEnableSelfCollision = true;
	float						mThickness = 0.025f;
	uint32						mHashBucketCount = 1024;
	uint32						mMaxCandidatePairs = 4096;
};

struct SoftBodySelfCollisionPair
{
	uint32						mVertexA = 0;
	uint32						mVertexB = 0;
};

class ACPH_EXPORT SoftBodyAdvancedSimulation final
{
public:
	static bool					sShouldFractureEdge(float inRestLength, float inCurrentLength, const SoftBodyFractureSettings &inSettings);
	static void					sBuildSelfCollisionPairs(const Vec3 *inPositions, uint32 inVertexCount, const SoftBodySelfCollisionSettings &inSettings, Array<SoftBodySelfCollisionPair> &outPairs);
};

ACPH_NAMESPACE_END