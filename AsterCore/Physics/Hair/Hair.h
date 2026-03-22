// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Hair/HairSettings.h>

ACPH_NAMESPACE_BEGIN

class ACPH_EXPORT Hair final
{
public:
	static bool					sShouldBreakStrandSegment(float inRestLength, float inCurrentLength, const HairStrandSettings &inSettings);
	static void					sBuildSelfCollisionPairs(const Vec3 *inPositions, uint32 inVertexCount, const HairSelfCollisionSettings &inSettings, Array<HairSelfCollisionPair> &outPairs);
};

ACPH_NAMESPACE_END
