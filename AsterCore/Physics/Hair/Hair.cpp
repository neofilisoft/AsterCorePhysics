// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Hair/Hair.h>
#include <AsterCore/Physics/Hair/HairSettings.h>
#include <cmath>
#include <unordered_map>

ACPH_NAMESPACE_BEGIN

namespace
{
	struct HairCellCoordinate
	{
		int		mX;
		int		mY;
		int		mZ;
	};

	static uint64 HashCoordinate(const HairCellCoordinate &inCoordinate)
	{
		const uint64 x = uint64(uint32(inCoordinate.mX * 73856093));
		const uint64 y = uint64(uint32(inCoordinate.mY * 19349663));
		const uint64 z = uint64(uint32(inCoordinate.mZ * 83492791));
		return x ^ (y << 1) ^ (z << 2);
	}
}

bool Hair::sShouldBreakStrandSegment(float inRestLength, float inCurrentLength, const HairStrandSettings &inSettings)
{
	if (!inSettings.mEnableTearing || inRestLength <= 0.0f || inCurrentLength <= inSettings.mMinimumEdgeLength)
		return false;

	return (inCurrentLength / inRestLength) >= inSettings.mStrainThreshold;
}

void Hair::sBuildSelfCollisionPairs(const Vec3 *inPositions, uint32 inVertexCount, const HairSelfCollisionSettings &inSettings, Array<HairSelfCollisionPair> &outPairs)
{
	outPairs.clear();
	if (!inSettings.mEnableSelfCollision || inPositions == nullptr || inVertexCount < 2)
		return;

	const float cell_size = HairSettingsUtility::sGetCollisionCellSize(inSettings);
	std::unordered_map<uint64, Array<uint32>> buckets;
	buckets.reserve(inSettings.mHashBucketCount);

	for (uint32 i = 0; i < inVertexCount; ++i)
	{
		const Vec3 position = inPositions[i];
		const HairCellCoordinate coordinate
		{
			int(std::floor(position.GetX() / cell_size)),
			int(std::floor(position.GetY() / cell_size)),
			int(std::floor(position.GetZ() / cell_size))
		};
		buckets[HashCoordinate(coordinate)].push_back(i);
	}

	const float max_distance_sq = Square(inSettings.mThickness);
	for (uint32 i = 0; i < inVertexCount && outPairs.size() < inSettings.mMaxCandidatePairs; ++i)
	{
		const Vec3 position = inPositions[i];
		const HairCellCoordinate center
		{
			int(std::floor(position.GetX() / cell_size)),
			int(std::floor(position.GetY() / cell_size)),
			int(std::floor(position.GetZ() / cell_size))
		};

		for (int z = -1; z <= 1 && outPairs.size() < inSettings.mMaxCandidatePairs; ++z)
			for (int y = -1; y <= 1 && outPairs.size() < inSettings.mMaxCandidatePairs; ++y)
				for (int x = -1; x <= 1 && outPairs.size() < inSettings.mMaxCandidatePairs; ++x)
				{
					const uint64 hash = HashCoordinate({ center.mX + x, center.mY + y, center.mZ + z });
					auto bucket = buckets.find(hash);
					if (bucket == buckets.end())
						continue;

					for (uint32 candidate : bucket->second)
					{
						if (candidate <= i)
							continue;

						if ((inPositions[candidate] - position).LengthSq() <= max_distance_sq)
							outPairs.push_back({ i, candidate });

						if (outPairs.size() >= inSettings.mMaxCandidatePairs)
							break;
					}
				}
	}
}

ACPH_NAMESPACE_END
