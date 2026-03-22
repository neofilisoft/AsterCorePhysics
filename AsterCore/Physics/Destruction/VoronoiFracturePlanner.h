// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Array.h>
#include <AsterCore/Geometry/AABox.h>
#include <AsterCore/Physics/Destruction/PreFracturedMesh.h>

ACPH_NAMESPACE_BEGIN

struct ACPH_EXPORT VoronoiFractureSettings
{
	AABox						mLocalBounds = AABox(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));
	uint32						mCellCount = 12;
	uint32						mSamplesX = 10;
	uint32						mSamplesY = 10;
	uint32						mSamplesZ = 10;
	uint64						mRandomSeed = 0;
	float						mDensity = 35.0f;
	float						mMinimumChunkExtent = 0.075f;
	float						mAnchorBandHeight = 0.2f;
	float						mConvexRadius = 0.02f;
};

struct ACPH_EXPORT VoronoiFractureCell
{
	Vec3						mSite = Vec3::sZero();
	Vec3						mCentroid = Vec3::sZero();
	AABox						mBounds;
	Array<Vec3>					mSamples;
	uint32						mSampleCount = 0;
};

class ACPH_EXPORT VoronoiFracturePlanner final : public NonCopyable
{
public:
	Array<VoronoiFractureCell>	Generate(const VoronoiFractureSettings &inSettings) const;
	void						AppendCellsAsBoxChunks(const VoronoiFractureSettings &inSettings, const BodyCreationSettings &inChunkTemplate, PreFracturedMesh &ioMesh, bool inAnchorLowestLayer) const;
	void						AppendCellsAsConvexChunks(const VoronoiFractureSettings &inSettings, const BodyCreationSettings &inChunkTemplate, PreFracturedMesh &ioMesh, bool inAnchorLowestLayer) const;
};

ACPH_NAMESPACE_END
