// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

ACPH_NAMESPACE_BEGIN

class ACPH_EXPORT SignedDistanceFieldGrid
{
public:
	void						Resize(uint32 inSizeX, uint32 inSizeY, uint32 inSizeZ);
	float &						At(uint32 inX, uint32 inY, uint32 inZ);
	float						At(uint32 inX, uint32 inY, uint32 inZ) const;

	void						SetOrigin(Vec3Arg inOrigin)						{ mOrigin = inOrigin; }
	void						SetCellSize(Vec3Arg inCellSize)					{ mCellSize = inCellSize; }
	Vec3						GetOrigin() const									{ return mOrigin; }
	Vec3						GetCellSize() const								{ return mCellSize; }

	float						SampleDistance(Vec3Arg inPosition) const;
	Vec3						EstimateNormal(Vec3Arg inPosition) const;

private:
	uint32						GetIndex(uint32 inX, uint32 inY, uint32 inZ) const;

	Array<float>				mSamples;
	uint32						mSizeX = 0;
	uint32						mSizeY = 0;
	uint32						mSizeZ = 0;
	Vec3						mOrigin = Vec3::sZero();
	Vec3						mCellSize = Vec3::sReplicate(1.0f);
};

ACPH_NAMESPACE_END