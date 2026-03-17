// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Collision/SDF/SignedDistanceFieldGrid.h>
#include <cmath>

ACPH_NAMESPACE_BEGIN

namespace
{
	static inline float SDFLerp(float inA, float inB, float inT)
	{
		return inA + (inB - inA) * inT;
	}
}

void SignedDistanceFieldGrid::Resize(uint32 inSizeX, uint32 inSizeY, uint32 inSizeZ)
{
	mSizeX = max(inSizeX, 1U);
	mSizeY = max(inSizeY, 1U);
	mSizeZ = max(inSizeZ, 1U);
	mSamples.resize(size_t(mSizeX) * mSizeY * mSizeZ, 0.0f);
}

uint32 SignedDistanceFieldGrid::GetIndex(uint32 inX, uint32 inY, uint32 inZ) const
{
	return (inZ * mSizeY + inY) * mSizeX + inX;
}

float &SignedDistanceFieldGrid::At(uint32 inX, uint32 inY, uint32 inZ)
{
	return mSamples[GetIndex(inX, inY, inZ)];
}

float SignedDistanceFieldGrid::At(uint32 inX, uint32 inY, uint32 inZ) const
{
	return mSamples[GetIndex(inX, inY, inZ)];
}

float SignedDistanceFieldGrid::SampleDistance(Vec3Arg inPosition) const
{
	if (mSamples.empty())
		return FLT_MAX;

	const Vec3 grid = (inPosition - mOrigin) / mCellSize;
	const float fx = Clamp(grid.GetX(), 0.0f, float(mSizeX - 1));
	const float fy = Clamp(grid.GetY(), 0.0f, float(mSizeY - 1));
	const float fz = Clamp(grid.GetZ(), 0.0f, float(mSizeZ - 1));

	const uint32 x0 = uint32(std::floor(fx));
	const uint32 y0 = uint32(std::floor(fy));
	const uint32 z0 = uint32(std::floor(fz));
	const uint32 x1 = min(x0 + 1, mSizeX - 1);
	const uint32 y1 = min(y0 + 1, mSizeY - 1);
	const uint32 z1 = min(z0 + 1, mSizeZ - 1);
	const float tx = fx - float(x0);
	const float ty = fy - float(y0);
	const float tz = fz - float(z0);

	const float c000 = At(x0, y0, z0);
	const float c100 = At(x1, y0, z0);
	const float c010 = At(x0, y1, z0);
	const float c110 = At(x1, y1, z0);
	const float c001 = At(x0, y0, z1);
	const float c101 = At(x1, y0, z1);
	const float c011 = At(x0, y1, z1);
	const float c111 = At(x1, y1, z1);

	const float c00 = SDFLerp(c000, c100, tx);
	const float c10 = SDFLerp(c010, c110, tx);
	const float c01 = SDFLerp(c001, c101, tx);
	const float c11 = SDFLerp(c011, c111, tx);
	const float c0 = SDFLerp(c00, c10, ty);
	const float c1 = SDFLerp(c01, c11, ty);
	return SDFLerp(c0, c1, tz);
}

Vec3 SignedDistanceFieldGrid::EstimateNormal(Vec3Arg inPosition) const
{
	const Vec3 epsilon = mCellSize * 0.5f;
	const float dx = SampleDistance(inPosition + Vec3(epsilon.GetX(), 0.0f, 0.0f)) - SampleDistance(inPosition - Vec3(epsilon.GetX(), 0.0f, 0.0f));
	const float dy = SampleDistance(inPosition + Vec3(0.0f, epsilon.GetY(), 0.0f)) - SampleDistance(inPosition - Vec3(0.0f, epsilon.GetY(), 0.0f));
	const float dz = SampleDistance(inPosition + Vec3(0.0f, 0.0f, epsilon.GetZ())) - SampleDistance(inPosition - Vec3(0.0f, 0.0f, epsilon.GetZ()));
	Vec3 normal(dx, dy, dz);
	if (normal.LengthSq() <= 1.0e-12f)
		return Vec3::sAxisY();
	return normal.Normalized();
}

ACPH_NAMESPACE_END
