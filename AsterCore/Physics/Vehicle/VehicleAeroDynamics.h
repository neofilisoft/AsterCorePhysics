// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Body/BodyID.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;

struct VehicleAeroSurface
{
	Vec3						mLocalPosition = Vec3::sZero();
	Vec3						mLocalNormal = Vec3::sAxisY();
	float						mArea = 1.0f;
	float						mLiftCoefficient = 0.0f;
	float						mDragCoefficient = 0.35f;
	float						mControlInfluence = 0.0f;
};

struct VehicleAeroDynamicsSettings
{
	Array<VehicleAeroSurface>	mSurfaces;
	float						mAirDensity = 1.225f;
};

class ACPH_EXPORT VehicleAeroDynamics final
{
public:
	static void					sApply(const VehicleAeroDynamicsSettings &inSettings, const BodyID &inBodyID, BodyInterface &ioBodyInterface, RVec3Arg inBodyPosition, QuatArg inBodyRotation, Vec3Arg inLinearVelocity, float inControlInput);
};

ACPH_NAMESPACE_END