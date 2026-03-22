// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Body/BodyID.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;

struct VehicleWaterInteractionSettings
{
	float						mWaterLevel = 0.0f;
	float						mReferenceDepth = 1.0f;
	float						mBuoyancyScale = 1.0f;
	float						mLinearDrag = 0.8f;
	float						mAngularDrag = 0.2f;
};

class ACPH_EXPORT VehicleWaterInteraction final
{
public:
	static float					sComputeSubmergedFraction(const VehicleWaterInteractionSettings &inSettings, RVec3Arg inCenterOfMass);
	static void					sApply(const VehicleWaterInteractionSettings &inSettings, const BodyID &inBodyID, BodyInterface &ioBodyInterface, RVec3Arg inCenterOfMass, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity, Vec3Arg inGravity);
};

ACPH_NAMESPACE_END