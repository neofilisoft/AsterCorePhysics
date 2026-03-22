// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Vehicle/VehicleWaterInteraction.h>
#include <AsterCore/Physics/Body/BodyInterface.h>

ACPH_NAMESPACE_BEGIN

float VehicleWaterInteraction::sComputeSubmergedFraction(const VehicleWaterInteractionSettings &inSettings, RVec3Arg inCenterOfMass)
{
	const float water_depth = float(inSettings.mWaterLevel - inCenterOfMass.GetY());
	if (water_depth <= 0.0f)
		return 0.0f;
	return Clamp(water_depth / max(inSettings.mReferenceDepth, 1.0e-4f), 0.0f, 1.0f);
}

void VehicleWaterInteraction::sApply(const VehicleWaterInteractionSettings &inSettings, const BodyID &inBodyID, BodyInterface &ioBodyInterface, RVec3Arg inCenterOfMass, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity, Vec3Arg inGravity)
{
	const float submerged_fraction = sComputeSubmergedFraction(inSettings, inCenterOfMass);
	if (submerged_fraction <= 0.0f)
		return;

	const Vec3 buoyancy = -inGravity * (inSettings.mBuoyancyScale * submerged_fraction);
	const Vec3 drag_force = -inLinearVelocity * (inSettings.mLinearDrag * submerged_fraction);
	const Vec3 drag_torque = -inAngularVelocity * (inSettings.mAngularDrag * submerged_fraction);
	ioBodyInterface.AddForceAndTorque(inBodyID, buoyancy + drag_force, drag_torque, EActivation::Activate);
}

ACPH_NAMESPACE_END