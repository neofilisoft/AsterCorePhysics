// AsterCore Physics Library
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Vehicle/VehicleAeroDynamics.h>
#include <AsterCore/Physics/Body/BodyInterface.h>

ACPH_NAMESPACE_BEGIN

void VehicleAeroDynamics::sApply(const VehicleAeroDynamicsSettings &inSettings, const BodyID &inBodyID, BodyInterface &ioBodyInterface, RVec3Arg inBodyPosition, QuatArg inBodyRotation, Vec3Arg inLinearVelocity, float inControlInput)
{
	const float speed_sq = inLinearVelocity.LengthSq();
	if (speed_sq <= 1.0e-6f)
		return;

	const Vec3 airflow = -inLinearVelocity;
	const Vec3 airflow_dir = airflow.Normalized();
	const float dynamic_pressure = 0.5f * inSettings.mAirDensity * speed_sq;

	for (const VehicleAeroSurface &surface : inSettings.mSurfaces)
	{
		const RVec3 force_position = inBodyPosition + RVec3(inBodyRotation * surface.mLocalPosition);
		const Vec3 world_normal = inBodyRotation * surface.mLocalNormal.Normalized();
		Vec3 lift_direction = airflow_dir.Cross(world_normal).Cross(airflow_dir);
		if (lift_direction.LengthSq() > 1.0e-6f)
			lift_direction = lift_direction.Normalized();
		else
			lift_direction = world_normal;

		const float lift = dynamic_pressure * surface.mArea * (surface.mLiftCoefficient + surface.mControlInfluence * inControlInput);
		const float drag = dynamic_pressure * surface.mArea * surface.mDragCoefficient;
		const Vec3 total_force = lift_direction * lift + airflow_dir * drag;
		ioBodyInterface.AddForce(inBodyID, total_force, force_position, EActivation::Activate);
	}
}

ACPH_NAMESPACE_END
