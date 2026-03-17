// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Fluid/ParticleFluidRigidBodyCoupling.h>
#include <AsterCore/Physics/Fluid/ParticleFluidSystem.h>
#include <AsterCore/Physics/Body/BodyInterface.h>
#include <AsterCore/Core/JobSystem.h>
#include <cmath>

ACPH_NAMESPACE_BEGIN

namespace
{
	template <class Body>
	void RunBatched(JobSystem &inJobSystem, uint32 inItemCount, uint32 inMinBatchSize, const char *inName, ColorArg inColor, Body &&inBody)
	{
		if (inItemCount == 0)
			return;

		const uint32 max_concurrency = max<uint32>(uint32(inJobSystem.GetMaxConcurrency()), 1);
		const uint32 batch_size = max<uint32>(inMinBatchSize, (inItemCount + max_concurrency - 1) / max_concurrency);
		const uint32 job_count = (inItemCount + batch_size - 1) / batch_size;
		if (job_count <= 1)
		{
			inBody(0, inItemCount);
			return;
		}

		JobSystem::Barrier *barrier = inJobSystem.CreateBarrier();
		for (uint32 job_index = 0; job_index < job_count; ++job_index)
		{
			const uint32 start = job_index * batch_size;
			const uint32 end = min(start + batch_size, inItemCount);
			JobSystem::JobHandle handle = inJobSystem.CreateJob(inName, inColor, [=, &inBody]() { inBody(start, end); });
			barrier->AddJob(handle);
		}
		inJobSystem.WaitForJobs(barrier);
		inJobSystem.DestroyBarrier(barrier);
	}
}

void ParticleFluidRigidBodyCoupling::sApply(const ParticleFluidSystem &inFluidSystem, const BodyID *inBodies, uint32 inBodyCount, BodyInterface &ioBodyInterface, JobSystem &inJobSystem, const ParticleFluidRigidBodyCouplingSettings &inSettings)
{
	if (inBodies == nullptr || inBodyCount == 0 || inFluidSystem.GetParticleCount() == 0)
		return;

	const ParticleFluidSettings &fluid_settings = inFluidSystem.GetSettings();
	const float interaction_radius = fluid_settings.mSmoothingRadius * max(inSettings.mInteractionRadiusScale, 1.0f);
	const float interaction_radius_sq = interaction_radius * interaction_radius;

	RunBatched(inJobSystem, inBodyCount, inSettings.mMinBatchSize, "FluidRigidCoupling", Color::sCyan, [&](uint32 inStart, uint32 inEnd)
	{
		for (uint32 body_index = inStart; body_index < inEnd; ++body_index)
		{
			const BodyID body_id = inBodies[body_index];
			const RVec3 center_of_mass = ioBodyInterface.GetCenterOfMassPosition(body_id);
			float displaced_mass = 0.0f;
			Vec3 pressure_direction = Vec3::sZero();

			for (uint32 particle_index = 0; particle_index < inFluidSystem.GetParticleCount(); ++particle_index)
			{
				const ParticleFluidParticle &particle = inFluidSystem.GetParticle(particle_index);
				const Vec3 delta = Vec3(center_of_mass - RVec3(particle.mPosition));
				const float distance_sq = delta.LengthSq();
				if (distance_sq > interaction_radius_sq)
					continue;

				const float distance = std::sqrt(distance_sq);
				const float weight = 1.0f - distance / max(interaction_radius, 1.0e-6f);
				displaced_mass += weight * fluid_settings.mParticleMass;
				if (distance > 1.0e-6f)
					pressure_direction += (delta / distance) * (particle.mPressure * weight);
			}

			if (displaced_mass <= 0.0f)
				continue;

			const Vec3 buoyancy_force = -fluid_settings.mGravity * (displaced_mass * inSettings.mBuoyancyScale);
			const Vec3 pressure_force = pressure_direction * inSettings.mPressureScale;
			ioBodyInterface.AddForce(body_id, buoyancy_force + pressure_force, center_of_mass, EActivation::Activate);
		}
	});
}

ACPH_NAMESPACE_END

