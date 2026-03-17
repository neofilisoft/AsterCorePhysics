// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>
#include <algorithm>
#include <cmath>

#include <AsterCore/Physics/Fluid/ParticleFluidSystem.h>
#include <AsterCore/Physics/Fluid/ParticleFluidRigidBodyCoupling.h>

ACPH_NAMESPACE_BEGIN

namespace
{
	static constexpr float cDensityEpsilon = 1.0e-6f;
}

ParticleFluidSystem::ParticleFluidSystem()
{
	mSharedBuffer = new ParticleFluidSharedBuffer();
	SetSettings(mSettings);
}

void ParticleFluidSystem::SetSettings(const ParticleFluidSettings &inSettings)
{
	mSettings = inSettings;
	mSmoothingRadiusSquared = Square(mSettings.mSmoothingRadius);

	const float h = mSettings.mSmoothingRadius;
	const float h2 = h * h;
	const float h6 = h2 * h2 * h2;
	const float h9 = h6 * h2 * h;

	mKernelPoly6Coefficient = 315.0f / (64.0f * ACPH_PI * h9);
	mKernelSpikyGradientCoefficient = -45.0f / (ACPH_PI * h6);
	mKernelViscosityLaplacianCoefficient = 45.0f / (ACPH_PI * h6);

	EnsureCapacity();
}

void ParticleFluidSystem::Clear()
{
	mParticles.clear();
	if (mSharedBuffer != nullptr)
		mSharedBuffer->Initialize(0);
	EnsureCapacity();
}

void ParticleFluidSystem::SetParticles(const Array<ParticleFluidParticle> &inParticles)
{
	mParticles = inParticles;
	EnsureCapacity();
}

void ParticleFluidSystem::AddParticle(const ParticleFluidParticle &inParticle)
{
	mParticles.push_back(inParticle);
	EnsureCapacity();
}

void ParticleFluidSystem::EnsureCapacity()
{
	const uint32 particle_count = uint32(mParticles.size());
	if (mSharedBuffer == nullptr)
		mSharedBuffer = new ParticleFluidSharedBuffer();
	if (mSharedBuffer->GetCapacity() < particle_count)
		mSharedBuffer->Initialize(particle_count);

	mParticleHashes.resize(particle_count);
	mSortedParticleIndices.resize(particle_count);
	mCellCounts.resize(max<uint32>(mSettings.mHashBucketCount, 1));
	mCellOffsets.resize(max<uint32>(mSettings.mHashBucketCount, 1) + 1);
}

auto ParticleFluidSystem::GetGridCoordinate(Vec3Arg inPosition) const -> GridCoordinate
{
	const float inv_cell_size = 1.0f / max(mSettings.mSmoothingRadius, 1.0e-6f);
	return
	{
		int(std::floor(inPosition.GetX() * inv_cell_size)),
		int(std::floor(inPosition.GetY() * inv_cell_size)),
		int(std::floor(inPosition.GetZ() * inv_cell_size))
	};
}

uint32 ParticleFluidSystem::HashCoordinate(const GridCoordinate &inCoordinate) const
{
	const uint32 bucket_count = max<uint32>(mSettings.mHashBucketCount, 1);
	uint32 x = uint32(inCoordinate.mX * 73856093);
	uint32 y = uint32(inCoordinate.mY * 19349663);
	uint32 z = uint32(inCoordinate.mZ * 83492791);
	return (x ^ y ^ z) % bucket_count;
}

void ParticleFluidSystem::RebuildSpatialHash()
{
	std::fill(mCellCounts.begin(), mCellCounts.end(), 0U);

	for (uint32 i = 0, particle_count = uint32(mParticles.size()); i < particle_count; ++i)
	{
		uint32 hash = HashCoordinate(GetGridCoordinate(mParticles[i].mPosition));
		mParticleHashes[i] = hash;
		++mCellCounts[hash];
	}

	uint32 running_offset = 0;
	for (uint32 bucket = 0, bucket_count = uint32(mCellCounts.size()); bucket < bucket_count; ++bucket)
	{
		mCellOffsets[bucket] = running_offset;
		running_offset += mCellCounts[bucket];
		mCellCounts[bucket] = 0;
	}
	mCellOffsets[mCellCounts.size()] = running_offset;

	for (uint32 i = 0, particle_count = uint32(mParticles.size()); i < particle_count; ++i)
	{
		uint32 hash = mParticleHashes[i];
		uint32 slot = mCellOffsets[hash] + mCellCounts[hash]++;
		mSortedParticleIndices[slot] = i;
	}
}

float ParticleFluidSystem::KernelPoly6(float inDistanceSquared) const
{
	if (inDistanceSquared >= mSmoothingRadiusSquared)
		return 0.0f;

	float delta = mSmoothingRadiusSquared - inDistanceSquared;
	return mKernelPoly6Coefficient * delta * delta * delta;
}

float ParticleFluidSystem::KernelSpikyGradient(float inDistance) const
{
	if (inDistance <= 0.0f || inDistance >= mSettings.mSmoothingRadius)
		return 0.0f;

	float delta = mSettings.mSmoothingRadius - inDistance;
	return mKernelSpikyGradientCoefficient * delta * delta;
}

float ParticleFluidSystem::KernelViscosityLaplacian(float inDistance) const
{
	if (inDistance >= mSettings.mSmoothingRadius)
		return 0.0f;

	return mKernelViscosityLaplacianCoefficient * (mSettings.mSmoothingRadius - inDistance);
}

void ParticleFluidSystem::ComputeDensityPressureRange(uint32 inStart, uint32 inEnd)
{
	for (uint32 i = inStart; i < inEnd; ++i)
	{
		ParticleFluidParticle &particle = mParticles[i];
		const GridCoordinate center = GetGridCoordinate(particle.mPosition);

		float density = 0.0f;
		for (int z = -1; z <= 1; ++z)
			for (int y = -1; y <= 1; ++y)
				for (int x = -1; x <= 1; ++x)
				{
					const uint32 hash = HashCoordinate({ center.mX + x, center.mY + y, center.mZ + z });
					for (uint32 sorted = mCellOffsets[hash], end = mCellOffsets[hash + 1]; sorted < end; ++sorted)
					{
						const ParticleFluidParticle &neighbor = mParticles[mSortedParticleIndices[sorted]];
						const float distance_squared = (neighbor.mPosition - particle.mPosition).LengthSq();
						density += mSettings.mParticleMass * KernelPoly6(distance_squared);
					}
				}

		particle.mDensity = max(density, cDensityEpsilon);
		particle.mPressure = mSettings.mPressureStiffness * max(particle.mDensity - mSettings.mRestDensity, 0.0f);
	}
}

void ParticleFluidSystem::ComputeForceRange(uint32 inStart, uint32 inEnd)
{
	for (uint32 i = inStart; i < inEnd; ++i)
	{
		ParticleFluidParticle &particle = mParticles[i];
		const GridCoordinate center = GetGridCoordinate(particle.mPosition);

		Vec3 pressure_force = Vec3::sZero();
		Vec3 viscosity_force = Vec3::sZero();

		for (int z = -1; z <= 1; ++z)
			for (int y = -1; y <= 1; ++y)
				for (int x = -1; x <= 1; ++x)
				{
					const uint32 hash = HashCoordinate({ center.mX + x, center.mY + y, center.mZ + z });
					for (uint32 sorted = mCellOffsets[hash], end = mCellOffsets[hash + 1]; sorted < end; ++sorted)
					{
						const uint32 neighbor_index = mSortedParticleIndices[sorted];
						if (neighbor_index == i)
							continue;

						const ParticleFluidParticle &neighbor = mParticles[neighbor_index];
						Vec3 delta = particle.mPosition - neighbor.mPosition;
						const float distance = delta.Length();
						if (distance <= 0.0f || distance >= mSettings.mSmoothingRadius)
							continue;

						const Vec3 direction = delta / distance;
						const float shared_pressure = (particle.mPressure + neighbor.mPressure) / (2.0f * max(neighbor.mDensity, cDensityEpsilon));
						pressure_force -= direction * (mSettings.mParticleMass * shared_pressure * KernelSpikyGradient(distance));
						viscosity_force += mSettings.mViscosity * mSettings.mParticleMass * (neighbor.mVelocity - particle.mVelocity) / max(neighbor.mDensity, cDensityEpsilon) * KernelViscosityLaplacian(distance);
					}
				}

		particle.mAcceleration = (pressure_force + viscosity_force) / max(particle.mDensity, cDensityEpsilon) + mSettings.mGravity;
	}
}

void ParticleFluidSystem::IntegrateRange(uint32 inStart, uint32 inEnd, float inDeltaTime)
{
	for (uint32 i = inStart; i < inEnd; ++i)
	{
		ParticleFluidParticle &particle = mParticles[i];
		particle.mVelocity += particle.mAcceleration * inDeltaTime;
		particle.mPosition += particle.mVelocity * inDeltaTime;

		for (int axis = 0; axis < 3; ++axis)
		{
			const float min_bound = mSettings.mBoundsMin[axis];
			const float max_bound = mSettings.mBoundsMax[axis];
			float position = particle.mPosition[axis];
			float velocity = particle.mVelocity[axis];

			if (position < min_bound)
			{
				position = min_bound;
				velocity = -velocity * mSettings.mBoundaryDamping;
			}
			else if (position > max_bound)
			{
				position = max_bound;
				velocity = -velocity * mSettings.mBoundaryDamping;
			}
		}
	}
}

template <class JobBody>
void ParticleFluidSystem::RunBatchedJobs(JobSystem &inJobSystem, const char *inName, ColorArg inColor, uint32 inItemCount, JobBody &&inJobBody) const
{
	if (inItemCount == 0)
		return;

	const uint32 max_concurrency = max<uint32>(uint32(inJobSystem.GetMaxConcurrency()), 1);
	const uint32 batch_size = max<uint32>(mSettings.mMinJobBatchSize, (inItemCount + max_concurrency - 1) / max_concurrency);
	const uint32 num_jobs = (inItemCount + batch_size - 1) / batch_size;

	if (num_jobs <= 1)
	{
		inJobBody(0, inItemCount);
		return;
	}

	JobSystem::Barrier *barrier = inJobSystem.CreateBarrier();
	for (uint32 job_index = 0; job_index < num_jobs; ++job_index)
	{
		const uint32 start = job_index * batch_size;
		const uint32 end = min(start + batch_size, inItemCount);
		JobSystem::JobHandle handle = inJobSystem.CreateJob(inName, inColor, [start, end, &inJobBody] { inJobBody(start, end); });
		barrier->AddJob(handle);
	}

	inJobSystem.WaitForJobs(barrier);
	inJobSystem.DestroyBarrier(barrier);
}

void ParticleFluidSystem::PublishRenderBuffer(JobSystem &inJobSystem)
{
	ParticleFluidRenderParticle *render_particles = mSharedBuffer->BeginWrite(uint32(mParticles.size()));
	RunBatchedJobs(inJobSystem, "ParticleFluidPublish", Color::sBlue, uint32(mParticles.size()), [this, render_particles](uint32 inStart, uint32 inEnd)
	{
		for (uint32 i = inStart; i < inEnd; ++i)
		{
			const ParticleFluidParticle &particle = mParticles[i];
			ParticleFluidRenderParticle &out_particle = render_particles[i];
			out_particle.mPosition[0] = particle.mPosition.GetX();
			out_particle.mPosition[1] = particle.mPosition.GetY();
			out_particle.mPosition[2] = particle.mPosition.GetZ();
			out_particle.mRadius = mSettings.mParticleRadius;
			out_particle.mVelocity[0] = particle.mVelocity.GetX();
			out_particle.mVelocity[1] = particle.mVelocity.GetY();
			out_particle.mVelocity[2] = particle.mVelocity.GetZ();
			out_particle.mDensity = particle.mDensity;
		}
	});
	mSharedBuffer->EndWrite(uint32(mParticles.size()));
}

void ParticleFluidSystem::SimulateCoupled(float inDeltaTime, JobSystem &inJobSystem, BodyInterface &ioBodyInterface, const BodyID *inBodies, uint32 inBodyCount, const ParticleFluidRigidBodyCouplingSettings &inCouplingSettings)
{
	Simulate(inDeltaTime, inJobSystem);
	ParticleFluidRigidBodyCoupling::sApply(*this, inBodies, inBodyCount, ioBodyInterface, inJobSystem, inCouplingSettings);
}

void ParticleFluidSystem::Simulate(float inDeltaTime, JobSystem &inJobSystem)
{
	if (mParticles.empty())
	{
		mSharedBuffer->Initialize(0);
		return;
	}

	EnsureCapacity();
	RebuildSpatialHash();

	const uint32 particle_count = uint32(mParticles.size());
	RunBatchedJobs(inJobSystem, "ParticleFluidDensity", Color::sRed, particle_count, [this](uint32 inStart, uint32 inEnd)
	{
		ComputeDensityPressureRange(inStart, inEnd);
	});

	RunBatchedJobs(inJobSystem, "ParticleFluidForce", Color::sGreen, particle_count, [this](uint32 inStart, uint32 inEnd)
	{
		ComputeForceRange(inStart, inEnd);
	});

	RunBatchedJobs(inJobSystem, "ParticleFluidIntegrate", Color::sYellow, particle_count, [this, inDeltaTime](uint32 inStart, uint32 inEnd)
	{
		IntegrateRange(inStart, inEnd, inDeltaTime);
	});

	PublishRenderBuffer(inJobSystem);
}

ACPH_NAMESPACE_END

