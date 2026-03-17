// AsterCore Physics Library (https://github.com/jrouwe/joltphysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Color.h>
#include <AsterCore/Core/JobSystem.h>
#include <AsterCore/Core/Reference.h>
#include <AsterCore/Physics/Body/BodyID.h>
#include <AsterCore/Physics/Fluid/ParticleFluidSharedBuffer.h>

ACPH_NAMESPACE_BEGIN

class BodyInterface;
struct ParticleFluidRigidBodyCouplingSettings;

struct ParticleFluidSettings
{
	float							mParticleRadius = 0.075f;
	float							mParticleMass = 1.0f;
	float							mSmoothingRadius = 0.2f;
	float							mRestDensity = 1000.0f;
	float							mPressureStiffness = 14.0f;
	float							mViscosity = 0.12f;
	float							mBoundaryDamping = 0.45f;
	uint32							mHashBucketCount = 4096;
	uint32							mMinJobBatchSize = 128;
	Vec3							mGravity = Vec3(0.0f, -9.81f, 0.0f);
	Vec3							mBoundsMin = Vec3(-50.0f, -50.0f, -50.0f);
	Vec3							mBoundsMax = Vec3(50.0f, 50.0f, 50.0f);
};

struct ParticleFluidParticle
{
	Vec3							mPosition = Vec3::sZero();
	Vec3							mVelocity = Vec3::sZero();
	Vec3							mAcceleration = Vec3::sZero();
	float							mDensity = 0.0f;
	float							mPressure = 0.0f;
};

class ACPH_EXPORT ParticleFluidSystem final : public NonCopyable
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	ParticleFluidSystem();

	void							SetSettings(const ParticleFluidSettings &inSettings);
	const ParticleFluidSettings &	GetSettings() const						{ return mSettings; }

	void							Clear();
	void							SetParticles(const Array<ParticleFluidParticle> &inParticles);
	void							AddParticle(const ParticleFluidParticle &inParticle);

	uint32							GetParticleCount() const						{ return uint32(mParticles.size()); }
	const ParticleFluidParticle &	GetParticle(uint32 inIndex) const			{ return mParticles[inIndex]; }
	Ref<ParticleFluidSharedBuffer>	GetSharedBuffer() const					{ return mSharedBuffer; }

	void							Simulate(float inDeltaTime, JobSystem &inJobSystem);
	void							SimulateCoupled(float inDeltaTime, JobSystem &inJobSystem, BodyInterface &ioBodyInterface, const BodyID *inBodies, uint32 inBodyCount, const ParticleFluidRigidBodyCouplingSettings &inCouplingSettings);

private:
	struct GridCoordinate
	{
		int							mX = 0;
		int							mY = 0;
		int							mZ = 0;
	};

	void							EnsureCapacity();
	void							RebuildSpatialHash();
	void							ComputeDensityPressureRange(uint32 inStart, uint32 inEnd);
	void							ComputeForceRange(uint32 inStart, uint32 inEnd);
	void							IntegrateRange(uint32 inStart, uint32 inEnd, float inDeltaTime);
	void							PublishRenderBuffer(JobSystem &inJobSystem);

	GridCoordinate					GetGridCoordinate(Vec3Arg inPosition) const;
	uint32							HashCoordinate(const GridCoordinate &inCoordinate) const;

	template <class JobBody>
	void							RunBatchedJobs(JobSystem &inJobSystem, const char *inName, ColorArg inColor, uint32 inItemCount, JobBody &&inJobBody) const;

	float							KernelPoly6(float inDistanceSquared) const;
	float							KernelSpikyGradient(float inDistance) const;
	float							KernelViscosityLaplacian(float inDistance) const;

	ParticleFluidSettings			mSettings;
	Array<ParticleFluidParticle>	mParticles;
	Array<uint32>					mParticleHashes;
	Array<uint32>					mSortedParticleIndices;
	Array<uint32>					mCellOffsets;
	Array<uint32>					mCellCounts;
	Ref<ParticleFluidSharedBuffer>	mSharedBuffer;
	float							mSmoothingRadiusSquared = 0.0f;
	float							mKernelPoly6Coefficient = 0.0f;
	float							mKernelSpikyGradientCoefficient = 0.0f;
	float							mKernelViscosityLaplacianCoefficient = 0.0f;
};

ACPH_NAMESPACE_END
