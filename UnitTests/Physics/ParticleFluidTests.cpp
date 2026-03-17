// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <AsterCore/Core/JobSystemSingleThreaded.h>
#include <AsterCore/Physics/Fluid/ParticleFluidSystem.h>

TEST_SUITE("ParticleFluidTests")
{
	TEST_CASE("SharedBufferPublishesLatestSnapshot")
	{
		ParticleFluidSharedBuffer buffer;
		buffer.Initialize(2);

		ParticleFluidRenderParticle *particles = buffer.BeginWrite(2);
		particles[0].mPosition[0] = 1.0f;
		particles[1].mPosition[0] = 2.0f;
		buffer.EndWrite(2);

		FelissRenderer3DParticleBufferView view = buffer.AcquireFelissRenderer3DView();
		CHECK(view.mCount == 2);
		CHECK(view.mStride == sizeof(ParticleFluidRenderParticle));
		CHECK(view.mParticles[0].mPosition[0] == 1.0f);
		CHECK(view.mParticles[1].mPosition[0] == 2.0f);
	}

	TEST_CASE("FluidStepPublishesParticlesToRendererView")
	{
		ParticleFluidSystem fluid;
		ParticleFluidSettings settings;
		settings.mBoundsMin = Vec3(-5.0f, -5.0f, -5.0f);
		settings.mBoundsMax = Vec3(5.0f, 5.0f, 5.0f);
		settings.mHashBucketCount = 64;
		settings.mMinJobBatchSize = 1;
		fluid.SetSettings(settings);

		Array<ParticleFluidParticle> particles;
		particles.push_back({ Vec3(0.0f, 0.0f, 0.0f), Vec3::sZero(), Vec3::sZero(), 0.0f, 0.0f });
		particles.push_back({ Vec3(0.1f, 0.0f, 0.0f), Vec3::sZero(), Vec3::sZero(), 0.0f, 0.0f });
		fluid.SetParticles(particles);

		JobSystemSingleThreaded jobs(16);
		fluid.Simulate(1.0f / 60.0f, jobs);

		FelissRenderer3DParticleBufferView view = fluid.GetSharedBuffer()->AcquireFelissRenderer3DView();
		CHECK(view.mCount == 2);
		CHECK(view.mParticles[0].mRadius == settings.mParticleRadius);
		CHECK(view.mParticles[0].mDensity > 0.0f);
	}
}