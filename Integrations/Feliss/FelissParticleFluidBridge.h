#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Physics/Fluid/ParticleFluidSharedBuffer.h>

namespace AsterIntegrations::Feliss
{
	struct ParticleFrame
	{
		const ACPH::ParticleFluidRenderParticle *mParticles = nullptr;
		ACPH::uint32 mCount = 0;
		ACPH::uint32 mStride = sizeof(ACPH::ParticleFluidRenderParticle);
		ACPH::uint32 mVersion = 0;
	};

	class FelissParticleFluidBridge final
	{
	public:
		static ParticleFrame Capture(const ACPH::ParticleFluidSharedBuffer &inSharedBuffer);
	};
}
