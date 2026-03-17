#include <Integrations/Feliss/FelissParticleFluidBridge.h>

namespace AsterIntegrations::Feliss
{
	ParticleFrame FelissParticleFluidBridge::Capture(const ACPH::ParticleFluidSharedBuffer &inSharedBuffer)
	{
		const ACPH::FelissRenderer3DParticleBufferView view = inSharedBuffer.AcquireFelissRenderer3DView();
		ParticleFrame frame;
		frame.mParticles = view.mParticles;
		frame.mCount = view.mCount;
		frame.mStride = view.mStride;
		frame.mVersion = view.mVersion;
		return frame;
	}
}
