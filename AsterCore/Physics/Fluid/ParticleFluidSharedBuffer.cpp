// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Fluid/ParticleFluidSharedBuffer.h>

ACPH_NAMESPACE_BEGIN

void ParticleFluidSharedBuffer::Initialize(uint32 inCapacity)
{
	unique_lock lock(mMutex);

	mCapacity = inCapacity;
	mCounts[0] = mCounts[1] = 0;
	mFrontBufferIndex = 0;
	mBackBufferIndex = 1;
	mVersion = 0;

	const size_t buffer_size = size_t(inCapacity) * sizeof(ParticleFluidRenderParticle);
	for (ByteBuffer &buffer : mBuffers)
		buffer.resize(buffer_size, 0);
}

uint32 ParticleFluidSharedBuffer::GetCapacity() const
{
	shared_lock lock(mMutex);
	return mCapacity;
}

ParticleFluidRenderParticle *ParticleFluidSharedBuffer::BeginWrite(uint32 inCount)
{
	unique_lock lock(mMutex);

	ACPH_ASSERT(inCount <= mCapacity);
	return reinterpret_cast<ParticleFluidRenderParticle *>(mBuffers[mBackBufferIndex].data());
}

void ParticleFluidSharedBuffer::EndWrite(uint32 inCount)
{
	unique_lock lock(mMutex);

	ACPH_ASSERT(inCount <= mCapacity);
	mCounts[mBackBufferIndex] = inCount;
	std::swap(mFrontBufferIndex, mBackBufferIndex);
	++mVersion;
}

FelissRenderer3DParticleBufferView ParticleFluidSharedBuffer::AcquireFelissRenderer3DView() const
{
	shared_lock lock(mMutex);

	FelissRenderer3DParticleBufferView view;
	view.mParticles = reinterpret_cast<const ParticleFluidRenderParticle *>(mBuffers[mFrontBufferIndex].data());
	view.mCount = mCounts[mFrontBufferIndex];
	view.mVersion = mVersion;
	return view;
}

ACPH_NAMESPACE_END
