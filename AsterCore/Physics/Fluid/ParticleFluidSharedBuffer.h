// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/ByteBuffer.h>
#include <AsterCore/Core/Mutex.h>
#include <AsterCore/Core/Reference.h>

ACPH_NAMESPACE_BEGIN

struct ParticleFluidRenderParticle
{
	float							mPosition[3] = { 0.0f, 0.0f, 0.0f };
	float							mRadius = 0.0f;
	float							mVelocity[3] = { 0.0f, 0.0f, 0.0f };
	float							mDensity = 0.0f;
};

struct FelissRenderer3DParticleBufferView
{
	const ParticleFluidRenderParticle *mParticles = nullptr;
	uint32							mCount = 0;
	uint32							mStride = sizeof(ParticleFluidRenderParticle);
	uint32							mVersion = 0;
};

class ACPH_EXPORT ParticleFluidSharedBuffer final : public RefTarget<ParticleFluidSharedBuffer>
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	void							Initialize(uint32 inCapacity);
	uint32							GetCapacity() const;
	ParticleFluidRenderParticle *	BeginWrite(uint32 inCount);
	void							EndWrite(uint32 inCount);
	FelissRenderer3DParticleBufferView AcquireFelissRenderer3DView() const;

private:
	mutable SharedMutex				mMutex;
	ByteBuffer						mBuffers[2];
	uint32							mCapacity = 0;
	uint32							mCounts[2] = { 0, 0 };
	uint32							mFrontBufferIndex = 0;
	uint32							mBackBufferIndex = 1;
	uint32							mVersion = 0;
};

ACPH_NAMESPACE_END