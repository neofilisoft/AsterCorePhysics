// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Geometry/AABox.h>
#include <AsterCore/Physics/Body/BodyID.h>

ACPH_NAMESPACE_BEGIN

struct PhysicsBodyPair
{
	BodyID						mBodyA;
	BodyID						mBodyB;
};

enum class EPhysicsComputeStage : uint32
{
	BroadPhase = 1 << 0,
	NarrowPhase = 1 << 1,
	SoftBody = 1 << 2,
	Fluid = 1 << 3,
	SignedDistanceField = 1 << 4,
};

struct PhysicsComputeBufferView
{
	const void *				mData = nullptr;
	uint32						mStride = 0;
	uint32						mCount = 0;
};

struct PhysicsComputeBroadPhaseRequest
{
	AABox						mWorldBounds;
	PhysicsComputeBufferView	mBounds;
	PhysicsComputeBufferView	mBodyIDs;
};

struct PhysicsComputeBroadPhaseResult
{
	PhysicsBodyPair *			mPairs = nullptr;
	uint32						mPairCapacity = 0;
	uint32						mPairCount = 0;
	bool						mUsedAccelerator = false;
};

struct PhysicsComputeNarrowPhaseRequest
{
	PhysicsComputeBufferView	mCandidatePairs;
	PhysicsComputeBufferView	mTransforms;
	PhysicsComputeBufferView	mShapeHandles;
};

struct PhysicsComputeNarrowPhaseResult
{
	uint32						mContactCount = 0;
	bool						mUsedAccelerator = false;
};

class ACPH_EXPORT PhysicsComputeBackend
{
public:
	virtual						~PhysicsComputeBackend() = default;

	virtual uint32				GetSupportedStages() const = 0;
	bool						Supports(EPhysicsComputeStage inStage) const			{ return (GetSupportedStages() & uint32(inStage)) != 0; }

	virtual bool				DispatchBroadPhase(const PhysicsComputeBroadPhaseRequest &inRequest, PhysicsComputeBroadPhaseResult &ioResult) = 0;
	virtual bool				DispatchNarrowPhase(const PhysicsComputeNarrowPhaseRequest &inRequest, PhysicsComputeNarrowPhaseResult &ioResult) = 0;
};

ACPH_NAMESPACE_END