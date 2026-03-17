// AsterCore Physics Library
// SPDX-FileCopyrightText: 2026 OpenAI
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Core/Mutex.h>
#include <AsterCore/Physics/GPU/PhysicsComputeBackend.h>

ACPH_NAMESPACE_BEGIN

class ACPH_EXPORT PhysicsComputeScheduler final : public NonCopyable
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	void						SetBackend(PhysicsComputeBackend *inBackend);
	PhysicsComputeBackend *		GetBackend() const;

	bool						TryDispatchBroadPhase(const PhysicsComputeBroadPhaseRequest &inRequest, PhysicsComputeBroadPhaseResult &ioResult) const;
	bool						TryDispatchNarrowPhase(const PhysicsComputeNarrowPhaseRequest &inRequest, PhysicsComputeNarrowPhaseResult &ioResult) const;

private:
	mutable SharedMutex			mMutex;
	PhysicsComputeBackend *		mBackend = nullptr;
};

ACPH_NAMESPACE_END