// AsterCore Physics Library (https://github.com/jrouwe/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/PhysicsUpdateContext.h>

ACPH_NAMESPACE_BEGIN

/// @cond INTERNAL
PhysicsUpdateContext::PhysicsUpdateContext(TempAllocator &inTempAllocator) :
	mTempAllocator(&inTempAllocator),
	mSteps(inTempAllocator)
{
}

PhysicsUpdateContext::~PhysicsUpdateContext()
{
	ACPH_ASSERT(mBodyPairs == nullptr);
	ACPH_ASSERT(mActiveConstraints == nullptr);
}
/// @endcond

ACPH_NAMESPACE_END
