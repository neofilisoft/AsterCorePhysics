// AsterCore Physics Library (https://github.com/jrouwe/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Collision/BroadPhase/BroadPhase.h>

ACPH_NAMESPACE_BEGIN

void BroadPhase::Init(BodyManager *inBodyManager, const BroadPhaseLayerInterface &inLayerInterface)
{
	mBodyManager = inBodyManager;
}

ACPH_NAMESPACE_END
