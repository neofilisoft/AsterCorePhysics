// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Physics/Constraints/TwoBodyConstraint.h>
#include <AsterCore/Physics/IslandBuilder.h>
#include <AsterCore/Physics/LargeIslandSplitter.h>
#include <AsterCore/Physics/Body/BodyManager.h>

#ifdef ACPH_DEBUG_RENDERER
	#include <AsterCore/Renderer/DebugRenderer.h>
#endif // ACPH_DEBUG_RENDERER

ACPH_NAMESPACE_BEGIN

ACPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(TwoBodyConstraintSettings)
{
	ACPH_ADD_BASE_CLASS(TwoBodyConstraintSettings, ConstraintSettings)
}

void TwoBodyConstraint::BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager)
{
	// Activate bodies
	BodyID body_ids[2];
	int num_bodies = 0;
	if (mBody1->IsDynamic() && !mBody1->IsActive())
		body_ids[num_bodies++] = mBody1->GetID();
	if (mBody2->IsDynamic() && !mBody2->IsActive())
		body_ids[num_bodies++] = mBody2->GetID();
	if (num_bodies > 0)
		inBodyManager.ActivateBodies(body_ids, num_bodies);

	// Link the bodies into the same island
	ioBuilder.LinkConstraint(inConstraintIndex, mBody1->GetIndexInActiveBodiesInternal(), mBody2->GetIndexInActiveBodiesInternal());
}

uint TwoBodyConstraint::BuildIslandSplits(LargeIslandSplitter &ioSplitter) const
{
	return ioSplitter.AssignSplit(mBody1, mBody2);
}

#ifdef ACPH_DEBUG_RENDERER

void TwoBodyConstraint::DrawConstraintReferenceFrame(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform() * GetConstraintToBody1Matrix();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform() * GetConstraintToBody2Matrix();
	inRenderer->DrawCoordinateSystem(transform1, 1.1f * mDrawConstraintSize);
	inRenderer->DrawCoordinateSystem(transform2, mDrawConstraintSize);
}

#endif // ACPH_DEBUG_RENDERER

ACPH_NAMESPACE_END
