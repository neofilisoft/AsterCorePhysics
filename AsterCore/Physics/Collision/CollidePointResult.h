// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Physics/Body/BodyID.h>
#include <AsterCore/Physics/Collision/Shape/SubShapeID.h>

ACPH_NAMESPACE_BEGIN

/// Structure that holds the result of colliding a point against a shape
class CollidePointResult
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	/// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. For point queries there is no sensible return value.
	inline float	GetEarlyOutFraction() const			{ return 0.0f; }

	BodyID			mBodyID;							///< Body that was hit
	SubShapeID		mSubShapeID2;						///< Sub shape ID of shape that we collided against
};

ACPH_NAMESPACE_END
