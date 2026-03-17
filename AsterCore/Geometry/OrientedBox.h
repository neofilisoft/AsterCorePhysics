// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Geometry/Triangle.h>
#include <AsterCore/Geometry/IndexedTriangle.h>
#include <AsterCore/Geometry/AABox.h>
#include <AsterCore/Math/Mat44.h>

ACPH_NAMESPACE_BEGIN

class AABox;

/// Oriented box
class ACPH_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] OrientedBox
{
public:
	ACPH_OVERRIDE_NEW_DELETE

	/// Constructor
					OrientedBox() = default;
					OrientedBox(Mat44Arg inOrientation, Vec3Arg inHalfExtents)			: mOrientation(inOrientation), mHalfExtents(inHalfExtents) { }

	/// Construct from axis aligned box and transform. Only works for rotation/translation matrix (no scaling / shearing).
					OrientedBox(Mat44Arg inOrientation, const AABox &inBox)				: OrientedBox(inOrientation.PreTranslated(inBox.GetCenter()), inBox.GetExtent()) { }

	/// Test if oriented box overlaps with axis aligned box each other
	bool			Overlaps(const AABox &inBox, float inEpsilon = 1.0e-6f) const;

	/// Test if two oriented boxes overlap each other
	bool			Overlaps(const OrientedBox &inBox, float inEpsilon = 1.0e-6f) const;

	Mat44			mOrientation;														///< Transform that positions and rotates the local space axis aligned box into world space
	Vec3			mHalfExtents;														///< Half extents (half the size of the edge) of the local space axis aligned box
};

ACPH_NAMESPACE_END
