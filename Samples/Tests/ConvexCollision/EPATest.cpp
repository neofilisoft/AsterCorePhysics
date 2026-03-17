// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/ConvexCollision/EPATest.h>
#include <AsterCore/Geometry/Sphere.h>
#include <AsterCore/Geometry/AABox.h>
#include <AsterCore/Geometry/ConvexSupport.h>
#include <AsterCore/Geometry/EPAPenetrationDepth.h>
#include <Utils/DebugRendererSP.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(EPATest)
{
	ACPH_ADD_BASE_CLASS(EPATest, Test)
}

void EPATest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	AABox box(Vec3(1, 1, -2), Vec3(2, 2, 2));
	Sphere sphere(Vec3(4, 4, 0), sqrt(8.0f) + 0.01f);
	Mat44 matrix = Mat44::sRotationTranslation(Quat::sRotation(Vec3(1, 1, 1).Normalized(), 0.25f * ACPH_PI), Vec3(1, 2, 3));
	bool intersecting = CollideBoxSphere(matrix, box, sphere);
	ACPH_ASSERT(intersecting);
	(void)intersecting; // For when asserts are off
}

bool EPATest::CollideBoxSphere(Mat44Arg inMatrix, const AABox &inBox, const Sphere &inSphere) const
{
	// Draw the box and shere
	DrawBoxSP(mDebugRenderer, inMatrix, inBox, Color::sGrey);
	DrawSphereSP(mDebugRenderer, inMatrix * inSphere.GetCenter(), inSphere.GetRadius(), Color::sGrey);

	// Transform the box and sphere according to inMatrix
	TransformedConvexObject transformed_box(inMatrix, inBox);
	TransformedConvexObject transformed_sphere(inMatrix, inSphere);

	// Run the EPA algorithm
	EPAPenetrationDepth epa;
	Vec3 v1 = Vec3::sAxisX(), pa1, pb1;
	bool intersect1 = epa.GetPenetrationDepth(transformed_box, transformed_box, 0.0f, transformed_sphere, transformed_sphere, 0.0f, 1.0e-2f, FLT_EPSILON, v1, pa1, pb1);

	// Draw iterative solution
	if (intersect1)
	{
		DrawMarkerSP(mDebugRenderer, pa1, Color::sRed, 1.0f);
		DrawMarkerSP(mDebugRenderer, pb1, Color::sGreen, 1.0f);
		DrawArrowSP(mDebugRenderer, pb1 + Vec3(0, 1, 0), pb1 + Vec3(0, 1, 0) + v1, Color::sYellow, 0.1f);
	}

	// Calculate analytical solution
	Vec3 pa2 = inBox.GetClosestPoint(inSphere.GetCenter());
	Vec3 v2 = inSphere.GetCenter() - pa2;
	bool intersect2 = v2.LengthSq() <= Square(inSphere.GetRadius());

	ACPH_ASSERT(intersect1 == intersect2);
	if (intersect1 && intersect2)
	{
		Vec3 pb2 = inSphere.GetCenter() - inSphere.GetRadius() * v2.NormalizedOr(Vec3::sZero());

		// Transform analytical solution
		v2 = inMatrix.Multiply3x3(v2);
		pa2 = inMatrix * pa2;
		pb2 = inMatrix * pb2;

		// Draw analytical solution
		DrawMarkerSP(mDebugRenderer, pa2, Color::sOrange, 1.0f);
		DrawMarkerSP(mDebugRenderer, pb2, Color::sYellow, 1.0f);

		// Check angle between v1 and v2
		float dot = v1.Dot(v2);
		float len = v1.Length() * v2.Length();
		float angle = RadiansToDegrees(ACos(dot / len));
		ACPH_ASSERT(angle < 0.1f);
		Trace("Angle = %.9g", (double)angle);

		// Check delta between contact on A
		Vec3 dpa = pa2 - pa1;
		ACPH_ASSERT(dpa.IsNearZero(Square(8.0e-4f)));
		Trace("Delta A = %.9g", (double)dpa.Length());

		// Check delta between contact on B
		Vec3 dpb = pb2 - pb1;
		ACPH_ASSERT(dpb.IsNearZero(Square(8.0e-4f)));
		Trace("Delta B = %.9g", (double)dpb.Length());
	}

	return intersect1;
}
