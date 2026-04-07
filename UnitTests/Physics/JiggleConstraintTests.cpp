// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include <AsterCore/Core/JobSystemSingleThreaded.h>
#include <AsterCore/Physics/Constraints/JiggleConstraint.h>

TEST_SUITE("JiggleConstraintTests")
{
	TEST_CASE("GravityPullsConstraintAwayFromAnchor")
	{
		JiggleConstraintSettings settings;
		settings.mStiffness = 10.0f;
		settings.mDamping = 1.0f;
		settings.mGravityInfluence = 1.0f;
		settings.mInertiaInfluence = 0.0f;
		settings.mMaxDistance = 10.0f;

		JiggleConstraint jiggle(settings);
		jiggle.Reset(Vec3::sZero(), Vec3::sZero());
		jiggle.Update(1.0f / 30.0f, Vec3::sZero(), Vec3::sZero(), Vec3(0.0f, -9.81f, 0.0f));

		CHECK(jiggle.GetPosition().GetY() < 0.0f);
	}

	TEST_CASE("InertiaAddsSecondaryMotionOnSuddenRootMovement")
	{
		JiggleConstraintSettings settings;
		settings.mStiffness = 20.0f;
		settings.mDamping = 2.0f;
		settings.mGravityInfluence = 0.0f;
		settings.mInertiaInfluence = 1.0f;
		settings.mMaxDistance = 10.0f;

		JiggleConstraint jiggle(settings);
		jiggle.Reset(Vec3::sZero(), Vec3::sZero());
		jiggle.Update(1.0f / 60.0f, Vec3(1.0f, 0.0f, 0.0f), Vec3(60.0f, 0.0f, 0.0f), Vec3::sZero());

		CHECK(jiggle.GetVelocity().GetX() < 0.0f);
	}

	TEST_CASE("BatchUpdateUsesJobSystemAcrossMultipleConstraints")
	{
		JiggleConstraintSettings settings;
		settings.mGravityInfluence = 1.0f;
		settings.mInertiaInfluence = 0.5f;
		settings.mMaxDistance = 10.0f;

		JiggleConstraint constraint_a(settings);
		JiggleConstraint constraint_b(settings);
		constraint_a.Reset(Vec3::sZero(), Vec3::sZero());
		constraint_b.Reset(Vec3(0.0f, 1.0f, 0.0f), Vec3::sZero());

		JiggleConstraint *constraints[] = { &constraint_a, &constraint_b };
		const Vec3 root_positions[] = { Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.5f, 0.0f) };
		const Vec3 root_velocities[] = { Vec3(10.0f, 0.0f, 0.0f), Vec3(0.0f, 5.0f, 0.0f) };

		JobSystemSingleThreaded jobs(16);
		JiggleConstraint::sUpdateBatch(jobs, 1.0f / 60.0f, Vec3(0.0f, -9.81f, 0.0f), constraints, root_positions, root_velocities, 2, 1);

		CHECK(constraint_a.GetPosition().GetX() != 0.0f);
		CHECK(constraint_b.GetPosition().GetY() < 1.5f);
	}
}