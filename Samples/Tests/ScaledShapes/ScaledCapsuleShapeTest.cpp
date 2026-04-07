// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/ScaledShapes/ScaledCapsuleShapeTest.h>
#include <AsterCore/Physics/Collision/Shape/CapsuleShape.h>
#include <AsterCore/Physics/Collision/Shape/ScaledShape.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <Layers.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(ScaledCapsuleShapeTest)
{
	ACPH_ADD_BASE_CLASS(ScaledCapsuleShapeTest, Test)
}

void ScaledCapsuleShapeTest::Initialize()
{
	// Floor
	CreateFloor();

	// Create capsule
	RefConst<CapsuleShape> capsule_shape = new CapsuleShape(2.0f, 0.5f);

	// Original shape
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(capsule_shape, RVec3(-20, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Uniformly scaled shape < 1
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(capsule_shape, Vec3::sReplicate(0.25f)), RVec3(-10, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Uniformly scaled shape > 1
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(capsule_shape, Vec3::sReplicate(2.0f)), RVec3(0, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Flipped in 2 axis
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(capsule_shape, Vec3(-1.5f, -1.5f, 1.5f)), RVec3(10, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Inside out
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(capsule_shape, Vec3(-0.75f, 0.75f, 0.75f)), RVec3(20, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);
}
