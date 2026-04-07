// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/ScaledShapes/ScaledBoxShapeTest.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Collision/Shape/ScaledShape.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <Layers.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(ScaledBoxShapeTest)
{
	ACPH_ADD_BASE_CLASS(ScaledBoxShapeTest, Test)
}

void ScaledBoxShapeTest::Initialize()
{
	// Floor
	CreateFloor();

	// Create box
	RefConst<BoxShape> box_shape = new BoxShape(Vec3(3, 2, 1.5f));

	// Original shape
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(box_shape, RVec3(-30, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Uniformly scaled shape < 1
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(box_shape, Vec3::sReplicate(0.25f)), RVec3(-20, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Uniformly scaled shape > 1
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(box_shape, Vec3::sReplicate(2.0f)), RVec3(-10, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Non-uniform scaled shape
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(box_shape, Vec3(0.25f, 0.5f, 1.5f)), RVec3(0, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Flipped in 2 axis
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(box_shape, Vec3(-0.25f, 0.5f, -1.5f)), RVec3(10, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);

	// Inside out
	mBodyInterface->CreateAndAddBody(BodyCreationSettings(new ScaledShape(box_shape, Vec3(-0.25f, 0.5f, 1.5f)), RVec3(20, 10, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);
}
