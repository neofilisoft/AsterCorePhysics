// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/General/WallTest.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <Layers.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(WallTest)
{
	ACPH_ADD_BASE_CLASS(WallTest, Test)
}

void WallTest::Initialize()
{
	// Floor
	CreateFloor();

	RefConst<Shape> box_shape = new BoxShape(Vec3(1.0f, 1.0f, 1.0f));

	// Wall
	for (int i = 0; i < 10; ++i)
		for (int j = i / 2; j < 50 - (i + 1) / 2; ++j)
		{
			RVec3 position(-50 + j * 2.0f + (i & 1? 1.0f : 0.0f), 1.0f + i * 3.0f, 0);
			mBodyInterface->CreateAndAddBody(BodyCreationSettings(box_shape, position, Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING), EActivation::Activate);
		}
}
