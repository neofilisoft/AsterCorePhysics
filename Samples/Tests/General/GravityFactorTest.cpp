// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/General/GravityFactorTest.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <Layers.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(GravityFactorTest)
{
	ACPH_ADD_BASE_CLASS(GravityFactorTest, Test)
}

void GravityFactorTest::Initialize()
{
	// Floor
	CreateFloor();

	RefConst<Shape> box = new BoxShape(Vec3(2.0f, 2.0f, 2.0f));

	// Bodies with increasing gravity fraction
	for (int i = 0; i <= 10; ++i)
	{
		Body &body = *mBodyInterface->CreateBody(BodyCreationSettings(box, RVec3(-50.0f + i * 10.0f, 25.0f, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING));
		body.GetMotionProperties()->SetGravityFactor(0.1f * i);
		mBodyInterface->AddBody(body.GetID(), EActivation::Activate);
		SetBodyLabel(body.GetID(), StringFormat("Gravity: %.1f", double(body.GetMotionProperties()->GetGravityFactor())));
	}
}
