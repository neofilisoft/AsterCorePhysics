// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/SoftBody/SoftBodySensorTest.h>
#include <AsterCore/Physics/Body/BodyCreationSettings.h>
#include <AsterCore/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <AsterCore/Physics/Collision/Shape/SphereShape.h>
#include <AsterCore/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <AsterCore/Physics/SoftBody/SoftBodyManifold.h>
#include <Utils/SoftBodyCreator.h>
#include <Renderer/DebugRendererImp.h>
#include <Layers.h>

ACPH_IMPLEMENT_RTTI_VIRTUAL(SoftBodySensorTest)
{
	ACPH_ADD_BASE_CLASS(SoftBodySensorTest, Test)
}

void SoftBodySensorTest::Initialize()
{
	// Install contact listener for soft bodies
	mPhysicsSystem->SetSoftBodyContactListener(this);

	// Floor
	CreateFloor();

	// Create cloth that's fixated at the corners
	SoftBodyCreationSettings cloth(SoftBodyCreator::CreateClothWithFixatedCorners(), RVec3(0, 10.0f, 0), Quat::sIdentity(), Layers::MOVING);
	mBodyInterface->CreateAndAddSoftBody(cloth, EActivation::Activate);

	// Some sensors to detect the cloth
	BodyCreationSettings cylinder_sensor(new TaperedCylinderShapeSettings(4.0f, 1.0f, 2.0f), RVec3(0, 6, 0), Quat::sRotation(Vec3::sAxisX(), 0.5f * ACPH_PI), EMotionType::Static, Layers::SENSOR);
	cylinder_sensor.mIsSensor = true;
	mBodyInterface->CreateAndAddBody(cylinder_sensor, EActivation::DontActivate);

	BodyCreationSettings sphere_sensor(new SphereShape(4.0f), RVec3(4, 5, 0), Quat::sIdentity(), EMotionType::Static, Layers::SENSOR);
	sphere_sensor.mIsSensor = true;
	mBodyInterface->CreateAndAddBody(sphere_sensor, EActivation::DontActivate);

	// Sphere that falls on the cloth to check that we don't ignore this collision
	BodyCreationSettings bcs(new SphereShape(1.0f), RVec3(0, 15, 0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
	bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
	bcs.mMassPropertiesOverride.mMass = 500.0f;
	mBodyInterface->CreateAndAddBody(bcs, EActivation::Activate);
}

void SoftBodySensorTest::OnSoftBodyContactAdded(const Body &inSoftBody, const SoftBodyManifold &inManifold)
{
	// Draw the vertices that are in contact
	RMat44 com = inSoftBody.GetCenterOfMassTransform();
	for (const SoftBodyVertex &v : inManifold.GetVertices())
		if (inManifold.HasContact(v))
			DebugRenderer::sInstance->DrawMarker(com * v.mPosition, Color::sGreen, 0.1f);

	// Draw the sensors that are in contact with the soft body
	for (uint i = 0; i < inManifold.GetNumSensorContacts(); ++i)
	{
		BodyID sensor_id = inManifold.GetSensorContactBodyID(i);
		BodyLockRead lock(mPhysicsSystem->GetBodyLockInterfaceNoLock(), sensor_id); // Can't lock in a callback
		if (lock.SucceededAndIsInBroadPhase())
		{
			AABox bounds = lock.GetBody().GetWorldSpaceBounds();
			DebugRenderer::sInstance->DrawWireBox(bounds, Color::sGreen);
		}
	}
}
