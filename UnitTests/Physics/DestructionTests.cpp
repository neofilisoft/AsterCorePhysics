// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"
#include <AsterCore/Geometry/AABox.h>
#include <AsterCore/Physics/Body/BodyLock.h>
#include <AsterCore/Physics/Collision/Shape/BoxShape.h>
#include <AsterCore/Physics/Destruction/DestructionSystem.h>
#include <AsterCore/Physics/Destruction/VoronoiFracturePlanner.h>
#include <AsterCore/Physics/Snapshot/PhysicsSnapshot.h>
#include <AsterCore/Physics/StateRecorderImpl.h>

TEST_SUITE("DestructionTests")
{
	static PreFracturedChunk sCreateBoxChunk(Vec3Arg inHalfExtent, Vec3Arg inLocalPosition, float inMass, bool inAnchored = false)
	{
		PreFracturedChunk chunk;
		chunk.mShape = new BoxShape(inHalfExtent);
		chunk.mLocalPosition = inLocalPosition;
		chunk.mMass = inMass;
		chunk.mAnchored = inAnchored;
		return chunk;
	}

	static BreakableBodySettings sCreateTwoChunkSettings(bool inSpawnStatic)
	{
		BreakableBodySettings settings;
		settings.mSpawnSupportedChunksAsStatic = inSpawnStatic;
		settings.mFractureDamageThreshold = 50.0f;
		settings.mFractureImpulseThreshold = 10.0f;
		settings.mDamageToImpulseScale = 0.5f;

		BodyCreationSettings chunk_template;
		chunk_template.mObjectLayer = Layers::MOVING;
		chunk_template.mMotionType = EMotionType::Dynamic;
		settings.mPreFracturedMesh.SetChunkTemplate(chunk_template);
		settings.mPreFracturedMesh.SetChunkTemplateOverrides(EChunkTemplateOverride::ObjectLayer);
		settings.mPreFracturedMesh.AddChunk(sCreateBoxChunk(Vec3(0.5f, 1.0f, 1.0f), Vec3(-0.5f, 0.0f, 0.0f), 18.0f));
		settings.mPreFracturedMesh.AddChunk(sCreateBoxChunk(Vec3(0.5f, 1.0f, 1.0f), Vec3(0.5f, 0.0f, 0.0f), 18.0f));
		return settings;
	}

	static BreakableBodySettings sCreateTowerSettings()
	{
		BreakableBodySettings settings;
		settings.mSpawnSupportedChunksAsStatic = true;
		settings.mFractureDamageThreshold = 20.0f;
		settings.mFractureImpulseThreshold = 5.0f;
		settings.mDamageToImpulseScale = 0.25f;

		BodyCreationSettings chunk_template;
		chunk_template.mObjectLayer = Layers::MOVING;
		chunk_template.mMotionType = EMotionType::Dynamic;
		settings.mPreFracturedMesh.SetChunkTemplate(chunk_template);
		settings.mPreFracturedMesh.SetChunkTemplateOverrides(EChunkTemplateOverride::ObjectLayer);
		settings.mPreFracturedMesh.AddChunk(sCreateBoxChunk(Vec3(0.5f, 0.5f, 0.5f), Vec3(0.0f, 0.0f, 0.0f), 20.0f, true));
		settings.mPreFracturedMesh.AddChunk(sCreateBoxChunk(Vec3(0.5f, 0.5f, 0.5f), Vec3(0.0f, 1.0f, 0.0f), 15.0f));
		settings.mPreFracturedMesh.AddChunk(sCreateBoxChunk(Vec3(0.5f, 0.5f, 0.5f), Vec3(0.0f, 2.0f, 0.0f), 10.0f));
		settings.mConnections.push_back({ 0, 1, 1.0f });
		settings.mConnections.push_back({ 1, 2, 1.0f });
		return settings;
	}

	static BreakableBodySettings sCreateVoronoiCollapseSettings()
	{
		BreakableBodySettings settings;
		settings.mSpawnSupportedChunksAsStatic = false;
		settings.mFractureDamageThreshold = 25.0f;
		settings.mFractureImpulseThreshold = 5.0f;
		settings.mDamageToImpulseScale = 0.35f;

		BodyCreationSettings chunk_template;
		chunk_template.mObjectLayer = Layers::MOVING;
		chunk_template.mMotionType = EMotionType::Dynamic;
		settings.mPreFracturedMesh.SetChunkTemplate(chunk_template);
		settings.mPreFracturedMesh.SetChunkTemplateOverrides(EChunkTemplateOverride::ObjectLayer);

		VoronoiFracturePlanner planner;
		VoronoiFractureSettings fracture_settings;
		fracture_settings.mLocalBounds = AABox(Vec3(-1.5f, -3.0f, -1.5f), Vec3(1.5f, 3.0f, 1.5f));
		fracture_settings.mCellCount = 18;
		fracture_settings.mSamplesX = 8;
		fracture_settings.mSamplesY = 14;
		fracture_settings.mSamplesZ = 8;
		fracture_settings.mRandomSeed = 20260321;
		fracture_settings.mDensity = 22.0f;
		fracture_settings.mConvexRadius = 0.015f;
		planner.AppendCellsAsConvexChunks(fracture_settings, chunk_template, settings.mPreFracturedMesh, false);

		return settings;
	}

	static void sCheckChunkDamping(PhysicsTestContext &ioContext, const BodyID &inBodyID, float inLinearDamping, float inAngularDamping)
	{
		BodyLockRead lock(ioContext.GetSystem()->GetBodyLockInterface(), inBodyID);
		REQUIRE(lock.Succeeded());
		const MotionProperties *motion_properties = lock.GetBody().GetMotionPropertiesUnchecked();
		REQUIRE(motion_properties != nullptr);
		CHECK_APPROX_EQUAL(motion_properties->GetLinearDamping(), inLinearDamping, 1.0e-6f);
		CHECK_APPROX_EQUAL(motion_properties->GetAngularDamping(), inAngularDamping, 1.0e-6f);
	}

	TEST_CASE("PreFracturedMeshInheritsIntactBodySettingsByDefault")
	{
		PreFracturedMesh mesh;
		BodyCreationSettings chunk_template;
		chunk_template.mObjectLayer = Layers::MOVING;
		mesh.SetChunkTemplate(chunk_template);
		mesh.SetChunkTemplateOverrides(EChunkTemplateOverride::ObjectLayer);
		mesh.AddChunk(sCreateBoxChunk(Vec3(1.0f, 2.0f, 3.0f), Vec3(1.0f, 0.0f, 0.0f), 12.0f));

		BodyCreationSettings intact_template(new BoxShape(Vec3(2.0f, 2.0f, 2.0f)), RVec3(10.0f, 5.0f, -2.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::NON_MOVING);
		intact_template.mMotionQuality = EMotionQuality::LinearCast;
		intact_template.mFriction = 0.73f;
		intact_template.mRestitution = 0.19f;
		intact_template.mLinearDamping = 0.13f;
		intact_template.mAngularDamping = 0.29f;
		intact_template.mGravityFactor = 0.67f;
		intact_template.mCollisionGroup = CollisionGroup(nullptr, 7, 11);

		BodyCreationSettings chunk_settings = mesh.CreateChunkCreationSettings(0, intact_template, RVec3(10.0f, 5.0f, -2.0f), Quat::sIdentity(), EMotionType::Dynamic);
		CHECK(chunk_settings.GetShape() != nullptr);
		CHECK_APPROX_EQUAL(chunk_settings.mPosition, RVec3(11.0f, 5.0f, -2.0f), 1.0e-6f);
		CHECK(chunk_settings.mObjectLayer == Layers::MOVING);
		CHECK(chunk_settings.mMotionType == EMotionType::Dynamic);
		CHECK(chunk_settings.mMotionQuality == EMotionQuality::LinearCast);
		CHECK_APPROX_EQUAL(chunk_settings.mFriction, 0.73f, 1.0e-6f);
		CHECK_APPROX_EQUAL(chunk_settings.mRestitution, 0.19f, 1.0e-6f);
		CHECK_APPROX_EQUAL(chunk_settings.mLinearDamping, 0.13f, 1.0e-6f);
		CHECK_APPROX_EQUAL(chunk_settings.mAngularDamping, 0.29f, 1.0e-6f);
		CHECK_APPROX_EQUAL(chunk_settings.mGravityFactor, 0.67f, 1.0e-6f);
		CHECK(chunk_settings.mCollisionGroup.GetGroupID() == 7);
		CHECK(chunk_settings.mCollisionGroup.GetSubGroupID() == 11);
		CHECK(chunk_settings.mMassPropertiesOverride.mMass == 12.0f);
	}

	TEST_CASE("DestructionSystemFracturesIntoRuntimeChunkBodiesAndPreservesMaterialState")
	{
		PhysicsTestContext context;
		BodyCreationSettings intact_settings(new BoxShape(Vec3(1.0f, 1.0f, 1.0f)), RVec3(0.0f, 5.0f, 0.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		intact_settings.mFriction = 0.81f;
		intact_settings.mRestitution = 0.14f;
		intact_settings.mLinearDamping = 0.18f;
		intact_settings.mAngularDamping = 0.31f;
		intact_settings.mGravityFactor = 0.42f;
		intact_settings.mMotionQuality = EMotionQuality::LinearCast;
		intact_settings.mCollisionGroup = CollisionGroup(nullptr, 5, 9);
		BodyID intact_body = context.GetBodyInterface().CreateAndAddBody(intact_settings, EActivation::Activate);

		DestructionSystem destruction;
		CHECK(destruction.RegisterBreakableBody(intact_body, sCreateTwoChunkSettings(false)));

		DestructionDamage damage;
		damage.mPosition = RVec3(0.0f, 5.0f, 0.0f);
		damage.mImpulse = Vec3(12.0f, 0.0f, 0.0f);
		damage.mDamage = 100.0f;
		CHECK(destruction.ApplyDamage(context.GetBodyInterface(), intact_body, damage));
		CHECK(destruction.ProcessPendingFractures(context.GetBodyInterface()));
		CHECK(destruction.IsFractured(intact_body));

		Array<BodyID> chunk_bodies = destruction.GetChunkBodies(intact_body);
		CHECK(chunk_bodies.size() == 2);
		for (const BodyID &chunk_body : chunk_bodies)
		{
			CHECK(context.GetBodyInterface().IsAdded(chunk_body));
			CHECK(context.GetBodyInterface().GetMotionType(chunk_body) == EMotionType::Dynamic);
			CHECK(context.GetBodyInterface().GetMotionQuality(chunk_body) == EMotionQuality::LinearCast);
			CHECK_APPROX_EQUAL(context.GetBodyInterface().GetFriction(chunk_body), 0.81f, 1.0e-6f);
			CHECK_APPROX_EQUAL(context.GetBodyInterface().GetRestitution(chunk_body), 0.14f, 1.0e-6f);
			CHECK_APPROX_EQUAL(context.GetBodyInterface().GetGravityFactor(chunk_body), 0.42f, 1.0e-6f);
			CHECK(context.GetBodyInterface().GetCollisionGroup(chunk_body).GetGroupID() == 5);
			CHECK(context.GetBodyInterface().GetCollisionGroup(chunk_body).GetSubGroupID() == 9);
			sCheckChunkDamping(context, chunk_body, 0.18f, 0.31f);
		}

		Array<DestructionEvent> events = destruction.ConsumeEvents();
		CHECK(events.size() >= 3);
	}

	TEST_CASE("AnchoredChunksStayStaticUntilSupportGraphBreaks")
	{
		PhysicsTestContext context;
		BodyCreationSettings intact_settings(new BoxShape(Vec3(0.75f, 1.5f, 0.75f)), RVec3(0.0f, 3.0f, 0.0f), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
		BodyID intact_body = context.GetBodyInterface().CreateAndAddBody(intact_settings, EActivation::DontActivate);

		DestructionSystem destruction;
		CHECK(destruction.RegisterBreakableBody(intact_body, sCreateTowerSettings()));

		DestructionDamage fracture_damage;
		fracture_damage.mPosition = RVec3(0.0f, 5.0f, 0.0f);
		fracture_damage.mDamage = 25.0f;
		fracture_damage.mRadius = 0.75f;
		fracture_damage.mForceFracture = true;
		CHECK(destruction.ApplyDamage(context.GetBodyInterface(), intact_body, fracture_damage));
		CHECK(destruction.ProcessPendingFractures(context.GetBodyInterface()));

		Array<BodyID> chunk_bodies = destruction.GetChunkBodies(intact_body);
		CHECK(chunk_bodies.size() == 3);
		CHECK(context.GetBodyInterface().GetMotionType(chunk_bodies[0]) == EMotionType::Static);
		CHECK(context.GetBodyInterface().GetMotionType(chunk_bodies[1]) == EMotionType::Static);
		CHECK(context.GetBodyInterface().GetMotionType(chunk_bodies[2]) == EMotionType::Dynamic);

		CHECK(destruction.BreakConnection(context.GetBodyInterface(), intact_body, 0, Vec3(0.0f, 4.0f, 0.0f)));
		CHECK(context.GetBodyInterface().GetMotionType(chunk_bodies[1]) == EMotionType::Dynamic);
	}

	TEST_CASE("DestructionSnapshotRestoresIntactAndFracturedTopologies")
	{
		PhysicsTestContext context;
		context.CreateFloor();

		BodyCreationSettings intact_settings(new BoxShape(Vec3(1.0f, 1.0f, 1.0f)), RVec3(0.0f, 6.0f, 0.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		BodyID intact_body = context.GetBodyInterface().CreateAndAddBody(intact_settings, EActivation::Activate);

		DestructionSystem destruction;
		CHECK(destruction.RegisterBreakableBody(intact_body, sCreateTwoChunkSettings(false)));

		PhysicsSnapshot snapshot;
		snapshot.AddParticipant(&destruction);

		StateRecorderImpl initial_state;
		snapshot.Save(initial_state, *context.GetSystem());
		const std::string initial_bytes = initial_state.GetData();

		DestructionDamage damage;
		damage.mPosition = RVec3(0.0f, 6.0f, 0.0f);
		damage.mImpulse = Vec3(14.0f, 2.0f, 0.0f);
		damage.mDamage = 125.0f;
		damage.mForceFracture = true;
		CHECK(destruction.ApplyDamage(context.GetBodyInterface(), intact_body, damage));
		CHECK(destruction.ProcessPendingFractures(context.GetBodyInterface()));
		context.Simulate(0.4f);

		StateRecorderImpl fractured_state;
		snapshot.Save(fractured_state, *context.GetSystem());
		const std::string fractured_bytes = fractured_state.GetData();
		CHECK(destruction.IsFractured(intact_body));
		CHECK(destruction.GetChunkBodies(intact_body).size() == 2);

		initial_state.Rewind();
		CHECK(snapshot.Restore(initial_state, *context.GetSystem()));
		CHECK(!destruction.IsFractured(intact_body));
		CHECK(destruction.GetChunkBodies(intact_body).empty());
		CHECK(context.GetBodyInterface().HasBody(intact_body));

		StateRecorderImpl verify_initial;
		snapshot.Save(verify_initial, *context.GetSystem());
		CHECK(verify_initial.GetData() == initial_bytes);

		fractured_state.Rewind();
		CHECK(snapshot.Restore(fractured_state, *context.GetSystem()));
		CHECK(destruction.IsFractured(intact_body));
		CHECK(destruction.GetChunkBodies(intact_body).size() == 2);

		StateRecorderImpl verify_fractured;
		snapshot.Save(verify_fractured, *context.GetSystem());
		CHECK(verify_fractured.GetData() == fractured_bytes);
	}

	TEST_CASE("DestructionRollbackIsDeterministicForLargeConvexCollapse")
	{
		PhysicsTestContext context;
		context.CreateFloor();

		BodyCreationSettings intact_settings(new BoxShape(Vec3(1.5f, 3.0f, 1.5f)), RVec3(0.0f, 3.2f, 0.0f), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
		BodyID intact_body = context.GetBodyInterface().CreateAndAddBody(intact_settings, EActivation::DontActivate);

		DestructionSystem destruction;
		CHECK(destruction.RegisterBreakableBody(intact_body, sCreateVoronoiCollapseSettings()));

		PhysicsSnapshot snapshot;
		snapshot.AddParticipant(&destruction);

		StateRecorderImpl initial_state;
		snapshot.Save(initial_state, *context.GetSystem());

		auto run_collapse = [&]() -> std::string
		{
			DestructionDamage damage;
			damage.mPosition = RVec3(0.25f, 3.5f, 0.0f);
			damage.mImpulse = Vec3(8.0f, 5.0f, 1.5f);
			damage.mDamage = 60.0f;
			damage.mRadius = 4.0f;
			damage.mForceFracture = true;
			CHECK(destruction.ApplyDamage(context.GetBodyInterface(), intact_body, damage));
			CHECK(destruction.ProcessPendingFractures(context.GetBodyInterface()));
			context.Simulate(1.25f);

			StateRecorderImpl final_state;
			snapshot.Save(final_state, *context.GetSystem());
			return final_state.GetData();
		};

		const std::string first_run = run_collapse();

		initial_state.Rewind();
		CHECK(snapshot.Restore(initial_state, *context.GetSystem()));
		const std::string second_run = run_collapse();

		CHECK(first_run == second_run);
	}

	TEST_CASE("VoronoiPlannerBuildsConvexChunksForPreFracturePipeline")
	{
		VoronoiFracturePlanner planner;
		VoronoiFractureSettings settings;
		settings.mLocalBounds = AABox(Vec3(-1.0f, -2.0f, -1.0f), Vec3(1.0f, 2.0f, 1.0f));
		settings.mCellCount = 6;
		settings.mSamplesX = 6;
		settings.mSamplesY = 8;
		settings.mSamplesZ = 6;
		settings.mRandomSeed = 1337;
		settings.mAnchorBandHeight = 0.35f;
		settings.mConvexRadius = 0.01f;

		PreFracturedMesh mesh;
		BodyCreationSettings chunk_template;
		chunk_template.mObjectLayer = Layers::MOVING;
		planner.AppendCellsAsConvexChunks(settings, chunk_template, mesh, true);

		CHECK(mesh.GetChunkCount() == 6);
		bool found_anchor = false;
		bool found_convex_chunk = false;
		for (uint32 chunk_index = 0; chunk_index < mesh.GetChunkCount(); ++chunk_index)
		{
			found_anchor = found_anchor || mesh.GetChunk(chunk_index).mAnchored;
			found_convex_chunk = found_convex_chunk || mesh.GetChunk(chunk_index).mShape->GetSubType() == EShapeSubType::ConvexHull;
		}

		CHECK(found_anchor);
		CHECK(found_convex_chunk);
	}
}
