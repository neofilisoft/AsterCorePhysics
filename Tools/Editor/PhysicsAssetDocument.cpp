#include <Tools/Editor/PhysicsAssetDocument.h>
#include <fstream>
#include <sstream>

namespace AsterEditor
{
	void PhysicsAssetDocument::AddBody(const PhysicsBodyAsset &inBody)
	{
		mBodies.push_back(inBody);
	}

	void PhysicsAssetDocument::AddSoftBody(const SoftBodyAsset &inSoftBody)
	{
		mSoftBodies.push_back(inSoftBody);
	}

	void PhysicsAssetDocument::AddConstraint(const ConstraintAsset &inConstraint)
	{
		mConstraints.push_back(inConstraint);
	}

	void PhysicsAssetDocument::AddDestructible(const DestructibleAsset &inDestructible)
	{
		mDestructibles.push_back(inDestructible);
	}

	std::string PhysicsAssetDocument::ExportToAsterAssetText() const
	{
		std::ostringstream stream;
		stream << "asset_format = ASTER_PHYSICS_ASSET\n";
		stream << "bodies = [\n";
		for (const PhysicsBodyAsset &body : mBodies)
			stream << "  { name = \"" << body.mName << "\", pos = [" << body.mPosition.GetX() << ", " << body.mPosition.GetY() << ", " << body.mPosition.GetZ() << "], half_extent = [" << body.mHalfExtent.GetX() << ", " << body.mHalfExtent.GetY() << ", " << body.mHalfExtent.GetZ() << "], mass = " << body.mMass << " },\n";
		stream << "]\nsoft_bodies = [\n";
		for (const SoftBodyAsset &soft_body : mSoftBodies)
			stream << "  { name = \"" << soft_body.mName << "\", particle_radius = " << soft_body.mParticleRadius << ", tear_threshold = " << soft_body.mTearThreshold << ", self_collision = " << (soft_body.mEnableSelfCollision ? "true" : "false") << " },\n";
		stream << "]\nconstraints = [\n";
		for (const ConstraintAsset &constraint : mConstraints)
			stream << "  { name = \"" << constraint.mName << "\", type = \"" << constraint.mType << "\", body_a = \"" << constraint.mBodyA << "\", body_b = \"" << constraint.mBodyB << "\" },\n";
		stream << "]\ndestructibles = [\n";
		for (const DestructibleAsset &destructible : mDestructibles)
		{
			stream << "  {\n";
			stream << "    name = \"" << destructible.mName << "\",\n";
			stream << "    source_body = \"" << destructible.mSourceBody << "\",\n";
			stream << "    fracture_mode = \"" << destructible.mFractureMode << "\",\n";
			stream << "    voronoi_preset = \"" << destructible.mVoronoiPreset << "\",\n";
			stream << "    fracture_damage_threshold = " << destructible.mFractureDamageThreshold << ",\n";
			stream << "    fracture_impulse_threshold = " << destructible.mFractureImpulseThreshold << ",\n";
			stream << "    damage_to_impulse_scale = " << destructible.mDamageToImpulseScale << ",\n";
			stream << "    spawn_supported_chunks_as_static = " << (destructible.mSpawnSupportedChunksAsStatic ? "true" : "false") << ",\n";
			stream << "    fracture_authoring = {\n";
			stream << "      pattern = \"" << destructible.mAuthoring.mPattern << "\",\n";
			stream << "      cluster_mode = \"" << destructible.mAuthoring.mClusterMode << "\",\n";
			stream << "      material_preset = \"" << destructible.mAuthoring.mMaterialPreset << "\",\n";
			stream << "      cell_count = " << destructible.mAuthoring.mCellCount << ",\n";
			stream << "      cluster_count = " << destructible.mAuthoring.mClusterCount << ",\n";
			stream << "      use_impact_points = " << (destructible.mAuthoring.mUseImpactPoints ? "true" : "false") << ",\n";
			stream << "      long_shard_bias = " << destructible.mAuthoring.mLongShardBias << ",\n";
			stream << "      tiny_polygon_bias = " << destructible.mAuthoring.mTinyPolygonBias << ",\n";
			stream << "      enable_static_to_dynamic_transition = " << (destructible.mAuthoring.mEnableStaticToDynamicTransition ? "true" : "false") << ",\n";
			stream << "      wake_impulse_threshold = " << destructible.mAuthoring.mWakeImpulseThreshold << ",\n";
			stream << "      wake_damage_threshold = " << destructible.mAuthoring.mWakeDamageThreshold << ",\n";
			stream << "      keep_sleeping_until_threshold = " << (destructible.mAuthoring.mKeepSleepingUntilThreshold ? "true" : "false") << ",\n";
			stream << "      hierarchy = { enable_merge = " << (destructible.mAuthoring.mHierarchy.mEnableMerge ? "true" : "false")
				<< ", enable_cull = " << (destructible.mAuthoring.mHierarchy.mEnableCull ? "true" : "false")
				<< ", merge_distance = " << destructible.mAuthoring.mHierarchy.mMergeDistance
				<< ", cull_distance = " << destructible.mAuthoring.mHierarchy.mCullDistance
				<< ", target_merged_chunk_count = " << destructible.mAuthoring.mHierarchy.mTargetMergedChunkCount << " },\n";
			stream << "      impact_points = [\n";
			for (const FractureImpactPointAsset &impact_point : destructible.mAuthoring.mImpactPoints)
				stream << "        { local_pos = [" << impact_point.mLocalPosition.GetX() << ", " << impact_point.mLocalPosition.GetY() << ", " << impact_point.mLocalPosition.GetZ() << "], radius = " << impact_point.mRadius << ", seed = " << impact_point.mSeed << ", weight = " << impact_point.mWeight << " },\n";
			stream << "      ]\n";
			stream << "    },\n";
			stream << "    chunks = [\n";
			for (const DestructionChunkAsset &chunk : destructible.mChunks)
				stream << "      { name = \"" << chunk.mName << "\", local_pos = [" << chunk.mLocalPosition.GetX() << ", " << chunk.mLocalPosition.GetY() << ", " << chunk.mLocalPosition.GetZ() << "], half_extent = [" << chunk.mHalfExtent.GetX() << ", " << chunk.mHalfExtent.GetY() << ", " << chunk.mHalfExtent.GetZ() << "], mass = " << chunk.mMass << ", anchored = " << (chunk.mAnchored ? "true" : "false") << " },\n";
			stream << "    ]\n";
			stream << "  },\n";
		}
		stream << "]\n";
		return stream.str();
	}

	bool PhysicsAssetDocument::WriteToFile(const char *inPath) const
	{
		std::ofstream file(inPath, std::ios::binary | std::ios::trunc);
		if (!file.is_open())
			return false;
		const std::string content = ExportToAsterAssetText();
		file.write(content.data(), std::streamsize(content.size()));
		return file.good();
	}
}
