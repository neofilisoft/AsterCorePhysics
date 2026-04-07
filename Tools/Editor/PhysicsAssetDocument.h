#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Core/Array.h>
#include <AsterCore/Math/Vec3.h>
#include <cstdint>
#include <string>
#include <vector>

namespace AsterEditor
{
	struct PhysicsBodyAsset
	{
		std::string mName;
		ACPH::Vec3 mPosition = ACPH::Vec3::sZero();
		ACPH::Vec3 mHalfExtent = ACPH::Vec3(0.5f, 0.5f, 0.5f);
		float mMass = 1.0f;
	};

	struct SoftBodyAsset
	{
		std::string mName;
		float mParticleRadius = 0.05f;
		float mTearThreshold = 1.75f;
		bool mEnableSelfCollision = true;
	};

	struct ConstraintAsset
	{
		std::string mName;
		std::string mType;
		std::string mBodyA;
		std::string mBodyB;
	};

	struct DestructionChunkAsset
	{
		std::string mName;
		ACPH::Vec3 mLocalPosition = ACPH::Vec3::sZero();
		ACPH::Vec3 mHalfExtent = ACPH::Vec3(0.5f, 0.5f, 0.5f);
		float mMass = 15.0f;
		bool mAnchored = false;
	};

	struct FractureImpactPointAsset
	{
		ACPH::Vec3 mLocalPosition = ACPH::Vec3::sZero();
		float mRadius = 0.75f;
		std::uint32_t mSeed = 0;
		float mWeight = 1.0f;
	};

	struct FractureChunkHierarchyAsset
	{
		bool mEnableMerge = true;
		bool mEnableCull = true;
		float mMergeDistance = 35.0f;
		float mCullDistance = 55.0f;
		std::uint32_t mTargetMergedChunkCount = 8;
	};

	struct FractureAuthoringAsset
	{
		std::string mPattern = "Voronoi";
		std::string mClusterMode = "ClusteredVoronoi";
		std::string mMaterialPreset = "Concrete";
		std::uint32_t mCellCount = 48;
		std::uint32_t mClusterCount = 6;
		bool mUseImpactPoints = true;
		float mLongShardBias = 0.0f;
		float mTinyPolygonBias = 0.0f;
		bool mEnableStaticToDynamicTransition = true;
		float mWakeImpulseThreshold = 45.0f;
		float mWakeDamageThreshold = 30.0f;
		bool mKeepSleepingUntilThreshold = true;
		FractureChunkHierarchyAsset mHierarchy;
		std::vector<FractureImpactPointAsset> mImpactPoints;
	};

	struct DestructibleAsset
	{
		std::string mName;
		std::string mSourceBody;
		std::string mFractureMode = "PreFractured";
		std::string mVoronoiPreset = "BuildingCollapse";
		float mFractureDamageThreshold = 125.0f;
		float mFractureImpulseThreshold = 60.0f;
		float mDamageToImpulseScale = 0.4f;
		bool mSpawnSupportedChunksAsStatic = true;
		FractureAuthoringAsset mAuthoring;
		std::vector<DestructionChunkAsset> mChunks;
	};

	class PhysicsAssetDocument
	{
	public:
		void AddBody(const PhysicsBodyAsset &inBody);
		void AddSoftBody(const SoftBodyAsset &inSoftBody);
		void AddConstraint(const ConstraintAsset &inConstraint);
		void AddDestructible(const DestructibleAsset &inDestructible);
		std::string ExportToAsterAssetText() const;
		bool WriteToFile(const char *inPath) const;

	private:
		ACPH::Array<PhysicsBodyAsset> mBodies;
		ACPH::Array<SoftBodyAsset> mSoftBodies;
		ACPH::Array<ConstraintAsset> mConstraints;
		ACPH::Array<DestructibleAsset> mDestructibles;
	};
}
