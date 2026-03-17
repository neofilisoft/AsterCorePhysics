#pragma once

#include <AsterCore/AsterCore.h>
#include <AsterCore/Core/Array.h>
#include <AsterCore/Math/Vec3.h>
#include <string>

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

	class PhysicsAssetDocument
	{
	public:
		void AddBody(const PhysicsBodyAsset &inBody);
		void AddSoftBody(const SoftBodyAsset &inSoftBody);
		void AddConstraint(const ConstraintAsset &inConstraint);
		std::string ExportToAsterAssetText() const;
		bool WriteToFile(const char *inPath) const;

	private:
		ACPH::Array<PhysicsBodyAsset> mBodies;
		ACPH::Array<SoftBodyAsset> mSoftBodies;
		ACPH::Array<ConstraintAsset> mConstraints;
	};
}
