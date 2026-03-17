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
