#include <Tools/Editor/PhysicsAssetDocument.h>
#include <AsterCore/Core/Memory.h>
#include <iostream>

int main(int argc, char **argv)
{
	const char *output_path = argc > 1? argv[1] : "aster_editor_export.apa";

	ACPH::RegisterDefaultAllocator();
	AsterEditor::PhysicsAssetDocument document;
	document.AddBody({ "Crate", ACPH::Vec3(0.0f, 1.0f, 0.0f), ACPH::Vec3(0.5f, 0.5f, 0.5f), 15.0f });
	document.AddBody({ "GatehouseShell", ACPH::Vec3(0.0f, 8.0f, 0.0f), ACPH::Vec3(2.5f, 6.0f, 2.5f), 240.0f });
	document.AddSoftBody({ "BannerCloth", 0.03f, 1.9f, true });
	document.AddConstraint({ "KingBannerJiggle", "Jiggle", "Crate", "BannerCloth" });

	AsterEditor::DestructibleAsset gatehouse;
	gatehouse.mName = "GatehouseCollapse";
	gatehouse.mSourceBody = "GatehouseShell";
	gatehouse.mFractureDamageThreshold = 180.0f;
	gatehouse.mFractureImpulseThreshold = 85.0f;
	gatehouse.mDamageToImpulseScale = 0.55f;
	gatehouse.mSpawnSupportedChunksAsStatic = true;
	gatehouse.mAuthoring.mPattern = "Voronoi";
	gatehouse.mAuthoring.mClusterMode = "ImpactCluster";
	gatehouse.mAuthoring.mMaterialPreset = "Stone";
	gatehouse.mAuthoring.mCellCount = 72;
	gatehouse.mAuthoring.mClusterCount = 9;
	gatehouse.mAuthoring.mWakeImpulseThreshold = 120.0f;
	gatehouse.mAuthoring.mWakeDamageThreshold = 55.0f;
	gatehouse.mAuthoring.mHierarchy.mMergeDistance = 48.0f;
	gatehouse.mAuthoring.mHierarchy.mCullDistance = 72.0f;
	gatehouse.mAuthoring.mHierarchy.mTargetMergedChunkCount = 6;
	gatehouse.mAuthoring.mImpactPoints.push_back({ ACPH::Vec3(0.0f, -1.5f, 1.25f), 1.2f, 1337u, 1.0f });
	gatehouse.mAuthoring.mImpactPoints.push_back({ ACPH::Vec3(0.0f, 2.5f, -0.75f), 0.9f, 2024u, 0.8f });
	gatehouse.mChunks.push_back({ "BaseWest", ACPH::Vec3(-1.2f, -4.0f, 0.0f), ACPH::Vec3(1.2f, 2.0f, 2.5f), 60.0f, true });
	gatehouse.mChunks.push_back({ "BaseEast", ACPH::Vec3(1.2f, -4.0f, 0.0f), ACPH::Vec3(1.2f, 2.0f, 2.5f), 60.0f, true });
	gatehouse.mChunks.push_back({ "MidSection", ACPH::Vec3(0.0f, 0.0f, 0.0f), ACPH::Vec3(2.4f, 2.0f, 2.5f), 70.0f, false });
	gatehouse.mChunks.push_back({ "Crown", ACPH::Vec3(0.0f, 4.0f, 0.0f), ACPH::Vec3(2.4f, 2.0f, 2.5f), 50.0f, false });
	document.AddDestructible(gatehouse);

	if (!document.WriteToFile(output_path))
	{
		std::cerr << "Failed to export physics asset to: " << output_path << std::endl;
		return 1;
	}

	std::cout << "Exported ASTER_PHYSICS_ASSET with destruction metadata to: " << output_path << std::endl;
	return 0;
}
