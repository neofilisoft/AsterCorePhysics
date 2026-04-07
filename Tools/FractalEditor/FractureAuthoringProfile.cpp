// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: Copyright 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <Tools/FractalEditor/FractureAuthoringProfile.h>

namespace AsterTools
{
	AsterEditor::FractureAuthoringAsset FractureAuthoringProfile::Create(FracturePattern inPattern, FractureMaterialPreset inMaterial)
	{
		AsterEditor::FractureAuthoringAsset asset;
		asset.mPattern = ToString(inPattern);
		asset.mClusterMode = inPattern == FracturePattern::ImpactCluster? "ImpactCluster" : "ClusteredVoronoi";
		asset.mMaterialPreset = ToString(inMaterial);

		switch (inMaterial)
		{
		case FractureMaterialPreset::Wood:
			asset.mCellCount = 30;
			asset.mClusterCount = 4;
			asset.mLongShardBias = 0.9f;
			asset.mTinyPolygonBias = 0.1f;
			asset.mWakeImpulseThreshold = 60.0f;
			break;
		case FractureMaterialPreset::Glass:
			asset.mCellCount = 96;
			asset.mClusterCount = 10;
			asset.mLongShardBias = 0.05f;
			asset.mTinyPolygonBias = 0.95f;
			asset.mWakeImpulseThreshold = 18.0f;
			asset.mWakeDamageThreshold = 12.0f;
			break;
		case FractureMaterialPreset::Stone:
			asset.mCellCount = 72;
			asset.mClusterCount = 8;
			asset.mLongShardBias = 0.25f;
			asset.mTinyPolygonBias = 0.2f;
			asset.mWakeImpulseThreshold = 110.0f;
			break;
		case FractureMaterialPreset::Concrete:
			asset.mCellCount = 64;
			asset.mClusterCount = 7;
			asset.mLongShardBias = 0.2f;
			asset.mTinyPolygonBias = 0.3f;
			asset.mWakeImpulseThreshold = 95.0f;
			break;
		case FractureMaterialPreset::Metal:
			asset.mCellCount = 28;
			asset.mClusterCount = 3;
			asset.mLongShardBias = 0.65f;
			asset.mTinyPolygonBias = 0.05f;
			asset.mWakeImpulseThreshold = 140.0f;
			break;
		case FractureMaterialPreset::Generic:
		default:
			break;
		}

		if (inPattern == FracturePattern::ImpactCluster)
		{
			asset.mUseImpactPoints = true;
			asset.mClusterCount += 2;
		}

		return asset;
	}

	void FractureAuthoringProfile::AddImpactPoint(AsterEditor::FractureAuthoringAsset &ioAsset, const ACPH::Vec3 &inLocalPosition, float inRadius, ACPH::uint32 inSeed, float inWeight)
	{
		ioAsset.mImpactPoints.push_back({ inLocalPosition, inRadius, inSeed, inWeight });
	}

	std::string FractureAuthoringProfile::ToString(FracturePattern inPattern)
	{
		switch (inPattern)
		{
		case FracturePattern::Voronoi: return "Voronoi";
		case FracturePattern::ClusteredVoronoi: return "ClusteredVoronoi";
		case FracturePattern::ImpactCluster: return "ImpactCluster";
		default: return "Voronoi";
		}
	}

	std::string FractureAuthoringProfile::ToString(FractureMaterialPreset inMaterial)
	{
		switch (inMaterial)
		{
		case FractureMaterialPreset::Wood: return "Wood";
		case FractureMaterialPreset::Glass: return "Glass";
		case FractureMaterialPreset::Stone: return "Stone";
		case FractureMaterialPreset::Concrete: return "Concrete";
		case FractureMaterialPreset::Metal: return "Metal";
		case FractureMaterialPreset::Generic:
		default: return "Generic";
		}
	}
}
