// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: Copyright 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#pragma once

#include <Tools/Editor/PhysicsAssetDocument.h>
#include <string>

namespace AsterTools
{
	enum class FracturePattern
	{
		Voronoi,
		ClusteredVoronoi,
		ImpactCluster
	};

	enum class FractureMaterialPreset
	{
		Generic,
		Wood,
		Glass,
		Stone,
		Concrete,
		Metal
	};

	class FractureAuthoringProfile
	{
	public:
		static AsterEditor::FractureAuthoringAsset Create(FracturePattern inPattern, FractureMaterialPreset inMaterial);
		static void AddImpactPoint(AsterEditor::FractureAuthoringAsset &ioAsset, const ACPH::Vec3 &inLocalPosition, float inRadius, ACPH::uint32 inSeed, float inWeight);
		static std::string ToString(FracturePattern inPattern);
		static std::string ToString(FractureMaterialPreset inMaterial);
	};
}
