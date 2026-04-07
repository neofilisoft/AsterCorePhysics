// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: Copyright 2026 Neofilisoft
// SPDX-License-Identifier: MIT

#include <Tools/Editor/PhysicsAssetDocument.h>
#include <Tools/FractalEditor/FractureAuthoringProfile.h>
#include <AsterCore/Core/Memory.h>
#include <iostream>

int main(int argc, char **argv)
{
	const char *output_path = argc > 1? argv[1] : "fractal_editor_export.apa";

	ACPH::RegisterDefaultAllocator();
	AsterEditor::PhysicsAssetDocument document;
	document.AddBody({ "TempleWindow", ACPH::Vec3(0.0f, 3.0f, 0.0f), ACPH::Vec3(1.8f, 2.4f, 0.15f), 18.0f });

	AsterEditor::DestructibleAsset window;
	window.mName = "TempleWindowGlass";
	window.mSourceBody = "TempleWindow";
	window.mFractureMode = "PreFractured";
	window.mVoronoiPreset = "ImpactCluster";
	window.mAuthoring = AsterTools::FractureAuthoringProfile::Create(AsterTools::FracturePattern::ImpactCluster, AsterTools::FractureMaterialPreset::Glass);
	AsterTools::FractureAuthoringProfile::AddImpactPoint(window.mAuthoring, ACPH::Vec3(0.25f, 0.35f, 0.0f), 0.55f, 1401u, 1.0f);
	window.mAuthoring.mHierarchy.mMergeDistance = 24.0f;
	window.mAuthoring.mHierarchy.mCullDistance = 42.0f;
	window.mChunks.push_back({ "PaneA", ACPH::Vec3(-0.4f, 0.5f, 0.0f), ACPH::Vec3(0.4f, 0.7f, 0.15f), 2.0f, false });
	window.mChunks.push_back({ "PaneB", ACPH::Vec3(0.45f, -0.2f, 0.0f), ACPH::Vec3(0.35f, 0.55f, 0.15f), 1.5f, false });
	document.AddDestructible(window);

	if (!document.WriteToFile(output_path))
	{
		std::cerr << "Failed to export fracture profile to: " << output_path << std::endl;
		return 1;
	}

	std::cout << "Exported Fractal Editor authoring asset to: " << output_path << std::endl;
	return 0;
}
