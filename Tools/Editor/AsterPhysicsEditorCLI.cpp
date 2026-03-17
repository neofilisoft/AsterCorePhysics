#include <Tools/Editor/PhysicsAssetDocument.h>
#include <iostream>

int main(int argc, char **argv)
{
	const char *output_path = argc > 1? argv[1] : "aster_editor_export.apa";

	AsterEditor::PhysicsAssetDocument document;
	document.AddBody({ "Crate", ACPH::Vec3(0.0f, 1.0f, 0.0f), ACPH::Vec3(0.5f, 0.5f, 0.5f), 15.0f });
	document.AddSoftBody({ "BannerCloth", 0.03f, 1.9f, true });
	document.AddConstraint({ "KingBannerJiggle", "Jiggle", "Crate", "BannerCloth" });

	if (!document.WriteToFile(output_path))
	{
		std::cerr << "Failed to export physics asset to: " << output_path << std::endl;
		return 1;
	}

	std::cout << "Exported ASTER_PHYSICS_ASSET to: " << output_path << std::endl;
	return 0;
}
