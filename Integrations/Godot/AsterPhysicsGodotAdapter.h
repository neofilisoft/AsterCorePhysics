#pragma once

#include <string>

namespace AsterIntegrations::Godot
{
	struct AdapterDescriptor
	{
		std::string mModuleName = "AsterPhysicsGodot";
		std::string mTargetPhysicsBackend = "ASTER_PHYSICS";
		bool mExposeEditorInspector = true;
	};
}
