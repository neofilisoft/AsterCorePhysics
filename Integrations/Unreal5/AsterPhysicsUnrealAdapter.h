#pragma once

#include <string>

namespace AsterIntegrations::Unreal5
{
	struct AdapterDescriptor
	{
		std::string mModuleName = "AsterPhysicsUnreal";
		std::string mTargetPhysicsBackend = "ASTER_PHYSICS";
		bool mReplaceChaos = false;
	};
}
