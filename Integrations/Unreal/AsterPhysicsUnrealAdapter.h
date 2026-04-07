#pragma once

#include <string>

namespace AsterIntegrations::Unreal
{
    struct AdapterDescriptor
    {
        std::string ModuleName = "AsterCorePhysicsRuntime";
        std::string IntegrationMode = "MiddlewarePlugin";
        bool ReplaceChaos = false;
        bool SupportsUE4 = true;
        bool SupportsUE5 = true;
    };
}
