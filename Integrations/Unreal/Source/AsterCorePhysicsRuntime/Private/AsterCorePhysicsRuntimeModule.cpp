#include "AsterCorePhysicsRuntimeModule.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"

IMPLEMENT_MODULE(FAsterCorePhysicsRuntimeModule, AsterCorePhysicsRuntime)

namespace
{
    template <typename T>
    static T LoadExport(void *LibraryHandle, const TCHAR *Name)
    {
        return reinterpret_cast<T>(FPlatformProcess::GetDllExport(LibraryHandle, Name));
    }
}

bool FAsterCoreCAPIFunctions::Load(const FString &DllPath)
{
    LibraryHandle = FPlatformProcess::GetDllHandle(*DllPath);
    if (LibraryHandle == nullptr)
    {
        return false;
    }

    Initialize = LoadExport<FInitialize>(LibraryHandle, TEXT("ACPH_CAPIInitialize"));
    Shutdown = LoadExport<FShutdown>(LibraryHandle, TEXT("ACPH_CAPIShutdown"));
    DefaultWorldSettings = LoadExport<FDefaultWorldSettings>(LibraryHandle, TEXT("ACPH_CAPIDefaultWorldSettings"));
    CreateWorld = LoadExport<FCreateWorld>(LibraryHandle, TEXT("ACPH_CAPICreateWorld"));
    DestroyWorld = LoadExport<FDestroyWorld>(LibraryHandle, TEXT("ACPH_CAPIDestroyWorld"));
    IsWorldValid = LoadExport<FIsWorldValid>(LibraryHandle, TEXT("ACPH_CAPIIsWorldValid"));
    GetLastError = LoadExport<FGetLastError>(LibraryHandle, TEXT("ACPH_CAPIGetLastError"));
    ResetWorld = LoadExport<FResetWorld>(LibraryHandle, TEXT("ACPH_CAPIResetWorld"));
    SetGravity = LoadExport<FSetGravity>(LibraryHandle, TEXT("ACPH_CAPISetGravity"));
    GetGravity = LoadExport<FGetGravity>(LibraryHandle, TEXT("ACPH_CAPIGetGravity"));
    WorldStep = LoadExport<FWorldStep>(LibraryHandle, TEXT("ACPH_CAPIWorldStep"));
    CreateBoxBody = LoadExport<FCreateBoxBody>(LibraryHandle, TEXT("ACPH_CAPICreateBoxBody"));
    CreateSphereBody = LoadExport<FCreateSphereBody>(LibraryHandle, TEXT("ACPH_CAPICreateSphereBody"));
    DestroyBody = LoadExport<FDestroyBody>(LibraryHandle, TEXT("ACPH_CAPIDestroyBody"));
    HasBody = LoadExport<FHasBody>(LibraryHandle, TEXT("ACPH_CAPIHasBody"));
    IsBodyActive = LoadExport<FIsBodyActive>(LibraryHandle, TEXT("ACPH_CAPIIsBodyActive"));
    GetBodyPosition = LoadExport<FGetBodyPosition>(LibraryHandle, TEXT("ACPH_CAPIGetBodyPosition"));
    SetBodyPosition = LoadExport<FSetBodyPosition>(LibraryHandle, TEXT("ACPH_CAPISetBodyPosition"));
    GetBodyRotation = LoadExport<FGetBodyRotation>(LibraryHandle, TEXT("ACPH_CAPIGetBodyRotation"));
    SetBodyRotation = LoadExport<FSetBodyRotation>(LibraryHandle, TEXT("ACPH_CAPISetBodyRotation"));
    GetBodyLinearVelocity = LoadExport<FGetBodyLinearVelocity>(LibraryHandle, TEXT("ACPH_CAPIGetBodyLinearVelocity"));
    SetBodyLinearVelocity = LoadExport<FSetBodyLinearVelocity>(LibraryHandle, TEXT("ACPH_CAPISetBodyLinearVelocity"));
    ApplyBodyImpulse = LoadExport<FApplyBodyImpulse>(LibraryHandle, TEXT("ACPH_CAPIApplyBodyImpulse"));

    const bool bReady = Initialize && Shutdown && DefaultWorldSettings && CreateWorld && DestroyWorld && IsWorldValid &&
        ResetWorld && SetGravity && WorldStep && CreateBoxBody && CreateSphereBody && DestroyBody && HasBody &&
        GetBodyPosition && SetBodyPosition && GetBodyRotation && SetBodyRotation && GetBodyLinearVelocity && SetBodyLinearVelocity && ApplyBodyImpulse;

    if (!bReady)
    {
        Unload();
        return false;
    }

    Initialize();
    return true;
}

void FAsterCoreCAPIFunctions::Unload()
{
    if (Shutdown)
    {
        Shutdown();
    }

    Initialize = nullptr;
    Shutdown = nullptr;
    DefaultWorldSettings = nullptr;
    CreateWorld = nullptr;
    DestroyWorld = nullptr;
    IsWorldValid = nullptr;
    GetLastError = nullptr;
    ResetWorld = nullptr;
    SetGravity = nullptr;
    GetGravity = nullptr;
    WorldStep = nullptr;
    CreateBoxBody = nullptr;
    CreateSphereBody = nullptr;
    DestroyBody = nullptr;
    HasBody = nullptr;
    IsBodyActive = nullptr;
    GetBodyPosition = nullptr;
    SetBodyPosition = nullptr;
    GetBodyRotation = nullptr;
    SetBodyRotation = nullptr;
    GetBodyLinearVelocity = nullptr;
    SetBodyLinearVelocity = nullptr;
    ApplyBodyImpulse = nullptr;

    if (LibraryHandle)
    {
        FPlatformProcess::FreeDllHandle(LibraryHandle);
        LibraryHandle = nullptr;
    }
}

bool FAsterCoreCAPIFunctions::IsReady() const
{
    return LibraryHandle != nullptr && CreateWorld != nullptr;
}

FAsterCorePhysicsRuntimeModule &FAsterCorePhysicsRuntimeModule::Get()
{
    return FModuleManager::LoadModuleChecked<FAsterCorePhysicsRuntimeModule>(TEXT("AsterCorePhysicsRuntime"));
}

void FAsterCorePhysicsRuntimeModule::StartupModule()
{
    LoadRuntime();
}

void FAsterCorePhysicsRuntimeModule::ShutdownModule()
{
    API.Unload();
}

bool FAsterCorePhysicsRuntimeModule::LoadRuntime()
{
    if (API.IsReady())
    {
        return true;
    }

    const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("AsterCorePhysics"));
    if (!Plugin.IsValid())
    {
        return false;
    }

    const FString BaseDir = Plugin->GetBaseDir();
    const FString DllPath = FPaths::Combine(BaseDir, TEXT("Binaries/ThirdParty/Win64/AsterCoreCAPI.dll"));
    return API.Load(DllPath);
}
