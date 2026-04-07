#pragma once

#include "Modules/ModuleManager.h"
#include "HAL/PlatformProcess.h"
#include "AsterCoreCAPI.h"

struct FAsterCoreCAPIFunctions
{
    using FInitialize = int (*)();
    using FShutdown = void (*)();
    using FDefaultWorldSettings = void (*)(ACPH_CAPIWorldSettings *);
    using FCreateWorld = ACPH_CAPIWorldHandle (*)(const ACPH_CAPIWorldSettings *);
    using FDestroyWorld = void (*)(ACPH_CAPIWorldHandle);
    using FIsWorldValid = int (*)(ACPH_CAPIWorldHandle);
    using FGetLastError = const char *(*)(ACPH_CAPIWorldHandle);
    using FResetWorld = void (*)(ACPH_CAPIWorldHandle);
    using FSetGravity = void (*)(ACPH_CAPIWorldHandle, ACPH_CAPIVec3);
    using FGetGravity = ACPH_CAPIVec3 (*)(ACPH_CAPIWorldHandle);
    using FWorldStep = int (*)(ACPH_CAPIWorldHandle, float, uint32);
    using FCreateBoxBody = uint32 (*)(ACPH_CAPIWorldHandle, ACPH_CAPIVec3, ACPH_CAPIVec3, int, float);
    using FCreateSphereBody = uint32 (*)(ACPH_CAPIWorldHandle, float, ACPH_CAPIVec3, int, float);
    using FDestroyBody = int (*)(ACPH_CAPIWorldHandle, uint32);
    using FHasBody = int (*)(ACPH_CAPIWorldHandle, uint32);
    using FIsBodyActive = int (*)(ACPH_CAPIWorldHandle, uint32);
    using FGetBodyPosition = ACPH_CAPIVec3 (*)(ACPH_CAPIWorldHandle, uint32);
    using FSetBodyPosition = void (*)(ACPH_CAPIWorldHandle, uint32, ACPH_CAPIVec3);
    using FGetBodyRotation = ACPH_CAPIQuat (*)(ACPH_CAPIWorldHandle, uint32);
    using FSetBodyRotation = void (*)(ACPH_CAPIWorldHandle, uint32, ACPH_CAPIQuat);
    using FGetBodyLinearVelocity = ACPH_CAPIVec3 (*)(ACPH_CAPIWorldHandle, uint32);
    using FSetBodyLinearVelocity = void (*)(ACPH_CAPIWorldHandle, uint32, ACPH_CAPIVec3);
    using FApplyBodyImpulse = void (*)(ACPH_CAPIWorldHandle, uint32, ACPH_CAPIVec3);

    void *LibraryHandle = nullptr;
    FInitialize Initialize = nullptr;
    FShutdown Shutdown = nullptr;
    FDefaultWorldSettings DefaultWorldSettings = nullptr;
    FCreateWorld CreateWorld = nullptr;
    FDestroyWorld DestroyWorld = nullptr;
    FIsWorldValid IsWorldValid = nullptr;
    FGetLastError GetLastError = nullptr;
    FResetWorld ResetWorld = nullptr;
    FSetGravity SetGravity = nullptr;
    FGetGravity GetGravity = nullptr;
    FWorldStep WorldStep = nullptr;
    FCreateBoxBody CreateBoxBody = nullptr;
    FCreateSphereBody CreateSphereBody = nullptr;
    FDestroyBody DestroyBody = nullptr;
    FHasBody HasBody = nullptr;
    FIsBodyActive IsBodyActive = nullptr;
    FGetBodyPosition GetBodyPosition = nullptr;
    FSetBodyPosition SetBodyPosition = nullptr;
    FGetBodyRotation GetBodyRotation = nullptr;
    FSetBodyRotation SetBodyRotation = nullptr;
    FGetBodyLinearVelocity GetBodyLinearVelocity = nullptr;
    FSetBodyLinearVelocity SetBodyLinearVelocity = nullptr;
    FApplyBodyImpulse ApplyBodyImpulse = nullptr;

    bool Load(const FString &DllPath);
    void Unload();
    bool IsReady() const;
};

class FAsterCorePhysicsRuntimeModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    static FAsterCorePhysicsRuntimeModule &Get();
    const FAsterCoreCAPIFunctions &GetAPI() const { return API; }
    bool IsRuntimeReady() const { return API.IsReady(); }

private:
    bool LoadRuntime();

    FAsterCoreCAPIFunctions API;
};
