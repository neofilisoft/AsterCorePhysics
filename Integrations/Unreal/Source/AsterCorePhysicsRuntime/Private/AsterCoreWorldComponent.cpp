#include "AsterCoreWorldComponent.h"

UAsterCoreWorldComponent::UAsterCoreWorldComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UAsterCoreWorldComponent::BeginPlay()
{
    Super::BeginPlay();

    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (!API.IsReady())
    {
        UE_LOG(LogTemp, Warning, TEXT("AsterCore runtime is not ready. Ensure AsterCoreCAPI.dll is present in the plugin Binaries/ThirdParty/Win64 directory."));
        return;
    }

    ACPH_CAPIWorldSettings Settings {};
    API.DefaultWorldSettings(&Settings);
    WorldHandle = API.CreateWorld(&Settings);
    if (WorldHandle)
    {
        API.SetGravity(WorldHandle, ToAsterVector(Gravity));
    }
}

void UAsterCoreWorldComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (WorldHandle && API.DestroyWorld)
    {
        API.DestroyWorld(WorldHandle);
        WorldHandle = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

void UAsterCoreWorldComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (bAutoStep && WorldHandle && API.WorldStep)
    {
        API.WorldStep(WorldHandle, DeltaTime, FMath::Max(1, CollisionSteps));
    }
}

bool UAsterCoreWorldComponent::IsWorldReady() const
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return WorldHandle != nullptr && API.IsWorldValid && API.IsWorldValid(WorldHandle) != 0;
}

int32 UAsterCoreWorldComponent::CreateBoxBody(FVector HalfExtents, FVector Location, FQuat Rotation, bool bDynamic, float Mass)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (!IsWorldReady() || !API.CreateBoxBody)
    {
        return INDEX_NONE;
    }

    const uint32 BodyId = API.CreateBoxBody(WorldHandle, ToAsterVector(HalfExtents), ToAsterVector(Location), bDynamic ? 1 : 0, Mass);
    API.SetBodyRotation(WorldHandle, BodyId, ToAsterQuat(Rotation));
    return static_cast<int32>(BodyId);
}

int32 UAsterCoreWorldComponent::CreateSphereBody(float Radius, FVector Location, FQuat Rotation, bool bDynamic, float Mass)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (!IsWorldReady() || !API.CreateSphereBody)
    {
        return INDEX_NONE;
    }

    const uint32 BodyId = API.CreateSphereBody(WorldHandle, Radius / 100.0f, ToAsterVector(Location), bDynamic ? 1 : 0, Mass);
    API.SetBodyRotation(WorldHandle, BodyId, ToAsterQuat(Rotation));
    return static_cast<int32>(BodyId);
}

bool UAsterCoreWorldComponent::DestroyBody(int32 BodyId)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return IsWorldReady() && API.DestroyBody && API.DestroyBody(WorldHandle, static_cast<uint32>(BodyId)) != 0;
}

bool UAsterCoreWorldComponent::HasBody(int32 BodyId) const
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return IsWorldReady() && API.HasBody && API.HasBody(WorldHandle, static_cast<uint32>(BodyId)) != 0;
}

FVector UAsterCoreWorldComponent::GetBodyLocation(int32 BodyId) const
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return HasBody(BodyId) && API.GetBodyPosition ? ToUnrealVector(API.GetBodyPosition(WorldHandle, static_cast<uint32>(BodyId))) : FVector::ZeroVector;
}

FQuat UAsterCoreWorldComponent::GetBodyRotation(int32 BodyId) const
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return HasBody(BodyId) && API.GetBodyRotation ? ToUnrealQuat(API.GetBodyRotation(WorldHandle, static_cast<uint32>(BodyId))) : FQuat::Identity;
}

void UAsterCoreWorldComponent::SetBodyTransform(int32 BodyId, FVector Location, FQuat Rotation)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (!HasBody(BodyId))
    {
        return;
    }

    API.SetBodyPosition(WorldHandle, static_cast<uint32>(BodyId), ToAsterVector(Location));
    API.SetBodyRotation(WorldHandle, static_cast<uint32>(BodyId), ToAsterQuat(Rotation));
}

void UAsterCoreWorldComponent::SetBodyLinearVelocity(int32 BodyId, FVector Velocity)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (HasBody(BodyId) && API.SetBodyLinearVelocity)
    {
        API.SetBodyLinearVelocity(WorldHandle, static_cast<uint32>(BodyId), ToAsterVector(Velocity));
    }
}

FVector UAsterCoreWorldComponent::GetBodyLinearVelocity(int32 BodyId) const
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    return HasBody(BodyId) && API.GetBodyLinearVelocity ? ToUnrealVector(API.GetBodyLinearVelocity(WorldHandle, static_cast<uint32>(BodyId))) : FVector::ZeroVector;
}

void UAsterCoreWorldComponent::ApplyBodyImpulse(int32 BodyId, FVector Impulse)
{
    const FAsterCoreCAPIFunctions &API = FAsterCorePhysicsRuntimeModule::Get().GetAPI();
    if (HasBody(BodyId) && API.ApplyBodyImpulse)
    {
        API.ApplyBodyImpulse(WorldHandle, static_cast<uint32>(BodyId), ToAsterVector(Impulse));
    }
}

ACPH_CAPIVec3 UAsterCoreWorldComponent::ToAsterVector(const FVector &Value)
{
    return ACPH_CAPIVec3 { static_cast<float>(Value.X / 100.0f), static_cast<float>(Value.Z / 100.0f), static_cast<float>(Value.Y / 100.0f) };
}

FVector UAsterCoreWorldComponent::ToUnrealVector(const ACPH_CAPIVec3 &Value)
{
    return FVector(Value.x * 100.0f, Value.z * 100.0f, Value.y * 100.0f);
}

ACPH_CAPIQuat UAsterCoreWorldComponent::ToAsterQuat(const FQuat &Value)
{
    return ACPH_CAPIQuat { static_cast<float>(Value.X), static_cast<float>(Value.Z), static_cast<float>(Value.Y), static_cast<float>(Value.W) };
}

FQuat UAsterCoreWorldComponent::ToUnrealQuat(const ACPH_CAPIQuat &Value)
{
    return FQuat(Value.x, Value.z, Value.y, Value.w);
}
