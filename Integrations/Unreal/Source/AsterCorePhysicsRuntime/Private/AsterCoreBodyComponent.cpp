#include "AsterCoreBodyComponent.h"
#include "AsterCoreWorldComponent.h"

UAsterCoreBodyComponent::UAsterCoreBodyComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UAsterCoreBodyComponent::BeginPlay()
{
    Super::BeginPlay();

    if (WorldComponent == nullptr)
    {
        WorldComponent = GetOwner() ? GetOwner()->FindComponentByClass<UAsterCoreWorldComponent>() : nullptr;
    }

    if (bCreateOnBeginPlay)
    {
        CreateBody();
    }
}

void UAsterCoreBodyComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    DestroyBody();
    Super::EndPlay(EndPlayReason);
}

void UAsterCoreBodyComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (WorldComponent == nullptr || BodyId == INDEX_NONE || GetOwner() == nullptr)
    {
        return;
    }

    if (bDynamicBody)
    {
        GetOwner()->SetActorLocationAndRotation(WorldComponent->GetBodyLocation(BodyId), WorldComponent->GetBodyRotation(BodyId));
    }
    else
    {
        WorldComponent->SetBodyTransform(BodyId, GetOwner()->GetActorLocation(), GetOwner()->GetActorQuat());
    }
}

void UAsterCoreBodyComponent::CreateBody()
{
    if (WorldComponent == nullptr || BodyId != INDEX_NONE || GetOwner() == nullptr || !WorldComponent->IsWorldReady())
    {
        return;
    }

    BodyId = Shape == EAsterCoreBodyShape::Sphere
        ? WorldComponent->CreateSphereBody(Radius, GetOwner()->GetActorLocation(), GetOwner()->GetActorQuat(), bDynamicBody, Mass)
        : WorldComponent->CreateBoxBody(HalfExtents, GetOwner()->GetActorLocation(), GetOwner()->GetActorQuat(), bDynamicBody, Mass);
}

void UAsterCoreBodyComponent::DestroyBody()
{
    if (WorldComponent && BodyId != INDEX_NONE)
    {
        WorldComponent->DestroyBody(BodyId);
        BodyId = INDEX_NONE;
    }
}

void UAsterCoreBodyComponent::ApplyImpulse(FVector Impulse)
{
    if (WorldComponent && BodyId != INDEX_NONE)
    {
        WorldComponent->ApplyBodyImpulse(BodyId, Impulse);
    }
}
