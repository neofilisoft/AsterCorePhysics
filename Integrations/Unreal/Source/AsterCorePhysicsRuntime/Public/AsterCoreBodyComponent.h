#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AsterCoreBodyComponent.generated.h"

class UAsterCoreWorldComponent;

UENUM(BlueprintType)
enum class EAsterCoreBodyShape : uint8
{
    Box,
    Sphere
};

UCLASS(ClassGroup=(AsterCore), BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent))
class ASTERCOREPHYSICSRUNTIME_API UAsterCoreBodyComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UAsterCoreBodyComponent();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void CreateBody();

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void DestroyBody();

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void ApplyImpulse(FVector Impulse);

protected:
    UPROPERTY(EditAnywhere, Category="AsterCore")
    UAsterCoreWorldComponent* WorldComponent = nullptr;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    EAsterCoreBodyShape Shape = EAsterCoreBodyShape::Box;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    bool bDynamicBody = true;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    float Mass = 1.0f;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    FVector HalfExtents = FVector(50.0f, 50.0f, 50.0f);

    UPROPERTY(EditAnywhere, Category="AsterCore")
    float Radius = 50.0f;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    bool bCreateOnBeginPlay = true;

private:
    int32 BodyId = INDEX_NONE;
};

