#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AsterCorePhysicsRuntimeModule.h"
#include "AsterCoreWorldComponent.generated.h"

UCLASS(ClassGroup=(AsterCore), BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent))
class ASTERCOREPHYSICSRUNTIME_API UAsterCoreWorldComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UAsterCoreWorldComponent();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    bool IsWorldReady() const;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    int32 CreateBoxBody(FVector HalfExtents, FVector Location, FQuat Rotation, bool bDynamic, float Mass);

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    int32 CreateSphereBody(float Radius, FVector Location, FQuat Rotation, bool bDynamic, float Mass);

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    bool DestroyBody(int32 BodyId);

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    bool HasBody(int32 BodyId) const;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    FVector GetBodyLocation(int32 BodyId) const;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    FQuat GetBodyRotation(int32 BodyId) const;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void SetBodyTransform(int32 BodyId, FVector Location, FQuat Rotation);

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void SetBodyLinearVelocity(int32 BodyId, FVector Velocity);

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    FVector GetBodyLinearVelocity(int32 BodyId) const;

    UFUNCTION(BlueprintCallable, Category="AsterCore")
    void ApplyBodyImpulse(int32 BodyId, FVector Impulse);

protected:
    UPROPERTY(EditAnywhere, Category="AsterCore")
    bool bAutoStep = true;

    UPROPERTY(EditAnywhere, Category="AsterCore", meta=(ClampMin="1"))
    int32 CollisionSteps = 1;

    UPROPERTY(EditAnywhere, Category="AsterCore")
    FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

private:
    static ACPH_CAPIVec3 ToAsterVector(const FVector &Value);
    static FVector ToUnrealVector(const ACPH_CAPIVec3 &Value);
    static ACPH_CAPIQuat ToAsterQuat(const FQuat &Value);
    static FQuat ToUnrealQuat(const ACPH_CAPIQuat &Value);

    ACPH_CAPIWorldHandle WorldHandle = nullptr;
};
