#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ParityFKRetarget.h"
#include "ParityTcpLink.h"

#include "PhysicalAnimation.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogPhysicalAnimation, Log, All);

class USkeletalMeshComponent;
class USpringArmComponent;
class UPhysicalMaterial;

// Drives the avatar from the motion model: each tick send the observation, wait for the
// model's joint actions, apply them as joint drive targets, step physics.
UCLASS(Blueprintable, meta = (BlueprintSpawnableComponent))
class MOTIONPROTOTYPING_API UPhysicalAnimation : public UActorComponent
{
	GENERATED_BODY()

public:
	UPhysicalAnimation();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(
		float DeltaTime,
		ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	// host:port of model/server.py; -MotionModel=host:port on the command line overrides it.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Motion Model")
	FString ModelAddress = TEXT("127.0.0.1:41911");

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Motion Model", meta = (ClampMin = "1.0"))
	float ModelTimeoutSeconds = 10.0f;

	// Input steers the heading the model is asked to walk in; it never moves the pawn directly.
	UFUNCTION(BlueprintCallable, Category = "Motion Model")
	void SetDirectHeadingFromWorldDirection(FVector WorldDirection);

private:
	bool ConnectToModel();
	void Disconnect(const FString& Reason);
	void TickModelLoop();

	void EnsureSimulationStarted();

	bool CanPublishObservation(FString& OutError) const;
	bool PublishObservation();

	void TickHold();

	void ForceFullBoneEvaluation();
	void SetPelvisPinned(bool bPinned);
	void DisableCharacterMovement();
	void ApplyContactOverrides();
	void UpdateFollowCamera();
	void ShowStatus(const FString& Message, const FColor& Color) const;

	UPROPERTY(Transient)
	TObjectPtr<USkeletalMeshComponent> SkeletalMesh;

	UPROPERTY(Transient)
	TObjectPtr<UPhysicalMaterial> ContactMaterial;

	TWeakObjectPtr<USpringArmComponent> FollowBoom;

	FParityTcpLink TcpLink;
	bool bConnected = false;
	int32 TimeoutMs = 10000;

	FParityRetargetCache RetargetCache;
	bool bSimulationStarted = false;

	bool bLoopReady = false;
	int32 NotReadyTicks = 0;

	bool bHoldStarted = false;
	bool bReleased = false;
	bool bPublishedThisTick = false;
	int32 WarmupTicksElapsed = 0;

	bool bLifted = false;

	double DescendPerTick = 0.0;
	double DescendRemaining = 0.0;

	int64 ObservationStep = 0;

	// Floor height under the avatar, in model metres; the model was trained with the floor at 0.
	TOptional<double> GroundZM;

	double TargetDirXy[2] = { 1.0, 0.0 };
	bool bTargetLatched = false;
};
