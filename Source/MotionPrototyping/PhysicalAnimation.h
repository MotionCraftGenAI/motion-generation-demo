// Copyright © 2026 MotionCraftGenAI. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Components/ArrowComponent.h"
#include "Retargeter.h"
#include "KafkaAnimationConsumer.h"
#include "PhysicalAnimation.generated.h"


UCLASS(Blueprintable, meta = (BlueprintSpawnableComponent))
class MOTIONPROTOTYPING_API UPhysicalAnimation : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UPhysicalAnimation();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, Category = "Movement")
	float VelocityThreshold = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Movement")
	float HeightOffset = -92.0f;

	int FrameIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Animation Playback")
	int UpdateFrequency = 1;
	int UpdateCounter;

	// Kafka Configuration
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka Configuration")
	FKafkaAnimationConfig KafkaConfig;

	// When enabled, world time dilation is controlled by Kafka message arrival
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka Configuration")
	bool bSyncToKafka = false;

	// Time dilation settings for smooth Kafka sync
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Sync", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float MinTimeDilation = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Sync", meta = (ClampMin = "0.1", ClampMax = "2.0"))
	float MaxTimeDilation = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time Sync")
	float TimeDilationSmoothingSpeed = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual Indicator")
	float DeltaArrowLength = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual Indicator")
	float DeltaArrowThickness = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual Indicator")
	FLinearColor DeltaArrowColor = FLinearColor::Green;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual Indicator")
	float DeltaArrowHeightOffset = 150.0f;

	FQuat GetRootRotation() const;

	FVector GetRootPositionDelta() const;
	double GetLastDeltaUpdateTime() const { return LastDeltaUpdateTime; }
	void ClearRootPositionDelta();

	UFUNCTION(BlueprintCallable, Category = "Kafka")
	void SendHeadingVector2D(const FVector2D& HeadingVector) const;

private:
	// Kafka consumer instance
	UPROPERTY()
	UKafkaAnimationConsumer* KafkaConsumer;

	// Visual indicator arrow
	UPROPERTY()
	UArrowComponent* DeltaArrow;

	// Animation data members
	FVector LastVelocity;
	USkeletalMeshComponent* SkeletalMesh;
	UPoseableMeshComponent* PoseableMesh;
	FRetargeterConfig MyRetargetConfig;

	FAnimationFrameData CurrentAnimationFrame;
	FQuat CurrentRootRotation;
	FVector LastRootPosition;
	FVector CurrentRootPositionDelta;
	double LastDeltaUpdateTime;

	float CurrentTimeDilation = 1.0f;
	bool bFirstKafkaFrameReceived = false;

	// Utility methods
	bool IsVelocitySignificant(const FVector& CurrentVelocity) const;
	FVector GetActorVelocity() const;
};
