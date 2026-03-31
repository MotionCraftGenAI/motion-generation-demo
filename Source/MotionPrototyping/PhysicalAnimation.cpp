// Copyright © 2026 MotionCraftGenAI. All rights reserved.


#include "PhysicalAnimation.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PawnMovementComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Character.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/ArrowComponent.h"


#define ROOT_NAME "pelvis"
#define ROOT_INDEX 0

#define CHEST_NAME "spine_04"
#define CHEST_INDEX 1

#define UPPER_ARM_R_NAME "upperarm_r"
#define UPPER_ARM_R_INDEX 3

#define LOWER_ARM_R_NAME "lowerarm_r"
#define LOWER_ARM_R_INDEX 4

#define HAND_R_NAME "hand_r"
#define HAND_R_INDEX 5

#define UPPER_ARM_L_NAME "upperarm_l"
#define UPPER_ARM_L_INDEX 6

#define LOWER_ARM_L_NAME "lowerarm_l"
#define LOWER_ARM_L_INDEX 7

#define HAND_L_NAME "hand_l"
#define HAND_L_INDEX 8

#define THIGH_R_NAME "thigh_r"
#define THIGH_R_INDEX 9

#define SHIN_R_NAME "calf_r"
#define SHIN_R_INDEX 10

#define FOOT_R_NAME "foot_r"
#define FOOT_R_INDEX 11

#define THIGH_L_NAME "thigh_l"
#define THIGH_L_INDEX 12

#define SHIN_L_NAME "calf_l"
#define SHIN_L_INDEX 13

#define FOOT_L_NAME "foot_l"
#define FOOT_L_INDEX 14

DEFINE_LOG_CATEGORY_STATIC(LogPhysicalAnimation, Log, All);


// Sets default values for this component's properties
UPhysicalAnimation::UPhysicalAnimation()
{
	PrimaryComponentTick.bCanEverTick = true;
	LastRootPosition = FVector::ZeroVector;
	CurrentRootPositionDelta = FVector::ZeroVector;
}

// Called when the game starts
void UPhysicalAnimation::BeginPlay()
{
	Super::BeginPlay();
	if (AActor* Owner = GetOwner(); ensure(Owner))
	{
		// Find the skeletal mesh component
		SkeletalMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();

		if (!SkeletalMesh)
		{
			UE_LOG(LogPhysicalAnimation, Error,
			       TEXT("UPhysicalAnimation on Actor '%s' requires a USkeletalMeshComponent, but none was found."),
			       *Owner->GetName());
		}
		else
		{
			// Create PoseableMeshComponent programmatically from the SkeletalMeshComponent
			PoseableMesh = NewObject<UPoseableMeshComponent>(Owner, TEXT("PoseableMesh"));
			PoseableMesh->SetSkinnedAssetAndUpdate(SkeletalMesh->GetSkeletalMeshAsset());
			PoseableMesh->AttachToComponent(Owner->GetRootComponent(),
			                                FAttachmentTransformRules::SnapToTargetIncludingScale);
			PoseableMesh->SetRelativeTransform(SkeletalMesh->GetRelativeTransform());
			PoseableMesh->RegisterComponent();

			// Hide the original skeletal mesh since we're using the poseable mesh for rendering
			SkeletalMesh->SetVisibility(false);
		}
	}

	FrameIndex = 0;
	UpdateCounter = UpdateFrequency;
	LastVelocity = FVector::ZeroVector;

	// Load retargeting configuration
	URetargeter::LoadRetargetingConfig(
		TEXT("C:/Users/szymo/Documents/blender_retarget_map_fin.json"), SkeletalMesh, PoseableMesh, MyRetargetConfig);

	// Initialize Kafka consumer
	KafkaConsumer = NewObject<UKafkaAnimationConsumer>(this);
	KafkaConsumer->Initialize(KafkaConfig);
}

void UPhysicalAnimation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (KafkaConsumer)
	{
		KafkaConsumer->Shutdown();
	}

	// Clean up arrow component
	if (DeltaArrow)
	{
		DeltaArrow->DestroyComponent();
		DeltaArrow = nullptr;
	}

	// Clean up poseable mesh component
	if (PoseableMesh)
	{
		PoseableMesh->DestroyComponent();
		PoseableMesh = nullptr;
	}

	// Restore skeletal mesh visibility
	if (SkeletalMesh)
	{
		SkeletalMesh->SetVisibility(true);
	}

	Super::EndPlay(EndPlayReason);
}

FQuat UPhysicalAnimation::GetRootRotation() const
{
	return CurrentRootRotation;
}

FVector UPhysicalAnimation::GetRootPositionDelta() const
{
	return CurrentRootPositionDelta;
}

void UPhysicalAnimation::ClearRootPositionDelta()
{
	CurrentRootPositionDelta = FVector::ZeroVector;
}

void UPhysicalAnimation::SendHeadingVector2D(const FVector2D& HeadingVector) const
{
	if (KafkaConsumer)
	{
		KafkaConsumer->SendHeadingVector2D(HeadingVector);
	}
}

FVector UPhysicalAnimation::GetActorVelocity() const
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return FVector::ZeroVector;
	}

	// First check for Character Movement Component
	if (const ACharacter* Character = Cast<ACharacter>(Owner))
	{
		if (const UCharacterMovementComponent* CharMove = Character->GetCharacterMovement())
		{
			return CharMove->Velocity;
		}
	}

	// Then check for any Movement Component
	if (const UMovementComponent* MovementComp = Owner->FindComponentByClass<UMovementComponent>())
	{
		return MovementComp->Velocity;
	}

	// Finally, try to get physics velocity from root component
	if (UPrimitiveComponent* PrimComp = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
	{
		return PrimComp->GetPhysicsLinearVelocity();
	}

	// If all else fails, calculate velocity from position change
	return Owner->GetVelocity();
}


// Called every frame
void UPhysicalAnimation::TickComponent(float DeltaTime, ELevelTick TickType,
                                       FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	UpdateCounter++;

	if (UpdateCounter >= UpdateFrequency)
	{
		UpdateCounter = 0;

		FAnimationFrameData NewFrameData;
		const bool bHasNewFrame = KafkaConsumer ? KafkaConsumer->GetNextFrame(NewFrameData) : false;

		if (!bHasNewFrame)
		{
			ClearRootPositionDelta();
			// return;
		}
		else
		{
			CurrentAnimationFrame = NewFrameData;
			FVector NewRootPosition = NewFrameData.RootPosition;
			// MappingTranslate->AxisMapping.ApplyMapping(MappingTranslate->QuatCorrectionFactor.RotateVector(NewFrameData.RootPosition));

			// On first frame, just initialize LastRootPosition without applying movement
			if (!bFirstKafkaFrameReceived)
			{
				LastRootPosition = NewRootPosition;
				bFirstKafkaFrameReceived = true;
			}
			else
			{
				// Calculate and apply delta
				CurrentRootPositionDelta = FVector(
					NewRootPosition.X - LastRootPosition.X,
					NewRootPosition.Y - LastRootPosition.Y,
					NewRootPosition.Z
				);
				UE_LOG(LogPhysicalAnimation, Log, TEXT("Delta R: %f, %s (instance: %p)"), NewRootPosition.Z,
				       *CurrentRootPositionDelta.ToString(), this);

				LastRootPosition = NewRootPosition;

				if (CurrentAnimationFrame.Quaternions.Num() > ROOT_INDEX)
				{
					const FBoneMapping* Mapping = MyRetargetConfig.DestinationBoneMap.Find(FName(ROOT_NAME));
					CurrentRootRotation = Mapping->QuatCorrectionFactor * (Mapping->AxisMapping.ApplyMapping(
						CurrentAnimationFrame.Quaternions[ROOT_INDEX]));
				}
			}

			FrameIndex++;
		}
	}
	// Apply quaternions to bones (using PoseableMesh for direct bone manipulation)
	if (PoseableMesh && CurrentAnimationFrame.Quaternions.Num() > HAND_L_INDEX)
	{
		FVector MeshRelativeLocation = PoseableMesh->GetRelativeLocation();
		MeshRelativeLocation.Z = LastRootPosition.Z * 98.69f / 0.88f - 98.69f + HeightOffset;
		// NOTE: We need to apply correction of 12.67 to Y position
		UE_LOG(LogPhysicalAnimation, Log, TEXT("Mesh position: %s"), *MeshRelativeLocation.ToString());
		PoseableMesh->SetRelativeLocation(MeshRelativeLocation);
		UE_LOG(LogPhysicalAnimation, Log, TEXT("Skeletal Mesh Skeletal Animation Frame, [%f, %f, %f, %f]"),
		       CurrentAnimationFrame.Quaternions[CHEST_INDEX].X, CurrentAnimationFrame.Quaternions[CHEST_INDEX].Y,
		       CurrentAnimationFrame.Quaternions[CHEST_INDEX].Z, CurrentAnimationFrame.Quaternions[CHEST_INDEX].W);
		URetargeter::RetargetSingleBoneDirect(FName(CHEST_NAME), CurrentAnimationFrame.Quaternions[CHEST_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(UPPER_ARM_R_NAME),
		                                      CurrentAnimationFrame.Quaternions[UPPER_ARM_R_INDEX], MyRetargetConfig,
		                                      CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(LOWER_ARM_R_NAME),
		                                      CurrentAnimationFrame.Quaternions[LOWER_ARM_R_INDEX], MyRetargetConfig,
		                                      CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(UPPER_ARM_L_NAME),
		                                      CurrentAnimationFrame.Quaternions[UPPER_ARM_L_INDEX], MyRetargetConfig,
		                                      CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(LOWER_ARM_L_NAME),
		                                      CurrentAnimationFrame.Quaternions[LOWER_ARM_L_INDEX], MyRetargetConfig,
		                                      CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(THIGH_R_NAME), CurrentAnimationFrame.Quaternions[THIGH_R_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(SHIN_R_NAME), CurrentAnimationFrame.Quaternions[SHIN_R_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(FOOT_R_NAME), CurrentAnimationFrame.Quaternions[FOOT_R_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(THIGH_L_NAME), CurrentAnimationFrame.Quaternions[THIGH_L_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(SHIN_L_NAME), CurrentAnimationFrame.Quaternions[SHIN_L_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
		URetargeter::RetargetSingleBoneDirect(FName(FOOT_L_NAME), CurrentAnimationFrame.Quaternions[FOOT_L_INDEX],
		                                      MyRetargetConfig, CurrentRootRotation);
	}
}


bool UPhysicalAnimation::IsVelocitySignificant(const FVector& CurrentVelocity) const
{
	return FVector::Distance(CurrentVelocity, LastVelocity) > VelocityThreshold;
}
