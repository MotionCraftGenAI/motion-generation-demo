// Copyright © 2026 MotionCraftGenAI. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Math/UnrealMathUtility.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/PoseableMeshComponent.h"
#include "Retargeter.generated.h"


USTRUCT(BlueprintType)
struct FAxisMapping
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	FName XAxisTarget;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	bool bXAxisInverted;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	FName YAxisTarget;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	bool bYAxisInverted;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	FName ZAxisTarget;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Retargeter")
	bool bZAxisInverted;

	FAxisMapping() = default;

	FAxisMapping(
		const FName InXAxisTarget, const bool bInXAxisInverted,
		const FName InYAxisTarget, const bool bInYAxisInverted,
		const FName InZAxisTarget, const bool bInZAxisInverted)
		: XAxisTarget(InXAxisTarget)
		  , bXAxisInverted(bInXAxisInverted)
		  , YAxisTarget(InYAxisTarget)
		  , bYAxisInverted(bInYAxisInverted)
		  , ZAxisTarget(InZAxisTarget)
		  , bZAxisInverted(bInZAxisInverted)
	{
	}

	static double GetComponentValue(const FName AxisName, const FQuat& SourceRotation)
	{
		if (AxisName == FName("Y"))
		{
			return SourceRotation.Y;
		}
		if (AxisName == FName("Z"))
		{
			return SourceRotation.Z;
		}
		// Default to X if the name is invalid or "X"
		return SourceRotation.X;
	};

	static double GetComponentValue(const FName AxisName, const FVector& SourceVector)
	{
		if (AxisName == FName("Y"))
		{
			return SourceVector.Y;
		}
		if (AxisName == FName("Z"))
		{
			return SourceVector.Z;
		}
		// Default to X if the name is invalid or "X"
		return SourceVector.X;
	};

	FQuat ApplyMapping(const FQuat& SourceRotation) const
	{
		double X = GetComponentValue(XAxisTarget, SourceRotation);
		if (bXAxisInverted)
		{
			X = -X;
		}

		double Y = GetComponentValue(YAxisTarget, SourceRotation);
		if (bYAxisInverted)
		{
			Y = -Y;
		}

		double Z = GetComponentValue(ZAxisTarget, SourceRotation);
		if (bZAxisInverted)
		{
			Z = -Z;
		}

		return FQuat(
			X, Y, Z, SourceRotation.W
		);
	};

	FVector ApplyMapping(const FVector& SourceVector) const
	{
		double X = GetComponentValue(XAxisTarget, SourceVector);
		if (bXAxisInverted)
		{
			X = -X;
		}

		double Y = GetComponentValue(YAxisTarget, SourceVector);
		if (bYAxisInverted)
		{
			Y = -Y;
		}

		double Z = GetComponentValue(ZAxisTarget, SourceVector);
		if (bZAxisInverted)
		{
			Z = -Z;
		}

		return FVector(X, Y, Z);
	};
};

// Represents a single bone mapping from the JSON configuration
USTRUCT(BlueprintType)
struct FBoneMapping
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Retargeter")
	FName SourceBoneName;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Retargeter")
	FName DestinationBoneName;

	// The rotational offset to apply to the source bone's rotation.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Retargeter")
	FQuat QuatCorrectionFactor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Retargeter")
	FAxisMapping AxisMapping;

	FBoneMapping() = default;

	FBoneMapping(
		const FName InSourceBoneName,
		const FName InDestinationBoneName,
		const FQuat& InQuatCorrectionFactor,
		const FAxisMapping& InAxisMapping)
		: SourceBoneName(InSourceBoneName)
		  , DestinationBoneName(InDestinationBoneName)
		  , QuatCorrectionFactor(InQuatCorrectionFactor)
		  , AxisMapping(InAxisMapping)
	{
	}
};

// Represents the entire JSON configuration file
USTRUCT(BlueprintType)
struct FRetargeterConfig
{
	GENERATED_BODY()

	// A map for fast lookups from a destination bone name to its mapping configuration.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Retargeter")
	TMap<FName, FBoneMapping> DestinationBoneMap;

	USkeletalMeshComponent* SkeletalMesh;
	UPoseableMeshComponent* PoseableMesh;
};

UCLASS()
class MOTIONPROTOTYPING_API URetargeter : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * Reads a KeeMap JSON configuration file from disk.
	 * @param FilePath The absolute path to the JSON file.
	 * @param OutConfig The configuration structure to populate.
	 * @return True if the file was read and parsed successfully.
	 */
	UFUNCTION(BlueprintCallable, Category = "Retargeting")
	static bool LoadRetargetingConfig(const FString& FilePath, USkeletalMeshComponent* SkeletalMesh,
	                                  UPoseableMeshComponent* PoseableMesh, FRetargeterConfig& OutConfig);

	/**
	 * Retargets a single bone's rotation.
	 * This is a low-level function that requires you to provide the component space rotations.
	 * @param DestinationBoneName The name of the bone on the source skeleton.
	 * @param SourceBoneComponentSpaceRotation The current component-space rotation of the source bone.
	 * @param DestinationParentComponentSpaceRotation The component-space rotation of the destination bone's parent.
	 * @param Config The loaded retargeting configuration.
	 * @return The calculated LOCAL-space rotation for the destination bone.
	 */
	UFUNCTION(BlueprintPure, Category = "Retargeting")
	static void RetargetSingleBone(
		const FName ConstraintName,
		const FQuat& SourceBoneActorSpaceRotation,
		const FRetargeterConfig& Config);

	/**
	 * Simplified retargeting - applies rotation directly to skeleton (no physics).
	 * Works like root rotation: applies QuatCorrectionFactor * AxisMapping.ApplyMapping(SourceRotation).
	 * @param BoneName The name of the bone to retarget.
	 * @param SourceBoneActorSpaceRotation The source rotation in actor space.
	 * @param Config The loaded retargeting configuration.
	 */
	UFUNCTION(BlueprintCallable, Category = "Retargeting")
	static void RetargetSingleBoneDirect(
		const FName BoneName,
		const FQuat& SourceBoneActorSpaceRotation,
		const FRetargeterConfig& Config,
		const FQuat& ComponentRotation);

private:
	static FConstraintInstance* FindConstraintByName(const FName ConstraintName, USkeletalMeshComponent* SkeletalMesh);
};
