#pragma once

#include "CoreMinimal.h"
#include "ParityProtocol.h"

class USkeletalMeshComponent;

// The model's 28 joint targets -> rotation of each of its 15 bodies relative to the root.
MOTIONPROTOTYPING_API bool ParityForwardKinematics(
	const TArray<double>& PdTargets,
	TArray<FQuat>& OutRootRelative);

// Conversions from UE (cm, left-handed) to the model's frame (m, right-handed).
FQuat ParityUeRotationToIsaac(const FQuat& UeRotation);

FVector ParityUeWorldPositionToIsaac(const FVector& UePositionCm);

FVector ParityUeWorldAxialToIsaac(const FVector& UeAxial);

MOTIONPROTOTYPING_API FName ParityRootBoneName();

struct FParityConstraintBind
{
	bool bValid = false;
	FQuat ChildToConstraint = FQuat::Identity;
	FQuat ParentToConstraint = FQuat::Identity;
	FQuat ParentActorBind = FQuat::Identity;
	int32 LiveAnchorBody = INDEX_NONE;
	FQuat HeldRelativeToAnchor = FQuat::Identity;
};

struct FParityRetargetCache
{
	bool bCaptured = false;
	TMap<FName, FParityConstraintBind> Constraints;
};

// Body rotations -> joint-local drive targets on the UE constraints. Also installs the training
// rig's gains, limits and armature on the first call.
MOTIONPROTOTYPING_API bool ParityApplyDriveTargets(
	USkeletalMeshComponent* Mesh,
	const TArray<FQuat>& RootRelative,
	FParityRetargetCache& Cache,
	FString& OutError);

// Reads the 15 model bodies off the simulated avatar.
MOTIONPROTOTYPING_API bool ParityCapturePhysicsState(
	USkeletalMeshComponent* Mesh,
	FParityPhysicsState& OutState,
	FString& OutError);
