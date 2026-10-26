#include "PhysicalAnimation.h"

#include "ParityObservation.h"

#include "Components/CapsuleComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"

DEFINE_LOG_CATEGORY(LogPhysicalAnimation);

namespace
{
	// Ticks the avatar is held in the model's default pose, pelvis pinned, before release.
	constexpr int32 WarmupTicks = 60;
	// The hold is lifted so the legs can straighten, then set down with this much clearance.
	constexpr double LiftCm = 12.0;
	constexpr double ClearanceCm = 5.0;

	// Contact material the model was trained with.
	constexpr float ContactFriction = 1.0f;
	constexpr float ContactRestitution = 0.0f;

	// Speed command; the model was only trained with index 0.
	constexpr double LocomotionIndex = 0.0;

	// Bones the model's skeleton has no joint for; welded so UE simulates the same rigid bodies.
	const TCHAR* WeldedBones[] = {
		TEXT("spine_02"), TEXT("spine_03"), TEXT("spine_05"),
		TEXT("clavicle_l"), TEXT("clavicle_r"),
		TEXT("hand_l"), TEXT("hand_r"),
		TEXT("neck_01"), TEXT("neck_02")
	};

	FQuat ConstraintFrameRelativeRotation(const FConstraintInstance& Constraint)
	{
		const FQuat ChildFrame = FChaosEngineInterface::GetGlobalPose(
			Constraint.ConstraintHandle, EConstraintFrame::Frame1).GetRotation();
		const FQuat ParentFrame = FChaosEngineInterface::GetGlobalPose(
			Constraint.ConstraintHandle, EConstraintFrame::Frame2).GetRotation();
		FQuat Relative = ParentFrame.Inverse() * ChildFrame;
		Relative.Normalize();
		return Relative;
	}
}

UPhysicalAnimation::UPhysicalAnimation()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UPhysicalAnimation::BeginPlay()
{
	Super::BeginPlay();

	AActor* Owner = GetOwner();
	SkeletalMesh = Owner ? Owner->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
	if (!SkeletalMesh)
	{
		UE_LOG(LogPhysicalAnimation, Error,
			TEXT("PhysicalAnimation requires a skeletal mesh on its owning actor"));
		SetComponentTickEnabled(false);
		return;
	}

	if (!ConnectToModel())
	{
		SetComponentTickEnabled(false);
	}
}

void UPhysicalAnimation::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	TcpLink.Close();
	bConnected = false;
	SkeletalMesh = nullptr;
	Super::EndPlay(EndPlayReason);
}

bool UPhysicalAnimation::ConnectToModel()
{
	FString Address = ModelAddress;
	FParse::Value(FCommandLine::Get(), TEXT("MotionModel="), Address);
	TimeoutMs = FMath::Max(1, FMath::RoundToInt(ModelTimeoutSeconds * 1000.0f));

	FString Error;
	if (Address.IsEmpty() || !TcpLink.Connect(Address, Error))
	{
		UE_LOG(LogPhysicalAnimation, Error,
			TEXT("[Model] cannot reach the motion model at %s: %s"), *Address, *Error);
		ShowStatus(FString::Printf(
			TEXT("Motion model not reachable at %s. Start it with: .venv\\Scripts\\python model/server.py, ")
			TEXT("then press Play again."), *Address), FColor::Red);
		return false;
	}
	bConnected = true;
	UE_LOG(LogPhysicalAnimation, Display,
		TEXT("[Model] connected to %s; observation out, action in, lockstep"), *Address);
	return true;
}

void UPhysicalAnimation::Disconnect(const FString& Reason)
{
	TcpLink.Close();
	bConnected = false;
	UE_LOG(LogPhysicalAnimation, Warning, TEXT("[Model] %s Nothing drives the avatar now."), *Reason);
	ShowStatus(Reason, FColor::Yellow);
}

void UPhysicalAnimation::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateFollowCamera();

	if (bConnected)
	{
		TickModelLoop();
	}
}

void UPhysicalAnimation::TickModelLoop()
{
	// No physics before observation 0 can be sent, so tick N always answers observation N.
	if (!bLoopReady)
	{
		FString Error;
		if (!CanPublishObservation(Error))
		{
			if (++NotReadyTicks == 30)
			{
				UE_LOG(LogPhysicalAnimation, Error, TEXT("[Model] cannot start: %s"), *Error);
				ShowStatus(FString::Printf(TEXT("Motion model cannot start: %s"), *Error), FColor::Red);
			}
			return;
		}
		bLoopReady = true;
	}

	EnsureSimulationStarted();
	if (!bReleased && !bHoldStarted)
	{
		bHoldStarted = true;
		ForceFullBoneEvaluation();
		SetPelvisPinned(true);
		DisableCharacterMovement();
		if (AActor* Owner = GetOwner())
		{
			Owner->SetActorLocation(Owner->GetActorLocation() + FVector(0.0, 0.0, LiftCm),
				false, nullptr, ETeleportType::TeleportPhysics);
			bLifted = true;
		}
		UE_LOG(LogPhysicalAnimation, Display,
			TEXT("[Model] holding the default pose for %d ticks before the model takes over"),
			WarmupTicks);
	}

	// Lockstep: observation out, then block for the action computed from it.
	bPublishedThisTick = PublishObservation();

	TArray<double> DofTargets;
	if (bConnected && bPublishedThisTick)
	{
		FString Reply;
		if (!TcpLink.Recv(Reply, TimeoutMs))
		{
			// A late answer would be applied to the wrong step from then on, so a timeout is fatal.
			Disconnect(TcpLink.IsConnected()
				? FString::Printf(TEXT("Motion model timed out (no answer within %d ms)."), TimeoutMs)
				: FString(TEXT("Motion model disconnected.")));
			return;
		}
		int64 Step = -1;
		FString Error;
		if (!ParityParseActionMessage(Reply, Step, DofTargets, Error))
		{
			Disconnect(FString::Printf(TEXT("Bad message from the motion model: %s"), *Error));
			return;
		}
		if (Step != ObservationStep - 1)
		{
			Disconnect(FString::Printf(
				TEXT("Motion model answered step %lld, expected %lld."), Step, ObservationStep - 1));
			return;
		}
	}

	// During the hold all targets are zero (the default pose); the model's actions are dropped.
	if (!bReleased)
	{
		DofTargets.Reset();
		if (bPublishedThisTick)
		{
			DofTargets.SetNumZeroed(ParityDofCount);
			TickHold();
		}
		else
		{
			UE_LOG(LogPhysicalAnimation, Error,
				TEXT("[Model] tick published nothing at observation %lld; holding the warmup ")
				TEXT("clock so tick and observation stay the same number"), ObservationStep);
		}
	}

	if (DofTargets.Num() == 0)
	{
		return;
	}

	TArray<FQuat> RootRelative;
	if (!ParityForwardKinematics(DofTargets, RootRelative))
	{
		UE_LOG(LogPhysicalAnimation, Error,
			TEXT("[Model] forward kinematics rejected %d joint targets"), DofTargets.Num());
		return;
	}
	FString Error;
	if (!ParityApplyDriveTargets(
		SkeletalMesh, RootRelative, RetargetCache, Error))
	{
		UE_LOG(LogPhysicalAnimation, Verbose, TEXT("[Model] deferring drive install: %s"), *Error);
	}
}

void UPhysicalAnimation::TickHold()
{
	++WarmupTicksElapsed;
	AActor* Owner = GetOwner();

	if (bLifted && WarmupTicksElapsed >= WarmupTicks / 2 && Owner && SkeletalMesh)
	{
		double LowestZ = TNumericLimits<double>::Max();
		for (FBodyInstance* Body : SkeletalMesh->Bodies)
		{
			if (Body && Body->IsValidBodyInstance())
			{
				LowestZ = FMath::Min(LowestZ, Body->GetBodyBounds().Min.Z);
			}
		}

		const FVector ActorLocation = Owner->GetActorLocation();
		double FloorZ = 0.0;
		FHitResult Hit;
		FCollisionQueryParams Params(TEXT("MotionSetDown"), false, Owner);
		if (GetWorld() && GetWorld()->LineTraceSingleByChannel(
			Hit, ActorLocation + FVector(0.0, 0.0, 200.0),
			ActorLocation - FVector(0.0, 0.0, 10000.0), ECC_WorldStatic, Params))
		{
			FloorZ = Hit.ImpactPoint.Z;
		}

		if (LowestZ < TNumericLimits<double>::Max())
		{
			// Descend gradually over a quarter of the hold; a single teleport yanks the limbs.
			const double Ticks = WarmupTicks / 4;
			DescendRemaining = FMath::Max(0.0, (LowestZ - FloorZ) - ClearanceCm);
			DescendPerTick = DescendRemaining / FMath::Max(1.0, Ticks);
		}
		bLifted = false;
	}

	if (DescendRemaining > 0.0 && Owner)
	{
		const double Step = FMath::Min(DescendPerTick, DescendRemaining);
		FVector Where = Owner->GetActorLocation();
		Where.Z -= Step;
		Owner->SetActorLocation(Where, false, nullptr, ETeleportType::TeleportPhysics);
		DescendRemaining -= Step;
		if (DescendRemaining <= KINDA_SMALL_NUMBER)
		{
			DescendRemaining = 0.0;
		}
	}

	if (WarmupTicksElapsed >= WarmupTicks)
	{
		SetPelvisPinned(false);
		bReleased = true;
		UE_LOG(LogPhysicalAnimation, Display,
			TEXT("[Model] released after %d ticks; the model has the avatar now"),
			WarmupTicksElapsed);
		ShowStatus(TEXT("Motion model driving. WASD steers the heading."), FColor::Green);
	}
}

bool UPhysicalAnimation::CanPublishObservation(FString& OutError) const
{
	FParityPhysicsState Probe;
	return ParityCapturePhysicsState(SkeletalMesh, Probe, OutError)
		&& Probe.BodiesWorld.Num() == ParityObsBodyCount;
}

bool UPhysicalAnimation::PublishObservation()
{
	FParityPhysicsState State;
	FString Error;
	if (!ParityCapturePhysicsState(SkeletalMesh, State, Error))
	{
		UE_LOG(LogPhysicalAnimation, Warning, TEXT("[Model] state capture failed: %s"), *Error);
		return false;
	}
	if (State.BodiesWorld.Num() != ParityObsBodyCount)
	{
		UE_LOG(LogPhysicalAnimation, Warning,
			TEXT("[Model] %d world body rows, need %d"), State.BodiesWorld.Num(), ParityObsBodyCount);
		return false;
	}

	// Root height must be above the floor: the model was trained with the floor at z = 0.
	if (!GroundZM.IsSet())
	{
		GroundZM = 0.0;
		AActor* Owner = GetOwner();
		UWorld* World = GetWorld();
		if (Owner && World)
		{
			// From the actor origin: a trace from above would hit the character's own capsule.
			FHitResult Hit;
			FCollisionQueryParams Params(TEXT("MotionObsGround"), true);
			Params.AddIgnoredActor(Owner);
			const FVector At = Owner->GetActorLocation();
			if (World->LineTraceSingleByChannel(
				Hit, At, At - FVector(0.0, 0.0, 5000.0), ECC_WorldStatic, Params))
			{
				GroundZM = ParityUeWorldPositionToIsaac(Hit.ImpactPoint).Z;
			}
			else
			{
				UE_LOG(LogPhysicalAnimation, Warning,
					TEXT("[Model] no ground under the avatar; root height is measured from z = 0"));
			}
		}
	}
	if (GroundZM.GetValue() != 0.0)
	{
		for (FParityBodyState& Row : State.BodiesWorld)
		{
			Row.Values[2] -= GroundZM.GetValue();
		}
	}

	if (!bTargetLatched)
	{
		const double* Root = State.BodiesWorld[0].Values;
		const double RootQuat[4] = { Root[3], Root[4], Root[5], Root[6] };
		const double Heading = ParityCalcHeading(RootQuat);
		TargetDirXy[0] = FMath::Cos(Heading);
		TargetDirXy[1] = FMath::Sin(Heading);
		bTargetLatched = true;
	}

	TArray<double> Observation;
	if (!ParityBuildObservation(State, TargetDirXy, LocomotionIndex, Observation, Error))
	{
		UE_LOG(LogPhysicalAnimation, Warning, TEXT("[Model] observation build failed: %s"), *Error);
		return false;
	}
	for (const double Value : Observation)
	{
		if (!FMath::IsFinite(Value))
		{
			Disconnect(TEXT("The simulation diverged (non-finite observation)."));
			return false;
		}
	}
	FString Json;
	if (!ParitySerializeObservation(ObservationStep, Observation, Json, Error))
	{
		UE_LOG(LogPhysicalAnimation, Warning, TEXT("[Model] observation serialize failed: %s"), *Error);
		return false;
	}
	if (!TcpLink.Send(Json))
	{
		Disconnect(TEXT("Motion model disconnected."));
		return false;
	}
	++ObservationStep;
	return true;
}

void UPhysicalAnimation::EnsureSimulationStarted()
{
	if (bSimulationStarted || !SkeletalMesh)
	{
		return;
	}
	bSimulationStarted = true;

	SkeletalMesh->SetSimulatePhysics(true);
	ApplyContactOverrides();

	int32 Welded = 0;
	for (const TCHAR* BoneName : WeldedBones)
	{
		FConstraintInstance* Constraint = SkeletalMesh->FindConstraintInstance(FName(BoneName));
		if (!Constraint)
		{
			continue;
		}
		// Rotate the child frame so the lock holds the bind pose rather than frame identity.
		if (Constraint->ConstraintHandle.IsValid())
		{
			const FQuat BindRelative = ConstraintFrameRelativeRotation(*Constraint);
			FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
			Frame1.SetRotation((Frame1.GetRotation() * BindRelative.Inverse()).GetNormalized());
			Constraint->SetRefFrame(EConstraintFrame::Frame1, Frame1);
		}
		Constraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Locked);
		Constraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Locked);
		Constraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Locked);
		Constraint->SetProjectionParams(true, 1.0f, 1.0f, 0.0f, 0.0f);
		++Welded;
	}
	UE_LOG(LogPhysicalAnimation, Display,
		TEXT("[Model] simulating %d bodies, %d undriven joints welded"),
		SkeletalMesh->Bodies.Num(), Welded);
}

void UPhysicalAnimation::ForceFullBoneEvaluation()
{
	// The observation is read from bone transforms, so every bone must update every tick.
	SkeletalMesh->SetForcedLOD(1);
	SkeletalMesh->bEnableUpdateRateOptimizations = false;
	SkeletalMesh->VisibilityBasedAnimTickOption =
		EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
}

void UPhysicalAnimation::SetPelvisPinned(bool bPinned)
{
	if (FBodyInstance* Root = SkeletalMesh->GetBodyInstance(ParityRootBoneName()))
	{
		Root->SetInstanceSimulatePhysics(!bPinned);
	}
}

void UPhysicalAnimation::DisableCharacterMovement()
{
	ACharacter* Character = Cast<ACharacter>(GetOwner());
	if (!Character)
	{
		return;
	}
	if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
	{
		Movement->StopMovementImmediately();
		Movement->DisableMovement();
		Movement->SetComponentTickEnabled(false);
	}
	if (UCapsuleComponent* Capsule = Character->GetCapsuleComponent())
	{
		Capsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
}

void UPhysicalAnimation::ApplyContactOverrides()
{
	if (!ContactMaterial)
	{
		ContactMaterial = NewObject<UPhysicalMaterial>(this, TEXT("MotionContactMaterial"));
	}
	ContactMaterial->Friction = ContactFriction;
	ContactMaterial->Restitution = ContactRestitution;
	ContactMaterial->FrictionCombineMode = EFrictionCombineMode::Max;
	ContactMaterial->bOverrideFrictionCombineMode = true;
	ContactMaterial->RestitutionCombineMode = EFrictionCombineMode::Min;
	ContactMaterial->bOverrideRestitutionCombineMode = true;

	for (FBodyInstance* Body : SkeletalMesh->Bodies)
	{
		if (Body)
		{
			Body->SetPhysMaterialOverride(ContactMaterial);
		}
	}

	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (*It == GetOwner())
			{
				continue;
			}
			for (UActorComponent* Component : It->GetComponents())
			{
				UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Component);
				if (Primitive && Primitive->IsCollisionEnabled())
				{
					Primitive->SetPhysMaterialOverride(ContactMaterial);
				}
			}
		}
	}
}

void UPhysicalAnimation::UpdateFollowCamera()
{
	if (!SkeletalMesh)
	{
		return;
	}
	if (!FollowBoom.IsValid())
	{
		FollowBoom = GetOwner() ? GetOwner()->FindComponentByClass<USpringArmComponent>() : nullptr;
		if (!FollowBoom.IsValid())
		{
			return;
		}
	}
	// The bodies walk away from the actor, so the camera arm follows the pelvis.
	const FVector Pelvis = SkeletalMesh->GetBoneLocation(ParityRootBoneName());
	if (!Pelvis.ContainsNaN())
	{
		FollowBoom->SetWorldLocation(Pelvis);
	}
}

void UPhysicalAnimation::SetDirectHeadingFromWorldDirection(FVector WorldDirection)
{
	WorldDirection.Z = 0.0;
	if (WorldDirection.IsNearlyZero())
	{
		return;
	}

	const FVector Policy = ParityUeWorldPositionToIsaac(WorldDirection.GetSafeNormal());
	const double Length = FMath::Sqrt(Policy.X * Policy.X + Policy.Y * Policy.Y);
	if (Length < UE_KINDA_SMALL_NUMBER)
	{
		return;
	}
	TargetDirXy[0] = Policy.X / Length;
	TargetDirXy[1] = Policy.Y / Length;
	bTargetLatched = true;
}

void UPhysicalAnimation::ShowStatus(const FString& Message, const FColor& Color) const
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(static_cast<uint64>(reinterpret_cast<UPTRINT>(this)),
			10.0f, Color, Message);
	}
}
