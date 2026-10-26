#include "ParityFKRetarget.h"

#include "Chaos/ChaosEngineInterface.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "HAL/IConsoleManager.h"

#include <cmath>

DEFINE_LOG_CATEGORY_STATIC(LogParityFK, Log, All);

namespace
{
	float ReadCVarFloat(const TCHAR* Name, bool& bFound)
	{
		if (const IConsoleVariable* Variable = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			bFound = true;
			return Variable->GetFloat();
		}
		bFound = false;
		return 0.0f;
	}

	// The skeleton the model was trained on: 15 bodies, 28 joint DoFs.
	constexpr int32 NumDofs = 28;
	constexpr int32 NumBodies = 15;

	struct FCanonicalJoint
	{
		int32 BodyIndex;
		int32 DofStart;
		int32 DofCount;
	};

	constexpr int32 BodyParents[NumBodies] = {
		-1, 0, 1, 1, 3, 4, 1, 6, 7, 0, 9, 10, 0, 12, 13
	};

	constexpr FCanonicalJoint CanonicalJoints[] = {
		{1, 0, 3}, {2, 3, 3}, {3, 6, 3}, {4, 9, 1},
		{6, 10, 3}, {7, 13, 1}, {9, 14, 3}, {10, 17, 1},
		{11, 18, 3}, {12, 21, 3}, {13, 24, 1}, {14, 25, 3}
	};

	enum class EParityAxis : uint8
	{
		X,
		Y,
		Z
	};

	struct FAxisMap
	{
		EParityAxis XSource;
		bool bInvertX;
		EParityAxis YSource;
		bool bInvertY;
		EParityAxis ZSource;
		bool bInvertZ;
	};

	struct FDrivenJoint
	{
		const TCHAR* Bone;
		int32 BodyIndex;
		int32 ParentDrivenBody;
		FQuat Correction;
		FAxisMap AxisMap;
	};

	// UE bone driven for each model body. Correction and AxisMap map the model's joint frame
	// onto the bone's.
	const FDrivenJoint DrivenJoints[] = {
		{TEXT("spine_04"), 1, INDEX_NONE,
			FQuat(0.5, 0.5, 0.5, -0.5),
			{EParityAxis::Z, true, EParityAxis::X, true, EParityAxis::Y, false}},
		{TEXT("head"), 2, INDEX_NONE,
			FQuat(0.5, 0.5, 0.5, -0.5),
			{EParityAxis::Z, true, EParityAxis::X, true, EParityAxis::Y, false}},
		{TEXT("upperarm_r"), 3, INDEX_NONE,
			FQuat(-0.5, 0.5, -0.5, -0.5),
			{EParityAxis::Z, true, EParityAxis::X, false, EParityAxis::Y, true}},
		{TEXT("lowerarm_r"), 4, 3,
			FQuat(-0.5, 0.5, -0.5, -0.5),
			{EParityAxis::Z, true, EParityAxis::X, false, EParityAxis::Y, true}},
		{TEXT("upperarm_l"), 6, INDEX_NONE,
			FQuat(-0.5, -0.5, 0.5, -0.5),
			{EParityAxis::Z, false, EParityAxis::X, true, EParityAxis::Y, true}},
		{TEXT("lowerarm_l"), 7, 6,
			FQuat(-0.5, -0.5, 0.5, -0.5),
			{EParityAxis::Z, false, EParityAxis::X, true, EParityAxis::Y, true}},
		{TEXT("thigh_r"), 9, INDEX_NONE,
			FQuat(-0.5, 0.5, 0.5, 0.5),
			{EParityAxis::Z, false, EParityAxis::X, false, EParityAxis::Y, false}},
		{TEXT("calf_r"), 10, 9,
			FQuat(-0.5, 0.5, 0.5, 0.5),
			{EParityAxis::Z, false, EParityAxis::X, false, EParityAxis::Y, false}},
		{TEXT("foot_r"), 11, 10,
			FQuat(-0.5, 0.5, 0.5, 0.5),
			{EParityAxis::Z, false, EParityAxis::X, false, EParityAxis::Y, false}},
		{TEXT("thigh_l"), 12, INDEX_NONE,
			FQuat(-0.5, -0.5, -0.5, 0.5),
			{EParityAxis::Z, true, EParityAxis::X, true, EParityAxis::Y, false}},
		{TEXT("calf_l"), 13, 12,
			FQuat(-0.5, -0.5, -0.5, 0.5),
			{EParityAxis::Z, true, EParityAxis::X, true, EParityAxis::Y, false}},
		{TEXT("foot_l"), 14, 13,
			FQuat(-0.5, -0.5, -0.5, 0.5),
			{EParityAxis::Z, true, EParityAxis::X, true, EParityAxis::Y, false}}
	};

	struct FMjcfGain
	{
		const TCHAR* Bone;
		double StiffnessNm;
		double DampingNms;
		double ArmatureKgM2;
	};

	// UE torque is kg*cm^2/s^2, the training rig's kg*m^2/s^2.
	constexpr double DriveGainScale = 1.0e4;

	struct FMjcfRangeLimit
	{
		const TCHAR* Bone;
		float HalfTwist;
		float CX, CY, CZ, CW;
	};

	// Knee and ankle twist ranges from training. CX..CW re-centre the limit frame on the model's
	// zero pose, since UE limits are symmetric about the frame.
	const FMjcfRangeLimit MjcfRangeLimits[] = {
		{TEXT("calf_l"), 80.000f, +0.130496f, +0.000001f, +0.000003f, +0.991449f},
		{TEXT("calf_r"), 80.000f, +0.126458f, -0.060265f, +0.083746f, +0.986592f},
		{TEXT("foot_l"), 55.000f, -0.236207f, -0.025789f, +0.007599f, +0.971331f},
		{TEXT("foot_r"), 55.000f, -0.236208f, +0.025789f, -0.007599f, +0.971331f},
	};

	// Joint stiffness, damping and armature of the training rig.
	const FMjcfGain MjcfGains[] = {
		{TEXT("spine_04"), 1000.0, 100.0, 0.02},
		{TEXT("head"), 100.0, 10.0, 0.01},
		{TEXT("upperarm_r"), 400.0, 40.0, 0.02},
		{TEXT("lowerarm_r"), 300.0, 30.0, 0.01},
		{TEXT("upperarm_l"), 400.0, 40.0, 0.02},
		{TEXT("lowerarm_l"), 300.0, 30.0, 0.01},
		{TEXT("thigh_r"), 500.0, 50.0, 0.02},
		{TEXT("calf_r"), 500.0, 50.0, 0.02},
		{TEXT("foot_r"), 400.0, 40.0, 0.01},
		{TEXT("thigh_l"), 500.0, 50.0, 0.02},
		{TEXT("calf_l"), 500.0, 50.0, 0.02},
		{TEXT("foot_l"), 400.0, 40.0, 0.01}
	};

	const FMjcfGain* FindMjcfJoint(const TCHAR* Bone)
	{
		for (const FMjcfGain& Gain : MjcfGains)
		{
			if (FCString::Strcmp(Gain.Bone, Bone) == 0)
			{
				return &Gain;
			}
		}
		return nullptr;
	}

	constexpr double ArmatureToUEInertia = 1.0e4;

	// The training rig adds armature (rotor inertia) to every joint; UE has none, so it is added
	// to the driven body's inertia instead.
	bool ApplyArmature(FBodyInstance* Body, double ArmatureKgM2, FString& OutReport)
	{
		if (!Body || ArmatureKgM2 <= 0.0)
		{
			return false;
		}
		const FVector Before = Body->GetBodyInertiaTensor();
		if (Before.GetMin() <= 0.0)
		{
			return false;
		}
		const double Add = ArmatureKgM2 * ArmatureToUEInertia;
		const FVector Wanted = Before + FVector(Add, Add, Add);

		FPhysicsCommand::ExecuteWrite(Body->GetPhysicsActorHandle(),
			[&Wanted](const FPhysicsActorHandle& Actor)
			{
				FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor, Wanted);
			});

		const FVector After = Body->GetBodyInertiaTensor();

		const FVector ComLocal = Body->GetMassSpaceLocal().GetLocation();
		OutReport = FString::Printf(
			TEXT("mass %.3f kg, com_local (%.2f, %.2f, %.2f) cm |%.2f| ; "
			     "inertia (%.1f, %.1f, %.1f) + %.0f -> (%.1f, %.1f, %.1f) kg*cm^2, "
			     "armature is %.0f%%/%.0f%%/%.0f%% of the result"),
			Body->GetBodyMass(), ComLocal.X, ComLocal.Y, ComLocal.Z, ComLocal.Size(),
			Before.X, Before.Y, Before.Z, Add, After.X, After.Y, After.Z,
			100.0 * Add / (Before.X + Add),
			100.0 * Add / (Before.Y + Add),
			100.0 * Add / (Before.Z + Add));

		return After.Equals(Wanted, 0.5);
	}

	struct FStateSlot
	{
		const TCHAR* Bone;
		int32 MappedBody;
	};

	const FStateSlot StateSlots[NumBodies] = {
		{TEXT("pelvis"), 0}, {TEXT("spine_04"), 1}, {TEXT("head"), 2},
		{TEXT("upperarm_r"), 3}, {TEXT("lowerarm_r"), 4}, {TEXT("hand_r"), 4},
		{TEXT("upperarm_l"), 6}, {TEXT("lowerarm_l"), 7}, {TEXT("hand_l"), 7},
		{TEXT("thigh_r"), 9}, {TEXT("calf_r"), 10}, {TEXT("foot_r"), 11},
		{TEXT("thigh_l"), 12}, {TEXT("calf_l"), 13}, {TEXT("foot_l"), 14}
	};

	constexpr double SqrtHalf = 0.70710678118654752440;

	const FQuat RootCorrection(0.0, 0.0, SqrtHalf, SqrtHalf);
	const FAxisMap RootAxisMap{
		EParityAxis::X, true, EParityAxis::Y, false, EParityAxis::Z, true};

	// Written out rather than FQuat::operator*, which uses FMA and rounds differently; on the
	// drive path one ulp is enough to change the whole run.
	FQuat QuatMultiply(const FQuat& A, const FQuat& B)
	{
		return FQuat(
			A.W * B.X + A.X * B.W + A.Y * B.Z - A.Z * B.Y,
			A.W * B.Y + A.Y * B.W + A.Z * B.X - A.X * B.Z,
			A.W * B.Z + A.Z * B.W + A.X * B.Y - A.Y * B.X,
			A.W * B.W - A.X * B.X - A.Y * B.Y - A.Z * B.Z);
	}

	FQuat FromAngleAxis(double Angle, int32 Axis)
	{
		const FVector Axes[3] = {
			FVector::XAxisVector, FVector::YAxisVector, FVector::ZAxisVector };
		return FQuat(Axes[FMath::Clamp(Axis, 0, 2)], Angle);
	}

	double AxisComponent(const FQuat& Q, EParityAxis Axis)
	{
		switch (Axis)
		{
		case EParityAxis::Y: return Q.Y;
		case EParityAxis::Z: return Q.Z;
		default: return Q.X;
		}
	}

	void WriteAxis(FQuat& Q, EParityAxis Axis, double Value)
	{
		switch (Axis)
		{
		case EParityAxis::Y: Q.Y = Value; break;
		case EParityAxis::Z: Q.Z = Value; break;
		default: Q.X = Value; break;
		}
	}

	FQuat ApplyAxisMap(const FQuat& Q, const FAxisMap& Map)
	{
		return FQuat(
			(Map.bInvertX ? -1.0 : 1.0) * AxisComponent(Q, Map.XSource),
			(Map.bInvertY ? -1.0 : 1.0) * AxisComponent(Q, Map.YSource),
			(Map.bInvertZ ? -1.0 : 1.0) * AxisComponent(Q, Map.ZSource),
			Q.W);
	}

	FQuat ApplyInverseAxisMap(const FQuat& Q, const FAxisMap& Map)
	{
		FQuat Result(0.0, 0.0, 0.0, Q.W);
		WriteAxis(Result, Map.XSource, Map.bInvertX ? -Q.X : Q.X);
		WriteAxis(Result, Map.YSource, Map.bInvertY ? -Q.Y : Q.Y);
		WriteAxis(Result, Map.ZSource, Map.bInvertZ ? -Q.Z : Q.Z);
		Result.Normalize();
		return Result;
	}

	FTransform ParityRefBoneComponentTransform(const USkeletalMeshComponent* Mesh, FName Bone)
	{
		const USkinnedAsset* Asset = Mesh ? Mesh->GetSkinnedAsset() : nullptr;
		if (!Asset)
		{
			return FTransform::Identity;
		}
		const FReferenceSkeleton& RefSkeleton = Asset->GetRefSkeleton();
		const TArray<FTransform>& RefPose = RefSkeleton.GetRefBonePose();
		int32 Index = RefSkeleton.FindBoneIndex(Bone);
		FTransform T = FTransform::Identity;
		while (RefPose.IsValidIndex(Index))
		{
			T = T * RefPose[Index];
			Index = RefSkeleton.GetParentIndex(Index);
		}
		return T;
	}

	const FDrivenJoint* FindDrivenJointByBody(int32 BodyIndex)
	{
		for (const FDrivenJoint& Joint : DrivenJoints)
		{
			if (Joint.BodyIndex == BodyIndex)
			{
				return &Joint;
			}
		}
		return nullptr;
	}

	FQuat ComputeAbsoluteTarget(const FDrivenJoint& Joint, const FQuat& RootRelative)
	{
		FQuat Result = Joint.Correction * ApplyAxisMap(RootRelative, Joint.AxisMap);
		Result.Normalize();
		return Result;
	}

	FConstraintInstance* FindConstraint(USkeletalMeshComponent* Mesh, FName Name)
	{
		if (!Mesh)
		{
			return nullptr;
		}
		for (FConstraintInstance* Constraint : Mesh->Constraints)
		{
			if (Constraint && Constraint->JointName == Name)
			{
				return Constraint;
			}
		}
		return nullptr;
	}

	void SetBodyValues(
		FParityBodyState& OutBody,
		const FVector& Position,
		const FQuat& Rotation,
		const FVector& LinearVelocity,
		const FVector& AngularVelocity)
	{
		OutBody.Values[0] = Position.X;
		OutBody.Values[1] = Position.Y;
		OutBody.Values[2] = Position.Z;
		OutBody.Values[3] = Rotation.X;
		OutBody.Values[4] = Rotation.Y;
		OutBody.Values[5] = Rotation.Z;
		OutBody.Values[6] = Rotation.W;
		OutBody.Values[7] = LinearVelocity.X;
		OutBody.Values[8] = LinearVelocity.Y;
		OutBody.Values[9] = LinearVelocity.Z;
		OutBody.Values[10] = AngularVelocity.X;
		OutBody.Values[11] = AngularVelocity.Y;
		OutBody.Values[12] = AngularVelocity.Z;
	}
}

bool ParityForwardKinematics(
	const TArray<double>& PdTargets,
	TArray<FQuat>& OutRootRelative)
{
	OutRootRelative.Reset();
	if (PdTargets.Num() != NumDofs)
	{
		return false;
	}
	for (double Value : PdTargets)
	{
		if (!std::isfinite(Value))
		{
			return false;
		}
	}

	const FQuat Identity = FQuat::Identity;
	TArray<FQuat> Local;
	Local.Init(Identity, NumBodies);
	for (const FCanonicalJoint& Joint : CanonicalJoints)
	{
		FQuat Accumulator = Identity;
		for (int32 AxisIndex = 0; AxisIndex < Joint.DofCount; ++AxisIndex)
		{
			const int32 CanonicalAxis = Joint.DofCount == 1 ? 1 : AxisIndex;
			const FQuat Step = FromAngleAxis(
				PdTargets[Joint.DofStart + AxisIndex], CanonicalAxis);
			Accumulator = AxisIndex == 0 ? Step : QuatMultiply(Accumulator, Step);
		}
		Local[Joint.BodyIndex] = Accumulator;
	}

	OutRootRelative.Init(Identity, NumBodies);
	for (int32 BodyIndex = 1; BodyIndex < NumBodies; ++BodyIndex)
	{
		const int32 ParentIndex = BodyParents[BodyIndex];
		OutRootRelative[BodyIndex] = ParentIndex == 0
			? Local[BodyIndex]
			: QuatMultiply(OutRootRelative[ParentIndex], Local[BodyIndex]);
	}
	return true;
}

FQuat ParityUeRotationToIsaac(const FQuat& UeRotation)
{
	return FQuat(-UeRotation.Y, -UeRotation.X, -UeRotation.Z, UeRotation.W).GetNormalized();
}

FVector ParityUeWorldPositionToIsaac(const FVector& UePositionCm)
{
	return FVector(UePositionCm.X, -UePositionCm.Y, UePositionCm.Z) / 100.0;
}

FVector ParityUeWorldAxialToIsaac(const FVector& UeAxial)
{
	return -FVector(UeAxial.X, -UeAxial.Y, UeAxial.Z);
}

FName ParityRootBoneName()
{
	return FName(StateSlots[0].Bone);
}

bool ParityApplyDriveTargets(
	USkeletalMeshComponent* Mesh,
	const TArray<FQuat>& RootRelative,
	FParityRetargetCache& Cache,
	FString& OutError)
{
	if (!Mesh || RootRelative.Num() != NumBodies)
	{
		OutError = TEXT("mesh missing or FK body count is not 15");
		return false;
	}

	if (!Cache.bCaptured)
	{
		TMap<FName, FParityConstraintBind> Captured;
		for (const FDrivenJoint& Joint : DrivenJoints)
		{
			const FName Bone(Joint.Bone);
			FConstraintInstance* Constraint = FindConstraint(Mesh, Bone);
			if (!Constraint || !Constraint->ConstraintHandle.IsValid())
			{
				OutError = FString::Printf(TEXT("constraint '%s' is not initialized"), Joint.Bone);
				return false;
			}

			// Put the joint pivot on the skeleton's bind pose, which is what the training rig uses.
			{
				const FTransform Child = ParityRefBoneComponentTransform(
					Mesh, Constraint->ConstraintBone1);
				const FTransform Parent = ParityRefBoneComponentTransform(
					Mesh, Constraint->ConstraintBone2);
				const FVector Wanted = Parent.InverseTransformPosition(Child.GetLocation());
				const FVector Authored =
					Constraint->GetRefFrame(EConstraintFrame::Frame2).GetLocation();
				if (!Wanted.Equals(Authored, 0.01f))
				{
					UE_LOG(LogParityFK, Verbose, TEXT("[Frames] %s pivot moved %.3f cm"),
						*Bone.ToString(), (Wanted - Authored).Size());
					Constraint->SetRefPosition(EConstraintFrame::Frame2, Wanted);
				}
			}

			for (const FMjcfRangeLimit& Lim : MjcfRangeLimits)
			{
				if (Bone != FName(Lim.Bone))
				{
					continue;
				}
				const FQuat Centre(Lim.CX, Lim.CY, Lim.CZ, Lim.CW);
				FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
				Frame1.SetRotation(
					(Frame1.GetRotation() * Centre.Inverse()).GetNormalized());
				Constraint->SetRefFrame(EConstraintFrame::Frame1, Frame1);

				// Hard stops, as in training; Chaos can silently ignore weak soft limits.
				Constraint->ProfileInstance.TwistLimit.bSoftConstraint = false;
				Constraint->ProfileInstance.ConeLimit.bSoftConstraint = false;
				Constraint->ProfileInstance.TwistLimit.Restitution = 0.0f;
				Constraint->ProfileInstance.ConeLimit.Restitution = 0.0f;
				Constraint->ProfileInstance.TwistLimit.ContactDistance = 0.0f;
				Constraint->ProfileInstance.ConeLimit.ContactDistance = 0.0f;
				Constraint->SetAngularTwistLimit(
					EAngularConstraintMotion::ACM_Limited, Lim.HalfTwist);
			}

			const FName ChildBone = Constraint->GetChildBoneName();
			const FName ParentBone = Constraint->GetParentBoneName();
			const FQuat ChildWorld = Mesh->GetBoneQuaternion(ChildBone);
			const FQuat ParentWorld = Mesh->GetBoneQuaternion(ParentBone);
			const FQuat ChildFrame = FChaosEngineInterface::GetGlobalPose(
				Constraint->ConstraintHandle, EConstraintFrame::Frame1).GetRotation();
			const FQuat ParentFrame = FChaosEngineInterface::GetGlobalPose(
				Constraint->ConstraintHandle, EConstraintFrame::Frame2).GetRotation();

			FParityConstraintBind Bind;
			Bind.ChildToConstraint = ChildWorld.Inverse() * ChildFrame;
			Bind.ParentToConstraint = ParentWorld.Inverse() * ParentFrame;
			Bind.ParentActorBind = Mesh->GetSocketTransform(ParentBone, RTS_Actor).GetRotation();

			// The shoulders' and head's parent bones are welded into the torso, so their parent target
			// follows the torso's target with the offset measured at bind.
			if (ParentBone.ToString().Contains(TEXT("clavicle"))
				|| ParentBone.ToString().Contains(TEXT("neck")))
			{
				Bind.LiveAnchorBody = 1;
				const FDrivenJoint* Torso = FindDrivenJointByBody(1);
				const FQuat TorsoActorBind =
					Mesh->GetSocketTransform(FName(Torso->Bone), RTS_Actor).GetRotation();
				Bind.HeldRelativeToAnchor = TorsoActorBind.Inverse() * Bind.ParentActorBind;
			}
			Bind.bValid = true;
			Captured.Add(Bone, Bind);

			// The training rig's gains replace the ones authored in the physics asset, in torque
			// mode: acceleration mode would give every axis of a joint the same response despite
			// their different inertia.
			if (const FMjcfGain* Mjcf = FindMjcfJoint(Joint.Bone))
			{
				float AuthoredSpring = 0.0f;
				float AuthoredDamping = 0.0f;
				float AuthoredForceLimit = 0.0f;
				Constraint->GetAngularDriveParams(AuthoredSpring, AuthoredDamping, AuthoredForceLimit);
				const double Spring = Mjcf->StiffnessNm * DriveGainScale;
				const double Damping = Mjcf->DampingNms * DriveGainScale;

				FString ArmatureReport;
				if (!ApplyArmature(Mesh->GetBodyInstance(Bone), Mjcf->ArmatureKgM2, ArmatureReport))
				{
					UE_LOG(LogParityFK, Warning, TEXT("[Drive] %s: armature not applied"), Joint.Bone);
				}
				Constraint->ProfileInstance.AngularDrive.SetAccelerationMode(false);
				Constraint->SetAngularDriveParams(Spring, Damping, AuthoredForceLimit);
				UE_LOG(LogParityFK, Verbose, TEXT("[Drive] %s Kp %.0f Kd %.0f N*m/rad; %s"),
					Joint.Bone, Mjcf->StiffnessNm, Mjcf->DampingNms, *ArmatureReport);
			}
		}
		Cache.Constraints = MoveTemp(Captured);
		Cache.bCaptured = true;

		// UE scales every drive gain by these (default 1.5); the training gains need 1.0.
		bool bFoundK = false;
		bool bFoundD = false;
		const float ScaleK = ReadCVarFloat(
			TEXT("p.Chaos.JointConstraint.AngularDriveStiffnessScale"), bFoundK);
		const float ScaleD = ReadCVarFloat(
			TEXT("p.Chaos.JointConstraint.AngularDriveDampingScale"), bFoundD);
		if (!bFoundK || !bFoundD
			|| !FMath::IsNearlyEqual(ScaleK, 1.0f, 1.e-3f)
			|| !FMath::IsNearlyEqual(ScaleD, 1.0f, 1.e-3f))
		{
			UE_LOG(LogParityFK, Warning,
				TEXT("Joint drive gains reach the solver scaled x%.4g (stiffness) x%.4g (damping); ")
				TEXT("set p.Chaos.JointConstraint.AngularDrive*Scale to 1.0"),
				ScaleK, ScaleD);
		}
	}

	for (const FDrivenJoint& Joint : DrivenJoints)
	{
		const FName Bone(Joint.Bone);
		FConstraintInstance* Constraint = FindConstraint(Mesh, Bone);
		const FParityConstraintBind* Bind = Cache.Constraints.Find(Bone);
		if (!Constraint || !Constraint->ConstraintHandle.IsValid() || !Bind || !Bind->bValid)
		{
			OutError = FString::Printf(TEXT("lost initialized constraint '%s'"), Joint.Bone);
			return false;
		}

		const FQuat ChildTarget = ComputeAbsoluteTarget(Joint, RootRelative[Joint.BodyIndex]);
		FQuat ParentTarget = Bind->ParentActorBind;
		if (Joint.ParentDrivenBody != INDEX_NONE)
		{
			const FDrivenJoint* ParentJoint = FindDrivenJointByBody(Joint.ParentDrivenBody);
			ParentTarget = ComputeAbsoluteTarget(
				*ParentJoint, RootRelative[Joint.ParentDrivenBody]);
		}
		else if (Bind->LiveAnchorBody != INDEX_NONE)
		{
			const FDrivenJoint* Anchor = FindDrivenJointByBody(Bind->LiveAnchorBody);
			ParentTarget = ComputeAbsoluteTarget(
				*Anchor, RootRelative[Bind->LiveAnchorBody]) * Bind->HeldRelativeToAnchor;
		}

		FQuat ConstraintTarget =
			(ParentTarget * Bind->ParentToConstraint).Inverse()
			* (ChildTarget * Bind->ChildToConstraint);
		ConstraintTarget.Normalize();

		Constraint->SetAngularDriveMode(EAngularDriveMode::SLERP);
		Constraint->SetOrientationDriveSLERP(true);
		Constraint->SetAngularVelocityDriveSLERP(true);
		Constraint->SetAngularOrientationTarget(ConstraintTarget);
	}

	OutError.Reset();
	return true;
}

bool ParityCapturePhysicsState(
	USkeletalMeshComponent* Mesh,
	FParityPhysicsState& OutState,
	FString& OutError)
{
	OutState = FParityPhysicsState();
	if (!Mesh)
	{
		OutError = TEXT("no skeletal mesh");
		return false;
	}

	const FTransform ActorFrame = Mesh->GetOwner()
		? Mesh->GetOwner()->GetActorTransform()
		: FTransform::Identity;

	TArray<FQuat> RootRelativeRotations;
	RootRelativeRotations.Init(FQuat::Identity, NumBodies);
	for (int32 BodyIndex = 0; BodyIndex < NumBodies; ++BodyIndex)
	{
		const int32 MappedBody = StateSlots[BodyIndex].MappedBody;
		if (MappedBody == 0)
		{
			const FQuat ActorPose =
				Mesh->GetSocketTransform(ParityRootBoneName(), RTS_Actor).GetRotation();
			RootRelativeRotations[BodyIndex] =
				ApplyInverseAxisMap(RootCorrection.Inverse() * ActorPose, RootAxisMap);

			// Fixed rotation between UE's pelvis bone and the model's root frame.
			static const FQuat RootWorldConvention(0.70710678f, 0.0f, 0.70710678f, 0.0f);
			RootRelativeRotations[BodyIndex] =
				(RootWorldConvention * RootRelativeRotations[BodyIndex]).GetNormalized();
		}
		else if (MappedBody != INDEX_NONE)
		{
			const FDrivenJoint* Mapping = FindDrivenJointByBody(MappedBody);
			const FQuat ActorPose =
				Mesh->GetSocketTransform(FName(StateSlots[BodyIndex].Bone), RTS_Actor).GetRotation();
			RootRelativeRotations[BodyIndex] = ApplyInverseAxisMap(
				Mapping->Correction.Inverse() * ActorPose,
				Mapping->AxisMap);
		}

		// The hands are welded to the forearms.
		if (BodyIndex == 5) RootRelativeRotations[BodyIndex] = RootRelativeRotations[4];
		if (BodyIndex == 8) RootRelativeRotations[BodyIndex] = RootRelativeRotations[7];
	}

	OutState.BodiesWorld.Reserve(NumBodies);
	for (int32 BodyIndex = 0; BodyIndex < NumBodies; ++BodyIndex)
	{
		const FName Bone(StateSlots[BodyIndex].Bone);
		const FBodyInstance* BodyInst = Mesh->GetBodyInstance(Bone);
		if (!BodyInst)
		{
			OutError = FString::Printf(TEXT("missing physics body '%s'"), *Bone.ToString());
			return false;
		}
		// Position of the physics body, as in training, not of the bone pivot.
		SetBodyValues(
			OutState.BodiesWorld.AddDefaulted_GetRef(),
			ParityUeWorldPositionToIsaac(BodyInst->GetUnrealWorldTransform().GetLocation()),
			ParityUeRotationToIsaac(ActorFrame.GetRotation())
				* RootRelativeRotations[BodyIndex],
			ParityUeWorldPositionToIsaac(Mesh->GetPhysicsLinearVelocity(Bone)),
			ParityUeWorldAxialToIsaac(Mesh->GetPhysicsAngularVelocityInRadians(Bone)));
	}
	OutError.Reset();
	return true;
}
