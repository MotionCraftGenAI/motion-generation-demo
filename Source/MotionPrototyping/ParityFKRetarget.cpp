#include "ParityFKRetarget.h"

#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PBDJointConstraintUtilities.h"
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
	float GDriveGainScale = 1.0e4f;
	FAutoConsoleVariableRef CVarDriveGainScale(
		TEXT("p.Parity.DriveGainScale"),
		GDriveGainScale,
		TEXT("Multiplier from MJCF N*m/rad onto UE angular drive units."),
		ECVF_Default);

	int32 GInjectMjcfGains = 1;
	FAutoConsoleVariableRef CVarInjectMjcfGains(
		TEXT("p.Parity.InjectMjcfGains"),
		GInjectMjcfGains,
		TEXT("1 to overwrite authored angular drive gains with the MJCF values."),
		ECVF_Default);

	int32 GFixConstraintFrames = 1;
	FAutoConsoleVariableRef CVarFixConstraintFrames(
		TEXT("p.Parity.FixConstraintFrames"),
		GFixConstraintFrames,
		TEXT("1 to move each constraint's Frame2 onto the reference-skeleton bone offset."));

	int32 GKneeLimits = 1;
	FAutoConsoleVariableRef CVarKneeLimits(
		TEXT("p.Parity.KneeLimits"),
		GKneeLimits,
		TEXT("Enforce the MJCF knee range (0..160 deg) on the UE calf constraints."));

	int32 GAnkleLimits = 1;
	FAutoConsoleVariableRef CVarAnkleLimits(
		TEXT("p.Parity.AnkleLimits"),
		GAnkleLimits,
		TEXT("Enforce the MJCF ankle ranges: 1 for the twist axis only (ankle_y, the one "
		     "that maps exactly), 2 for all three."));

	struct FMjcfRangeLimit
	{
		const TCHAR* Bone;
		const int32* Enable;
		float HalfTwist;
		float HalfSwing1;
		float HalfSwing2;
		float CX, CY, CZ, CW;
	};

	// Knee and ankle ranges from training, expressed in UE's twist/swing axes. CX..CW re-centre
	// the limit frame on the model's zero pose, since UE limits are symmetric about the frame.
	const FMjcfRangeLimit MjcfRangeLimits[] = {
		{TEXT("calf_l"), &GKneeLimits,  80.000f, -1.0f,   -1.0f,
		 +0.130496f, +0.000001f, +0.000003f, +0.991449f},
		{TEXT("calf_r"), &GKneeLimits,  80.000f, -1.0f,   -1.0f,
		 +0.126458f, -0.060265f, +0.083746f, +0.986592f},
		{TEXT("foot_l"), &GAnkleLimits, 55.000f, 35.624f, 26.684f,
		 -0.236207f, -0.025789f, +0.007599f, +0.971331f},
		{TEXT("foot_r"), &GAnkleLimits, 55.000f, 35.624f, 26.684f,
		 -0.236208f, +0.025789f, -0.007599f, +0.971331f},
	};

	int32 GDriveNeck = 1;
	FAutoConsoleVariableRef CVarDriveNeck(
		TEXT("p.Parity.DriveNeck"),
		GDriveNeck,
		TEXT("1 to drive the head from neck_x/y/z, 0 to weld it as before."));

	int32 GObsRootExact = 0;
	FAutoConsoleVariableRef CVarObsRootExact(
		TEXT("p.Parity.ObsRootExact"),
		GObsRootExact,
		TEXT("1 to report the pelvis with the exact measured convention, 0 the snapped one."));

	int32 GClavicleAnchorBind = 1;
	FAutoConsoleVariableRef CVarClavicleAnchorBind(
		TEXT("p.Parity.ClavicleAnchorBind"),
		GClavicleAnchorBind,
		TEXT("1 to anchor the clavicle weld offset against the torso's bind "
		     "rotation, 0 for the old neutral-target anchor."),
		ECVF_Default);

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

	int32 GDriveMode = 0;
	FAutoConsoleVariableRef CVarDriveMode(
		TEXT("p.Parity.DriveMode"),
		GDriveMode,
		TEXT("0 = SLERP drive (one rotation-vector error), 1 = TwistAndSwing (per-axis)."));

	// Torque mode, not acceleration: the model's joints differ in inertia per axis.
	int32 GTorqueMode = 1;
	FAutoConsoleVariableRef CVarTorqueMode(
		TEXT("p.Parity.TorqueMode"),
		GTorqueMode,
		TEXT("1 to drive in torque units (bAccelerationMode off), which is the only "
		     "mode that can reproduce a joint whose axes differ in inertia."),
		ECVF_Default);

	int32 GEmulateArmature = 1;
	FAutoConsoleVariableRef CVarEmulateArmature(
		TEXT("p.Parity.EmulateArmature"),
		GEmulateArmature,
		TEXT("1 to add the MJCF's armature to each driven body's inertia tensor, "
		     "per axis, so UE's effective joint inertia matches Isaac's."),
		ECVF_Default);

	bool FindMjcfGain(const TCHAR* Bone, double& OutStiffness, double& OutDamping)
	{
		for (const FMjcfGain& Gain : MjcfGains)
		{
			if (FCString::Strcmp(Gain.Bone, Bone) == 0)
			{
				OutStiffness = Gain.StiffnessNm * GDriveGainScale;
				OutDamping = Gain.DampingNms * GDriveGainScale;
				return true;
			}
		}
		return false;
	}

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

FVector ParityUePositionToIsaac(const FVector& UePositionCm)
{
	return FVector(UePositionCm.Y, UePositionCm.X, UePositionCm.Z) / 100.0;
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

bool ParityIsNeckDriven()
{
	return GDriveNeck != 0;
}

FName ParityRootBoneName()
{
	return FName(StateSlots[0].Bone);
}

bool ParityApplyDriveTargets(
	USkeletalMeshComponent* Mesh,
	const TArray<FQuat>& RootRelative,
	FParityRetargetCache& Cache,
	float& OutWorstReadbackDegrees,
	FString& OutError)
{
	OutWorstReadbackDegrees = 0.0f;
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
			if (GFixConstraintFrames != 0)
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
					UE_LOG(LogParityFK, Display,
						TEXT("[Frames] %s Frame2 (%+.3f %+.3f %+.3f) -> (%+.3f %+.3f %+.3f) cm, "
						     "moved %.3f cm"),
						*Bone.ToString(),
						Authored.X, Authored.Y, Authored.Z,
						Wanted.X, Wanted.Y, Wanted.Z,
						(Wanted - Authored).Size());
					Constraint->SetRefPosition(EConstraintFrame::Frame2, Wanted);
				}
			}

			for (const FMjcfRangeLimit& Lim : MjcfRangeLimits)
			{
				if (Bone != FName(Lim.Bone) || *Lim.Enable == 0)
				{
					continue;
				}

				const FQuat Centre(Lim.CX, Lim.CY, Lim.CZ, Lim.CW);
				FTransform Frame1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
				Frame1.SetRotation(
					(Frame1.GetRotation() * Centre.Inverse()).GetNormalized());
				Constraint->SetRefFrame(EConstraintFrame::Frame1, Frame1);

				UE_LOG(LogParityFK, Display,
					TEXT("[Limit] %s authored soft twist=%d(%.0f/%.0f) cone=%d(%.0f/%.0f) ")
					TEXT("restitution twist=%.2f cone=%.2f contact twist=%.2f cone=%.2f"),
					Lim.Bone,
					Constraint->ProfileInstance.TwistLimit.bSoftConstraint ? 1 : 0,
					Constraint->ProfileInstance.TwistLimit.Stiffness,
					Constraint->ProfileInstance.TwistLimit.Damping,
					Constraint->ProfileInstance.ConeLimit.bSoftConstraint ? 1 : 0,
					Constraint->ProfileInstance.ConeLimit.Stiffness,
					Constraint->ProfileInstance.ConeLimit.Damping,
					Constraint->ProfileInstance.TwistLimit.Restitution,
					Constraint->ProfileInstance.ConeLimit.Restitution,
					Constraint->ProfileInstance.TwistLimit.ContactDistance,
					Constraint->ProfileInstance.ConeLimit.ContactDistance);

				// Hard stops, as in training; Chaos can silently ignore weak soft limits.
				Constraint->ProfileInstance.TwistLimit.bSoftConstraint = false;
				Constraint->ProfileInstance.ConeLimit.bSoftConstraint = false;
				Constraint->ProfileInstance.TwistLimit.Restitution = 0.0f;
				Constraint->ProfileInstance.ConeLimit.Restitution = 0.0f;
				Constraint->ProfileInstance.TwistLimit.ContactDistance = 0.0f;
				Constraint->ProfileInstance.ConeLimit.ContactDistance = 0.0f;

				if (Lim.HalfTwist >= 0.0f)
				{
					Constraint->SetAngularTwistLimit(
						EAngularConstraintMotion::ACM_Limited, Lim.HalfTwist);
				}

				const bool bSwings = (*Lim.Enable >= 2);
				if (Lim.HalfSwing1 >= 0.0f && bSwings)
				{
					Constraint->SetAngularSwing1Limit(
						EAngularConstraintMotion::ACM_Limited, Lim.HalfSwing1);
				}
				if (Lim.HalfSwing2 >= 0.0f && bSwings)
				{
					Constraint->SetAngularSwing2Limit(
						EAngularConstraintMotion::ACM_Limited, Lim.HalfSwing2);
				}

				Chaos::FReal Tw = 0.0, S1 = 0.0, S2 = 0.0;
				Chaos::FPBDJointUtilities::GetSwingTwistAngles(
					Chaos::FRotation3::Identity, Centre, Tw, S1, S2);
				UE_LOG(LogParityFK, Display,
					TEXT("[Limit] %s neutral shifted by twist %+.3f swing1 %+.3f ")
					TEXT("swing2 %+.3f deg; limits twist %.3f swing1 %.3f swing2 %.3f"),
					Lim.Bone,
					FMath::RadiansToDegrees((float)Tw),
					FMath::RadiansToDegrees((float)S1),
					FMath::RadiansToDegrees((float)S2),
					Lim.HalfTwist, Lim.HalfSwing1, Lim.HalfSwing2);
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
				const FQuat NeutralTorsoTarget =
					ComputeAbsoluteTarget(*Torso, FQuat::Identity);
				const FQuat TorsoActorBind =
					Mesh->GetSocketTransform(FName(Torso->Bone), RTS_Actor).GetRotation();
				Bind.HeldRelativeToAnchor =
					(GClavicleAnchorBind != 0 ? TorsoActorBind : NeutralTorsoTarget)
						.Inverse() * Bind.ParentActorBind;
				UE_LOG(LogParityFK, Display,
					TEXT("[Drive] %s weld anchor on %s: held=%.4f deg, "
					     "bind_vs_neutral=%.4f deg (anchor_bind=%d)"),
					Joint.Bone, *ParentBone.ToString(),
					FMath::RadiansToDegrees(
						Bind.HeldRelativeToAnchor.GetAngle()),
					FMath::RadiansToDegrees(
						TorsoActorBind.AngularDistance(NeutralTorsoTarget)),
					GClavicleAnchorBind);
			}
			Bind.bValid = true;
			Captured.Add(Bone, Bind);

			float AuthoredSpring = 0.0f;
			float AuthoredDamping = 0.0f;
			float AuthoredForceLimit = 0.0f;
			Constraint->GetAngularDriveParams(AuthoredSpring, AuthoredDamping, AuthoredForceLimit);

			double MjcfSpring = 0.0;
			double MjcfDamping = 0.0;
			if (GInjectMjcfGains != 0 && FindMjcfGain(Joint.Bone, MjcfSpring, MjcfDamping))
			{
				FString ArmatureReport = TEXT("armature not applied");
				const FMjcfGain* Mjcf = FindMjcfJoint(Joint.Bone);
				FBodyInstance* Body = Mesh->GetBodyInstance(Bone);
				if (GEmulateArmature != 0 && Mjcf
					&& !ApplyArmature(Body, Mjcf->ArmatureKgM2, ArmatureReport))
				{
					ArmatureReport = TEXT("armature FAILED: no inertia on the body yet");
				}

				Constraint->ProfileInstance.AngularDrive.SetAccelerationMode(
					GTorqueMode == 0);
				Constraint->SetAngularDriveParams(
					MjcfSpring, MjcfDamping, AuthoredForceLimit);

				const FVector Inertia = Body ? Body->GetBodyInertiaTensor() : FVector::ZeroVector;
				const auto Wn = [&](double I) {
					return I > 0.0 ? FMath::Sqrt(MjcfSpring / I) : 0.0;
				};

				{
					const FTransform F1 = Constraint->GetRefFrame(EConstraintFrame::Frame1);
					const FTransform F2 = Constraint->GetRefFrame(EConstraintFrame::Frame2);
					const FQuat Rel = (F2.GetRotation().Inverse() * F1.GetRotation()).GetNormalized();
					UE_LOG(LogParityFK, Display,
						TEXT("[Frames] %s F1 pos (%.4f, %.4f, %.4f) rot %.4f deg about ")
						TEXT("(%.3f, %.3f, %.3f) | F2 pos (%.4f, %.4f, %.4f) rot %.4f deg about ")
						TEXT("(%.3f, %.3f, %.3f) | F1-vs-F2 %.4f deg"),
						Joint.Bone,
						F1.GetLocation().X, F1.GetLocation().Y, F1.GetLocation().Z,
						FMath::RadiansToDegrees(F1.GetRotation().GetAngle()),
						F1.GetRotation().GetRotationAxis().X,
						F1.GetRotation().GetRotationAxis().Y,
						F1.GetRotation().GetRotationAxis().Z,
						F2.GetLocation().X, F2.GetLocation().Y, F2.GetLocation().Z,
						FMath::RadiansToDegrees(F2.GetRotation().GetAngle()),
						F2.GetRotation().GetRotationAxis().X,
						F2.GetRotation().GetRotationAxis().Y,
						F2.GetRotation().GetRotationAxis().Z,
						FMath::RadiansToDegrees(Rel.GetAngle()));
				}

				{
					const FConstraintProfileProperties& Profile = Constraint->ProfileInstance;
					UE_LOG(LogParityFK, Display,
						TEXT("[Limits] %s motion twist=%d swing1=%d swing2=%d  ")
						TEXT("limits twist=%.2f swing1=%.2f swing2=%.2f deg  ")
						TEXT("soft twist=%d (%.0f/%.0f) cone=%d (%.0f/%.0f)  ")
						TEXT("restitution twist=%.2f cone=%.2f  contact twist=%.2f cone=%.2f"),
						Joint.Bone,
						static_cast<int32>(Constraint->GetAngularTwistMotion()),
						static_cast<int32>(Constraint->GetAngularSwing1Motion()),
						static_cast<int32>(Constraint->GetAngularSwing2Motion()),
						Profile.TwistLimit.TwistLimitDegrees,
						Profile.ConeLimit.Swing1LimitDegrees,
						Profile.ConeLimit.Swing2LimitDegrees,
						static_cast<int32>(Profile.TwistLimit.bSoftConstraint),
						Profile.TwistLimit.Stiffness, Profile.TwistLimit.Damping,
						static_cast<int32>(Profile.ConeLimit.bSoftConstraint),
						Profile.ConeLimit.Stiffness, Profile.ConeLimit.Damping,
						Profile.TwistLimit.Restitution, Profile.ConeLimit.Restitution,
						Profile.TwistLimit.ContactDistance, Profile.ConeLimit.ContactDistance);
				}

				// UE scales every drive gain by these (default 1.5); they must be 1.0.
				{
					bool bFoundK = false;
					bool bFoundD = false;
					const float ScaleK = ReadCVarFloat(
						TEXT("p.Chaos.JointConstraint.AngularDriveStiffnessScale"), bFoundK);
					const float ScaleD = ReadCVarFloat(
						TEXT("p.Chaos.JointConstraint.AngularDriveDampingScale"), bFoundD);
					const bool bUnity = bFoundK && bFoundD
						&& FMath::IsNearlyEqual(ScaleK, 1.0f, 1.e-3f)
						&& FMath::IsNearlyEqual(ScaleD, 1.0f, 1.e-3f);
					static bool bScalesReported = false;
					if (!bScalesReported)
					{
						bScalesReported = true;
						UE_LOG(LogParityFK, Display,
							TEXT("[ParityScales] angular drive stiffness x%.4g damping x%.4g -> %s"),
							ScaleK, ScaleD,
							bUnity ? TEXT("ok") : TEXT("NOT UNITY: gains reach the solver scaled"));
					}
				}

				UE_LOG(LogParityFK, Display,
					TEXT("[Drive] %s Kp %.0f -> %.0f, Kd %.0f -> %.0f (MJCF %.0f/%.0f "
					     "N*m/rad x %.0f) acceleration_mode=%d force_limit=%.1f%s\n"
					     "         %s\n"
					     "         wn about body axes: %.1f / %.1f / %.1f rad/s"),
					Joint.Bone, AuthoredSpring, MjcfSpring, AuthoredDamping, MjcfDamping,
					MjcfSpring / GDriveGainScale,
					MjcfDamping / GDriveGainScale, GDriveGainScale,
					static_cast<int32>(
						Constraint->ProfileInstance.AngularDrive.bAccelerationMode),
					AuthoredForceLimit,
					(AuthoredForceLimit <= 0.0f || AuthoredForceLimit >= UE_MAX_FLT)
						? TEXT(" (unclamped)") : TEXT(" (CLAMPED)"),
					*ArmatureReport,
					Wn(Inertia.X), Wn(Inertia.Y), Wn(Inertia.Z));
			}
			else
			{
				UE_LOG(LogParityFK, Display,
					TEXT("[Drive] %s left at authored %.1f/%.1f "
					     "(p.Parity.InjectMjcfGains=%d); the MJCF asks for %.0f/%.0f N*m/rad"),
					Joint.Bone, AuthoredSpring, AuthoredDamping, GInjectMjcfGains,
					MjcfSpring > 0.0 ? MjcfSpring / GDriveGainScale : 0.0,
					MjcfDamping > 0.0 ? MjcfDamping / GDriveGainScale : 0.0);
			}
		}
		Cache.Constraints = MoveTemp(Captured);
		Cache.bCaptured = true;
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

		if (GDriveMode == 0)
		{
			Constraint->SetAngularDriveMode(EAngularDriveMode::SLERP);
			Constraint->SetOrientationDriveSLERP(true);
			Constraint->SetAngularVelocityDriveSLERP(true);
		}
		else
		{
			Constraint->SetOrientationDriveSLERP(false);
			Constraint->SetAngularVelocityDriveSLERP(false);
			Constraint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
			Constraint->SetOrientationDriveTwistAndSwing(true, true);
			Constraint->SetAngularVelocityDriveTwistAndSwing(true, true);
		}
		Constraint->SetAngularOrientationTarget(ConstraintTarget);

		const FQuat Readback = Constraint->GetAngularOrientationTarget().Quaternion();
		OutWorstReadbackDegrees = FMath::Max(
			OutWorstReadbackDegrees,
			FMath::RadiansToDegrees(ConstraintTarget.AngularDistance(Readback)));
	}

	OutError.Reset();
	return true;
}

bool ParityCapturePhysicsState(
	USkeletalMeshComponent* Mesh,
	int64 FrameId,
	int64 TimestampNs,
	FParityPhysicsState& OutState,
	FString& OutError)
{
	OutState = FParityPhysicsState();
	if (!Mesh || FrameId < 0 || TimestampNs < 0)
	{
		OutError = TEXT("mesh/frame/timestamp invalid");
		return false;
	}

	OutState.FrameId = FrameId;
	OutState.TimestampNs = TimestampNs;
	OutState.Bodies.Reserve(NumBodies);

	const FTransform ActorFrame = Mesh->GetOwner()
		? Mesh->GetOwner()->GetActorTransform()
		: FTransform::Identity;

	const FVector RootPosition = ParityUePositionToIsaac(
		ActorFrame.InverseTransformPosition(
			Mesh->GetBoneLocation(ParityRootBoneName())));
	OutState.RootPositionM[0] = RootPosition.X;
	OutState.RootPositionM[1] = RootPosition.Y;
	OutState.RootPositionM[2] = RootPosition.Z;

	const FVector RootWorld =
		ParityUeWorldPositionToIsaac(Mesh->GetBoneLocation(ParityRootBoneName()));
	OutState.RootWorldPositionM[0] = RootWorld.X;
	OutState.RootWorldPositionM[1] = RootWorld.Y;
	OutState.RootWorldPositionM[2] = RootWorld.Z;

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
			static const FQuat RootWorldConventionSnapped(
				0.70710678f, 0.0f, 0.70710678f, 0.0f);
			static const FQuat RootWorldConventionExact(
				0.70642950f, -0.02885777f, 0.70659945f, 0.02935481f);
			const FQuat& RootWorldConvention = (GObsRootExact != 0)
				? RootWorldConventionExact : RootWorldConventionSnapped;
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

	for (int32 BodyIndex = 0; BodyIndex < NumBodies; ++BodyIndex)
	{
		const FName Bone(StateSlots[BodyIndex].Bone);
		if (!Mesh->GetBodyInstance(Bone))
		{
			OutError = FString::Printf(TEXT("missing physics body '%s'"), *Bone.ToString());
			return false;
		}
		const FVector Position = ParityUePositionToIsaac(
			ActorFrame.InverseTransformPosition(Mesh->GetBoneLocation(Bone)));

		const FVector LinearVelocity = ParityUePositionToIsaac(
			ActorFrame.InverseTransformVectorNoScale(
				Mesh->GetPhysicsLinearVelocity(Bone)));
		const FVector ActorAngularVelocity = ActorFrame.InverseTransformVectorNoScale(
			Mesh->GetPhysicsAngularVelocityInRadians(Bone));

		// The axis swap is a reflection, so axial vectors also flip sign.
		const FVector AngularVelocity = -FVector(
			ActorAngularVelocity.Y,
			ActorAngularVelocity.X,
			ActorAngularVelocity.Z);

		// World rows feed the observation; position is the physics body's, as in training, not
		// the bone pivot.
		const FBodyInstance* BodyInst = Mesh->GetBodyInstance(Bone);
		const FVector WorldBodyLocation = BodyInst
			? BodyInst->GetUnrealWorldTransform().GetLocation()
			: Mesh->GetBoneLocation(Bone);
		SetBodyValues(
			OutState.BodiesWorld.AddDefaulted_GetRef(),
			ParityUeWorldPositionToIsaac(WorldBodyLocation),
			ParityUeRotationToIsaac(ActorFrame.GetRotation())
				* RootRelativeRotations[BodyIndex],
			ParityUeWorldPositionToIsaac(Mesh->GetPhysicsLinearVelocity(Bone)),
			ParityUeWorldAxialToIsaac(Mesh->GetPhysicsAngularVelocityInRadians(Bone)));

		SetBodyValues(
			OutState.Bodies.AddDefaulted_GetRef(),
			Position,
			RootRelativeRotations[BodyIndex],
			LinearVelocity,
			AngularVelocity);
	}
	OutError.Reset();
	return true;
}
