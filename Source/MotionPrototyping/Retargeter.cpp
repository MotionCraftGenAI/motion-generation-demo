// Copyright © 2026 MotionCraftGenAI. All rights reserved.

#include "Retargeter.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"


// Main function to load and parse the JSON configuration.
bool URetargeter::LoadRetargetingConfig(const FString& FilePath, USkeletalMeshComponent* SkeletalMesh,
                                        UPoseableMeshComponent* PoseableMesh, FRetargeterConfig& OutConfig)
{
	OutConfig.DestinationBoneMap.Add(FName("pelvis_translation"), FBoneMapping(
		                                 FName("pelvis"), // Source bone name (from CALM)
		                                 FName("pelvis"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 // FQuat(0.5, 0.5, 0.5, -0.5),
		                                 FQuat(0.0, 0.0, 0.7071, 0.7071),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Y"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Z"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("pelvis"), FBoneMapping(
		                                 FName("pelvis"), // Source bone name (from CALM)
		                                 FName("pelvis"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(0.0, 0.0, 1 / sqrt(2.0), 1 / sqrt(2.0)),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("X"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("Y"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Z"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("spine_04"), FBoneMapping(
		                                 FName("torso"), // Source bone name (from CALM)
		                                 FName("spine_04"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(0.5, 0.5, 0.5, -0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("upperarm_l"), FBoneMapping(
		                                 FName("left_upper_arm"), // Source bone name (from CALM)
		                                 FName("upperarm_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, -0.5, 0.5, -0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("lowerarm_l"), FBoneMapping(
		                                 FName("left_lower_arm"), // Source bone name (from CALM)
		                                 FName("lowerarm_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, -0.5, 0.5, -0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("hand_l"), FBoneMapping(
		                                 FName("left_hand"), // Source bone name (from CALM)
		                                 FName("hand_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.7071, 0.0, 0.7071, 0.0),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("Y"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("X"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("upperarm_r"), FBoneMapping(
		                                 FName("right_upper_arm"), // Source bone name (from CALM)
		                                 FName("upperarm_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, 0.5, -0.5, -0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("lowerarm_r"), FBoneMapping(
		                                 FName("right_lower_arm"), // Source bone name (from CALM)
		                                 FName("lowerarm_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, 0.5, -0.5, -0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("hand_r"), FBoneMapping(
		                                 FName("right_hand"), // Source bone name (from CALM)
		                                 FName("hand_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(0.7071, 0.0, 0.7071, 0.0),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("Y"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("X"), // Z axis of destination (target) bone
			                                 true // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("thigh_l"), FBoneMapping(
		                                 FName("left_thigh"), // Source bone name (from CALM)
		                                 FName("thigh_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, -0.5, -0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("calf_l"), FBoneMapping(
		                                 FName("left_shin"), // Source bone name (from CALM)
		                                 FName("calf_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, -0.5, -0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("foot_l"), FBoneMapping(
		                                 FName("left_foot"), // Source bone name (from CALM)
		                                 FName("foot_l"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, -0.5, -0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 true, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 true, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("thigh_r"), FBoneMapping(
		                                 FName("right_thigh"), // Source bone name (from CALM)
		                                 FName("thigh_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, 0.5, 0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("calf_r"), FBoneMapping(
		                                 FName("right_shin"), // Source bone name (from CALM)
		                                 FName("calf_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, 0.5, 0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.DestinationBoneMap.Add(FName("foot_r"), FBoneMapping(
		                                 FName("right_foot"), // Source bone name (from CALM)
		                                 FName("foot_r"), // Destination bone name (in Unreal)
		                                 // Quat rotation to correct for T-pose to point in the default direction (Z up)
		                                 FQuat(-0.5, 0.5, 0.5, 0.5),
		                                 // Correct axis mapping from CALM to Unreal (based on T-pose)
		                                 FAxisMapping(
			                                 FName("Z"), // X axis of destination (target) bone
			                                 false, // Invert X axis
			                                 FName("X"), // Y axis of destination (target) bone
			                                 false, // Invert Y axis
			                                 FName("Y"), // Z axis of destination (target) bone
			                                 false // Invert Z axis
		                                 )
	                                 ));
	OutConfig.SkeletalMesh = SkeletalMesh;
	OutConfig.PoseableMesh = PoseableMesh;
	return true;
}


FConstraintInstance* URetargeter::FindConstraintByName(const FName ConstraintName, USkeletalMeshComponent* SkeletalMesh)
{
	for (FConstraintInstance* Constraint : SkeletalMesh->Constraints)
	{
		if (Constraint->JointName == ConstraintName)
		{
			return Constraint;
		}
	}

	return nullptr;
}


// Low-level function to retarget a single bone.
void URetargeter::RetargetSingleBone(const FName ConstraintName, const FQuat& SourceBoneActorSpaceRotation,
                                     const FRetargeterConfig& Config)
{
	// Find the mapping for the source bone
	const FBoneMapping* Mapping = Config.DestinationBoneMap.Find(ConstraintName);
	if (!Mapping)
	{
		UE_LOG(LogTemp, Log, TEXT("Failed to find name %s"), *ConstraintName.ToString());
		// If no mapping exists, raise error
		return;
	}

	FQuat AbsoluteRotationInTargetReferenceFrame = Mapping->QuatCorrectionFactor * (Mapping->AxisMapping.ApplyMapping(
		SourceBoneActorSpaceRotation));
	FConstraintInstance* Constraint = FindConstraintByName(ConstraintName, Config.SkeletalMesh);

	FName ChildName = Constraint->GetChildBoneName();
	FName ParentName = Constraint->GetParentBoneName();

	FQuat BoneParentAbsQuat = Config.SkeletalMesh->GetSocketTransform(ParentName, RTS_Actor).GetRotation();
	FQuat LocalRotationRelativeToActorReferenceFrame = BoneParentAbsQuat.Inverse() *
		AbsoluteRotationInTargetReferenceFrame;

	FQuat BoneParentQuat = Config.SkeletalMesh->GetBoneQuaternion(ParentName);
	FQuat BoneChildQuat = Config.SkeletalMesh->GetBoneQuaternion(ChildName);

	// Relative rotation of 0 (FQuat::Identity) is when BoneParentQuat == BoneChildQuat.
	// We want to move it by the target relative rotation.
	FQuat BoneChildTargetQuat = BoneParentQuat * LocalRotationRelativeToActorReferenceFrame;

	// Now we need to map the bone rotation to constraint rotation.
	FQuat ConstraintParentQuat = FChaosEngineInterface::GetGlobalPose(Constraint->ConstraintHandle,
	                                                                  EConstraintFrame::Frame2).GetRotation();
	FQuat ConstraintChildQuat = FChaosEngineInterface::GetGlobalPose(Constraint->ConstraintHandle,
	                                                                 EConstraintFrame::Frame1).GetRotation();

	// Transformation from Skeletal Mesh to Constraints frame of reference.
	FQuat BoneToConstraintChildQuat = BoneChildQuat.Inverse() * ConstraintChildQuat;
	// Transform absolute target child bone orientation to constraint frame of reference.
	FQuat ConstraintChildZeroQuat = BoneChildTargetQuat * BoneToConstraintChildQuat;

	// Calculate the final delta between current parent rotation in constraint frame of reference and
	// the target child rotation in constraint frame of reference. (Constraint target is relative rotation).
	FQuat ConstraintTargetOrientationQuat = ConstraintParentQuat.Inverse() * ConstraintChildZeroQuat;
	ConstraintTargetOrientationQuat.Normalize();

	Constraint->SetAngularOrientationTarget(ConstraintTargetOrientationQuat);
}

// Simplified retargeting - applies rotation directly to skeleton (no physics).
void URetargeter::RetargetSingleBoneDirect(const FName BoneName, const FQuat& SourceBoneActorSpaceRotation,
                                           const FRetargeterConfig& Config, const FQuat& ComponentRotation)
{
	// Find the mapping for the bone
	const FBoneMapping* Mapping = Config.DestinationBoneMap.Find(BoneName);
	if (!Mapping)
	{
		UE_LOG(LogTemp, Log, TEXT("Failed to find name %s"), *BoneName.ToString());
		return;
	}

	// Apply simplified retargeting (same as root rotation)
	FQuat RetargetedRotation = Mapping->QuatCorrectionFactor * (Mapping->AxisMapping.ApplyMapping(
		SourceBoneActorSpaceRotation));

	// Apply rotation directly to the poseable mesh bone in world space
	Config.PoseableMesh->SetBoneRotationByName(BoneName, (ComponentRotation * RetargetedRotation).Rotator(),
	                                           EBoneSpaces::WorldSpace);
}