#pragma once

#include "CoreMinimal.h"

// Position xyz, quaternion xyzw, linear velocity xyz, angular velocity xyz.
struct FParityBodyState
{
	double Values[13] = {};
};

struct FParityPhysicsState
{
	int64 FrameId = -1;
	int64 TimestampNs = -1;
	double RootPositionM[3] = {};
	double RootWorldPositionM[3] = {};

	// World frame, absolute rotations: what the observation is built from.
	TArray<FParityBodyState> BodiesWorld;
	// Actor frame, root-relative rotations.
	TArray<FParityBodyState> Bodies;
};
