#pragma once

#include "CoreMinimal.h"

// Position xyz, quaternion xyzw, linear velocity xyz, angular velocity xyz.
struct FParityBodyState
{
	double Values[13] = {};
};

// The model's 15 bodies in the world frame and the model's axes; the observation is built from it.
struct FParityPhysicsState
{
	TArray<FParityBodyState> BodiesWorld;
};
