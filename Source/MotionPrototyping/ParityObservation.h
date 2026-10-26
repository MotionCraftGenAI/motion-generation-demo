#pragma once

#include "CoreMinimal.h"
#include "ParityProtocol.h"

inline constexpr int32 ParityObsBodyCount = 15;

// The observation the model was trained on (CALM's compute_humanoid_observations_max).
inline constexpr int32 ParityObsBaseSize =
	1                               // root height
	+ (ParityObsBodyCount - 1) * 3  // body positions relative to the root
	+ ParityObsBodyCount * 6        // body rotations, tangent-normal form
	+ ParityObsBodyCount * 3        // linear velocities
	+ ParityObsBodyCount * 3;       // angular velocities
static_assert(ParityObsBaseSize == 223,
	"the observation layout changed; the policy checkpoint expects 223 base dims");

// Heading task: target direction, speed index.
inline constexpr int32 ParityObsTaskSize = 5;
inline constexpr int32 ParityObsSize = ParityObsBaseSize + ParityObsTaskSize;

inline constexpr int32 ParityDofCount = 28;

double ParityCalcHeading(const double QuatXyzw[4]);

bool ParityBuildObservation(
	const FParityPhysicsState& State,
	const double TargetDirWorldXy[2],
	double TargetLocomotionIndex,
	TArray<double>& OutObservation,
	FString& OutError);

bool ParitySerializeObservation(
	int64 Step,
	const TArray<double>& Observation,
	FString& OutJson,
	FString& OutError);

bool ParityParseActionMessage(
	const FString& Json,
	int64& OutStep,
	TArray<double>& OutDofTargets,
	FString& OutError);

// Actions in [-1, 1] -> joint targets, by the same scale and offset as in training.
void ParityActionsToDofTargets(const TArray<double>& Actions, TArray<double>& OutDofTargets);
