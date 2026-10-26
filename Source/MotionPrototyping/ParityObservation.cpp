#include "ParityObservation.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
#include "ParityPdAction.inl"

	static_assert(ParityPdDofCount == ParityDofCount,
		"the generated PD table and ParityDofCount disagree about the rig's DoF count");

	void IsaacQuatRotate(const FQuat& Q, const double V[3], double Out[3])
	{
		const FVector Rotated = Q.RotateVector(FVector(V[0], V[1], V[2]));
		Out[0] = Rotated.X;
		Out[1] = Rotated.Y;
		Out[2] = Rotated.Z;
	}

	void IsaacQuatToTanNorm(const FQuat& Q, double Out[6])
	{
		const double RefTan[3] = { 1.0, 0.0, 0.0 };
		const double RefNorm[3] = { 0.0, 0.0, 1.0 };
		IsaacQuatRotate(Q, RefTan, Out);
		IsaacQuatRotate(Q, RefNorm, Out + 3);
	}
}

double ParityCalcHeading(const double QuatXyzw[4])
{
	const FQuat Q(QuatXyzw[0], QuatXyzw[1], QuatXyzw[2], QuatXyzw[3]);
	const double RefDir[3] = { 1.0, 0.0, 0.0 };
	double RotDir[3];
	IsaacQuatRotate(Q, RefDir, RotDir);
	return FMath::Atan2(RotDir[1], RotDir[0]);
}

bool ParityBuildObservation(
	const FParityPhysicsState& State,
	const double TargetDirWorldXy[2],
	double TargetLocomotionIndex,
	TArray<double>& OutObservation,
	FString& OutError)
{
	if (State.BodiesWorld.Num() != ParityObsBodyCount)
	{
		OutError = FString::Printf(
			TEXT("expected %d world body rows, got %d"),
			ParityObsBodyCount, State.BodiesWorld.Num());
		return false;
	}

	const double* Root = State.BodiesWorld[0].Values;
	const double RootPos[3] = { Root[0], Root[1], Root[2] };

	const double RootQuat[4] = { Root[3], Root[4], Root[5], Root[6] };
	const double Heading = ParityCalcHeading(RootQuat);
	// Everything is expressed in the root's heading frame, so the observation ignores yaw.
	const FQuat HeadingInv(
		0.0, 0.0, FMath::Sin(-0.5 * Heading), FMath::Cos(-0.5 * Heading));

	OutObservation.Reset(ParityObsSize);

	OutObservation.Add(RootPos[2]);

	for (int32 Body = 1; Body < ParityObsBodyCount; ++Body)
	{
		const double* Row = State.BodiesWorld[Body].Values;
		const double Delta[3] = {
			Row[0] - RootPos[0], Row[1] - RootPos[1], Row[2] - RootPos[2] };
		double Local[3];
		IsaacQuatRotate(HeadingInv, Delta, Local);
		OutObservation.Add(Local[0]);
		OutObservation.Add(Local[1]);
		OutObservation.Add(Local[2]);
	}

	for (int32 Body = 0; Body < ParityObsBodyCount; ++Body)
	{
		const double* Row = State.BodiesWorld[Body].Values;

		const FQuat BodyQuat(Row[3], Row[4], Row[5], Row[6]);

		const FQuat Local = HeadingInv * BodyQuat;
		double TanNorm[6];
		IsaacQuatToTanNorm(Local, TanNorm);
		for (int32 Element = 0; Element < 6; ++Element)
		{
			OutObservation.Add(TanNorm[Element]);
		}
	}

	for (int32 Offset : { 7, 10 })
	{
		for (int32 Body = 0; Body < ParityObsBodyCount; ++Body)
		{
			const double* Row = State.BodiesWorld[Body].Values;
			const double Vel[3] = { Row[Offset], Row[Offset + 1], Row[Offset + 2] };
			double Local[3];
			IsaacQuatRotate(HeadingInv, Vel, Local);
			OutObservation.Add(Local[0]);
			OutObservation.Add(Local[1]);
			OutObservation.Add(Local[2]);
		}
	}

	if (OutObservation.Num() != ParityObsBaseSize)
	{
		OutError = FString::Printf(
			TEXT("built %d base elements, expected %d"),
			OutObservation.Num(), ParityObsBaseSize);
		return false;
	}

	double DirNorm = FMath::Sqrt(
		TargetDirWorldXy[0] * TargetDirWorldXy[0]
		+ TargetDirWorldXy[1] * TargetDirWorldXy[1]);
	if (DirNorm < UE_DOUBLE_SMALL_NUMBER)
	{
		DirNorm = 1.0;
	}
	const double Dir3[3] = {
		TargetDirWorldXy[0] / DirNorm, TargetDirWorldXy[1] / DirNorm, 0.0 };
	double LocalDir[3];
	IsaacQuatRotate(HeadingInv, Dir3, LocalDir);

	// Task: target direction, speed index, facing direction (the demo commands both the same).
	OutObservation.Add(LocalDir[0]);
	OutObservation.Add(LocalDir[1]);
	OutObservation.Add(TargetLocomotionIndex);
	OutObservation.Add(LocalDir[0]);
	OutObservation.Add(LocalDir[1]);

	if (OutObservation.Num() != ParityObsSize)
	{
		OutError = FString::Printf(
			TEXT("built %d elements, expected %d"), OutObservation.Num(), ParityObsSize);
		return false;
	}
	return true;
}

bool ParitySerializeObservation(
	int64 Step,
	const TArray<double>& Observation,
	FString& OutJson,
	FString& OutError)
{
	if (Observation.Num() != ParityObsSize)
	{
		OutError = FString::Printf(
			TEXT("refusing to publish %d elements, expected %d"),
			Observation.Num(), ParityObsSize);
		return false;
	}

	TStringBuilder<8192> Builder;

	Builder << TEXT("{\"step\":") << Step;
	Builder << TEXT(",\"observations\":[[");
	for (int32 Index = 0; Index < Observation.Num(); ++Index)
	{
		if (Index > 0)
		{
			Builder << TEXT(",");
		}
		// Seven significant digits, as in the reference run the model was verified against
		// (an exact float32 round trip would need nine).
		Builder.Appendf(TEXT("%.7g"), Observation[Index]);
	}
	Builder << TEXT("]]}");
	OutJson = Builder.ToString();
	return true;
}

void ParityActionsToDofTargets(const TArray<double>& Actions, TArray<double>& OutDofTargets)
{
	const int32 Count = FMath::Min(Actions.Num(), ParityDofCount);
	OutDofTargets.SetNumZeroed(ParityDofCount);
	for (int32 Index = 0; Index < Count; ++Index)
	{
		OutDofTargets[Index] =
			GPdActionOffset[Index] + GPdActionScale[Index] * Actions[Index];
	}
}

bool ParityParseActionMessage(
	const FString& Json,
	int64& OutStep,
	TArray<double>& OutDofTargets,
	FString& OutError)
{
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	TSharedPtr<FJsonObject> Object;
	if (!FJsonSerializer::Deserialize(Reader, Object) || !Object.IsValid())
	{
		OutError = TEXT("action message is not valid JSON");
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>* Rows = nullptr;
	if (!Object->TryGetArrayField(TEXT("actions"), Rows) || Rows->Num() == 0)
	{
		OutError = TEXT("action message has no 'actions' array");
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>* Row = nullptr;
	if (!(*Rows)[0]->TryGetArray(Row))
	{
		OutError = TEXT("actions[0] is not an array");
		return false;
	}
	if (Row->Num() != ParityDofCount)
	{
		OutError = FString::Printf(
			TEXT("actions[0] has %d elements, expected %d"), Row->Num(), ParityDofCount);
		return false;
	}

	TArray<double> Actions;
	Actions.Reserve(ParityDofCount);
	for (const TSharedPtr<FJsonValue>& Value : *Row)
	{
		Actions.Add(Value->AsNumber());
	}
	ParityActionsToDofTargets(Actions, OutDofTargets);

	double Step = 0.0;
	if (Object->TryGetNumberField(TEXT("step"), Step))
	{
		OutStep = static_cast<int64>(Step);
	}
	else
	{
		OutStep = -1;
	}
	return true;
}
