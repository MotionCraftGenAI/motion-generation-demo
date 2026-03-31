// Copyright Epic Games, Inc. All Rights Reserved.

#include "MotionPrototypingGameMode.h"
#include "MotionPrototypingCharacter.h"
#include "UObject/ConstructorHelpers.h"

AMotionPrototypingGameMode::AMotionPrototypingGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(
		TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != nullptr)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
