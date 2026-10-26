// Copyright Epic Games, Inc. All Rights Reserved.

#include "MotionPrototypingGameMode.h"
#include "MotionPrototypingCharacter.h"
#include "UObject/ConstructorHelpers.h"

AMotionPrototypingGameMode::AMotionPrototypingGameMode()
{
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
