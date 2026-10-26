// Copyright Epic Games, Inc. All Rights Reserved.

#include "MotionPrototypingCharacter.h"
#include "Engine/LocalPlayer.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/Controller.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputActionValue.h"

#include "PhysicalAnimation.h"

DEFINE_LOG_CATEGORY(LogTemplateCharacter);

AMotionPrototypingCharacter::AMotionPrototypingCharacter()
{
	GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;

	GetCharacterMovement()->bOrientRotationToMovement = true;
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 500.0f, 0.0f);

	GetCharacterMovement()->JumpZVelocity = 700.f;
	GetCharacterMovement()->AirControl = 0.35f;
	GetCharacterMovement()->MaxWalkSpeed = 500.f;
	GetCharacterMovement()->MinAnalogWalkSpeed = 20.f;
	GetCharacterMovement()->BrakingDecelerationWalking = 2000.f;
	GetCharacterMovement()->BrakingDecelerationFalling = 1500.0f;

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootComponent);
	CameraBoom->TargetArmLength = 400.0f;
	CameraBoom->bUsePawnControlRotation = true;

	CameraBoom->bEnableCameraLag = true;
	CameraBoom->bEnableCameraRotationLag = true;
	CameraBoom->CameraLagSpeed = 12.0f;
	CameraBoom->CameraRotationLagSpeed = 12.0f;

	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
	FollowCamera->bUsePawnControlRotation = false;

	ParityAnimationComponent = CreateDefaultSubobject<UPhysicalAnimation>(TEXT("ParityAnimationComponent"));
	PrimaryActorTick.AddPrerequisite(ParityAnimationComponent, ParityAnimationComponent->PrimaryComponentTick);
}

void AMotionPrototypingCharacter::BeginPlay()
{
	Super::BeginPlay();

	if (ParityAnimationComponent == nullptr)
	{
		ParityAnimationComponent = FindComponentByClass<UPhysicalAnimation>();
		UE_LOG(LogTemplateCharacter, Warning,
			TEXT("ParityAnimationComponent was None; re-resolved from the actor: %s"),
			ParityAnimationComponent ? TEXT("found") : TEXT("still missing"));
	}
}

void AMotionPrototypingCharacter::NotifyControllerChanged()
{
	Super::NotifyControllerChanged();

	if (APlayerController* PlayerController = Cast<APlayerController>(Controller))
	{
		if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PlayerController->GetLocalPlayer()))
		{
			Subsystem->AddMappingContext(DefaultMappingContext, 0);
		}
	}
}

void AMotionPrototypingCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	if (UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(PlayerInputComponent)) {
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Started, this, &ACharacter::Jump);
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Completed, this, &ACharacter::StopJumping);

		if (MoveAction)
		{
			EnhancedInputComponent->BindAction(MoveAction, ETriggerEvent::Triggered, this,
				&AMotionPrototypingCharacter::Move);
		}
		else
		{
			UE_LOG(LogTemplateCharacter, Warning,
				TEXT("MoveAction is not set on %s; the heading command cannot be steered"),
				*GetNameSafe(this));
		}

		EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered, this, &AMotionPrototypingCharacter::Look);
	}
	else
	{
		UE_LOG(LogTemplateCharacter, Error, TEXT("'%s' Failed to find an Enhanced Input component! This template is built to use the Enhanced Input system. If you intend to use the legacy system, then you will need to update this C++ file."), *GetNameSafe(this));
	}
}

void AMotionPrototypingCharacter::Move(const FInputActionValue& Value)
{
	// Camera-relative input steers the heading the model walks in; physics moves the pawn.
	const FVector2D Input = Value.Get<FVector2D>();
	if (Input.IsNearlyZero() || Controller == nullptr || ParityAnimationComponent == nullptr)
	{
		return;
	}

	const FRotator YawOnly(0.0, Controller->GetControlRotation().Yaw, 0.0);
	const FVector Forward = FRotationMatrix(YawOnly).GetUnitAxis(EAxis::X);
	const FVector Right = FRotationMatrix(YawOnly).GetUnitAxis(EAxis::Y);
	ParityAnimationComponent->SetDirectHeadingFromWorldDirection(
		Forward * Input.Y + Right * Input.X);
}

void AMotionPrototypingCharacter::Look(const FInputActionValue& Value)
{
	FVector2D LookAxisVector = Value.Get<FVector2D>();

	if (Controller != nullptr)
	{
		AddControllerYawInput(LookAxisVector.X);
		AddControllerPitchInput(LookAxisVector.Y);
	}
}
