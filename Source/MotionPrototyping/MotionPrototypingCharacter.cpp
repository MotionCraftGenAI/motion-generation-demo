// Copyright Epic Games, Inc. All Rights Reserved.
// Modifications Copyright © 2026 MotionCraftGenAI. All rights reserved.

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

//////////////////////////////////////////////////////////////////////////
// AMotionPrototypingCharacter

AMotionPrototypingCharacter::AMotionPrototypingCharacter()
{
	// Set size for collision capsule
	GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

	// Don't rotate when the controller rotates. Let that just affect the camera.
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;

	// Configure character movement
	GetCharacterMovement()->bOrientRotationToMovement = true; // Character moves in the direction of input...	
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 500.0f, 0.0f); // ...at this rotation rate

	// Note: For faster iteration times these variables, and many more, can be tweaked in the Character Blueprint
	// instead of recompiling to adjust them
	GetCharacterMovement()->JumpZVelocity = 700.f;
	GetCharacterMovement()->AirControl = 0.35f;
	GetCharacterMovement()->MaxWalkSpeed = 500.f;
	GetCharacterMovement()->MinAnalogWalkSpeed = 20.f;
	GetCharacterMovement()->BrakingDecelerationWalking = 2000.f;
	GetCharacterMovement()->BrakingDecelerationFalling = 1500.0f;

	// Create a camera boom (pulls in towards the player if there is a collision)
	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootComponent);
	CameraBoom->TargetArmLength = 400.0f; // The camera follows at this distance behind the character	
	CameraBoom->bUsePawnControlRotation = true; // Rotate the arm based on the controller

	// Create a follow camera
	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
	// Attach the camera to the end of the boom and let the boom adjust to match the controller orientation
	FollowCamera->bUsePawnControlRotation = false; // Camera does not rotate relative to arm

	// Note: The skeletal mesh and anim blueprint references on the Mesh component (inherited from Character)
	// are set in the derived blueprint asset named ThirdPersonCharacter (to avoid direct content references in C++)

	PhysicalAnimationComponent = CreateDefaultSubobject<UPhysicalAnimation>(TEXT("PhysicalAnimationComponent"));
}

void AMotionPrototypingCharacter::BeginPlay()
{
	Super::BeginPlay();

	UE_LOG(LogTemplateCharacter, Log, TEXT("PhysicalAnimationComponent from constructor: %p"),
	       PhysicalAnimationComponent);

	// Find the actual component that was created (constructor pointer may be wrong due to Blueprint/CDO issues)
	TArray<UPhysicalAnimation*> FoundComponents;
	GetComponents<UPhysicalAnimation>(FoundComponents);
	UE_LOG(LogTemplateCharacter, Log, TEXT("Found %d PhysicalAnimation components on this actor"),
	       FoundComponents.Num());
	for (UPhysicalAnimation* Comp : FoundComponents)
	{
		UE_LOG(LogTemplateCharacter, Log, TEXT("  - Component: %p"), Comp);
		PhysicalAnimationComponent = Comp; // Use the last one found (or you could add logic to pick the right one)
	}
}

void AMotionPrototypingCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (PhysicalAnimationComponent)
	{
		UE_LOG(LogTemplateCharacter, Log, TEXT("Delta R2: %s (instance: %p)"),
		       *PhysicalAnimationComponent->GetRootPositionDelta().ToString(), PhysicalAnimationComponent);
		float RootDeltaScale = 98.69f / 0.88f;
		const FVector DeltaMovement = PhysicalAnimationComponent->GetRootPositionDelta() * RootDeltaScale;
		UE_LOG(LogTemplateCharacter, Log, TEXT("%f, %f, %f, %f"), DeltaMovement.X, DeltaMovement.Y, DeltaMovement.Z,
		       RootDeltaScale);
		if (!DeltaMovement.IsNearlyZero())
		{
			FVector UpdatedDelta = FVector(DeltaMovement.Y + GetActorLocation().X,
			                               DeltaMovement.X + GetActorLocation().Y,
			                               GetActorLocation().Z);

			SetActorLocation(UpdatedDelta);

			const FQuat RootRotation = PhysicalAnimationComponent->GetRootRotation();
			const FRotator TargetRotation = RootRotation.Rotator();

			// Only use Yaw (ignore pitch and roll for character rotation)
			const FRotator CurrentRotation = GetActorRotation();
			UE_LOG(LogTemplateCharacter, Log, TEXT("%s; %s"), *CurrentRotation.Quaternion().ToString(),
			       *RootRotation.ToString());
			const FRotator NewRotation = FRotator(
				TargetRotation.Pitch, // Keep current pitch
				TargetRotation.Yaw, // Use AI's yaw
				TargetRotation.Roll // Keep current roll
			);

			SetActorRotation(NewRotation);
		}
	}
}


//////////////////////////////////////////////////////////////////////////
// Input

void AMotionPrototypingCharacter::NotifyControllerChanged()
{
	Super::NotifyControllerChanged();

	// Add Input Mapping Context
	if (APlayerController* PlayerController = Cast<APlayerController>(Controller))
	{
		if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<
			UEnhancedInputLocalPlayerSubsystem>(PlayerController->GetLocalPlayer()))
		{
			Subsystem->AddMappingContext(DefaultMappingContext, 0);
		}
	}
}

void AMotionPrototypingCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	// Set up action bindings
	if (UEnhancedInputComponent* EnhancedInputComponent = Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		// Jumping
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Started, this, &ACharacter::Jump);
		EnhancedInputComponent->BindAction(JumpAction, ETriggerEvent::Completed, this, &ACharacter::StopJumping);

		// Moving
		EnhancedInputComponent->BindAction(MoveAction, ETriggerEvent::Triggered, this,
		                                   &AMotionPrototypingCharacter::Move);

		// Looking
		EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered, this,
		                                   &AMotionPrototypingCharacter::Look);
	}
	else
	{
		UE_LOG(LogTemplateCharacter, Error,
		       TEXT(
			       "'%s' Failed to find an Enhanced Input component! This template is built to use the Enhanced Input system. If you intend to use the legacy system, then you will need to update this C++ file."
		       ), *GetNameSafe(this));
	}
}

void AMotionPrototypingCharacter::Move(const FInputActionValue& Value)
{
	// 1. Validate the component exists
	if (PhysicalAnimationComponent == nullptr)
	{
		UE_LOG(LogTemplateCharacter, Error, TEXT("PhysicalAnimationComponent is NULL in Move!"));
		return;
	}

	if (Controller != nullptr)
	{
		const FVector2D InputVector = Value.Get<FVector2D>();

		const FRotator Rotation = Controller->GetControlRotation();
		const FRotator YawRotation(0, Rotation.Yaw, 0);

		const FVector ForwardDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
		const FVector RightDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);

		const FVector WorldDirection = (ForwardDirection * InputVector.Y) + (RightDirection * InputVector.X);
		PhysicalAnimationComponent->SendHeadingVector2D(FVector2D(WorldDirection.Y, WorldDirection.X));
	}
}

void AMotionPrototypingCharacter::Look(const FInputActionValue& Value)
{
	// input is a Vector2D
	FVector2D LookAxisVector = Value.Get<FVector2D>();

	if (Controller != nullptr)
	{
		// add yaw and pitch input to controller
		AddControllerYawInput(LookAxisVector.X);
		AddControllerPitchInput(LookAxisVector.Y);
	}
}
