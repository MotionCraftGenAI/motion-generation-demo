// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MotionPrototyping : ModuleRules
{
	public MotionPrototyping(ReadOnlyTargetRules Target) : base(Target)
	{
		if (Target.Platform == UnrealTargetPlatform.Win64)
			bUseRTTI = true;

		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new[]
		{
			"Chaos", "Core", "CoreUObject", "Engine", "InputCore", "Json", "JsonUtilities", "PhysicsCore",
			"EnhancedInput"
		});

		PrivateDependencyModuleNames.AddRange(new[]
		{
			"EasyKafka",
			"KafkaLib",
			"KafkaConsumer",
			"KafkaProducer",
			"KafkaAdmin"
		});

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}