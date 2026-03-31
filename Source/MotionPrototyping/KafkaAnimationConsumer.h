// Copyright © 2026 MotionCraftGenAI. All rights reserved.
// Kafka Animation Consumer - Handles Kafka message consumption for animation data

#pragma once

#include "CoreMinimal.h"
#include "EasyKafkaSubsystem.h"
#include "KafkaConsumer/Public/KafkaConsumer.h"
#include "KafkaAnimationConsumer.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogKafkaAnimation, Log, All);

/**
 * Represents a single frame of animation data received from Kafka
 */
USTRUCT(BlueprintType)
struct FAnimationFrameData
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	TArray<FQuat> Quaternions;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	FVector RootPosition;

	UPROPERTY(BlueprintReadOnly, Category = "Animation")
	int32 StepNumber;

	FAnimationFrameData() : RootPosition(FVector::ZeroVector), StepNumber(0)
	{
	}
};

/**
 * Configuration for the Kafka Animation Consumer
 */
USTRUCT(BlueprintType)
struct FKafkaAnimationConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString BootstrapServers = TEXT("localhost:9092");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString Username = TEXT("");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString Password = TEXT("");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString DataTopic = TEXT("motion-animation-data");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString HeadingTopic = TEXT("humanoid-heading-commands");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	FString ConsumerGroup = TEXT("motion-animation-consumer");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kafka")
	bool bEnableLogging = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Buffer")
	int32 BufferCapacity = 12;
};

/**
 * Manages Kafka connection and animation frame buffering
 */
UCLASS(BlueprintType)
class MOTIONPROTOTYPING_API UKafkaAnimationConsumer : public UObject
{
	GENERATED_BODY()

public:
	UKafkaAnimationConsumer();

	/**
	 * Initialize Kafka connections based on configuration
	 * @param Config The Kafka configuration settings
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	void Initialize(const FKafkaAnimationConfig& Config);

	/**
	 * Shutdown all Kafka connections
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	void Shutdown();

	/**
	 * Check if the Kafka producer is initialized and ready
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	bool IsProducerInitialized() const;

	/**
	 * Check if the Kafka consumer is connected
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	bool IsConsumerConnected() const;

	/**
	 * Send a message to a Kafka topic
	 * @param Topic The topic to send to
	 * @param Message The message content
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	void ProduceMessage(const FString& Topic, const FString& Message) const;

	/**
	 * Send a 2D heading vector to the configured heading topic
	 * @param HeadingVector The heading vector to send
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	void SendHeadingVector2D(const FVector2D& HeadingVector) const;

	/**
	 * Get the next animation frame from the buffer
	 * @param OutFrameData The frame data if available
	 * @return True if a frame was available, false if buffer was empty
	 */
	UFUNCTION(BlueprintCallable, Category = "Buffer")
	bool GetNextFrame(FAnimationFrameData& OutFrameData);

	/**
	 * Get the current buffer fill level
	 */
	UFUNCTION(BlueprintCallable, Category = "Buffer")
	int32 GetBufferFillLevel() const;

	/**
	 * Parse animation frame data from a JSON string
	 * @param JsonString The JSON string to parse
	 * @param OutFrameData The parsed frame data
	 * @return True if parsing succeeded
	 */
	UFUNCTION(BlueprintCallable, Category = "Kafka")
	static bool ParseAnimationFrameFromJson(const FString& JsonString, FAnimationFrameData& OutFrameData);

private:
	void InitializeProducer();
	void InitializeConsumer();
	void ShutdownProducer();
	void ShutdownConsumer();

	void OnMessageReceived(const TArray<FConsumerRecord>& Messages);

	TSharedPtr<FEasyKafkaModule> EasyKafka;

	FKafkaAnimationConfig CurrentConfig;

	// Connection state
	bool bProducerEnabled;
	bool bConsumerEnabled;
	bool bConsumerConnected;
	bool bConsumerConsuming;

	// Buffer management
	FCriticalSection BufferCriticalSection;
	TArray<FAnimationFrameData> FrameBuffer;
	int32 BufferReadIndex;
	int32 BufferWriteIndex;
};
