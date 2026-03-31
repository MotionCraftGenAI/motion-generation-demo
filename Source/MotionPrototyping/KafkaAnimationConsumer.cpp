// Copyright © 2026 MotionCraftGenAI. All rights reserved.
// Kafka Animation Consumer - Implementation

#include "KafkaAnimationConsumer.h"
#include "Misc/DateTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY(LogKafkaAnimation);

UKafkaAnimationConsumer::UKafkaAnimationConsumer()
	: bProducerEnabled(false)
	  , bConsumerEnabled(false)
	  , bConsumerConnected(false)
	  , bConsumerConsuming(false)
	  , BufferReadIndex(0)
	  , BufferWriteIndex(0)
{
}

void UKafkaAnimationConsumer::Initialize(const FKafkaAnimationConfig& Config)
{
	CurrentConfig = Config;

	EasyKafka = GEngine->GetEngineSubsystem<UEasyKafkaSubsystem>()->GetEasyKafka();
	if (!EasyKafka.IsValid())
	{
		UE_LOG(LogKafkaAnimation, Error, TEXT("Failed to get EasyKafka subsystem"));
		return;
	}

	// Initialize buffer
	BufferReadIndex = 0;
	BufferWriteIndex = 0;
	FrameBuffer.SetNum(Config.BufferCapacity);

	// Determine which components to enable based on topic configuration
	bProducerEnabled = !Config.HeadingTopic.IsEmpty();
	bConsumerEnabled = !Config.DataTopic.IsEmpty();

	if (bProducerEnabled)
	{
		InitializeProducer();
	}

	if (bConsumerEnabled)
	{
		InitializeConsumer();
	}
}

void UKafkaAnimationConsumer::Shutdown()
{
	if (!EasyKafka.IsValid())
	{
		return;
	}

	if (bProducerEnabled)
	{
		ShutdownProducer();
	}

	if (bConsumerEnabled)
	{
		ShutdownConsumer();
	}

	EasyKafka->ShutdownModule();
	UE_LOG(LogKafkaAnimation, Log, TEXT("Kafka module shut down"));
}

void UKafkaAnimationConsumer::InitializeProducer()
{
	EasyKafka->GetProducer()->StartupModule();
	EasyKafka->GetProducer()->CreateProducer(
		CurrentConfig.BootstrapServers,
		CurrentConfig.Username,
		CurrentConfig.Password
	);

	UE_LOG(LogKafkaAnimation, Log, TEXT("Kafka producer initialized. Server: %s"), *CurrentConfig.BootstrapServers);
}

void UKafkaAnimationConsumer::InitializeConsumer()
{
	TMap<EKafkaConsumerConfig, FString> KafkaConfig;
	KafkaConfig.Add(EKafkaConsumerConfig::ENABLE_AUTO_COMMIT, TEXT("false"));
	KafkaConfig.Add(EKafkaConsumerConfig::GROUP_ID, CurrentConfig.ConsumerGroup);
	KafkaConfig.Add(EKafkaConsumerConfig::AUTO_OFFSET_RESET, TEXT("latest"));

	EKafkaLogLevel KafkaLogLevel = CurrentConfig.bEnableLogging ? EKafkaLogLevel::DEBUG : EKafkaLogLevel::ERR;

	EasyKafka->GetConsumer()->CreateConsumer(
		CurrentConfig.BootstrapServers,
		CurrentConfig.Username,
		CurrentConfig.Password,
		KafkaConfig,
		static_cast<int>(KafkaLogLevel)
	);

	// Bind message callback
	EasyKafka->GetConsumer()->OnNewMessage().AddUObject(
		this,
		&UKafkaAnimationConsumer::OnMessageReceived
	);

	EasyKafka->StartupModule();

	// Subscribe to topic
	EasyKafka->GetConsumer()->Subscribe({CurrentConfig.DataTopic});

	// Start consuming
	EasyKafka->GetConsumer()->StartConsuming();

	bConsumerConnected = true;
	bConsumerConsuming = true;

	UE_LOG(LogKafkaAnimation, Log,
	       TEXT("Kafka consumer initialized. Server: %s, Topic: %s, Group: %s"),
	       *CurrentConfig.BootstrapServers, *CurrentConfig.DataTopic, *CurrentConfig.ConsumerGroup);
}

void UKafkaAnimationConsumer::ShutdownProducer()
{
	EasyKafka->GetProducer()->ShutdownModule();
	UE_LOG(LogKafkaAnimation, Log, TEXT("Kafka producer shut down"));
}

void UKafkaAnimationConsumer::ShutdownConsumer()
{
	if (!bConsumerConnected)
	{
		return;
	}

	EasyKafka->GetConsumer()->StopConsuming();
	EasyKafka->GetConsumer()->ShutdownModule();
	bConsumerConnected = false;

	UE_LOG(LogKafkaAnimation, Log, TEXT("Kafka consumer shut down"));
}

bool UKafkaAnimationConsumer::IsProducerInitialized() const
{
	return EasyKafka.IsValid() && EasyKafka->GetProducer().IsValid();
}

bool UKafkaAnimationConsumer::IsConsumerConnected() const
{
	return bConsumerConnected;
}

void UKafkaAnimationConsumer::ProduceMessage(const FString& Topic, const FString& Message) const
{
	if (!IsProducerInitialized())
	{
		UE_LOG(LogKafkaAnimation, Warning,
		       TEXT("Kafka Producer not initialized. Cannot send message to topic '%s': %s"),
		       *Topic, *Message);
		return;
	}

	EasyKafka->GetProducer()->ProduceRecord(Topic, Message);
}

void UKafkaAnimationConsumer::SendHeadingVector2D(const FVector2D& HeadingVector) const
{
	if (!IsProducerInitialized())
	{
		UE_LOG(LogKafkaAnimation, Warning, TEXT("Kafka Producer not initialized. Cannot send heading vector."));
		return;
	}

	const TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
	JsonObject->SetNumberField(TEXT("x"), HeadingVector.X);
	JsonObject->SetNumberField(TEXT("y"), HeadingVector.Y);

	FString JsonString;
	if (const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
		FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer))
	{
		ProduceMessage(CurrentConfig.HeadingTopic, JsonString);
	}
	else
	{
		UE_LOG(LogKafkaAnimation, Error, TEXT("Failed to serialize heading vector to JSON."));
	}
}

void UKafkaAnimationConsumer::OnMessageReceived(const TArray<FConsumerRecord>& Messages)
{
	FScopeLock Lock(&BufferCriticalSection);

	const FConsumerRecord Message = Messages.Last();
	if (FAnimationFrameData FrameData; ParseAnimationFrameFromJson(Message.Value, FrameData))
	{
		const int32 NextWriteIndex = (BufferWriteIndex + 1) % CurrentConfig.BufferCapacity;

		// If buffer is full, advance read index to drop oldest frame
		if (NextWriteIndex == BufferReadIndex)
		{
			BufferReadIndex = (BufferReadIndex + 1) % CurrentConfig.BufferCapacity;
		}

		// Write to buffer
		FrameBuffer[BufferWriteIndex] = FrameData;
		UE_LOG(LogKafkaAnimation, Log, TEXT("[%f] Received Kafka message for step: %d"),
		       (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds(), FrameData.StepNumber);
		BufferWriteIndex = NextWriteIndex;
	}
}

bool UKafkaAnimationConsumer::GetNextFrame(FAnimationFrameData& OutFrameData)
{
	FScopeLock Lock(&BufferCriticalSection);

	// Check if buffer is empty
	if (BufferReadIndex == BufferWriteIndex)
	{
		return false;
	}

	// Read from buffer
	OutFrameData = FrameBuffer[BufferReadIndex];
	BufferReadIndex = (BufferReadIndex + 1) % CurrentConfig.BufferCapacity;

	return true;
}

int32 UKafkaAnimationConsumer::GetBufferFillLevel() const
{
	if (BufferWriteIndex >= BufferReadIndex)
	{
		return BufferWriteIndex - BufferReadIndex;
	}
	// Wrapped around
	return (CurrentConfig.BufferCapacity - BufferReadIndex) + BufferWriteIndex;
}

bool UKafkaAnimationConsumer::ParseAnimationFrameFromJson(const FString& JsonString, FAnimationFrameData& OutFrameData)
{
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
	TSharedPtr<FJsonObject> JsonObject;

	if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
	{
		UE_LOG(LogKafkaAnimation, Error, TEXT("Failed to deserialize JSON: %s"), *JsonString);
		return false;
	}

	const TSharedPtr<FJsonObject> Data = JsonObject->GetObjectField(TEXT("d"));
	if (!Data.IsValid())
	{
		UE_LOG(LogKafkaAnimation, Error, TEXT("JSON 'd' object not found or invalid."));
		return false;
	}

	// Parse 'q' (quaternions)
	if (const TArray<TSharedPtr<FJsonValue>>* QuaternionArrayPtr = nullptr;
		Data->TryGetArrayField(TEXT("q"), QuaternionArrayPtr))
	{
		for (const TSharedPtr<FJsonValue>& QuatValue : *QuaternionArrayPtr)
		{
			if (const TArray<TSharedPtr<FJsonValue>>* QuatComponentsPtr = nullptr;
				QuatValue->TryGetArray(QuatComponentsPtr))
			{
				if (QuatComponentsPtr->Num() >= 4)
				{
					FQuat Quat(
						(*QuatComponentsPtr)[0]->AsNumber(),
						(*QuatComponentsPtr)[1]->AsNumber(),
						(*QuatComponentsPtr)[2]->AsNumber(),
						(*QuatComponentsPtr)[3]->AsNumber()
					);
					OutFrameData.Quaternions.Add(Quat);
				}
			}
		}
	}
	else
	{
		UE_LOG(LogKafkaAnimation, Warning, TEXT("JSON 'q' array not found or invalid."));
	}

	// Parse 'r' (root position)
	if (const TArray<TSharedPtr<FJsonValue>>* RootPositionArrayPtr = nullptr;
		Data->TryGetArrayField(TEXT("r"), RootPositionArrayPtr))
	{
		if (RootPositionArrayPtr->Num() >= 3)
		{
			OutFrameData.RootPosition = FVector(
				(*RootPositionArrayPtr)[0]->AsNumber(),
				(*RootPositionArrayPtr)[1]->AsNumber(),
				(*RootPositionArrayPtr)[2]->AsNumber()
			);
		}
		else
		{
			UE_LOG(LogKafkaAnimation, Warning, TEXT("JSON 'r' array found but does not have 3 components."));
		}
	}
	else
	{
		UE_LOG(LogKafkaAnimation, Warning, TEXT("JSON 'r' array not found or invalid."));
	}

	// Parse 's' (step number)
	if (int32 StepNum; Data->TryGetNumberField(TEXT("s"), StepNum))
	{
		OutFrameData.StepNumber = StepNum;
	}
	else
	{
		UE_LOG(LogKafkaAnimation, Warning, TEXT("JSON 's' field not found or invalid."));
	}

	return OutFrameData.Quaternions.Num() > 0 || OutFrameData.RootPosition != FVector::ZeroVector || OutFrameData.
		StepNumber != 0;
}
