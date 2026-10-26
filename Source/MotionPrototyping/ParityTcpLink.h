#pragma once

#include "CoreMinimal.h"

class FSocket;

// Client side of the model socket: 4-byte big-endian length, then UTF-8 JSON.
struct FParityTcpLink
{
	bool Connect(const FString& HostPort, FString& OutError);
	bool Send(const FString& Payload);
	bool Recv(FString& OutPayload, int32 TimeoutMs);
	void Close();
	bool IsConnected() const { return bConnected && Socket != nullptr; }

private:
	FSocket* Socket = nullptr;
	bool bConnected = false;
};
