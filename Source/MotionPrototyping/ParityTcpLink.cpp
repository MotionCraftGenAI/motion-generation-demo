#include "ParityTcpLink.h"

#include "Common/TcpSocketBuilder.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

namespace
{
	bool RecvExact(FSocket* Socket, uint8* Buffer, int32 Count, double Deadline,
		bool& bOutPeerClosed)
	{
		int32 Offset = 0;
		while (Offset < Count)
		{
			const double Remaining = Deadline - FPlatformTime::Seconds();
			if (Remaining <= 0.0)
			{
				return false;
			}
			if (!Socket->Wait(ESocketWaitConditions::WaitForRead,
					FTimespan::FromSeconds(Remaining)))
			{
				return false;
			}
			int32 Read = 0;
			if (!Socket->Recv(Buffer + Offset, Count - Offset, Read) || Read <= 0)
			{
				bOutPeerClosed = true;
				return false;
			}
			Offset += Read;
		}
		return true;
	}
}

bool FParityTcpLink::Connect(const FString& HostPort, FString& OutError)
{
	FString Host;
	FString PortText;
	if (!HostPort.Split(TEXT(":"), &Host, &PortText))
	{
		OutError = FString::Printf(TEXT("expected host:port, got '%s'"), *HostPort);
		return false;
	}
	const int32 Port = FCString::Atoi(*PortText);

	ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	if (!Subsystem)
	{
		OutError = TEXT("no socket subsystem");
		return false;
	}

	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	bool bAddrValid = false;
	Addr->SetIp(*Host, bAddrValid);
	if (!bAddrValid)
	{
		const FAddressInfoResult Info = Subsystem->GetAddressInfo(
			*Host, nullptr, EAddressInfoFlags::Default, NAME_None);
		if (Info.Results.Num() == 0)
		{
			OutError = FString::Printf(TEXT("cannot resolve '%s'"), *Host);
			return false;
		}
		Addr = Info.Results[0].Address->Clone();
	}
	Addr->SetPort(Port);

	Socket = Subsystem->CreateSocket(NAME_Stream, TEXT("ParityTcp"), Addr->GetProtocolType());
	if (!Socket)
	{
		OutError = TEXT("CreateSocket failed");
		return false;
	}
	Socket->SetNonBlocking(false);
	// One small message each way per tick; Nagle's algorithm would hold each one back.
	Socket->SetNoDelay(true);

	bConnected = Socket->Connect(*Addr);
	if (!bConnected)
	{
		OutError = FString::Printf(TEXT("connect to %s failed"), *Addr->ToString(true));
		Subsystem->DestroySocket(Socket);
		Socket = nullptr;
		return false;
	}
	return true;
}

bool FParityTcpLink::Send(const FString& Payload)
{
	if (!IsConnected())
	{
		return false;
	}
	FTCHARToUTF8 Utf8(*Payload);
	const int32 Length = Utf8.Length();

	const uint8 Header[4] = {
		static_cast<uint8>((Length >> 24) & 0xFF),
		static_cast<uint8>((Length >> 16) & 0xFF),
		static_cast<uint8>((Length >> 8) & 0xFF),
		static_cast<uint8>(Length & 0xFF),
	};

	int32 Sent = 0;
	if (!Socket->Send(Header, 4, Sent) || Sent != 4)
	{
		bConnected = false;
		return false;
	}
	const uint8* Data = reinterpret_cast<const uint8*>(Utf8.Get());
	int32 Offset = 0;
	while (Offset < Length)
	{
		int32 Chunk = 0;
		if (!Socket->Send(Data + Offset, Length - Offset, Chunk) || Chunk <= 0)
		{
			bConnected = false;
			return false;
		}
		Offset += Chunk;
	}
	return true;
}

bool FParityTcpLink::Recv(FString& OutPayload, int32 TimeoutMs)
{
	if (!IsConnected())
	{
		return false;
	}
	const double Deadline = FPlatformTime::Seconds() + TimeoutMs / 1000.0;

	bool bPeerClosed = false;
	uint8 Header[4];
	if (!RecvExact(Socket, Header, 4, Deadline, bPeerClosed))
	{
		bConnected = !bPeerClosed;
		return false;
	}
	const int32 Length = (static_cast<int32>(Header[0]) << 24)
		| (static_cast<int32>(Header[1]) << 16)
		| (static_cast<int32>(Header[2]) << 8)
		| static_cast<int32>(Header[3]);

	if (Length <= 0 || Length > (4 << 20))
	{
		bConnected = false;
		return false;
	}

	TArray<uint8> Buffer;
	Buffer.SetNumUninitialized(Length + 1);
	if (!RecvExact(Socket, Buffer.GetData(), Length, Deadline, bPeerClosed))
	{
		bConnected = false;
		return false;
	}
	Buffer[Length] = 0;
	OutPayload = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer.GetData())));
	return true;
}

void FParityTcpLink::Close()
{
	if (Socket)
	{
		Socket->Close();
		if (ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
		{
			Subsystem->DestroySocket(Socket);
		}
		Socket = nullptr;
	}
	bConnected = false;
}
