// Copyright JaguarEngine Team. All Rights Reserved.

#include "JaguarEngineModule.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"

#define LOCTEXT_NAMESPACE "FJaguarEngineModule"

DEFINE_LOG_CATEGORY(LogJaguarEngine);

void FJaguarEngineModule::StartupModule()
{
    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarEngine module starting up..."));

    // Load native library
    if (LoadNativeLibrary())
    {
        UE_LOG(LogJaguarEngine, Log, TEXT("JaguarEngine native library loaded successfully"));
        UE_LOG(LogJaguarEngine, Log, TEXT("Native engine version: %s"), *GetNativeEngineVersion());
    }
    else
    {
        UE_LOG(LogJaguarEngine, Warning, TEXT("Failed to load JaguarEngine native library - simulation features will be limited"));
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarEngine module startup complete"));
}

void FJaguarEngineModule::ShutdownModule()
{
    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarEngine module shutting down..."));

    UnloadNativeLibrary();

    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarEngine module shutdown complete"));
}

FString FJaguarEngineModule::GetNativeEngineVersion() const
{
    if (!bNativeLibraryLoaded)
    {
        return TEXT("Not loaded");
    }

    // Would call native function here
    // For now return placeholder
    return TEXT("1.4.0");
}

bool FJaguarEngineModule::LoadNativeLibrary()
{
    // Determine library path based on platform
    FString LibraryPath;
    FString LibraryName;

#if PLATFORM_WINDOWS
    LibraryName = TEXT("jaguar.dll");
#elif PLATFORM_LINUX
    LibraryName = TEXT("libjaguar.so");
#elif PLATFORM_MAC
    LibraryName = TEXT("libjaguar.dylib");
#else
    UE_LOG(LogJaguarEngine, Error, TEXT("Unsupported platform for JaguarEngine"));
    return false;
#endif

    // Try multiple paths
    TArray<FString> SearchPaths;

    // Plugin binaries folder
    SearchPaths.Add(FPaths::Combine(
        IPluginManager::Get().FindPlugin(TEXT("JaguarEngine"))->GetBaseDir(),
        TEXT("Binaries"),
        FPlatformProcess::GetBinariesSubdirectory(),
        LibraryName));

    // Engine binaries
    SearchPaths.Add(FPaths::Combine(
        FPaths::EngineDir(),
        TEXT("Binaries"),
        FPlatformProcess::GetBinariesSubdirectory(),
        LibraryName));

    // Project binaries
    SearchPaths.Add(FPaths::Combine(
        FPaths::ProjectDir(),
        TEXT("Binaries"),
        FPlatformProcess::GetBinariesSubdirectory(),
        LibraryName));

    // System path (development)
    SearchPaths.Add(LibraryName);

    for (const FString& Path : SearchPaths)
    {
        if (FPaths::FileExists(Path))
        {
            LibraryPath = Path;
            break;
        }
    }

    if (LibraryPath.IsEmpty())
    {
        UE_LOG(LogJaguarEngine, Warning, TEXT("Could not find JaguarEngine library. Searched paths:"));
        for (const FString& Path : SearchPaths)
        {
            UE_LOG(LogJaguarEngine, Warning, TEXT("  %s"), *Path);
        }
        return false;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Loading JaguarEngine library from: %s"), *LibraryPath);

    NativeLibraryHandle = FPlatformProcess::GetDllHandle(*LibraryPath);

    if (NativeLibraryHandle == nullptr)
    {
        UE_LOG(LogJaguarEngine, Error, TEXT("Failed to load library: %s"), *LibraryPath);
        return false;
    }

    bNativeLibraryLoaded = true;
    return true;
}

void FJaguarEngineModule::UnloadNativeLibrary()
{
    if (NativeLibraryHandle != nullptr)
    {
        FPlatformProcess::FreeDllHandle(NativeLibraryHandle);
        NativeLibraryHandle = nullptr;
        bNativeLibraryLoaded = false;
    }
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FJaguarEngineModule, JaguarEngine)
