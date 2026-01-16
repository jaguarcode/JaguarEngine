// Copyright JaguarEngine Team. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

DECLARE_LOG_CATEGORY_EXTERN(LogJaguarEngine, Log, All);

/**
 * JaguarEngine Unreal Engine 5 Plugin Module
 *
 * Provides integration between JaguarEngine physics simulation
 * and Unreal Engine for high-fidelity multi-domain simulation.
 */
class FJaguarEngineModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    /**
     * Get the module instance
     */
    static FJaguarEngineModule& Get()
    {
        return FModuleManager::LoadModuleChecked<FJaguarEngineModule>("JaguarEngine");
    }

    /**
     * Check if the module is loaded
     */
    static bool IsAvailable()
    {
        return FModuleManager::Get().IsModuleLoaded("JaguarEngine");
    }

    /**
     * Get the native JaguarEngine version string
     */
    FString GetNativeEngineVersion() const;

    /**
     * Check if native library is loaded
     */
    bool IsNativeLibraryLoaded() const { return bNativeLibraryLoaded; }

private:
    /** Handle to the native library */
    void* NativeLibraryHandle = nullptr;

    /** Flag indicating native library load status */
    bool bNativeLibraryLoaded = false;

    /** Load the native JaguarEngine library */
    bool LoadNativeLibrary();

    /** Unload the native JaguarEngine library */
    void UnloadNativeLibrary();
};
