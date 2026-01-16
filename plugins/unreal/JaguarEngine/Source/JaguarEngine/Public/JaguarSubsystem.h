// Copyright JaguarEngine Team. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "JaguarTypes.h"
#include "JaguarSubsystem.generated.h"

class UJaguarEntityComponent;

/**
 * JaguarEngine World Subsystem
 *
 * Central manager for JaguarEngine simulation within an Unreal World.
 * Handles simulation lifecycle, entity registration, and coordinate transforms.
 */
UCLASS()
class JAGUARENGINE_API UJaguarSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    UJaguarSubsystem();

    //~ Begin UWorldSubsystem Interface
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;
    virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
    //~ End UWorldSubsystem Interface

    /**
     * Tick the JaguarEngine simulation
     * @param DeltaTime Time since last tick in seconds
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void TickSimulation(float DeltaTime);

    /**
     * Start the simulation
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void StartSimulation();

    /**
     * Stop the simulation
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void StopSimulation();

    /**
     * Pause the simulation
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void PauseSimulation();

    /**
     * Resume the simulation from pause
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void ResumeSimulation();

    /**
     * Step the simulation by a fixed amount
     * @param StepTime Time to step in seconds
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void StepSimulation(float StepTime);

    /**
     * Get current simulation state
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Simulation")
    EJaguarSimulationState GetSimulationState() const { return SimulationState; }

    /**
     * Get current simulation time
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Simulation")
    double GetSimulationTime() const { return SimulationTime; }

    /**
     * Get simulation statistics
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Simulation")
    FJaguarSimulationStats GetSimulationStats() const;

    /**
     * Set simulation time scale
     * @param TimeScale Multiplier for simulation time (1.0 = real-time)
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Simulation")
    void SetTimeScale(float TimeScale);

    /**
     * Get current time scale
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Simulation")
    float GetTimeScale() const { return CurrentTimeScale; }

    // ----- Entity Management -----

    /**
     * Register an entity component with the subsystem
     * @param Component The entity component to register
     * @return Entity ID assigned
     */
    int64 RegisterEntity(UJaguarEntityComponent* Component);

    /**
     * Unregister an entity component
     * @param EntityId The entity ID to unregister
     */
    void UnregisterEntity(int64 EntityId);

    /**
     * Get entity component by ID
     * @param EntityId The entity ID to find
     * @return Entity component or nullptr
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entities")
    UJaguarEntityComponent* GetEntityById(int64 EntityId) const;

    /**
     * Get all registered entity IDs
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entities")
    TArray<int64> GetAllEntityIds() const;

    /**
     * Get entity count
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entities")
    int32 GetEntityCount() const { return RegisteredEntities.Num(); }

    // ----- Coordinate Systems -----

    /**
     * Set the simulation origin in geodetic coordinates
     * @param Origin The geodetic origin for local coordinate transforms
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Coordinates")
    void SetSimulationOrigin(const FJaguarGeodeticPosition& Origin);

    /**
     * Get the current simulation origin
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Coordinates")
    FJaguarGeodeticPosition GetSimulationOrigin() const { return SimulationOrigin; }

    /**
     * Convert geodetic position to Unreal coordinates
     * @param Geodetic Geodetic position (lat/lon/alt)
     * @return Unreal world position
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Coordinates")
    FVector GeodeticToUnreal(const FJaguarGeodeticPosition& Geodetic) const;

    /**
     * Convert Unreal coordinates to geodetic position
     * @param UnrealPosition Unreal world position
     * @return Geodetic position
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Coordinates")
    FJaguarGeodeticPosition UnrealToGeodetic(const FVector& UnrealPosition) const;

    /**
     * Convert Unreal rotation to JaguarEngine orientation (NED)
     * @param UnrealRotation Unreal rotation
     * @return NED orientation quaternion
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Coordinates")
    FQuat UnrealRotationToNED(const FRotator& UnrealRotation) const;

    /**
     * Convert NED orientation to Unreal rotation
     * @param NEDOrientation NED orientation quaternion
     * @return Unreal rotation
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Coordinates")
    FRotator NEDToUnrealRotation(const FQuat& NEDOrientation) const;

    // ----- Environment -----

    /**
     * Get atmospheric conditions at a position
     * @param Position Geodetic position
     * @return Atmospheric conditions
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Environment")
    FJaguarAtmosphere GetAtmosphereAt(const FJaguarGeodeticPosition& Position) const;

    /**
     * Get terrain information at a position
     * @param Position Geodetic position
     * @return Terrain information
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Environment")
    FJaguarTerrainInfo GetTerrainAt(const FJaguarGeodeticPosition& Position) const;

    /**
     * Set global wind parameters
     * @param WindVelocity Wind velocity in NED (m/s)
     * @param TurbulenceIntensity Turbulence intensity (0-1)
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Environment")
    void SetWind(const FVector& WindVelocity, float TurbulenceIntensity = 0.0f);

    // ----- Network -----

    /**
     * Configure network settings
     * @param Settings Network configuration
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Network")
    void ConfigureNetwork(const FJaguarNetworkSettings& Settings);

    /**
     * Get current network settings
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Network")
    FJaguarNetworkSettings GetNetworkSettings() const { return NetworkSettings; }

    /**
     * Connect to DIS/HLA network
     * @return True if connection successful
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Network")
    bool ConnectNetwork();

    /**
     * Disconnect from DIS/HLA network
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Network")
    void DisconnectNetwork();

    /**
     * Check if connected to network
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Network")
    bool IsNetworkConnected() const { return bNetworkConnected; }

    // ----- Events -----

    /** Broadcast when simulation state changes */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnSimulationStateChanged, EJaguarSimulationState, NewState);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnSimulationStateChanged OnSimulationStateChanged;

    /** Broadcast when an entity is spawned */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnEntitySpawned, int64, EntityId);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnEntitySpawned OnEntitySpawned;

    /** Broadcast when an entity is destroyed */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnEntityDestroyed, int64, EntityId);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnEntityDestroyed OnEntityDestroyed;

    /** Broadcast when a collision is detected */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnCollisionDetected, int64, EntityA, int64, EntityB, FVector, ContactPoint);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnCollisionDetected OnCollisionDetected;

protected:
    /** Current simulation state */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    EJaguarSimulationState SimulationState = EJaguarSimulationState::Stopped;

    /** Current simulation time */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    double SimulationTime = 0.0;

    /** Time scale multiplier */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    float CurrentTimeScale = 1.0f;

    /** Simulation origin for coordinate transforms */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    FJaguarGeodeticPosition SimulationOrigin;

    /** Network settings */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    FJaguarNetworkSettings NetworkSettings;

    /** Network connection status */
    UPROPERTY(BlueprintReadOnly, Category = "JaguarEngine")
    bool bNetworkConnected = false;

private:
    /** Map of entity ID to component */
    UPROPERTY()
    TMap<int64, TWeakObjectPtr<UJaguarEntityComponent>> RegisteredEntities;

    /** Next entity ID to assign */
    int64 NextEntityId = 1;

    /** Wall clock time tracking */
    double WallClockStartTime = 0.0;

    /** Performance tracking */
    double LastPhysicsTimeMs = 0.0;
    double LastCollisionTimeMs = 0.0;
    double LastNetworkTimeMs = 0.0;

    /** Native engine handle (opaque pointer to jaguar::Engine) */
    void* NativeEngineHandle = nullptr;

    /** Initialize native engine */
    bool InitializeNativeEngine();

    /** Shutdown native engine */
    void ShutdownNativeEngine();

    /** Synchronize entity states with native engine */
    void SyncEntityStates();

    /** Process collision events from native engine */
    void ProcessCollisionEvents();

    /** Process network updates */
    void ProcessNetworkUpdates();
};
