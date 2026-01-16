// Copyright JaguarEngine Team. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "JaguarTypes.h"
#include "JaguarEntityComponent.generated.h"

class UJaguarSubsystem;

/**
 * JaguarEngine Entity Component
 *
 * Attach this component to any actor to make it a JaguarEngine entity.
 * The component handles synchronization between Unreal actors and
 * JaguarEngine physics simulation.
 */
UCLASS(ClassGroup = (JaguarEngine), meta = (BlueprintSpawnableComponent, DisplayName = "Jaguar Entity"))
class JAGUARENGINE_API UJaguarEntityComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UJaguarEntityComponent();

    //~ Begin UActorComponent Interface
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    //~ End UActorComponent Interface

    // ----- Entity Configuration -----

    /** Entity name for identification */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Identity")
    FString EntityName = TEXT("Entity");

    /** Domain this entity belongs to */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Identity")
    EJaguarDomain Domain = EJaguarDomain::Air;

    /** Entity kind classification */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Identity")
    EJaguarEntityKind EntityKind = EJaguarEntityKind::Platform;

    /** Path to entity XML configuration file */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Configuration", meta = (FilePathFilter = "xml"))
    FFilePath EntityConfigPath;

    /** Integration mode with Unreal physics */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Configuration")
    EJaguarIntegrationMode IntegrationMode = EJaguarIntegrationMode::ReplaceUEPhysics;

    /** Update rate for network publishing (Hz) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Network", meta = (ClampMin = "1.0", ClampMax = "60.0"))
    float NetworkUpdateRate = 10.0f;

    /** Dead reckoning threshold for network updates (meters) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Entity|Network", meta = (ClampMin = "0.1"))
    float DeadReckoningThreshold = 1.0f;

    // ----- Runtime State -----

    /**
     * Get entity ID
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    int64 GetEntityId() const { return EntityId; }

    /**
     * Get current entity state
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    FJaguarEntityState GetEntityState() const { return CurrentState; }

    /**
     * Get geodetic position
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    FJaguarGeodeticPosition GetGeodeticPosition() const { return CurrentState.Position; }

    /**
     * Set geodetic position
     * @param Position New geodetic position
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity")
    void SetGeodeticPosition(const FJaguarGeodeticPosition& Position);

    /**
     * Get velocity in body frame (m/s)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    FVector GetVelocity() const { return CurrentState.Velocity; }

    /**
     * Set velocity in body frame (m/s)
     * @param Velocity New velocity
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity")
    void SetVelocity(const FVector& Velocity);

    /**
     * Get orientation quaternion
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    FQuat GetOrientation() const { return CurrentState.Orientation; }

    /**
     * Set orientation
     * @param Orientation New orientation quaternion
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity")
    void SetOrientation(const FQuat& Orientation);

    /**
     * Get Euler angles (degrees)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    FRotator GetEulerAngles() const { return CurrentState.EulerAngles; }

    /**
     * Get airspeed (m/s) - valid for air domain
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Air")
    float GetAirspeed() const;

    /**
     * Get altitude above sea level (meters)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    double GetAltitudeMSL() const { return CurrentState.Position.Altitude; }

    /**
     * Get altitude above ground level (meters)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    double GetAltitudeAGL() const;

    /**
     * Get Mach number - valid for air domain
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Air")
    float GetMachNumber() const;

    /**
     * Get angle of attack (degrees) - valid for air domain
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Air")
    float GetAngleOfAttack() const;

    /**
     * Get sideslip angle (degrees) - valid for air domain
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Air")
    float GetSideslipAngle() const;

    /**
     * Get ground speed (m/s)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity")
    float GetGroundSpeed() const;

    // ----- Control Inputs -----

    /**
     * Set control input value
     * @param ControlName Name of the control (e.g., "elevator", "throttle")
     * @param Value Control value (typically -1 to 1 or 0 to 1)
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity|Control")
    void SetControlInput(const FString& ControlName, float Value);

    /**
     * Get control input value
     * @param ControlName Name of the control
     * @return Current control value
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Control")
    float GetControlInput(const FString& ControlName) const;

    /**
     * Set multiple control inputs at once
     * @param Controls Map of control names to values
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity|Control")
    void SetControlInputs(const TMap<FString, float>& Controls);

    // ----- Property System -----

    /**
     * Get a property value by path
     * @param PropertyPath JaguarEngine property path (e.g., "position/h-agl-ft")
     * @return Property value or 0 if not found
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Properties")
    double GetProperty(const FString& PropertyPath) const;

    /**
     * Set a property value by path
     * @param PropertyPath JaguarEngine property path
     * @param Value New property value
     * @return True if property was found and set
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity|Properties")
    bool SetProperty(const FString& PropertyPath, double Value);

    /**
     * Check if a property exists
     * @param PropertyPath JaguarEngine property path
     * @return True if property exists
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Properties")
    bool HasProperty(const FString& PropertyPath) const;

    // ----- Damage System -----

    /**
     * Apply damage to this entity
     * @param DamageAmount Normalized damage amount (0-1)
     * @param DamageType Type of damage
     * @param HitLocation World location of damage
     * @param HitDirection Direction damage came from
     */
    UFUNCTION(BlueprintCallable, Category = "JaguarEngine|Entity|Damage")
    void ApplyDamage(float DamageAmount, FName DamageType, FVector HitLocation, FVector HitDirection);

    /**
     * Get current health (0-1)
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Damage")
    float GetHealth() const { return CurrentHealth; }

    /**
     * Check if entity is destroyed
     */
    UFUNCTION(BlueprintPure, Category = "JaguarEngine|Entity|Damage")
    bool IsDestroyed() const { return bIsDestroyed; }

    // ----- Events -----

    /** Called when entity state is updated from simulation */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnEntityStateUpdated, const FJaguarEntityState&, NewState);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnEntityStateUpdated OnEntityStateUpdated;

    /** Called when entity is damaged */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnEntityDamaged, float, DamageAmount, float, RemainingHealth);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnEntityDamaged OnEntityDamaged;

    /** Called when entity is destroyed */
    DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnEntityDestroyedEvent);
    UPROPERTY(BlueprintAssignable, Category = "JaguarEngine|Events")
    FOnEntityDestroyedEvent OnEntityDestroyedEvent;

protected:
    /** Apply state from JaguarEngine to actor transform */
    virtual void ApplyStateToActor();

    /** Read state from actor transform to JaguarEngine */
    virtual void ReadStateFromActor();

    /** Called when simulation updates entity state */
    virtual void OnSimulationStateReceived(const FJaguarEntityState& NewState);

private:
    friend class UJaguarSubsystem;

    /** Assigned entity ID */
    UPROPERTY()
    int64 EntityId = 0;

    /** Current entity state */
    UPROPERTY()
    FJaguarEntityState CurrentState;

    /** Current control inputs */
    UPROPERTY()
    TMap<FString, float> ControlInputs;

    /** Current health */
    UPROPERTY()
    float CurrentHealth = 1.0f;

    /** Is entity destroyed */
    UPROPERTY()
    bool bIsDestroyed = false;

    /** Cached subsystem reference */
    UPROPERTY()
    TWeakObjectPtr<UJaguarSubsystem> CachedSubsystem;

    /** Time since last network update */
    float TimeSinceNetworkUpdate = 0.0f;

    /** Last published state for dead reckoning comparison */
    FJaguarEntityState LastPublishedState;

    /** Native entity handle (opaque pointer to jaguar entity) */
    void* NativeEntityHandle = nullptr;

    /** Register with subsystem */
    void RegisterWithSubsystem();

    /** Unregister from subsystem */
    void UnregisterFromSubsystem();

    /** Synchronize with native engine */
    void SyncWithNativeEngine();

    /** Check if state has changed enough to publish */
    bool ShouldPublishState() const;
};
