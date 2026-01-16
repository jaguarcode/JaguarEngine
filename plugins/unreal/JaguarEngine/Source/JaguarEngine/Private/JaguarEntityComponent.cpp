// Copyright JaguarEngine Team. All Rights Reserved.

#include "JaguarEntityComponent.h"
#include "JaguarSubsystem.h"
#include "JaguarEngineModule.h"
#include "GameFramework/Actor.h"

UJaguarEntityComponent::UJaguarEntityComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;

    // Default control inputs
    ControlInputs.Add(TEXT("elevator"), 0.0f);
    ControlInputs.Add(TEXT("aileron"), 0.0f);
    ControlInputs.Add(TEXT("rudder"), 0.0f);
    ControlInputs.Add(TEXT("throttle"), 0.0f);
    ControlInputs.Add(TEXT("collective"), 0.0f);
    ControlInputs.Add(TEXT("flaps"), 0.0f);
    ControlInputs.Add(TEXT("gear"), 1.0f);
    ControlInputs.Add(TEXT("speedbrake"), 0.0f);
}

void UJaguarEntityComponent::BeginPlay()
{
    Super::BeginPlay();

    // Initialize state from actor
    ReadStateFromActor();

    // Register with subsystem
    RegisterWithSubsystem();
}

void UJaguarEntityComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Unregister from subsystem
    UnregisterFromSubsystem();

    Super::EndPlay(EndPlayReason);
}

void UJaguarEntityComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bIsDestroyed)
    {
        return;
    }

    // Synchronize with native engine
    SyncWithNativeEngine();

    // Apply state to actor based on integration mode
    switch (IntegrationMode)
    {
        case EJaguarIntegrationMode::ReplaceUEPhysics:
        case EJaguarIntegrationMode::JaguarOnly:
            ApplyStateToActor();
            break;

        case EJaguarIntegrationMode::Parallel:
            // Both systems run, state is for reference only
            break;

        case EJaguarIntegrationMode::Synchronized:
            // Blend between UE and Jaguar physics
            ApplyStateToActor();
            break;
    }

    // Network update check
    TimeSinceNetworkUpdate += DeltaTime;
    if (TimeSinceNetworkUpdate >= 1.0f / NetworkUpdateRate)
    {
        if (ShouldPublishState())
        {
            // Would publish to network
            LastPublishedState = CurrentState;
        }
        TimeSinceNetworkUpdate = 0.0f;
    }

    // Broadcast state update event
    OnEntityStateUpdated.Broadcast(CurrentState);
}

void UJaguarEntityComponent::SetGeodeticPosition(const FJaguarGeodeticPosition& Position)
{
    CurrentState.Position = Position;

    // Update actor if in appropriate mode
    if (IntegrationMode == EJaguarIntegrationMode::ReplaceUEPhysics ||
        IntegrationMode == EJaguarIntegrationMode::JaguarOnly)
    {
        ApplyStateToActor();
    }
}

void UJaguarEntityComponent::SetVelocity(const FVector& Velocity)
{
    CurrentState.Velocity = Velocity;
}

void UJaguarEntityComponent::SetOrientation(const FQuat& Orientation)
{
    CurrentState.Orientation = Orientation;
    CurrentState.EulerAngles = Orientation.Rotator();
}

float UJaguarEntityComponent::GetAirspeed() const
{
    return CurrentState.Velocity.Size();
}

double UJaguarEntityComponent::GetAltitudeAGL() const
{
    if (CachedSubsystem.IsValid())
    {
        FJaguarTerrainInfo Terrain = CachedSubsystem->GetTerrainAt(CurrentState.Position);
        return CurrentState.Position.Altitude - Terrain.Elevation;
    }
    return CurrentState.Position.Altitude;
}

float UJaguarEntityComponent::GetMachNumber() const
{
    if (CachedSubsystem.IsValid())
    {
        FJaguarAtmosphere Atmo = CachedSubsystem->GetAtmosphereAt(CurrentState.Position);
        return GetAirspeed() / Atmo.SpeedOfSound;
    }
    return GetAirspeed() / 340.29f;  // Sea level speed of sound
}

float UJaguarEntityComponent::GetAngleOfAttack() const
{
    // Would come from native engine
    // Simplified calculation
    if (CurrentState.Velocity.Size() < 1.0f)
    {
        return 0.0f;
    }

    FVector VelBodyNorm = CurrentState.Velocity.GetSafeNormal();
    float Alpha = FMath::RadiansToDegrees(FMath::Atan2(-VelBodyNorm.Z, VelBodyNorm.X));
    return Alpha;
}

float UJaguarEntityComponent::GetSideslipAngle() const
{
    if (CurrentState.Velocity.Size() < 1.0f)
    {
        return 0.0f;
    }

    FVector VelBodyNorm = CurrentState.Velocity.GetSafeNormal();
    float Beta = FMath::RadiansToDegrees(FMath::Asin(VelBodyNorm.Y));
    return Beta;
}

float UJaguarEntityComponent::GetGroundSpeed() const
{
    // Horizontal components of velocity
    return FMath::Sqrt(CurrentState.Velocity.X * CurrentState.Velocity.X +
                       CurrentState.Velocity.Y * CurrentState.Velocity.Y);
}

void UJaguarEntityComponent::SetControlInput(const FString& ControlName, float Value)
{
    ControlInputs.FindOrAdd(ControlName) = Value;
}

float UJaguarEntityComponent::GetControlInput(const FString& ControlName) const
{
    const float* Value = ControlInputs.Find(ControlName);
    return Value ? *Value : 0.0f;
}

void UJaguarEntityComponent::SetControlInputs(const TMap<FString, float>& Controls)
{
    for (const auto& Pair : Controls)
    {
        ControlInputs.FindOrAdd(Pair.Key) = Pair.Value;
    }
}

double UJaguarEntityComponent::GetProperty(const FString& PropertyPath) const
{
    // Would query native engine property system
    // Provide some common properties directly

    if (PropertyPath == TEXT("position/h-sl-meters") ||
        PropertyPath == TEXT("position/altitude"))
    {
        return CurrentState.Position.Altitude;
    }
    if (PropertyPath == TEXT("position/latitude-deg"))
    {
        return CurrentState.Position.Latitude;
    }
    if (PropertyPath == TEXT("position/longitude-deg"))
    {
        return CurrentState.Position.Longitude;
    }
    if (PropertyPath == TEXT("velocities/u-fps") ||
        PropertyPath == TEXT("velocity/u"))
    {
        return CurrentState.Velocity.X * 3.28084;  // m/s to ft/s
    }
    if (PropertyPath == TEXT("velocities/v-fps") ||
        PropertyPath == TEXT("velocity/v"))
    {
        return CurrentState.Velocity.Y * 3.28084;
    }
    if (PropertyPath == TEXT("velocities/w-fps") ||
        PropertyPath == TEXT("velocity/w"))
    {
        return CurrentState.Velocity.Z * 3.28084;
    }
    if (PropertyPath == TEXT("attitude/phi-deg") ||
        PropertyPath == TEXT("attitude/roll"))
    {
        return CurrentState.EulerAngles.Roll;
    }
    if (PropertyPath == TEXT("attitude/theta-deg") ||
        PropertyPath == TEXT("attitude/pitch"))
    {
        return CurrentState.EulerAngles.Pitch;
    }
    if (PropertyPath == TEXT("attitude/psi-deg") ||
        PropertyPath == TEXT("attitude/yaw"))
    {
        return CurrentState.EulerAngles.Yaw;
    }

    UE_LOG(LogJaguarEngine, Verbose, TEXT("Unknown property: %s"), *PropertyPath);
    return 0.0;
}

bool UJaguarEntityComponent::SetProperty(const FString& PropertyPath, double Value)
{
    // Would set in native engine property system
    UE_LOG(LogJaguarEngine, Verbose, TEXT("SetProperty(%s, %f) - native engine not available"),
        *PropertyPath, Value);
    return false;
}

bool UJaguarEntityComponent::HasProperty(const FString& PropertyPath) const
{
    // Common properties we support
    static TSet<FString> KnownProperties = {
        TEXT("position/h-sl-meters"),
        TEXT("position/altitude"),
        TEXT("position/latitude-deg"),
        TEXT("position/longitude-deg"),
        TEXT("velocities/u-fps"),
        TEXT("velocities/v-fps"),
        TEXT("velocities/w-fps"),
        TEXT("velocity/u"),
        TEXT("velocity/v"),
        TEXT("velocity/w"),
        TEXT("attitude/phi-deg"),
        TEXT("attitude/theta-deg"),
        TEXT("attitude/psi-deg"),
        TEXT("attitude/roll"),
        TEXT("attitude/pitch"),
        TEXT("attitude/yaw")
    };

    return KnownProperties.Contains(PropertyPath);
}

void UJaguarEntityComponent::ApplyDamage(float DamageAmount, FName DamageType, FVector HitLocation, FVector HitDirection)
{
    if (bIsDestroyed)
    {
        return;
    }

    float PreviousHealth = CurrentHealth;
    CurrentHealth = FMath::Clamp(CurrentHealth - DamageAmount, 0.0f, 1.0f);

    UE_LOG(LogJaguarEngine, Log, TEXT("Entity '%s' damaged: %.2f -> %.2f (type: %s)"),
        *EntityName, PreviousHealth, CurrentHealth, *DamageType.ToString());

    OnEntityDamaged.Broadcast(DamageAmount, CurrentHealth);

    if (CurrentHealth <= 0.0f && !bIsDestroyed)
    {
        bIsDestroyed = true;
        CurrentState.bIsActive = false;

        UE_LOG(LogJaguarEngine, Log, TEXT("Entity '%s' destroyed"), *EntityName);
        OnEntityDestroyedEvent.Broadcast();
    }
}

void UJaguarEntityComponent::ApplyStateToActor()
{
    AActor* Owner = GetOwner();
    if (!Owner || !CachedSubsystem.IsValid())
    {
        return;
    }

    // Convert geodetic position to Unreal coordinates
    FVector WorldPos = CachedSubsystem->GeodeticToUnreal(CurrentState.Position);

    // Convert NED orientation to Unreal rotation
    FRotator WorldRot = CachedSubsystem->NEDToUnrealRotation(CurrentState.Orientation);

    // Apply to actor
    Owner->SetActorLocationAndRotation(WorldPos, WorldRot, false, nullptr, ETeleportType::TeleportPhysics);
}

void UJaguarEntityComponent::ReadStateFromActor()
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        return;
    }

    if (CachedSubsystem.IsValid())
    {
        // Convert Unreal position to geodetic
        CurrentState.Position = CachedSubsystem->UnrealToGeodetic(Owner->GetActorLocation());

        // Convert Unreal rotation to NED
        CurrentState.Orientation = CachedSubsystem->UnrealRotationToNED(Owner->GetActorRotation());
        CurrentState.EulerAngles = Owner->GetActorRotation();
    }
    else
    {
        // Fallback - just use Unreal coordinates
        FVector Loc = Owner->GetActorLocation();
        CurrentState.Position.Altitude = Loc.Z / 100.0;  // cm to m
    }

    // Get velocity from physics if available
    if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent()))
    {
        if (RootPrim->IsSimulatingPhysics())
        {
            FVector Vel = RootPrim->GetPhysicsLinearVelocity();
            // Convert cm/s to m/s
            CurrentState.Velocity = Vel / 100.0f;

            FVector AngVel = RootPrim->GetPhysicsAngularVelocityInRadians();
            CurrentState.AngularVelocity = AngVel;
        }
    }

    CurrentState.Timestamp = CachedSubsystem.IsValid() ? CachedSubsystem->GetSimulationTime() : 0.0;
}

void UJaguarEntityComponent::OnSimulationStateReceived(const FJaguarEntityState& NewState)
{
    CurrentState = NewState;
    CurrentState.EntityId = EntityId;
    CurrentState.EntityName = EntityName;
}

void UJaguarEntityComponent::RegisterWithSubsystem()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    UJaguarSubsystem* Subsystem = World->GetSubsystem<UJaguarSubsystem>();
    if (Subsystem)
    {
        CachedSubsystem = Subsystem;
        EntityId = Subsystem->RegisterEntity(this);

        CurrentState.EntityId = EntityId;
        CurrentState.EntityName = EntityName;
        CurrentState.Domain = Domain;
        CurrentState.Kind = EntityKind;

        UE_LOG(LogJaguarEngine, Log, TEXT("Entity '%s' registered with subsystem, ID: %lld"),
            *EntityName, EntityId);
    }
    else
    {
        UE_LOG(LogJaguarEngine, Warning, TEXT("Entity '%s' could not find JaguarSubsystem"),
            *EntityName);
    }
}

void UJaguarEntityComponent::UnregisterFromSubsystem()
{
    if (CachedSubsystem.IsValid() && EntityId != 0)
    {
        CachedSubsystem->UnregisterEntity(EntityId);
        UE_LOG(LogJaguarEngine, Log, TEXT("Entity '%s' unregistered from subsystem"),
            *EntityName);
    }

    EntityId = 0;
    CachedSubsystem.Reset();
}

void UJaguarEntityComponent::SyncWithNativeEngine()
{
    if (!NativeEntityHandle)
    {
        // No native entity - read state from actor
        ReadStateFromActor();
        return;
    }

    // Would sync control inputs to native engine
    // Then read back state

    // For now, just update timestamp
    if (CachedSubsystem.IsValid())
    {
        CurrentState.Timestamp = CachedSubsystem->GetSimulationTime();
    }
}

bool UJaguarEntityComponent::ShouldPublishState() const
{
    // Check dead reckoning threshold
    FVector PosDiff = CurrentState.Position.ToECEF() - LastPublishedState.Position.ToECEF();
    float Distance = PosDiff.Size();

    if (Distance > DeadReckoningThreshold)
    {
        return true;
    }

    // Check orientation difference
    float AngleDiff = CurrentState.Orientation.AngularDistance(LastPublishedState.Orientation);
    if (FMath::RadiansToDegrees(AngleDiff) > 1.0f)  // 1 degree threshold
    {
        return true;
    }

    return false;
}
