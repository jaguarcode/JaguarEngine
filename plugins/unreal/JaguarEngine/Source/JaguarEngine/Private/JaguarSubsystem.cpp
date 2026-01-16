// Copyright JaguarEngine Team. All Rights Reserved.

#include "JaguarSubsystem.h"
#include "JaguarEntityComponent.h"
#include "JaguarEngineModule.h"
#include "Engine/World.h"
#include "TimerManager.h"

// WGS84 constants
namespace WGS84
{
    constexpr double SemiMajorAxis = 6378137.0;
    constexpr double Flattening = 1.0 / 298.257223563;
    constexpr double SemiMinorAxis = SemiMajorAxis * (1.0 - Flattening);
    constexpr double EccentricitySquared = 2.0 * Flattening - Flattening * Flattening;
}

UJaguarSubsystem::UJaguarSubsystem()
{
    // Default origin: Edwards AFB, CA
    SimulationOrigin.Latitude = 34.905;
    SimulationOrigin.Longitude = -117.884;
    SimulationOrigin.Altitude = 702.0;
}

void UJaguarSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarSubsystem initializing for world: %s"),
        *GetWorld()->GetName());

    // Initialize native engine
    if (InitializeNativeEngine())
    {
        UE_LOG(LogJaguarEngine, Log, TEXT("Native engine initialized successfully"));
    }
    else
    {
        UE_LOG(LogJaguarEngine, Warning, TEXT("Failed to initialize native engine - using fallback mode"));
    }

    WallClockStartTime = FPlatformTime::Seconds();
}

void UJaguarSubsystem::Deinitialize()
{
    UE_LOG(LogJaguarEngine, Log, TEXT("JaguarSubsystem deinitializing..."));

    // Stop simulation
    StopSimulation();

    // Disconnect network
    DisconnectNetwork();

    // Shutdown native engine
    ShutdownNativeEngine();

    // Clear entities
    RegisteredEntities.Empty();

    Super::Deinitialize();
}

bool UJaguarSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
    // Create subsystem for all game worlds
    if (UWorld* World = Cast<UWorld>(Outer))
    {
        return World->IsGameWorld();
    }
    return false;
}

void UJaguarSubsystem::TickSimulation(float DeltaTime)
{
    if (SimulationState != EJaguarSimulationState::Running)
    {
        return;
    }

    const double StartTime = FPlatformTime::Seconds();

    // Apply time scale
    float ScaledDeltaTime = DeltaTime * CurrentTimeScale;

    // Sync entity states to native engine
    SyncEntityStates();

    // Step physics (would call native engine)
    // jaguar::Engine::step(NativeEngineHandle, ScaledDeltaTime);
    const double PhysicsEndTime = FPlatformTime::Seconds();
    LastPhysicsTimeMs = (PhysicsEndTime - StartTime) * 1000.0;

    // Process collisions
    ProcessCollisionEvents();
    const double CollisionEndTime = FPlatformTime::Seconds();
    LastCollisionTimeMs = (CollisionEndTime - PhysicsEndTime) * 1000.0;

    // Process network
    if (bNetworkConnected)
    {
        ProcessNetworkUpdates();
    }
    const double NetworkEndTime = FPlatformTime::Seconds();
    LastNetworkTimeMs = (NetworkEndTime - CollisionEndTime) * 1000.0;

    // Update simulation time
    SimulationTime += ScaledDeltaTime;
}

void UJaguarSubsystem::StartSimulation()
{
    if (SimulationState == EJaguarSimulationState::Running)
    {
        return;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Starting simulation..."));

    EJaguarSimulationState OldState = SimulationState;
    SimulationState = EJaguarSimulationState::Running;
    WallClockStartTime = FPlatformTime::Seconds();

    OnSimulationStateChanged.Broadcast(SimulationState);

    UE_LOG(LogJaguarEngine, Log, TEXT("Simulation started with %d entities"), RegisteredEntities.Num());
}

void UJaguarSubsystem::StopSimulation()
{
    if (SimulationState == EJaguarSimulationState::Stopped)
    {
        return;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Stopping simulation..."));

    SimulationState = EJaguarSimulationState::Stopped;
    SimulationTime = 0.0;

    OnSimulationStateChanged.Broadcast(SimulationState);
}

void UJaguarSubsystem::PauseSimulation()
{
    if (SimulationState != EJaguarSimulationState::Running)
    {
        return;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Pausing simulation at t=%.3f"), SimulationTime);

    SimulationState = EJaguarSimulationState::Paused;
    OnSimulationStateChanged.Broadcast(SimulationState);
}

void UJaguarSubsystem::ResumeSimulation()
{
    if (SimulationState != EJaguarSimulationState::Paused)
    {
        return;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Resuming simulation from t=%.3f"), SimulationTime);

    SimulationState = EJaguarSimulationState::Running;
    OnSimulationStateChanged.Broadcast(SimulationState);
}

void UJaguarSubsystem::StepSimulation(float StepTime)
{
    if (SimulationState == EJaguarSimulationState::Running)
    {
        PauseSimulation();
    }

    UE_LOG(LogJaguarEngine, Verbose, TEXT("Stepping simulation by %.4fs"), StepTime);

    EJaguarSimulationState OldState = SimulationState;
    SimulationState = EJaguarSimulationState::Stepping;

    TickSimulation(StepTime);

    SimulationState = OldState;
}

FJaguarSimulationStats UJaguarSubsystem::GetSimulationStats() const
{
    FJaguarSimulationStats Stats;

    Stats.SimulationTime = SimulationTime;
    Stats.WallClockTime = FPlatformTime::Seconds() - WallClockStartTime;

    if (Stats.WallClockTime > 0.0)
    {
        Stats.RealTimeRatio = static_cast<float>(Stats.SimulationTime / Stats.WallClockTime);
    }

    Stats.ActiveEntityCount = RegisteredEntities.Num();
    Stats.PhysicsStepTimeMs = static_cast<float>(LastPhysicsTimeMs);
    Stats.CollisionTimeMs = static_cast<float>(LastCollisionTimeMs);
    Stats.NetworkTimeMs = static_cast<float>(LastNetworkTimeMs);

    // Memory would come from native engine
    Stats.MemoryUsageMB = 0.0f;

    return Stats;
}

void UJaguarSubsystem::SetTimeScale(float TimeScale)
{
    CurrentTimeScale = FMath::Clamp(TimeScale, 0.0f, 100.0f);
    UE_LOG(LogJaguarEngine, Log, TEXT("Time scale set to %.2fx"), CurrentTimeScale);
}

int64 UJaguarSubsystem::RegisterEntity(UJaguarEntityComponent* Component)
{
    if (!Component)
    {
        return 0;
    }

    int64 EntityId = NextEntityId++;
    RegisteredEntities.Add(EntityId, Component);

    UE_LOG(LogJaguarEngine, Log, TEXT("Registered entity '%s' with ID %lld"),
        *Component->EntityName, EntityId);

    OnEntitySpawned.Broadcast(EntityId);

    return EntityId;
}

void UJaguarSubsystem::UnregisterEntity(int64 EntityId)
{
    if (RegisteredEntities.Remove(EntityId) > 0)
    {
        UE_LOG(LogJaguarEngine, Log, TEXT("Unregistered entity ID %lld"), EntityId);
        OnEntityDestroyed.Broadcast(EntityId);
    }
}

UJaguarEntityComponent* UJaguarSubsystem::GetEntityById(int64 EntityId) const
{
    const TWeakObjectPtr<UJaguarEntityComponent>* Found = RegisteredEntities.Find(EntityId);
    if (Found && Found->IsValid())
    {
        return Found->Get();
    }
    return nullptr;
}

TArray<int64> UJaguarSubsystem::GetAllEntityIds() const
{
    TArray<int64> Ids;
    RegisteredEntities.GetKeys(Ids);
    return Ids;
}

void UJaguarSubsystem::SetSimulationOrigin(const FJaguarGeodeticPosition& Origin)
{
    SimulationOrigin = Origin;
    UE_LOG(LogJaguarEngine, Log, TEXT("Simulation origin set to %s"), *Origin.ToString());
}

FVector UJaguarSubsystem::GeodeticToUnreal(const FJaguarGeodeticPosition& Geodetic) const
{
    // Convert geodetic to local NED, then to UE coordinates

    // First convert both to ECEF
    FVector OriginECEF = SimulationOrigin.ToECEF();
    FVector PointECEF = Geodetic.ToECEF();

    // Compute local NED offset
    double LatRad = FMath::DegreesToRadians(SimulationOrigin.Latitude);
    double LonRad = FMath::DegreesToRadians(SimulationOrigin.Longitude);

    double SinLat = FMath::Sin(LatRad);
    double CosLat = FMath::Cos(LatRad);
    double SinLon = FMath::Sin(LonRad);
    double CosLon = FMath::Cos(LonRad);

    // ECEF to NED rotation matrix
    FVector Diff = PointECEF - OriginECEF;

    double North = -SinLat * CosLon * Diff.X - SinLat * SinLon * Diff.Y + CosLat * Diff.Z;
    double East = -SinLon * Diff.X + CosLon * Diff.Y;
    double Down = -CosLat * CosLon * Diff.X - CosLat * SinLon * Diff.Y - SinLat * Diff.Z;

    // NED to Unreal: X=North, Y=East, Z=-Down, convert m to cm
    return FVector(North * 100.0, East * 100.0, -Down * 100.0);
}

FJaguarGeodeticPosition UJaguarSubsystem::UnrealToGeodetic(const FVector& UnrealPosition) const
{
    // Convert Unreal to NED (cm to m)
    double North = UnrealPosition.X / 100.0;
    double East = UnrealPosition.Y / 100.0;
    double Down = -UnrealPosition.Z / 100.0;

    // Convert origin to ECEF
    FVector OriginECEF = SimulationOrigin.ToECEF();

    double LatRad = FMath::DegreesToRadians(SimulationOrigin.Latitude);
    double LonRad = FMath::DegreesToRadians(SimulationOrigin.Longitude);

    double SinLat = FMath::Sin(LatRad);
    double CosLat = FMath::Cos(LatRad);
    double SinLon = FMath::Sin(LonRad);
    double CosLon = FMath::Cos(LonRad);

    // NED to ECEF rotation (transpose of ECEF to NED)
    double dx = -SinLat * CosLon * North - SinLon * East - CosLat * CosLon * Down;
    double dy = -SinLat * SinLon * North + CosLon * East - CosLat * SinLon * Down;
    double dz = CosLat * North - SinLat * Down;

    FVector PointECEF = OriginECEF + FVector(dx, dy, dz);

    return FJaguarGeodeticPosition::FromECEF(PointECEF);
}

FQuat UJaguarSubsystem::UnrealRotationToNED(const FRotator& UnrealRotation) const
{
    // Unreal: X-forward, Y-right, Z-up
    // NED: X-north, Y-east, Z-down

    // Convert to radians and swap axes
    FQuat UEQuat = UnrealRotation.Quaternion();

    // Apply coordinate transform
    // This is a simplified transform - full implementation would account for geodetic frame
    return FQuat(UEQuat.X, UEQuat.Y, -UEQuat.Z, UEQuat.W);
}

FRotator UJaguarSubsystem::NEDToUnrealRotation(const FQuat& NEDOrientation) const
{
    // Inverse of above transform
    FQuat UEQuat(NEDOrientation.X, NEDOrientation.Y, -NEDOrientation.Z, NEDOrientation.W);
    return UEQuat.Rotator();
}

FJaguarAtmosphere UJaguarSubsystem::GetAtmosphereAt(const FJaguarGeodeticPosition& Position) const
{
    FJaguarAtmosphere Atmo;

    // US Standard Atmosphere 1976 implementation
    double h = Position.Altitude;

    // Troposphere (0-11km)
    if (h < 11000.0)
    {
        double T0 = 288.15;
        double L = 0.0065;
        double P0 = 101325.0;
        double g = 9.80665;
        double M = 0.0289644;
        double R = 8.31447;

        Atmo.Temperature = T0 - L * h;
        Atmo.Pressure = P0 * FMath::Pow(Atmo.Temperature / T0, g * M / (R * L));
        Atmo.Density = Atmo.Pressure * M / (R * Atmo.Temperature);
    }
    // Stratosphere (11-20km)
    else if (h < 20000.0)
    {
        double T1 = 216.65;
        double P1 = 22632.0;
        double h1 = 11000.0;
        double g = 9.80665;
        double M = 0.0289644;
        double R = 8.31447;

        Atmo.Temperature = T1;
        Atmo.Pressure = P1 * FMath::Exp(-g * M * (h - h1) / (R * T1));
        Atmo.Density = Atmo.Pressure * M / (R * T1);
    }
    else
    {
        // Simplified above 20km
        Atmo.Temperature = 216.65;
        Atmo.Pressure = 5474.87 * FMath::Exp(-(h - 20000.0) / 6341.62);
        Atmo.Density = Atmo.Pressure * 0.0289644 / (8.31447 * Atmo.Temperature);
    }

    // Speed of sound
    Atmo.SpeedOfSound = FMath::Sqrt(1.4 * 287.05 * Atmo.Temperature);

    return Atmo;
}

FJaguarTerrainInfo UJaguarSubsystem::GetTerrainAt(const FJaguarGeodeticPosition& Position) const
{
    FJaguarTerrainInfo Info;

    // Would query native terrain system
    // For now, use Unreal's line trace

    FVector WorldPos = GeodeticToUnreal(Position);
    FVector TraceStart = WorldPos + FVector(0, 0, 100000.0);  // 1km up
    FVector TraceEnd = WorldPos - FVector(0, 0, 100000.0);    // 1km down

    FHitResult HitResult;
    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = true;

    if (GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, ECC_WorldStatic, QueryParams))
    {
        // Convert hit to geodetic altitude
        FJaguarGeodeticPosition HitGeo = UnrealToGeodetic(HitResult.Location);
        Info.Elevation = HitGeo.Altitude;
        Info.Normal = HitResult.Normal;

        // Get surface type from physical material
        if (HitResult.PhysMaterial.IsValid())
        {
            Info.FrictionCoefficient = HitResult.PhysMaterial->Friction;
        }
    }

    return Info;
}

void UJaguarSubsystem::SetWind(const FVector& WindVelocity, float TurbulenceIntensity)
{
    // Would set in native engine
    UE_LOG(LogJaguarEngine, Log, TEXT("Wind set to (%.1f, %.1f, %.1f) m/s, turbulence %.2f"),
        WindVelocity.X, WindVelocity.Y, WindVelocity.Z, TurbulenceIntensity);
}

void UJaguarSubsystem::ConfigureNetwork(const FJaguarNetworkSettings& Settings)
{
    NetworkSettings = Settings;
    UE_LOG(LogJaguarEngine, Log, TEXT("Network configured: DIS=%s, HLA=%s"),
        Settings.bEnableDIS ? TEXT("Yes") : TEXT("No"),
        Settings.bEnableHLA ? TEXT("Yes") : TEXT("No"));
}

bool UJaguarSubsystem::ConnectNetwork()
{
    if (bNetworkConnected)
    {
        return true;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Connecting to network..."));

    // Would initialize DIS/HLA connections via native library
    // For now, simulate success
    bNetworkConnected = true;

    UE_LOG(LogJaguarEngine, Log, TEXT("Network connected"));
    return true;
}

void UJaguarSubsystem::DisconnectNetwork()
{
    if (!bNetworkConnected)
    {
        return;
    }

    UE_LOG(LogJaguarEngine, Log, TEXT("Disconnecting from network..."));

    // Would cleanup network connections
    bNetworkConnected = false;

    UE_LOG(LogJaguarEngine, Log, TEXT("Network disconnected"));
}

bool UJaguarSubsystem::InitializeNativeEngine()
{
    if (!FJaguarEngineModule::IsAvailable() || !FJaguarEngineModule::Get().IsNativeLibraryLoaded())
    {
        return false;
    }

    // Would create native engine instance
    // NativeEngineHandle = jaguar::Engine::create();

    return true;
}

void UJaguarSubsystem::ShutdownNativeEngine()
{
    if (NativeEngineHandle)
    {
        // Would destroy native engine
        // jaguar::Engine::destroy(NativeEngineHandle);
        NativeEngineHandle = nullptr;
    }
}

void UJaguarSubsystem::SyncEntityStates()
{
    for (auto& Pair : RegisteredEntities)
    {
        if (Pair.Value.IsValid())
        {
            Pair.Value->SyncWithNativeEngine();
        }
    }
}

void UJaguarSubsystem::ProcessCollisionEvents()
{
    // Would query native engine for collision events and broadcast
}

void UJaguarSubsystem::ProcessNetworkUpdates()
{
    // Would process incoming/outgoing network updates
}

// Geodetic position helper implementations
FVector FJaguarGeodeticPosition::ToECEF() const
{
    double LatRad = FMath::DegreesToRadians(Latitude);
    double LonRad = FMath::DegreesToRadians(Longitude);

    double SinLat = FMath::Sin(LatRad);
    double CosLat = FMath::Cos(LatRad);
    double SinLon = FMath::Sin(LonRad);
    double CosLon = FMath::Cos(LonRad);

    // Prime vertical radius of curvature
    double N = WGS84::SemiMajorAxis / FMath::Sqrt(1.0 - WGS84::EccentricitySquared * SinLat * SinLat);

    double X = (N + Altitude) * CosLat * CosLon;
    double Y = (N + Altitude) * CosLat * SinLon;
    double Z = (N * (1.0 - WGS84::EccentricitySquared) + Altitude) * SinLat;

    return FVector(X, Y, Z);
}

FJaguarGeodeticPosition FJaguarGeodeticPosition::FromECEF(const FVector& ECEF)
{
    FJaguarGeodeticPosition Result;

    double X = ECEF.X;
    double Y = ECEF.Y;
    double Z = ECEF.Z;

    // Longitude is easy
    Result.Longitude = FMath::RadiansToDegrees(FMath::Atan2(Y, X));

    // Iterative solution for latitude and altitude (Bowring's method)
    double p = FMath::Sqrt(X * X + Y * Y);
    double Lat = FMath::Atan2(Z, p * (1.0 - WGS84::EccentricitySquared));

    for (int i = 0; i < 10; ++i)
    {
        double SinLat = FMath::Sin(Lat);
        double N = WGS84::SemiMajorAxis / FMath::Sqrt(1.0 - WGS84::EccentricitySquared * SinLat * SinLat);
        Lat = FMath::Atan2(Z + WGS84::EccentricitySquared * N * SinLat, p);
    }

    Result.Latitude = FMath::RadiansToDegrees(Lat);

    double SinLat = FMath::Sin(Lat);
    double CosLat = FMath::Cos(Lat);
    double N = WGS84::SemiMajorAxis / FMath::Sqrt(1.0 - WGS84::EccentricitySquared * SinLat * SinLat);

    if (FMath::Abs(CosLat) > 1e-10)
    {
        Result.Altitude = p / CosLat - N;
    }
    else
    {
        Result.Altitude = FMath::Abs(Z) - WGS84::SemiMinorAxis;
    }

    return Result;
}

FVector FJaguarGeodeticPosition::ToUnrealVector(const FJaguarGeodeticPosition& Origin) const
{
    // This would use the subsystem's conversion
    // Simplified standalone implementation
    FVector OriginECEF = Origin.ToECEF();
    FVector PointECEF = ToECEF();
    FVector Diff = PointECEF - OriginECEF;

    // Very simplified - doesn't account for Earth curvature properly
    return FVector(Diff.X * 100.0, Diff.Y * 100.0, Diff.Z * 100.0);
}

FJaguarGeodeticPosition FJaguarGeodeticPosition::FromUnrealVector(const FVector& Vector, const FJaguarGeodeticPosition& Origin)
{
    FVector OriginECEF = Origin.ToECEF();
    FVector PointECEF = OriginECEF + FVector(Vector.X / 100.0, Vector.Y / 100.0, Vector.Z / 100.0);
    return FromECEF(PointECEF);
}
