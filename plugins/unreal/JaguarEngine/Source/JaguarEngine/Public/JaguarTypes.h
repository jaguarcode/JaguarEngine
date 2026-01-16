// Copyright JaguarEngine Team. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "JaguarTypes.generated.h"

/**
 * Domain type for JaguarEngine entities
 */
UENUM(BlueprintType)
enum class EJaguarDomain : uint8
{
    Air        UMETA(DisplayName = "Air"),
    Land       UMETA(DisplayName = "Land"),
    Sea        UMETA(DisplayName = "Sea"),
    Space      UMETA(DisplayName = "Space"),
    Subsurface UMETA(DisplayName = "Subsurface"),
    Other      UMETA(DisplayName = "Other")
};

/**
 * Entity kind enumeration (DIS compatible)
 */
UENUM(BlueprintType)
enum class EJaguarEntityKind : uint8
{
    Other           = 0 UMETA(DisplayName = "Other"),
    Platform        = 1 UMETA(DisplayName = "Platform"),
    Munition        = 2 UMETA(DisplayName = "Munition"),
    LifeForm        = 3 UMETA(DisplayName = "Life Form"),
    Environmental   = 4 UMETA(DisplayName = "Environmental"),
    CulturalFeature = 5 UMETA(DisplayName = "Cultural Feature"),
    Supply          = 6 UMETA(DisplayName = "Supply"),
    Radio           = 7 UMETA(DisplayName = "Radio"),
    Expendable      = 8 UMETA(DisplayName = "Expendable"),
    SensorEmitter   = 9 UMETA(DisplayName = "Sensor/Emitter")
};

/**
 * Simulation state enumeration
 */
UENUM(BlueprintType)
enum class EJaguarSimulationState : uint8
{
    Stopped   UMETA(DisplayName = "Stopped"),
    Running   UMETA(DisplayName = "Running"),
    Paused    UMETA(DisplayName = "Paused"),
    Stepping  UMETA(DisplayName = "Stepping")
};

/**
 * Integration mode for physics
 */
UENUM(BlueprintType)
enum class EJaguarIntegrationMode : uint8
{
    ReplaceUEPhysics  UMETA(DisplayName = "Replace UE Physics"),
    Parallel          UMETA(DisplayName = "Parallel (Both Active)"),
    JaguarOnly        UMETA(DisplayName = "Jaguar Only"),
    Synchronized      UMETA(DisplayName = "Synchronized")
};

/**
 * Coordinate system type
 */
UENUM(BlueprintType)
enum class EJaguarCoordinateSystem : uint8
{
    ECEF      UMETA(DisplayName = "ECEF (Earth-Centered Earth-Fixed)"),
    LLA       UMETA(DisplayName = "LLA (Latitude/Longitude/Altitude)"),
    NED       UMETA(DisplayName = "NED (North-East-Down)"),
    ENU       UMETA(DisplayName = "ENU (East-North-Up)"),
    UE        UMETA(DisplayName = "Unreal Engine (Z-Up, cm)")
};

/**
 * Geodetic position (WGS84)
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarGeodeticPosition
{
    GENERATED_BODY()

    /** Latitude in degrees (-90 to +90) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position", meta = (ClampMin = "-90.0", ClampMax = "90.0"))
    double Latitude = 0.0;

    /** Longitude in degrees (-180 to +180) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position", meta = (ClampMin = "-180.0", ClampMax = "180.0"))
    double Longitude = 0.0;

    /** Altitude above WGS84 ellipsoid in meters */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Position")
    double Altitude = 0.0;

    FJaguarGeodeticPosition() = default;

    FJaguarGeodeticPosition(double InLatitude, double InLongitude, double InAltitude)
        : Latitude(InLatitude), Longitude(InLongitude), Altitude(InAltitude) {}

    /** Convert to FVector (UE coordinates, requires origin) */
    FVector ToUnrealVector(const FJaguarGeodeticPosition& Origin) const;

    /** Create from FVector (UE coordinates, requires origin) */
    static FJaguarGeodeticPosition FromUnrealVector(const FVector& Vector, const FJaguarGeodeticPosition& Origin);

    /** Convert to ECEF coordinates */
    FVector ToECEF() const;

    /** Create from ECEF coordinates */
    static FJaguarGeodeticPosition FromECEF(const FVector& ECEF);

    bool operator==(const FJaguarGeodeticPosition& Other) const
    {
        return FMath::IsNearlyEqual(Latitude, Other.Latitude, 1e-9) &&
               FMath::IsNearlyEqual(Longitude, Other.Longitude, 1e-9) &&
               FMath::IsNearlyEqual(Altitude, Other.Altitude, 1e-3);
    }

    FString ToString() const
    {
        return FString::Printf(TEXT("(%.6f, %.6f, %.1fm)"), Latitude, Longitude, Altitude);
    }
};

/**
 * Entity state for JaguarEngine entities
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarEntityState
{
    GENERATED_BODY()

    /** Entity unique identifier */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Identity")
    int64 EntityId = 0;

    /** Entity name */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Identity")
    FString EntityName;

    /** Domain type */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Identity")
    EJaguarDomain Domain = EJaguarDomain::Other;

    /** Entity kind */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Identity")
    EJaguarEntityKind Kind = EJaguarEntityKind::Other;

    /** Geodetic position */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FJaguarGeodeticPosition Position;

    /** Velocity in body frame (m/s) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FVector Velocity = FVector::ZeroVector;

    /** Acceleration in body frame (m/s²) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FVector Acceleration = FVector::ZeroVector;

    /** Orientation (body to world) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FQuat Orientation = FQuat::Identity;

    /** Angular velocity in body frame (rad/s) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FVector AngularVelocity = FVector::ZeroVector;

    /** Euler angles (Roll, Pitch, Yaw) in degrees */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Kinematics")
    FRotator EulerAngles = FRotator::ZeroRotator;

    /** Simulation timestamp */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time")
    double Timestamp = 0.0;

    /** Is entity active in simulation */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Status")
    bool bIsActive = true;

    /** Is entity controlled locally */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Status")
    bool bIsLocal = true;
};

/**
 * Atmospheric conditions
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarAtmosphere
{
    GENERATED_BODY()

    /** Temperature in Kelvin */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere")
    double Temperature = 288.15;

    /** Pressure in Pascals */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere")
    double Pressure = 101325.0;

    /** Density in kg/m³ */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere")
    double Density = 1.225;

    /** Speed of sound in m/s */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere")
    double SpeedOfSound = 340.29;

    /** Wind velocity in NED frame (m/s) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind")
    FVector WindVelocity = FVector::ZeroVector;

    /** Turbulence intensity (0-1) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wind", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float TurbulenceIntensity = 0.0f;
};

/**
 * Terrain query result
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarTerrainInfo
{
    GENERATED_BODY()

    /** Terrain elevation at query point (meters MSL) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    double Elevation = 0.0;

    /** Terrain normal vector */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    FVector Normal = FVector::UpVector;

    /** Surface type identifier */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    int32 SurfaceType = 0;

    /** Surface friction coefficient */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    float FrictionCoefficient = 0.7f;

    /** Is point over water */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    bool bIsWater = false;

    /** Water depth if over water (meters) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain")
    double WaterDepth = 0.0;
};

/**
 * Simulation statistics
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarSimulationStats
{
    GENERATED_BODY()

    /** Current simulation time */
    UPROPERTY(BlueprintReadOnly, Category = "Time")
    double SimulationTime = 0.0;

    /** Wall clock time elapsed */
    UPROPERTY(BlueprintReadOnly, Category = "Time")
    double WallClockTime = 0.0;

    /** Real-time ratio (sim time / wall time) */
    UPROPERTY(BlueprintReadOnly, Category = "Performance")
    float RealTimeRatio = 1.0f;

    /** Number of active entities */
    UPROPERTY(BlueprintReadOnly, Category = "Entities")
    int32 ActiveEntityCount = 0;

    /** Physics step time in milliseconds */
    UPROPERTY(BlueprintReadOnly, Category = "Performance")
    float PhysicsStepTimeMs = 0.0f;

    /** Collision detection time in milliseconds */
    UPROPERTY(BlueprintReadOnly, Category = "Performance")
    float CollisionTimeMs = 0.0f;

    /** Network update time in milliseconds */
    UPROPERTY(BlueprintReadOnly, Category = "Performance")
    float NetworkTimeMs = 0.0f;

    /** Memory usage in MB */
    UPROPERTY(BlueprintReadOnly, Category = "Resources")
    float MemoryUsageMB = 0.0f;
};

/**
 * DIS/HLA network settings
 */
USTRUCT(BlueprintType)
struct JAGUARENGINE_API FJaguarNetworkSettings
{
    GENERATED_BODY()

    /** Enable DIS protocol */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS")
    bool bEnableDIS = false;

    /** DIS exercise ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS", meta = (EditCondition = "bEnableDIS"))
    int32 DISExerciseId = 1;

    /** DIS site ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS", meta = (EditCondition = "bEnableDIS"))
    int32 DISSiteId = 1;

    /** DIS application ID */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS", meta = (EditCondition = "bEnableDIS"))
    int32 DISApplicationId = 1;

    /** DIS broadcast address */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS", meta = (EditCondition = "bEnableDIS"))
    FString DISBroadcastAddress = TEXT("255.255.255.255");

    /** DIS port */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DIS", meta = (EditCondition = "bEnableDIS"))
    int32 DISPort = 3000;

    /** Enable HLA federation */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HLA")
    bool bEnableHLA = false;

    /** HLA federation name */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HLA", meta = (EditCondition = "bEnableHLA"))
    FString HLAFederationName = TEXT("JaguarSimulation");

    /** HLA federate name */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HLA", meta = (EditCondition = "bEnableHLA"))
    FString HLAFederateName = TEXT("JaguarUE5");

    /** HLA RTI host */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HLA", meta = (EditCondition = "bEnableHLA"))
    FString HLARTIHost = TEXT("localhost");

    /** HLA RTI port */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HLA", meta = (EditCondition = "bEnableHLA"))
    int32 HLARTIPort = 8989;
};
