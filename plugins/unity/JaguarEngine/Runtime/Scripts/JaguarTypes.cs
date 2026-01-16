// Copyright JaguarEngine Team. All Rights Reserved.

using System;
using UnityEngine;
using Unity.Mathematics;

namespace JaguarEngine
{
    /// <summary>
    /// Domain type for JaguarEngine entities
    /// </summary>
    public enum JaguarDomain
    {
        Air,
        Land,
        Sea,
        Space,
        Subsurface,
        Other
    }

    /// <summary>
    /// Entity kind enumeration (DIS compatible)
    /// </summary>
    public enum JaguarEntityKind
    {
        Other = 0,
        Platform = 1,
        Munition = 2,
        LifeForm = 3,
        Environmental = 4,
        CulturalFeature = 5,
        Supply = 6,
        Radio = 7,
        Expendable = 8,
        SensorEmitter = 9
    }

    /// <summary>
    /// Simulation state enumeration
    /// </summary>
    public enum JaguarSimulationState
    {
        Stopped,
        Running,
        Paused,
        Stepping
    }

    /// <summary>
    /// Integration mode for physics
    /// </summary>
    public enum JaguarIntegrationMode
    {
        ReplaceUnityPhysics,
        Parallel,
        JaguarOnly,
        Synchronized
    }

    /// <summary>
    /// Coordinate system type
    /// </summary>
    public enum JaguarCoordinateSystem
    {
        ECEF,
        LLA,
        NED,
        ENU,
        Unity
    }

    /// <summary>
    /// Geodetic position (WGS84)
    /// </summary>
    [Serializable]
    public struct GeodeticPosition : IEquatable<GeodeticPosition>
    {
        /// <summary>Latitude in degrees (-90 to +90)</summary>
        [Range(-90f, 90f)]
        public double latitude;

        /// <summary>Longitude in degrees (-180 to +180)</summary>
        [Range(-180f, 180f)]
        public double longitude;

        /// <summary>Altitude above WGS84 ellipsoid in meters</summary>
        public double altitude;

        // WGS84 constants
        private const double SemiMajorAxis = 6378137.0;
        private const double Flattening = 1.0 / 298.257223563;
        private const double SemiMinorAxis = SemiMajorAxis * (1.0 - Flattening);
        private const double EccentricitySquared = 2.0 * Flattening - Flattening * Flattening;

        public GeodeticPosition(double lat, double lon, double alt)
        {
            latitude = lat;
            longitude = lon;
            altitude = alt;
        }

        /// <summary>
        /// Convert to ECEF coordinates
        /// </summary>
        public double3 ToECEF()
        {
            double latRad = math.radians(latitude);
            double lonRad = math.radians(longitude);

            double sinLat = math.sin(latRad);
            double cosLat = math.cos(latRad);
            double sinLon = math.sin(lonRad);
            double cosLon = math.cos(lonRad);

            double N = SemiMajorAxis / math.sqrt(1.0 - EccentricitySquared * sinLat * sinLat);

            double x = (N + altitude) * cosLat * cosLon;
            double y = (N + altitude) * cosLat * sinLon;
            double z = (N * (1.0 - EccentricitySquared) + altitude) * sinLat;

            return new double3(x, y, z);
        }

        /// <summary>
        /// Create from ECEF coordinates
        /// </summary>
        public static GeodeticPosition FromECEF(double3 ecef)
        {
            double x = ecef.x;
            double y = ecef.y;
            double z = ecef.z;

            double lon = math.degrees(math.atan2(y, x));

            double p = math.sqrt(x * x + y * y);
            double lat = math.atan2(z, p * (1.0 - EccentricitySquared));

            // Iterative solution (Bowring's method)
            for (int i = 0; i < 10; i++)
            {
                double sinLat = math.sin(lat);
                double N = SemiMajorAxis / math.sqrt(1.0 - EccentricitySquared * sinLat * sinLat);
                lat = math.atan2(z + EccentricitySquared * N * sinLat, p);
            }

            double sinLatFinal = math.sin(lat);
            double cosLatFinal = math.cos(lat);
            double N_final = SemiMajorAxis / math.sqrt(1.0 - EccentricitySquared * sinLatFinal * sinLatFinal);

            double alt;
            if (math.abs(cosLatFinal) > 1e-10)
            {
                alt = p / cosLatFinal - N_final;
            }
            else
            {
                alt = math.abs(z) - SemiMinorAxis;
            }

            return new GeodeticPosition(math.degrees(lat), lon, alt);
        }

        /// <summary>
        /// Convert to Unity world position relative to origin
        /// </summary>
        public Vector3 ToUnityPosition(GeodeticPosition origin)
        {
            double3 originECEF = origin.ToECEF();
            double3 pointECEF = ToECEF();
            double3 diff = pointECEF - originECEF;

            double latRad = math.radians(origin.latitude);
            double lonRad = math.radians(origin.longitude);

            double sinLat = math.sin(latRad);
            double cosLat = math.cos(latRad);
            double sinLon = math.sin(lonRad);
            double cosLon = math.cos(lonRad);

            // ECEF to NED rotation
            double north = -sinLat * cosLon * diff.x - sinLat * sinLon * diff.y + cosLat * diff.z;
            double east = -sinLon * diff.x + cosLon * diff.y;
            double down = -cosLat * cosLon * diff.x - cosLat * sinLon * diff.y - sinLat * diff.z;

            // NED to Unity: X=East, Y=Up, Z=North
            return new Vector3((float)east, (float)(-down), (float)north);
        }

        /// <summary>
        /// Create from Unity world position relative to origin
        /// </summary>
        public static GeodeticPosition FromUnityPosition(Vector3 position, GeodeticPosition origin)
        {
            // Unity to NED: North=Z, East=X, Down=-Y
            double north = position.z;
            double east = position.x;
            double down = -position.y;

            double latRad = math.radians(origin.latitude);
            double lonRad = math.radians(origin.longitude);

            double sinLat = math.sin(latRad);
            double cosLat = math.cos(latRad);
            double sinLon = math.sin(lonRad);
            double cosLon = math.cos(lonRad);

            // NED to ECEF rotation (transpose)
            double dx = -sinLat * cosLon * north - sinLon * east - cosLat * cosLon * down;
            double dy = -sinLat * sinLon * north + cosLon * east - cosLat * sinLon * down;
            double dz = cosLat * north - sinLat * down;

            double3 originECEF = origin.ToECEF();
            double3 pointECEF = originECEF + new double3(dx, dy, dz);

            return FromECEF(pointECEF);
        }

        public bool Equals(GeodeticPosition other)
        {
            return math.abs(latitude - other.latitude) < 1e-9 &&
                   math.abs(longitude - other.longitude) < 1e-9 &&
                   math.abs(altitude - other.altitude) < 1e-3;
        }

        public override bool Equals(object obj)
        {
            return obj is GeodeticPosition other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(latitude, longitude, altitude);
        }

        public override string ToString()
        {
            return $"({latitude:F6}, {longitude:F6}, {altitude:F1}m)";
        }

        public static bool operator ==(GeodeticPosition left, GeodeticPosition right) => left.Equals(right);
        public static bool operator !=(GeodeticPosition left, GeodeticPosition right) => !left.Equals(right);
    }

    /// <summary>
    /// Entity state for JaguarEngine entities
    /// </summary>
    [Serializable]
    public struct JaguarEntityState
    {
        /// <summary>Entity unique identifier</summary>
        public long entityId;

        /// <summary>Entity name</summary>
        public string entityName;

        /// <summary>Domain type</summary>
        public JaguarDomain domain;

        /// <summary>Entity kind</summary>
        public JaguarEntityKind kind;

        /// <summary>Geodetic position</summary>
        public GeodeticPosition position;

        /// <summary>Velocity in body frame (m/s)</summary>
        public Vector3 velocity;

        /// <summary>Acceleration in body frame (m/s²)</summary>
        public Vector3 acceleration;

        /// <summary>Orientation (body to world)</summary>
        public Quaternion orientation;

        /// <summary>Angular velocity in body frame (rad/s)</summary>
        public Vector3 angularVelocity;

        /// <summary>Euler angles (degrees)</summary>
        public Vector3 eulerAngles;

        /// <summary>Simulation timestamp</summary>
        public double timestamp;

        /// <summary>Is entity active in simulation</summary>
        public bool isActive;

        /// <summary>Is entity controlled locally</summary>
        public bool isLocal;
    }

    /// <summary>
    /// Atmospheric conditions
    /// </summary>
    [Serializable]
    public struct JaguarAtmosphere
    {
        /// <summary>Temperature in Kelvin</summary>
        public double temperature;

        /// <summary>Pressure in Pascals</summary>
        public double pressure;

        /// <summary>Density in kg/m³</summary>
        public double density;

        /// <summary>Speed of sound in m/s</summary>
        public double speedOfSound;

        /// <summary>Wind velocity in NED frame (m/s)</summary>
        public Vector3 windVelocity;

        /// <summary>Turbulence intensity (0-1)</summary>
        [Range(0f, 1f)]
        public float turbulenceIntensity;

        /// <summary>
        /// Get US Standard Atmosphere at altitude
        /// </summary>
        public static JaguarAtmosphere GetStandardAtmosphere(double altitudeMeters)
        {
            JaguarAtmosphere atmo = new JaguarAtmosphere();

            // US Standard Atmosphere 1976
            if (altitudeMeters < 11000.0)
            {
                // Troposphere
                double T0 = 288.15;
                double L = 0.0065;
                double P0 = 101325.0;
                double g = 9.80665;
                double M = 0.0289644;
                double R = 8.31447;

                atmo.temperature = T0 - L * altitudeMeters;
                atmo.pressure = P0 * math.pow(atmo.temperature / T0, g * M / (R * L));
                atmo.density = atmo.pressure * M / (R * atmo.temperature);
            }
            else if (altitudeMeters < 20000.0)
            {
                // Stratosphere (isothermal)
                double T1 = 216.65;
                double P1 = 22632.0;
                double h1 = 11000.0;
                double g = 9.80665;
                double M = 0.0289644;
                double R = 8.31447;

                atmo.temperature = T1;
                atmo.pressure = P1 * math.exp(-g * M * (altitudeMeters - h1) / (R * T1));
                atmo.density = atmo.pressure * M / (R * T1);
            }
            else
            {
                // Simplified above 20km
                atmo.temperature = 216.65;
                atmo.pressure = 5474.87 * math.exp(-(altitudeMeters - 20000.0) / 6341.62);
                atmo.density = atmo.pressure * 0.0289644 / (8.31447 * atmo.temperature);
            }

            atmo.speedOfSound = math.sqrt(1.4 * 287.05 * atmo.temperature);
            atmo.windVelocity = Vector3.zero;
            atmo.turbulenceIntensity = 0f;

            return atmo;
        }
    }

    /// <summary>
    /// Terrain query result
    /// </summary>
    [Serializable]
    public struct JaguarTerrainInfo
    {
        /// <summary>Terrain elevation at query point (meters MSL)</summary>
        public double elevation;

        /// <summary>Terrain normal vector</summary>
        public Vector3 normal;

        /// <summary>Surface type identifier</summary>
        public int surfaceType;

        /// <summary>Surface friction coefficient</summary>
        public float frictionCoefficient;

        /// <summary>Is point over water</summary>
        public bool isWater;

        /// <summary>Water depth if over water (meters)</summary>
        public double waterDepth;
    }

    /// <summary>
    /// Simulation statistics
    /// </summary>
    [Serializable]
    public struct JaguarSimulationStats
    {
        /// <summary>Current simulation time</summary>
        public double simulationTime;

        /// <summary>Wall clock time elapsed</summary>
        public double wallClockTime;

        /// <summary>Real-time ratio (sim time / wall time)</summary>
        public float realTimeRatio;

        /// <summary>Number of active entities</summary>
        public int activeEntityCount;

        /// <summary>Physics step time in milliseconds</summary>
        public float physicsStepTimeMs;

        /// <summary>Collision detection time in milliseconds</summary>
        public float collisionTimeMs;

        /// <summary>Network update time in milliseconds</summary>
        public float networkTimeMs;

        /// <summary>Memory usage in MB</summary>
        public float memoryUsageMB;
    }

    /// <summary>
    /// DIS/HLA network settings
    /// </summary>
    [Serializable]
    public struct JaguarNetworkSettings
    {
        /// <summary>Enable DIS protocol</summary>
        public bool enableDIS;

        /// <summary>DIS exercise ID</summary>
        public int disExerciseId;

        /// <summary>DIS site ID</summary>
        public int disSiteId;

        /// <summary>DIS application ID</summary>
        public int disApplicationId;

        /// <summary>DIS broadcast address</summary>
        public string disBroadcastAddress;

        /// <summary>DIS port</summary>
        public int disPort;

        /// <summary>Enable HLA federation</summary>
        public bool enableHLA;

        /// <summary>HLA federation name</summary>
        public string hlaFederationName;

        /// <summary>HLA federate name</summary>
        public string hlaFederateName;

        /// <summary>HLA RTI host</summary>
        public string hlaRTIHost;

        /// <summary>HLA RTI port</summary>
        public int hlaRTIPort;

        /// <summary>Get default settings</summary>
        public static JaguarNetworkSettings Default => new JaguarNetworkSettings
        {
            enableDIS = false,
            disExerciseId = 1,
            disSiteId = 1,
            disApplicationId = 1,
            disBroadcastAddress = "255.255.255.255",
            disPort = 3000,
            enableHLA = false,
            hlaFederationName = "JaguarSimulation",
            hlaFederateName = "JaguarUnity",
            hlaRTIHost = "localhost",
            hlaRTIPort = 8989
        };
    }
}
