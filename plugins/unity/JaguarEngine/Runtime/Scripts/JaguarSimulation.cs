// Copyright JaguarEngine Team. All Rights Reserved.

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using Unity.Mathematics;

namespace JaguarEngine
{
    /// <summary>
    /// Central simulation manager for JaguarEngine in Unity.
    /// Handles simulation lifecycle, entity registration, and coordinate transforms.
    /// </summary>
    public class JaguarSimulation : MonoBehaviour
    {
        #region Singleton
        private static JaguarSimulation _instance;
        public static JaguarSimulation Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindObjectOfType<JaguarSimulation>();
                    if (_instance == null)
                    {
                        var go = new GameObject("JaguarSimulation");
                        _instance = go.AddComponent<JaguarSimulation>();
                        DontDestroyOnLoad(go);
                    }
                }
                return _instance;
            }
        }
        #endregion

        #region Inspector Fields
        [Header("Simulation Origin")]
        [Tooltip("Geodetic origin for local coordinate transforms")]
        public GeodeticPosition simulationOrigin = new GeodeticPosition(34.905, -117.884, 702.0);

        [Header("Time Settings")]
        [Tooltip("Time scale multiplier (1.0 = real-time)")]
        [Range(0f, 100f)]
        public float timeScale = 1.0f;

        [Tooltip("Fixed physics timestep in seconds")]
        [Range(0.001f, 0.1f)]
        public float fixedTimestep = 0.01f;

        [Header("Network Settings")]
        public JaguarNetworkSettings networkSettings = JaguarNetworkSettings.Default;

        [Header("Debug")]
        public bool showDebugInfo = false;
        #endregion

        #region Events
        public event Action<JaguarSimulationState> OnSimulationStateChanged;
        public event Action<long> OnEntitySpawned;
        public event Action<long> OnEntityDestroyed;
        public event Action<long, long, Vector3> OnCollisionDetected;
        #endregion

        #region Properties
        public JaguarSimulationState SimulationState { get; private set; } = JaguarSimulationState.Stopped;
        public double SimulationTime { get; private set; } = 0.0;
        public bool IsNetworkConnected { get; private set; } = false;
        public int EntityCount => _registeredEntities.Count;
        #endregion

        #region Private Fields
        private Dictionary<long, JaguarEntity> _registeredEntities = new Dictionary<long, JaguarEntity>();
        private long _nextEntityId = 1;
        private double _wallClockStartTime = 0.0;
        private double _lastPhysicsTimeMs = 0.0;
        private double _lastCollisionTimeMs = 0.0;
        private double _lastNetworkTimeMs = 0.0;
        private IntPtr _nativeEngineHandle = IntPtr.Zero;
        private bool _nativeLibraryLoaded = false;
        #endregion

        #region Native Library Interface
        private const string NativeLibrary = "jaguar";

        // Native function declarations would go here
        // [DllImport(NativeLibrary)]
        // private static extern IntPtr jaguar_engine_create();

        private bool LoadNativeLibrary()
        {
            // Attempt to load native library
            try
            {
                // _nativeEngineHandle = jaguar_engine_create();
                _nativeLibraryLoaded = true;
                Debug.Log("[JaguarEngine] Native library loaded successfully");
                return true;
            }
            catch (DllNotFoundException)
            {
                Debug.LogWarning("[JaguarEngine] Native library not found - simulation features limited");
                return false;
            }
            catch (Exception e)
            {
                Debug.LogError($"[JaguarEngine] Failed to load native library: {e.Message}");
                return false;
            }
        }

        private void UnloadNativeLibrary()
        {
            if (_nativeEngineHandle != IntPtr.Zero)
            {
                // jaguar_engine_destroy(_nativeEngineHandle);
                _nativeEngineHandle = IntPtr.Zero;
            }
            _nativeLibraryLoaded = false;
        }
        #endregion

        #region Unity Lifecycle
        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);

            LoadNativeLibrary();
        }

        private void OnDestroy()
        {
            if (_instance == this)
            {
                StopSimulation();
                DisconnectNetwork();
                UnloadNativeLibrary();
                _instance = null;
            }
        }

        private void FixedUpdate()
        {
            if (SimulationState == JaguarSimulationState.Running)
            {
                TickSimulation(Time.fixedDeltaTime);
            }
        }

        private void OnGUI()
        {
            if (showDebugInfo)
            {
                DrawDebugInfo();
            }
        }
        #endregion

        #region Simulation Control
        /// <summary>
        /// Tick the JaguarEngine simulation
        /// </summary>
        public void TickSimulation(float deltaTime)
        {
            if (SimulationState != JaguarSimulationState.Running)
                return;

            var startTime = Time.realtimeSinceStartupAsDouble;

            float scaledDeltaTime = deltaTime * timeScale;

            // Sync entity states to native engine
            SyncEntityStates();

            var physicsEndTime = Time.realtimeSinceStartupAsDouble;
            _lastPhysicsTimeMs = (physicsEndTime - startTime) * 1000.0;

            // Process collisions
            ProcessCollisionEvents();

            var collisionEndTime = Time.realtimeSinceStartupAsDouble;
            _lastCollisionTimeMs = (collisionEndTime - physicsEndTime) * 1000.0;

            // Process network
            if (IsNetworkConnected)
            {
                ProcessNetworkUpdates();
            }

            var networkEndTime = Time.realtimeSinceStartupAsDouble;
            _lastNetworkTimeMs = (networkEndTime - collisionEndTime) * 1000.0;

            SimulationTime += scaledDeltaTime;
        }

        /// <summary>
        /// Start the simulation
        /// </summary>
        public void StartSimulation()
        {
            if (SimulationState == JaguarSimulationState.Running)
                return;

            Debug.Log("[JaguarEngine] Starting simulation...");

            SimulationState = JaguarSimulationState.Running;
            _wallClockStartTime = Time.realtimeSinceStartupAsDouble;

            OnSimulationStateChanged?.Invoke(SimulationState);

            Debug.Log($"[JaguarEngine] Simulation started with {_registeredEntities.Count} entities");
        }

        /// <summary>
        /// Stop the simulation
        /// </summary>
        public void StopSimulation()
        {
            if (SimulationState == JaguarSimulationState.Stopped)
                return;

            Debug.Log("[JaguarEngine] Stopping simulation...");

            SimulationState = JaguarSimulationState.Stopped;
            SimulationTime = 0.0;

            OnSimulationStateChanged?.Invoke(SimulationState);
        }

        /// <summary>
        /// Pause the simulation
        /// </summary>
        public void PauseSimulation()
        {
            if (SimulationState != JaguarSimulationState.Running)
                return;

            Debug.Log($"[JaguarEngine] Pausing simulation at t={SimulationTime:F3}");

            SimulationState = JaguarSimulationState.Paused;
            OnSimulationStateChanged?.Invoke(SimulationState);
        }

        /// <summary>
        /// Resume the simulation from pause
        /// </summary>
        public void ResumeSimulation()
        {
            if (SimulationState != JaguarSimulationState.Paused)
                return;

            Debug.Log($"[JaguarEngine] Resuming simulation from t={SimulationTime:F3}");

            SimulationState = JaguarSimulationState.Running;
            OnSimulationStateChanged?.Invoke(SimulationState);
        }

        /// <summary>
        /// Step the simulation by a fixed amount
        /// </summary>
        public void StepSimulation(float stepTime)
        {
            if (SimulationState == JaguarSimulationState.Running)
                PauseSimulation();

            var oldState = SimulationState;
            SimulationState = JaguarSimulationState.Stepping;

            TickSimulation(stepTime);

            SimulationState = oldState;
        }

        /// <summary>
        /// Get simulation statistics
        /// </summary>
        public JaguarSimulationStats GetSimulationStats()
        {
            double wallClockTime = Time.realtimeSinceStartupAsDouble - _wallClockStartTime;

            return new JaguarSimulationStats
            {
                simulationTime = SimulationTime,
                wallClockTime = wallClockTime,
                realTimeRatio = wallClockTime > 0 ? (float)(SimulationTime / wallClockTime) : 1f,
                activeEntityCount = _registeredEntities.Count,
                physicsStepTimeMs = (float)_lastPhysicsTimeMs,
                collisionTimeMs = (float)_lastCollisionTimeMs,
                networkTimeMs = (float)_lastNetworkTimeMs,
                memoryUsageMB = GC.GetTotalMemory(false) / (1024f * 1024f)
            };
        }
        #endregion

        #region Entity Management
        /// <summary>
        /// Register an entity with the simulation
        /// </summary>
        public long RegisterEntity(JaguarEntity entity)
        {
            if (entity == null)
                return 0;

            long entityId = _nextEntityId++;
            _registeredEntities[entityId] = entity;

            Debug.Log($"[JaguarEngine] Registered entity '{entity.entityName}' with ID {entityId}");

            OnEntitySpawned?.Invoke(entityId);

            return entityId;
        }

        /// <summary>
        /// Unregister an entity
        /// </summary>
        public void UnregisterEntity(long entityId)
        {
            if (_registeredEntities.Remove(entityId))
            {
                Debug.Log($"[JaguarEngine] Unregistered entity ID {entityId}");
                OnEntityDestroyed?.Invoke(entityId);
            }
        }

        /// <summary>
        /// Get entity by ID
        /// </summary>
        public JaguarEntity GetEntityById(long entityId)
        {
            _registeredEntities.TryGetValue(entityId, out var entity);
            return entity;
        }

        /// <summary>
        /// Get all entity IDs
        /// </summary>
        public IEnumerable<long> GetAllEntityIds() => _registeredEntities.Keys;
        #endregion

        #region Coordinate Transforms
        /// <summary>
        /// Convert geodetic position to Unity coordinates
        /// </summary>
        public Vector3 GeodeticToUnity(GeodeticPosition geodetic)
        {
            return geodetic.ToUnityPosition(simulationOrigin);
        }

        /// <summary>
        /// Convert Unity coordinates to geodetic position
        /// </summary>
        public GeodeticPosition UnityToGeodetic(Vector3 unityPosition)
        {
            return GeodeticPosition.FromUnityPosition(unityPosition, simulationOrigin);
        }

        /// <summary>
        /// Convert Unity rotation to NED orientation
        /// </summary>
        public Quaternion UnityRotationToNED(Quaternion unityRotation)
        {
            // Unity: X-right, Y-up, Z-forward
            // NED: X-north, Y-east, Z-down
            // This is a simplified transform
            return new Quaternion(unityRotation.z, unityRotation.x, -unityRotation.y, unityRotation.w);
        }

        /// <summary>
        /// Convert NED orientation to Unity rotation
        /// </summary>
        public Quaternion NEDToUnityRotation(Quaternion nedOrientation)
        {
            return new Quaternion(nedOrientation.y, -nedOrientation.z, nedOrientation.x, nedOrientation.w);
        }
        #endregion

        #region Environment
        /// <summary>
        /// Get atmospheric conditions at a position
        /// </summary>
        public JaguarAtmosphere GetAtmosphereAt(GeodeticPosition position)
        {
            return JaguarAtmosphere.GetStandardAtmosphere(position.altitude);
        }

        /// <summary>
        /// Get terrain information at a position
        /// </summary>
        public JaguarTerrainInfo GetTerrainAt(GeodeticPosition position)
        {
            JaguarTerrainInfo info = new JaguarTerrainInfo();

            Vector3 worldPos = GeodeticToUnity(position);
            Vector3 rayStart = worldPos + Vector3.up * 10000f;
            Vector3 rayEnd = worldPos + Vector3.down * 10000f;

            if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, 20000f))
            {
                GeodeticPosition hitGeo = UnityToGeodetic(hit.point);
                info.elevation = hitGeo.altitude;
                info.normal = hit.normal;

                if (hit.collider.material != null)
                {
                    info.frictionCoefficient = hit.collider.material.staticFriction;
                }
            }

            return info;
        }
        #endregion

        #region Network
        /// <summary>
        /// Configure network settings
        /// </summary>
        public void ConfigureNetwork(JaguarNetworkSettings settings)
        {
            networkSettings = settings;
            Debug.Log($"[JaguarEngine] Network configured: DIS={settings.enableDIS}, HLA={settings.enableHLA}");
        }

        /// <summary>
        /// Connect to DIS/HLA network
        /// </summary>
        public bool ConnectNetwork()
        {
            if (IsNetworkConnected)
                return true;

            Debug.Log("[JaguarEngine] Connecting to network...");

            // Would initialize network connections
            IsNetworkConnected = true;

            Debug.Log("[JaguarEngine] Network connected");
            return true;
        }

        /// <summary>
        /// Disconnect from network
        /// </summary>
        public void DisconnectNetwork()
        {
            if (!IsNetworkConnected)
                return;

            Debug.Log("[JaguarEngine] Disconnecting from network...");

            IsNetworkConnected = false;

            Debug.Log("[JaguarEngine] Network disconnected");
        }
        #endregion

        #region Private Methods
        private void SyncEntityStates()
        {
            foreach (var kvp in _registeredEntities)
            {
                kvp.Value?.SyncWithSimulation();
            }
        }

        private void ProcessCollisionEvents()
        {
            // Would query native engine for collision events
        }

        private void ProcessNetworkUpdates()
        {
            // Would process DIS/HLA network updates
        }

        private void DrawDebugInfo()
        {
            var stats = GetSimulationStats();

            GUILayout.BeginArea(new Rect(10, 10, 300, 200));
            GUILayout.BeginVertical("box");

            GUILayout.Label($"JaguarEngine Simulation");
            GUILayout.Label($"State: {SimulationState}");
            GUILayout.Label($"Time: {SimulationTime:F3}s");
            GUILayout.Label($"Entities: {stats.activeEntityCount}");
            GUILayout.Label($"RTR: {stats.realTimeRatio:F2}x");
            GUILayout.Label($"Physics: {stats.physicsStepTimeMs:F2}ms");
            GUILayout.Label($"Network: {IsNetworkConnected}");

            GUILayout.EndVertical();
            GUILayout.EndArea();
        }
        #endregion
    }
}
