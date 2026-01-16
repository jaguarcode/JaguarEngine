// Copyright JaguarEngine Team. All Rights Reserved.

using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace JaguarEngine
{
    /// <summary>
    /// Entity component for JaguarEngine simulation in Unity.
    /// Attach to GameObjects to make them simulation-aware entities.
    /// </summary>
    [AddComponentMenu("JaguarEngine/Jaguar Entity")]
    [DisallowMultipleComponent]
    public class JaguarEntity : MonoBehaviour
    {
        #region Inspector Fields
        [Header("Entity Identity")]
        [Tooltip("Human-readable entity name")]
        public string entityName = "Entity";

        [Tooltip("Domain type for this entity")]
        public JaguarDomain domain = JaguarDomain.Air;

        [Tooltip("Entity kind (DIS compatible)")]
        public JaguarEntityKind kind = JaguarEntityKind.Platform;

        [Header("Physics Integration")]
        [Tooltip("How Jaguar physics integrates with Unity physics")]
        public JaguarIntegrationMode integrationMode = JaguarIntegrationMode.ReplaceUnityPhysics;

        [Header("Network Settings")]
        [Tooltip("Whether this entity is locally controlled")]
        public bool isLocalEntity = true;

        [Tooltip("Network update rate in Hz")]
        [Range(1f, 60f)]
        public float networkUpdateRate = 15f;

        [Tooltip("Dead reckoning threshold in meters")]
        [Range(0.1f, 100f)]
        public float deadReckoningThreshold = 1.0f;

        [Header("Damage")]
        [Tooltip("Current health (0-1)")]
        [Range(0f, 1f)]
        public float currentHealth = 1.0f;
        #endregion

        #region Events
        public event Action<JaguarEntityState> OnStateUpdated;
        public event Action<float, float> OnDamageReceived;
        public event Action OnEntityDestroyed;
        #endregion

        #region Properties
        public long EntityId { get; private set; } = 0;
        public JaguarEntityState CurrentState { get; private set; }
        public bool IsDestroyed { get; private set; } = false;
        public bool IsRegistered => EntityId != 0;
        #endregion

        #region Private Fields
        private JaguarSimulation _simulation;
        private IntPtr _nativeEntityHandle = IntPtr.Zero;
        private JaguarEntityState _lastPublishedState;
        private float _timeSinceNetworkUpdate = 0f;

        // Control inputs (flight controls, vehicle controls, etc.)
        private Dictionary<string, float> _controlInputs = new Dictionary<string, float>();
        #endregion

        #region Unity Lifecycle
        private void Awake()
        {
            InitializeControlInputs();
        }

        private void Start()
        {
            _simulation = JaguarSimulation.Instance;
            RegisterWithSimulation();
            ReadStateFromTransform();
        }

        private void OnDestroy()
        {
            UnregisterFromSimulation();
        }

        private void FixedUpdate()
        {
            if (IsDestroyed || _simulation == null)
                return;

            SyncWithSimulation();
        }
        #endregion

        #region Registration
        private void RegisterWithSimulation()
        {
            if (_simulation == null)
            {
                Debug.LogWarning($"[JaguarEntity] '{entityName}' could not find JaguarSimulation instance");
                return;
            }

            EntityId = _simulation.RegisterEntity(this);

            // Initialize state
            var state = CurrentState;
            state.entityId = EntityId;
            state.entityName = entityName;
            state.domain = domain;
            state.kind = kind;
            state.isLocal = isLocalEntity;
            state.isActive = true;
            CurrentState = state;

            Debug.Log($"[JaguarEntity] '{entityName}' registered with ID {EntityId}");
        }

        private void UnregisterFromSimulation()
        {
            if (_simulation != null && EntityId != 0)
            {
                _simulation.UnregisterEntity(EntityId);
                Debug.Log($"[JaguarEntity] '{entityName}' unregistered from simulation");
            }
            EntityId = 0;
        }
        #endregion

        #region Synchronization
        /// <summary>
        /// Called by JaguarSimulation during tick
        /// </summary>
        public void SyncWithSimulation()
        {
            if (_nativeEntityHandle != IntPtr.Zero)
            {
                // Sync control inputs to native engine
                SyncControlInputsToNative();

                // Read state back from native engine
                ReadStateFromNative();
            }
            else
            {
                // No native entity - read state from transform
                ReadStateFromTransform();
            }

            // Apply state to transform based on integration mode
            switch (integrationMode)
            {
                case JaguarIntegrationMode.ReplaceUnityPhysics:
                case JaguarIntegrationMode.JaguarOnly:
                    ApplyStateToTransform();
                    break;

                case JaguarIntegrationMode.Parallel:
                    // Both systems run, state is for reference only
                    break;

                case JaguarIntegrationMode.Synchronized:
                    // Blend between Unity and Jaguar physics
                    ApplyStateToTransform();
                    break;
            }

            // Check network update
            _timeSinceNetworkUpdate += Time.fixedDeltaTime;
            if (_timeSinceNetworkUpdate >= 1f / networkUpdateRate)
            {
                if (ShouldPublishState())
                {
                    PublishState();
                    _lastPublishedState = CurrentState;
                }
                _timeSinceNetworkUpdate = 0f;
            }

            // Fire state update event
            OnStateUpdated?.Invoke(CurrentState);
        }

        private void ReadStateFromTransform()
        {
            if (_simulation == null)
                return;

            var state = CurrentState;

            // Convert Unity position to geodetic
            state.position = _simulation.UnityToGeodetic(transform.position);

            // Store orientation
            state.orientation = transform.rotation;
            state.eulerAngles = transform.eulerAngles;

            // Get velocity from Rigidbody if available
            var rb = GetComponent<Rigidbody>();
            if (rb != null && !rb.isKinematic)
            {
                // Transform world velocity to body frame
                state.velocity = transform.InverseTransformDirection(rb.linearVelocity);
                state.angularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
            }

            state.timestamp = _simulation.SimulationTime;
            CurrentState = state;
        }

        private void ApplyStateToTransform()
        {
            if (_simulation == null)
                return;

            // Convert geodetic position to Unity
            Vector3 worldPos = _simulation.GeodeticToUnity(CurrentState.position);

            // Apply to transform
            transform.SetPositionAndRotation(worldPos, CurrentState.orientation);
        }

        private void ReadStateFromNative()
        {
            // Would read state from native JaguarEngine
            // For now, just update timestamp
            if (_simulation != null)
            {
                var state = CurrentState;
                state.timestamp = _simulation.SimulationTime;
                CurrentState = state;
            }
        }

        private void SyncControlInputsToNative()
        {
            // Would sync control inputs to native JaguarEngine
        }

        private bool ShouldPublishState()
        {
            if (!isLocalEntity)
                return false;

            // Check position difference
            Vector3 currentECEF = (Vector3)CurrentState.position.ToECEF();
            Vector3 lastECEF = (Vector3)_lastPublishedState.position.ToECEF();
            float distance = Vector3.Distance(currentECEF, lastECEF);

            if (distance > deadReckoningThreshold)
                return true;

            // Check orientation difference
            float angleDiff = Quaternion.Angle(CurrentState.orientation, _lastPublishedState.orientation);
            if (angleDiff > 1f) // 1 degree threshold
                return true;

            return false;
        }

        private void PublishState()
        {
            // Would publish to DIS/HLA network
        }
        #endregion

        #region Position & Orientation
        /// <summary>
        /// Set entity position in geodetic coordinates
        /// </summary>
        public void SetGeodeticPosition(GeodeticPosition position)
        {
            var state = CurrentState;
            state.position = position;
            CurrentState = state;

            if (integrationMode == JaguarIntegrationMode.ReplaceUnityPhysics ||
                integrationMode == JaguarIntegrationMode.JaguarOnly)
            {
                ApplyStateToTransform();
            }
        }

        /// <summary>
        /// Set entity velocity in body frame (m/s)
        /// </summary>
        public void SetVelocity(Vector3 velocity)
        {
            var state = CurrentState;
            state.velocity = velocity;
            CurrentState = state;
        }

        /// <summary>
        /// Set entity orientation
        /// </summary>
        public void SetOrientation(Quaternion orientation)
        {
            var state = CurrentState;
            state.orientation = orientation;
            state.eulerAngles = orientation.eulerAngles;
            CurrentState = state;
        }

        /// <summary>
        /// Get geodetic position
        /// </summary>
        public GeodeticPosition GetGeodeticPosition() => CurrentState.position;

        /// <summary>
        /// Get velocity magnitude (airspeed for aircraft)
        /// </summary>
        public float GetAirspeed() => CurrentState.velocity.magnitude;

        /// <summary>
        /// Get ground speed (horizontal velocity magnitude)
        /// </summary>
        public float GetGroundSpeed()
        {
            return Mathf.Sqrt(CurrentState.velocity.x * CurrentState.velocity.x +
                              CurrentState.velocity.y * CurrentState.velocity.y);
        }

        /// <summary>
        /// Get altitude above ground level
        /// </summary>
        public double GetAltitudeAGL()
        {
            if (_simulation == null)
                return CurrentState.position.altitude;

            var terrain = _simulation.GetTerrainAt(CurrentState.position);
            return CurrentState.position.altitude - terrain.elevation;
        }

        /// <summary>
        /// Get Mach number
        /// </summary>
        public float GetMachNumber()
        {
            if (_simulation == null)
                return GetAirspeed() / 340.29f;

            var atmo = _simulation.GetAtmosphereAt(CurrentState.position);
            return GetAirspeed() / (float)atmo.speedOfSound;
        }

        /// <summary>
        /// Get angle of attack (degrees)
        /// </summary>
        public float GetAngleOfAttack()
        {
            if (CurrentState.velocity.magnitude < 1f)
                return 0f;

            Vector3 velNorm = CurrentState.velocity.normalized;
            return Mathf.Rad2Deg * Mathf.Atan2(-velNorm.z, velNorm.x);
        }

        /// <summary>
        /// Get sideslip angle (degrees)
        /// </summary>
        public float GetSideslipAngle()
        {
            if (CurrentState.velocity.magnitude < 1f)
                return 0f;

            Vector3 velNorm = CurrentState.velocity.normalized;
            return Mathf.Rad2Deg * Mathf.Asin(velNorm.y);
        }
        #endregion

        #region Control Inputs
        private void InitializeControlInputs()
        {
            // Default flight control inputs
            _controlInputs["elevator"] = 0f;
            _controlInputs["aileron"] = 0f;
            _controlInputs["rudder"] = 0f;
            _controlInputs["throttle"] = 0f;
            _controlInputs["collective"] = 0f;
            _controlInputs["flaps"] = 0f;
            _controlInputs["gear"] = 1f;
            _controlInputs["speedbrake"] = 0f;

            // Ground vehicle controls
            _controlInputs["steering"] = 0f;
            _controlInputs["brake"] = 0f;
        }

        /// <summary>
        /// Set a control input value
        /// </summary>
        public void SetControlInput(string controlName, float value)
        {
            _controlInputs[controlName] = value;
        }

        /// <summary>
        /// Get a control input value
        /// </summary>
        public float GetControlInput(string controlName)
        {
            return _controlInputs.TryGetValue(controlName, out float value) ? value : 0f;
        }

        /// <summary>
        /// Set multiple control inputs at once
        /// </summary>
        public void SetControlInputs(Dictionary<string, float> controls)
        {
            foreach (var kvp in controls)
            {
                _controlInputs[kvp.Key] = kvp.Value;
            }
        }

        /// <summary>
        /// Get all control inputs
        /// </summary>
        public IReadOnlyDictionary<string, float> GetAllControlInputs() => _controlInputs;
        #endregion

        #region Property System
        /// <summary>
        /// Get a simulation property value (JSBSim-style path)
        /// </summary>
        public double GetProperty(string propertyPath)
        {
            // Common properties provided directly
            switch (propertyPath)
            {
                case "position/h-sl-meters":
                case "position/altitude":
                    return CurrentState.position.altitude;

                case "position/latitude-deg":
                    return CurrentState.position.latitude;

                case "position/longitude-deg":
                    return CurrentState.position.longitude;

                case "velocities/u-fps":
                case "velocity/u":
                    return CurrentState.velocity.x * 3.28084; // m/s to ft/s

                case "velocities/v-fps":
                case "velocity/v":
                    return CurrentState.velocity.y * 3.28084;

                case "velocities/w-fps":
                case "velocity/w":
                    return CurrentState.velocity.z * 3.28084;

                case "attitude/phi-deg":
                case "attitude/roll":
                    return CurrentState.eulerAngles.z;

                case "attitude/theta-deg":
                case "attitude/pitch":
                    return CurrentState.eulerAngles.x;

                case "attitude/psi-deg":
                case "attitude/yaw":
                    return CurrentState.eulerAngles.y;

                case "aero/alpha-deg":
                    return GetAngleOfAttack();

                case "aero/beta-deg":
                    return GetSideslipAngle();

                case "velocities/mach":
                    return GetMachNumber();

                case "velocities/vc-kts":
                    return GetAirspeed() * 1.94384; // m/s to knots

                default:
                    Debug.LogWarning($"[JaguarEntity] Unknown property: {propertyPath}");
                    return 0.0;
            }
        }

        /// <summary>
        /// Set a simulation property value
        /// </summary>
        public bool SetProperty(string propertyPath, double value)
        {
            // Would set in native engine
            Debug.LogWarning($"[JaguarEntity] SetProperty({propertyPath}, {value}) - native engine not available");
            return false;
        }
        #endregion

        #region Damage System
        /// <summary>
        /// Apply damage to this entity
        /// </summary>
        public void ApplyDamage(float damage, string damageType = "generic", Vector3? hitLocation = null, Vector3? hitDirection = null)
        {
            if (IsDestroyed)
                return;

            float previousHealth = currentHealth;
            currentHealth = Mathf.Clamp01(currentHealth - damage);

            Debug.Log($"[JaguarEntity] '{entityName}' damaged: {previousHealth:F2} -> {currentHealth:F2} (type: {damageType})");

            OnDamageReceived?.Invoke(damage, currentHealth);

            if (currentHealth <= 0f && !IsDestroyed)
            {
                IsDestroyed = true;
                var state = CurrentState;
                state.isActive = false;
                CurrentState = state;

                Debug.Log($"[JaguarEntity] '{entityName}' destroyed");
                OnEntityDestroyed?.Invoke();
            }
        }

        /// <summary>
        /// Repair this entity
        /// </summary>
        public void Repair(float amount)
        {
            if (IsDestroyed)
                return;

            currentHealth = Mathf.Clamp01(currentHealth + amount);
        }

        /// <summary>
        /// Reset entity to full health
        /// </summary>
        public void ResetHealth()
        {
            currentHealth = 1f;
            IsDestroyed = false;
            var state = CurrentState;
            state.isActive = true;
            CurrentState = state;
        }
        #endregion

        #region Network State Reception
        /// <summary>
        /// Called when receiving state update from network (DIS/HLA)
        /// </summary>
        public void OnNetworkStateReceived(JaguarEntityState newState)
        {
            if (isLocalEntity)
                return; // Don't override local entity state

            var state = newState;
            state.entityId = EntityId;
            state.entityName = entityName;
            state.isLocal = false;
            CurrentState = state;

            ApplyStateToTransform();
        }
        #endregion

        #region Debug
        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying)
                return;

            // Draw velocity vector
            Gizmos.color = Color.green;
            Vector3 worldVel = transform.TransformDirection(CurrentState.velocity);
            Gizmos.DrawRay(transform.position, worldVel.normalized * 10f);

            // Draw body axes
            Gizmos.color = Color.red;
            Gizmos.DrawRay(transform.position, transform.right * 5f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, transform.up * 5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, transform.forward * 5f);
        }
        #endregion
    }
}
