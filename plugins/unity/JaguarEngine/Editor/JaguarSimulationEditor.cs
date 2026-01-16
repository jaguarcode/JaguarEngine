// Copyright JaguarEngine Team. All Rights Reserved.

using UnityEngine;
using UnityEditor;

namespace JaguarEngine.Editor
{
    /// <summary>
    /// Custom editor for JaguarSimulation component
    /// </summary>
    [CustomEditor(typeof(JaguarSimulation))]
    public class JaguarSimulationEditor : UnityEditor.Editor
    {
        private bool _showOriginFoldout = true;
        private bool _showTimeFoldout = true;
        private bool _showNetworkFoldout = false;
        private bool _showStatsFoldout = false;

        private SerializedProperty _simulationOrigin;
        private SerializedProperty _timeScale;
        private SerializedProperty _fixedTimestep;
        private SerializedProperty _networkSettings;
        private SerializedProperty _showDebugInfo;

        private void OnEnable()
        {
            _simulationOrigin = serializedObject.FindProperty("simulationOrigin");
            _timeScale = serializedObject.FindProperty("timeScale");
            _fixedTimestep = serializedObject.FindProperty("fixedTimestep");
            _networkSettings = serializedObject.FindProperty("networkSettings");
            _showDebugInfo = serializedObject.FindProperty("showDebugInfo");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            JaguarSimulation sim = (JaguarSimulation)target;

            // Header
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("JaguarEngine Simulation", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            // Status display
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.LabelField("Status:", GUILayout.Width(50));
                GUI.enabled = false;
                EditorGUILayout.EnumPopup(sim.SimulationState);
                GUI.enabled = true;
            }

            if (Application.isPlaying)
            {
                EditorGUILayout.LabelField($"Simulation Time: {sim.SimulationTime:F3}s");
                EditorGUILayout.LabelField($"Entity Count: {sim.EntityCount}");
                EditorGUILayout.LabelField($"Network: {(sim.IsNetworkConnected ? "Connected" : "Disconnected")}");
                EditorGUILayout.Space();
            }

            // Simulation Origin
            _showOriginFoldout = EditorGUILayout.Foldout(_showOriginFoldout, "Simulation Origin", true);
            if (_showOriginFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(_simulationOrigin.FindPropertyRelative("latitude"),
                    new GUIContent("Latitude (deg)"));
                EditorGUILayout.PropertyField(_simulationOrigin.FindPropertyRelative("longitude"),
                    new GUIContent("Longitude (deg)"));
                EditorGUILayout.PropertyField(_simulationOrigin.FindPropertyRelative("altitude"),
                    new GUIContent("Altitude (m)"));

                if (GUILayout.Button("Set to Edwards AFB"))
                {
                    _simulationOrigin.FindPropertyRelative("latitude").doubleValue = 34.905;
                    _simulationOrigin.FindPropertyRelative("longitude").doubleValue = -117.884;
                    _simulationOrigin.FindPropertyRelative("altitude").doubleValue = 702.0;
                }

                if (GUILayout.Button("Set to Nellis AFB"))
                {
                    _simulationOrigin.FindPropertyRelative("latitude").doubleValue = 36.236;
                    _simulationOrigin.FindPropertyRelative("longitude").doubleValue = -115.034;
                    _simulationOrigin.FindPropertyRelative("altitude").doubleValue = 570.0;
                }

                EditorGUI.indentLevel--;
            }

            // Time Settings
            _showTimeFoldout = EditorGUILayout.Foldout(_showTimeFoldout, "Time Settings", true);
            if (_showTimeFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(_timeScale, new GUIContent("Time Scale"));
                EditorGUILayout.PropertyField(_fixedTimestep, new GUIContent("Fixed Timestep (s)"));
                EditorGUI.indentLevel--;
            }

            // Network Settings
            _showNetworkFoldout = EditorGUILayout.Foldout(_showNetworkFoldout, "Network Settings", true);
            if (_showNetworkFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(_networkSettings);
                EditorGUI.indentLevel--;
            }

            // Debug
            EditorGUILayout.PropertyField(_showDebugInfo, new GUIContent("Show Debug Info"));

            // Runtime Statistics
            if (Application.isPlaying)
            {
                _showStatsFoldout = EditorGUILayout.Foldout(_showStatsFoldout, "Runtime Statistics", true);
                if (_showStatsFoldout)
                {
                    EditorGUI.indentLevel++;
                    var stats = sim.GetSimulationStats();
                    EditorGUILayout.LabelField($"Real-Time Ratio: {stats.realTimeRatio:F2}x");
                    EditorGUILayout.LabelField($"Physics Step: {stats.physicsStepTimeMs:F2}ms");
                    EditorGUILayout.LabelField($"Collision: {stats.collisionTimeMs:F2}ms");
                    EditorGUILayout.LabelField($"Network: {stats.networkTimeMs:F2}ms");
                    EditorGUILayout.LabelField($"Memory: {stats.memoryUsageMB:F1}MB");
                    EditorGUI.indentLevel--;
                }
            }

            // Control Buttons
            EditorGUILayout.Space();
            if (Application.isPlaying)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (sim.SimulationState == JaguarSimulationState.Stopped)
                    {
                        if (GUILayout.Button("Start"))
                            sim.StartSimulation();
                    }
                    else if (sim.SimulationState == JaguarSimulationState.Running)
                    {
                        if (GUILayout.Button("Pause"))
                            sim.PauseSimulation();
                        if (GUILayout.Button("Stop"))
                            sim.StopSimulation();
                    }
                    else if (sim.SimulationState == JaguarSimulationState.Paused)
                    {
                        if (GUILayout.Button("Resume"))
                            sim.ResumeSimulation();
                        if (GUILayout.Button("Step"))
                            sim.StepSimulation(0.1f);
                        if (GUILayout.Button("Stop"))
                            sim.StopSimulation();
                    }
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    if (!sim.IsNetworkConnected)
                    {
                        if (GUILayout.Button("Connect Network"))
                            sim.ConnectNetwork();
                    }
                    else
                    {
                        if (GUILayout.Button("Disconnect Network"))
                            sim.DisconnectNetwork();
                    }
                }
            }
            else
            {
                EditorGUILayout.HelpBox("Enter Play Mode to control simulation.", MessageType.Info);
            }

            serializedObject.ApplyModifiedProperties();

            if (GUI.changed)
            {
                EditorUtility.SetDirty(target);
            }
        }
    }

    /// <summary>
    /// Custom editor for JaguarEntity component
    /// </summary>
    [CustomEditor(typeof(JaguarEntity))]
    public class JaguarEntityEditor : UnityEditor.Editor
    {
        private bool _showIdentityFoldout = true;
        private bool _showPhysicsFoldout = true;
        private bool _showNetworkFoldout = false;
        private bool _showStateFoldout = false;
        private bool _showControlsFoldout = false;

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            JaguarEntity entity = (JaguarEntity)target;

            // Header
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("JaguarEngine Entity", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            // Runtime Status
            if (Application.isPlaying)
            {
                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField("Entity ID:", GUILayout.Width(80));
                    EditorGUILayout.LabelField(entity.EntityId.ToString());
                }

                using (new EditorGUILayout.HorizontalScope())
                {
                    EditorGUILayout.LabelField("Status:", GUILayout.Width(80));
                    EditorGUILayout.LabelField(entity.IsDestroyed ? "Destroyed" :
                        (entity.IsRegistered ? "Active" : "Unregistered"));
                }

                EditorGUILayout.Space();
            }

            // Identity Section
            _showIdentityFoldout = EditorGUILayout.Foldout(_showIdentityFoldout, "Entity Identity", true);
            if (_showIdentityFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(serializedObject.FindProperty("entityName"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("domain"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("kind"));
                EditorGUI.indentLevel--;
            }

            // Physics Integration
            _showPhysicsFoldout = EditorGUILayout.Foldout(_showPhysicsFoldout, "Physics Integration", true);
            if (_showPhysicsFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(serializedObject.FindProperty("integrationMode"));
                EditorGUI.indentLevel--;
            }

            // Network Settings
            _showNetworkFoldout = EditorGUILayout.Foldout(_showNetworkFoldout, "Network Settings", true);
            if (_showNetworkFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.PropertyField(serializedObject.FindProperty("isLocalEntity"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("networkUpdateRate"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("deadReckoningThreshold"));
                EditorGUI.indentLevel--;
            }

            // Damage
            EditorGUILayout.PropertyField(serializedObject.FindProperty("currentHealth"));

            // Runtime State Display
            if (Application.isPlaying && entity.IsRegistered)
            {
                _showStateFoldout = EditorGUILayout.Foldout(_showStateFoldout, "Current State", true);
                if (_showStateFoldout)
                {
                    EditorGUI.indentLevel++;
                    var state = entity.CurrentState;

                    EditorGUILayout.LabelField("Position:");
                    EditorGUI.indentLevel++;
                    EditorGUILayout.LabelField($"Lat: {state.position.latitude:F6}°");
                    EditorGUILayout.LabelField($"Lon: {state.position.longitude:F6}°");
                    EditorGUILayout.LabelField($"Alt: {state.position.altitude:F1}m");
                    EditorGUI.indentLevel--;

                    EditorGUILayout.LabelField("Velocity:");
                    EditorGUI.indentLevel++;
                    EditorGUILayout.LabelField($"Airspeed: {entity.GetAirspeed():F1} m/s ({entity.GetAirspeed() * 1.94384f:F1} kts)");
                    EditorGUILayout.LabelField($"Ground Speed: {entity.GetGroundSpeed():F1} m/s");
                    EditorGUILayout.LabelField($"Mach: {entity.GetMachNumber():F3}");
                    EditorGUI.indentLevel--;

                    EditorGUILayout.LabelField("Attitude:");
                    EditorGUI.indentLevel++;
                    EditorGUILayout.LabelField($"AoA: {entity.GetAngleOfAttack():F1}°");
                    EditorGUILayout.LabelField($"Sideslip: {entity.GetSideslipAngle():F1}°");
                    EditorGUILayout.LabelField($"AGL: {entity.GetAltitudeAGL():F1}m");
                    EditorGUI.indentLevel--;

                    EditorGUI.indentLevel--;
                }

                // Control Inputs
                _showControlsFoldout = EditorGUILayout.Foldout(_showControlsFoldout, "Control Inputs", true);
                if (_showControlsFoldout)
                {
                    EditorGUI.indentLevel++;
                    var controls = entity.GetAllControlInputs();
                    foreach (var kvp in controls)
                    {
                        EditorGUILayout.LabelField($"{kvp.Key}: {kvp.Value:F2}");
                    }
                    EditorGUI.indentLevel--;
                }
            }

            serializedObject.ApplyModifiedProperties();
        }
    }

    /// <summary>
    /// Menu items for JaguarEngine
    /// </summary>
    public static class JaguarEngineMenu
    {
        [MenuItem("GameObject/JaguarEngine/Simulation Manager", false, 10)]
        private static void CreateSimulationManager(MenuCommand menuCommand)
        {
            GameObject go = new GameObject("JaguarSimulation");
            go.AddComponent<JaguarSimulation>();
            GameObjectUtility.SetParentAndAlign(go, menuCommand.context as GameObject);
            Undo.RegisterCreatedObjectUndo(go, "Create JaguarSimulation");
            Selection.activeObject = go;
        }

        [MenuItem("GameObject/JaguarEngine/Entity (Air)", false, 11)]
        private static void CreateAirEntity(MenuCommand menuCommand)
        {
            CreateEntity(menuCommand, "Aircraft", JaguarDomain.Air);
        }

        [MenuItem("GameObject/JaguarEngine/Entity (Land)", false, 12)]
        private static void CreateLandEntity(MenuCommand menuCommand)
        {
            CreateEntity(menuCommand, "Vehicle", JaguarDomain.Land);
        }

        [MenuItem("GameObject/JaguarEngine/Entity (Sea)", false, 13)]
        private static void CreateSeaEntity(MenuCommand menuCommand)
        {
            CreateEntity(menuCommand, "Ship", JaguarDomain.Sea);
        }

        private static void CreateEntity(MenuCommand menuCommand, string name, JaguarDomain domain)
        {
            GameObject go = new GameObject(name);
            var entity = go.AddComponent<JaguarEntity>();
            entity.entityName = name;
            entity.domain = domain;
            GameObjectUtility.SetParentAndAlign(go, menuCommand.context as GameObject);
            Undo.RegisterCreatedObjectUndo(go, $"Create Jaguar {name}");
            Selection.activeObject = go;
        }

        [MenuItem("JaguarEngine/Documentation", false, 100)]
        private static void OpenDocumentation()
        {
            Application.OpenURL("https://jaguarengine.readthedocs.io");
        }

        [MenuItem("JaguarEngine/About", false, 101)]
        private static void ShowAbout()
        {
            EditorUtility.DisplayDialog(
                "JaguarEngine",
                "JaguarEngine Unity Integration\n" +
                "Version 1.4.0\n\n" +
                "Multi-domain physics simulation engine for Unity.\n" +
                "Provides high-fidelity air, land, sea, and space domain physics\n" +
                "with DIS/HLA interoperability.\n\n" +
                "Copyright JaguarEngine Team. All Rights Reserved.",
                "OK");
        }
    }
}
