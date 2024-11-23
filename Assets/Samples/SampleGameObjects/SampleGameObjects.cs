// -----------------------------------------------------------------------
// <copyright file="SampleGameObjects.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Mathematics;
    using UnityEngine;
    using UnityEngine.Assertions;

    internal class SampleGameObjects : MonoBehaviour
    {
        private static SampleGameObjects instance;

        private static bool gameQuiting = false;

        private Simulator simulator;

        private Camera mainCamera;
        private float2 currentGoal;
        private Plane ground = new Plane(Vector3.up, 0);

        [SerializeField]
        private Transform goalIndicator;

        public static SampleGameObjects Instance
        {
            get
            {
                if (instance == null)
                {
                    if (gameQuiting)
                    {
                        return null;
                    }

                    instance = FindAnyObjectByType<SampleGameObjects>();
                    Assert.IsNotNull(instance);
                }

                return instance;
            }
        }

        public static Simulator GetSimulator()
        {
            if (gameQuiting)
            {
                return null;
            }

            if (Instance.simulator == null)
            {
                var simulator = new Simulator();

                simulator.SetTimeStep(1 / 60f);
                simulator.SetAgentDefaults(10f, 10, 4f, 4f, 0.5f, 0.2f, new float2(0f, 0f));

                Instance.simulator = simulator;
            }

            return Instance.simulator;
        }

        public static float2 GetGoal()
        {
            if (gameQuiting)
            {
                return default;
            }

            return instance.currentGoal;
        }

        private void UpdateGoalIndicator()
        {
            if (this.goalIndicator == null)
            {
                return;
            }

            var goalPosition = this.goalIndicator.transform.position;
            this.goalIndicator.transform.position = new Vector3(this.currentGoal.x, goalPosition.y, this.currentGoal.y);
        }

        private void OnEnable()
        {
            this.mainCamera = Camera.main;
            this.UpdateGoalIndicator();
        }

        private void OnDestroy()
        {
            this.simulator.Clear();

            this.simulator.Dispose();
        }

        private void OnGUI()
        {
            GUILayout.Label($"Agents:{this.simulator.GetNumAgents()}");
            GUILayout.Label($"FPS:{1f / Time.deltaTime}");
        }

        private void Update()
        {
            this.simulator.DoStep();

            if (Input.GetMouseButton(0))
            {
                if (this.mainCamera == null)
                {
                    return;
                }

                var position = Input.mousePosition;
                var ray = this.mainCamera.ScreenPointToRay(position);
                if (this.ground.Raycast(ray, out var enter))
                {
                    Vector3 worldPosition = ray.GetPoint(enter);
                    this.currentGoal = new float2(worldPosition.x, worldPosition.z);
                    this.UpdateGoalIndicator();
                }
            }
        }

        private void LateUpdate()
        {
            this.simulator.EnsureCompleted();
        }

        private void OnApplicationQuit()
        {
            gameQuiting = true;
        }
    }
}
