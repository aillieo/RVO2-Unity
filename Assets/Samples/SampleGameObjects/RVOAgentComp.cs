// -----------------------------------------------------------------------
// <copyright file="RVOAgentComp.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Mathematics;
    using UnityEngine;

    internal class RVOAgentComp : MonoBehaviour
    {
        private static readonly float stoppingDistance = 5f;

        private int agentId;

        [SerializeField]
        private float maxSpeed = 0.5f;

        private void OnEnable()
        {
            var simulator = SampleGameObjects.GetSimulator();
            var position = new float2(this.transform.position.x, this.transform.position.z);
            this.agentId = simulator.AddAgent(position);
            simulator.SetAgentMaxSpeed(this.agentId, this.maxSpeed);
            var radius = 0.5f * this.transform.localScale.x;
            simulator.SetAgentRadius(this.agentId, radius);
        }

        private void OnDisable()
        {
            var simulator = SampleGameObjects.GetSimulator();
            if (simulator == null)
            {
                return;
            }

            simulator.RemoveAgent(this.agentId);
            this.agentId = default;
        }

        private void SetPreferredVelocities(float2 newGoal)
        {
            var simulator = SampleGameObjects.GetSimulator();

            float2 goalVector = newGoal - simulator.GetAgentPosition(this.agentId);

            if (math.lengthsq(goalVector) > stoppingDistance * stoppingDistance)
            {
                goalVector = math.normalize(goalVector);
                goalVector += (float2)UnityEngine.Random.insideUnitCircle * 0.001f;
            }

            simulator.SetAgentPrefVelocity(this.agentId, goalVector);
        }

        private void Update()
        {
            var simulator = SampleGameObjects.GetSimulator();

            var goal = SampleGameObjects.GetGoal();
            this.SetPreferredVelocities(goal);

            var position2 = simulator.GetAgentPosition(this.agentId);
            var prevPosition = this.transform.position;
            this.transform.position = new Vector3(position2.x, prevPosition.y, position2.y);
        }
    }
}
