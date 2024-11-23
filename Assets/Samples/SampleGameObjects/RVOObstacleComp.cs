// -----------------------------------------------------------------------
// <copyright file="RVOObstacleComp.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using UnityEngine;

    internal class RVOObstacleComp : MonoBehaviour
    {
        private int obstacleId;

        private void OnEnable()
        {
            var simulator = SampleGameObjects.GetSimulator();
            var meshFilter = this.GetComponent<MeshFilter>();
            var vertices = ObstacleHelper.CalculateBoundingPolygon(meshFilter);
            this.obstacleId = simulator.AddObstacle(vertices);
        }

        private void OnDisable()
        {
            var simulator = SampleGameObjects.GetSimulator();
            if (simulator == null)
            {
                return;
            }

            simulator.RemoveObstacle(this.obstacleId);
            this.obstacleId = default;
        }
    }
}
