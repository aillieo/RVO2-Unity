// -----------------------------------------------------------------------
// <copyright file="Blocks.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Blocks.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/*
 * Example file showing a demo with 100 agents split in four groups initially
 * positioned in four corners of the environment. Each agent attempts to move to
 * other side of the environment through a narrow passage generated by four
 * obstacles. There is no roadmap to guide the agents around the obstacles.
 */

namespace RVO
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using Unity.Mathematics;
    using UnityEngine;
    using Random = System.Random;

    internal class Blocks : MonoBehaviour
    {
        /// <summary>
        /// Store the goals of the agents.
        /// </summary>
        private Dictionary<int, float2> goals;

        /// <summary>
        /// Random number generator.
        /// </summary>
        private Random random;

        private Simulator simulator;

        private List<int> obstacles;

        private void Start()
        {
            EditorUtils.DrawGizmosSceneView(true);

            this.random = new Random(0);
            this.simulator = new Simulator();
            this.obstacles = new List<int>();

            this.StartCoroutine(this.Main());
        }

        private void SetupScenario()
        {
            this.goals = new Dictionary<int, float2>();

            // Specify the global time step of the simulation.
            this.simulator.SetTimeStep(0.25f);

            // Specify the default parameters for agents that are subsequently added.
            this.simulator.SetAgentDefaults(15f, 10, 5f, 5f, 2f, 2f, new float2(0f, 0f));

            // Add agents, specifying their start position, and store their
            // goals on the opposite side of the environment.
            for (var i = 0; i < 5; ++i)
            {
                for (var j = 0; j < 5; ++j)
                {
                    var agentId = this.simulator.AddAgent(new float2(55f + (i * 10f), 55f + (j * 10f)));
                    var goal = -this.simulator.GetAgentPosition(agentId);
                    this.goals.Add(agentId, goal);

                    agentId = this.simulator.AddAgent(new float2(-55f - (i * 10f), 55f + (j * 10f)));
                    goal = -this.simulator.GetAgentPosition(agentId);
                    this.goals.Add(agentId, goal);

                    agentId = this.simulator.AddAgent(new float2(55f + (i * 10f), -55f - (j * 10f)));
                    goal = -this.simulator.GetAgentPosition(agentId);
                    this.goals.Add(agentId, goal);

                    agentId = this.simulator.AddAgent(new float2(-55f - (i * 10f), -55f - (j * 10f)));
                    goal = -this.simulator.GetAgentPosition(agentId);
                    this.goals.Add(agentId, goal);
                }
            }

            // Add (polygonal) obstacles, specifying their vertices in
            // counterclockwise order.
            IList<float2> obstacle1 = new List<float2>
            {
                new float2(-10f, 40f),
                new float2(-40f, 40f),
                new float2(-40f, 10f),
                new float2(-10f, 10f),
            };
            var obstacle1Id = this.simulator.AddObstacle(obstacle1);
            this.obstacles.Add(obstacle1Id);

            IList<float2> obstacle2 = new List<float2>
            {
                new float2(10f, 40f),
                new float2(10f, 10f),
                new float2(40f, 10f),
                new float2(40f, 40f),
            };
            var obstacle2Id = this.simulator.AddObstacle(obstacle2);
            this.obstacles.Add(obstacle2Id);

            IList<float2> obstacle3 = new List<float2>
            {
                new float2(10f, -40f),
                new float2(40f, -40f),
                new float2(40f, -10f),
                new float2(10f, -10f),
            };
            var obstacle3Id = this.simulator.AddObstacle(obstacle3);
            this.obstacles.Add(obstacle3Id);

            IList<float2> obstacle4 = new List<float2>
            {
                new float2(-10f, -40f),
                new float2(-10f, -10f),
                new float2(-40f, -10f),
                new float2(-40f, -40f),
            };
            var obstacle4Id = this.simulator.AddObstacle(obstacle4);
            this.obstacles.Add(obstacle4Id);
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            foreach (var obstacle in this.obstacles)
            {
                var first = this.simulator.GetFirstObstacleVertexId(obstacle);

                var current = first;

                while (true)
                {
                    var next = this.simulator.GetNextObstacleVertexId(current);

                    float2 p0 = this.simulator.GetObstacleVertex(current);
                    float2 p1 = this.simulator.GetObstacleVertex(next);

                    Gizmos.DrawLine((Vector2)p0, (Vector2)p1);

                    if (next == first)
                    {
                        break;
                    }

                    current = next;
                }
            }

            foreach (var pair in this.goals)
            {
                var agentId = pair.Key;
                float2 position = this.simulator.GetAgentPosition(agentId);
                Gizmos.DrawSphere((Vector2)position, 2);
            }
        }

        private void SetPreferredVelocities()
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            foreach (var pair in this.goals)
            {
                var agentId = pair.Key;
                var goal = pair.Value;
                float2 goalVector = goal - this.simulator.GetAgentPosition(agentId);

                if (math.lengthsq(goalVector) > 1f)
                {
                    goalVector = math.normalize(goalVector);
                }

                this.simulator.SetAgentPrefVelocity(agentId, goalVector);

                // Perturb a little to avoid deadlocks due to perfect symmetry.
                var angle = (float)this.random.NextDouble() * 2f * (float)Math.PI;
                var dist = (float)this.random.NextDouble() * 0.0001f;

                this.simulator.SetAgentPrefVelocity(
                    agentId,
                    this.simulator.GetAgentPrefVelocity(agentId) + (dist * new float2((float)Math.Cos(angle), (float)Math.Sin(angle))));
            }
        }

        private bool ReachedGoal()
        {
            // Check if all agents have reached their goals.
            foreach (var pair in this.goals)
            {
                var agentId = pair.Key;
                var goal = pair.Value;
                if (math.lengthsq(
                    this.simulator.GetAgentPosition(agentId) - goal)
                        > this.simulator.GetAgentRadius(agentId) * this.simulator.GetAgentRadius(agentId))
                {
                    return false;
                }
            }

            return true;
        }

        private IEnumerator Main()
        {
            do
            {
                // Set up the scenario.
                this.SetupScenario();

                // Perform (and manipulate) the simulation.
                do
                {
                    this.SetPreferredVelocities();

                    this.simulator.DoStep();

                    yield return null;

                    this.simulator.EnsureCompleted();
                }
                while (!this.ReachedGoal());

                yield return new WaitForSeconds(1);

                this.simulator.Clear();
            }
            while (true);
        }

        private void OnDestroy()
        {
            this.simulator.Clear();
            this.simulator.Dispose();
        }
    }
}
