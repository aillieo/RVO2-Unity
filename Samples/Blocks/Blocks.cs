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
        private IList<float2> goals;

        /// <summary>
        /// Random number generator.
        /// </summary>
        private Random random;

        private Simulator simulator;

        private void Start()
        {
            this.random = new Random(0);
            this.simulator = new Simulator();

            this.StartCoroutine(this.Main());
        }

        private void setupScenario()
        {
            this.goals = new List<float2>();

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
                    this.simulator.AddAgent(new float2(55f + (i * 10f), 55f + (j * 10f)));
                    this.goals.Add(new float2(-75f, -75f));

                    this.simulator.AddAgent(new float2(-55f - (i * 10f), 55f + (j * 10f)));
                    this.goals.Add(new float2(75f, -75f));

                    this.simulator.AddAgent(new float2(55f + (i * 10f), -55f - (j * 10f)));
                    this.goals.Add(new float2(-75f, 75f));

                    this.simulator.AddAgent(new float2(-55f - (i * 10f), -55f - (j * 10f)));
                    this.goals.Add(new float2(75f, 75f));
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
            this.simulator.AddObstacle(obstacle1);

            IList<float2> obstacle2 = new List<float2>
            {
                new float2(10f, 40f),
                new float2(10f, 10f),
                new float2(40f, 10f),
                new float2(40f, 40f),
            };
            this.simulator.AddObstacle(obstacle2);

            IList<float2> obstacle3 = new List<float2>
            {
                new float2(10f, -40f),
                new float2(40f, -40f),
                new float2(40f, -10f),
                new float2(10f, -10f),
            };
            this.simulator.AddObstacle(obstacle3);

            IList<float2> obstacle4 = new List<float2>
            {
                new float2(-10f, -40f),
                new float2(-10f, -10f),
                new float2(-40f, -10f),
                new float2(-40f, -40f),
            };
            this.simulator.AddObstacle(obstacle4);

            // Process the obstacles so that they are accounted for in the simulation.
            this.simulator.ProcessObstacles();
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            for (var i = 0; i < this.simulator.GetNumObstacleVertices(); ++i)
            {
                var last = i;

                while (true)
                {
                    var next = this.simulator.GetNextObstacleVertexNo(last);

                    float2 p0 = this.simulator.GetObstacleVertex(last);
                    float2 p1 = this.simulator.GetObstacleVertex(next);

                    Gizmos.DrawLine((Vector2)p0, (Vector2)p1);

                    if (next == i)
                    {
                        break;
                    }

                    last = next;
                }

                i = last;
            }

            for (var i = 0; i < this.simulator.GetNumAgents(); ++i)
            {
                float2 position = this.simulator.GetAgentPosition(i);
                Gizmos.DrawSphere((Vector2)position, 2);
            }
        }

        private void setPreferredVelocities()
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            for (var i = 0; i < this.simulator.GetNumAgents(); ++i)
            {
                float2 goalVector = this.goals[i] - this.simulator.GetAgentPosition(i);

                if (math.lengthsq(goalVector) > 1f)
                {
                    goalVector = math.normalize(goalVector);
                }

                this.simulator.SetAgentPrefVelocity(i, goalVector);

                // Perturb a little to avoid deadlocks due to perfect symmetry.
                var angle = (float)this.random.NextDouble() * 2f * (float)Math.PI;
                var dist = (float)this.random.NextDouble() * 0.0001f;

                this.simulator.SetAgentPrefVelocity(
                    i,
                    this.simulator.GetAgentPrefVelocity(i) + (dist * new float2((float)Math.Cos(angle), (float)Math.Sin(angle))));
            }
        }

        private bool reachedGoal()
        {
            // Check if all agents have reached their goals.
            for (var i = 0; i < this.simulator.GetNumAgents(); ++i)
            {
                if (math.lengthsq(this.simulator.GetAgentPosition(i) - this.goals[i]) > 400f)
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
                this.setupScenario();

                // Perform (and manipulate) the simulation.
                do
                {
                    this.setPreferredVelocities();

                    this.simulator.DoStep();

                    yield return null;

                    this.simulator.EnsureCompleted();
                }
                while (!this.reachedGoal());

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
