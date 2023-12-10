// -----------------------------------------------------------------------
// <copyright file="Circle.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Circle.cs
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
 * Example file showing a demo with 250 agents initially positioned evenly
 * distributed on a circle attempting to move to the antipodal position on the
 * circle.
 */

namespace RVO
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using Unity.Jobs;
    using Unity.Mathematics;
    using UnityEngine;

    internal class Circle : MonoBehaviour
    {
        /* Store the goals of the agents. */
        private IList<float2> goals;

        private Simulator simulator;

        private void Start()
        {
            this.simulator = new Simulator();

            this.StartCoroutine(this.Main());
        }

        private void setupScenario()
        {
            this.goals = new List<float2>();

            /* Specify the global time step of the simulation. */
            this.simulator.setTimeStep(0.25f);

            /*
             * Specify the default parameters for agents that are subsequently
             * added.
             */
            this.simulator.setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, new float2(0.0f, 0.0f));

            /*
             * Add agents, specifying their start position, and store their
             * goals on the opposite side of the environment.
             */
            for (int i = 0; i < 250; ++i)
            {
                this.simulator.addAgent(200.0f *
                    new float2(
                        (float)Math.Cos(i * 2.0f * Math.PI / 250.0f),
                        (float)Math.Sin(i * 2.0f * Math.PI / 250.0f)));
                this.goals.Add(-this.simulator.getAgentPosition(i));
            }
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.CompleteImmediate();

            for (int i = 0; i < this.simulator.getNumAgents(); ++i)
            {
                float2 position = this.simulator.getAgentPosition(i);
                Gizmos.DrawSphere((Vector2)position, 1);
            }
        }

        private void setPreferredVelocities()
        {
            /*
             * Set the preferred velocity to be a vector of unit magnitude
             * (speed) in the direction of the goal.
             */
            for (int i = 0; i < this.simulator.getNumAgents(); ++i)
            {
                float2 goalVector = this.goals[i] - this.simulator.getAgentPosition(i);

                if (math.lengthsq(goalVector) > 1.0f)
                {
                    goalVector = math.normalize(goalVector);
                }

                this.simulator.setAgentPrefVelocity(i, goalVector);
            }
        }

        private bool reachedGoal()
        {
            /* Check if all agents have reached their goals. */
            for (int i = 0; i < this.simulator.getNumAgents(); ++i)
            {
                if (math.lengthsq(this.simulator.getAgentPosition(i) - this.goals[i]) > this.simulator.getAgentRadius(i) * this.simulator.getAgentRadius(i))
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
                /* Set up the scenario. */
                this.setupScenario();

                /* Perform (and manipulate) the simulation. */
                do
                {
                    this.setPreferredVelocities();

                    JobHandle jobHandle = this.simulator.doStep();

                    yield return null;

                    jobHandle.Complete();
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
