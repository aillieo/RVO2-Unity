// -----------------------------------------------------------------------
// <copyright file="Interactive.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Collections.Generic;
    using Unity.Jobs;
    using Unity.Mathematics;
    using UnityEngine;
    using UnityEngine.Profiling;
    using Random = System.Random;

    public class Interactive : MonoBehaviour
    {
        /** Random number generator. */
        private Random random = new Random(0);

        private Simulator simulator;

        private CustomSampler sampler;

        private void setupScenario()
        {
            /* Specify the global time step of the simulation. */
            this.simulator.setTimeStep(0.25f);

            /*
             * Specify the default parameters for agents that are subsequently
             * added.
             */
            this.simulator.setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f, new float2(0.0f, 0.0f));

            /*
             * Add agents, specifying their start position.
             */
            for (int i = 0; i < 5; ++i)
            {
                for (int j = 0; j < 5; ++j)
                {
                    this.simulator.addAgent(new float2(55.0f + (i * 10.0f), 55.0f + (j * 10.0f)));

                    this.simulator.addAgent(new float2(-55.0f - (i * 10.0f), 55.0f + (j * 10.0f)));

                    this.simulator.addAgent(new float2(55.0f + (i * 10.0f), -55.0f - (j * 10.0f)));

                    this.simulator.addAgent(new float2(-55.0f - (i * 10.0f), -55.0f - (j * 10.0f)));
                }
            }

            /*
             * Add (polygonal) obstacles, specifying their vertices in
             * counterclockwise order.
             */
            IList<float2> obstacle1 = new List<float2>
            {
                new float2(-10.0f, 40.0f),
                new float2(-40.0f, 40.0f),
                new float2(-40.0f, 10.0f),
                new float2(-10.0f, 10.0f),
            };
            this.simulator.addObstacle(obstacle1);

            IList<float2> obstacle2 = new List<float2>
            {
                new float2(10.0f, 40.0f),
                new float2(10.0f, 10.0f),
                new float2(40.0f, 10.0f),
                new float2(40.0f, 40.0f),
            };
            this.simulator.addObstacle(obstacle2);

            IList<float2> obstacle3 = new List<float2>
            {
                new float2(10.0f, -40.0f),
                new float2(40.0f, -40.0f),
                new float2(40.0f, -10.0f),
                new float2(10.0f, -10.0f),
            };
            this.simulator.addObstacle(obstacle3);

            IList<float2> obstacle4 = new List<float2>
            {
                new float2(-10.0f, -40.0f),
                new float2(-10.0f, -10.0f),
                new float2(-40.0f, -10.0f),
                new float2(-40.0f, -40.0f),
            };
            this.simulator.addObstacle(obstacle4);

            /*
             * Process the obstacles so that they are accounted for in the
             * simulation.
             */
            this.simulator.processObstacles();
        }

        private void OnDrawGizmos()
        {
            if (this.simulator == null)
            {
                return;
            }

            for (int i = 0; i < this.simulator.getNumObstacleVertices(); ++i)
            {
                float2 o = this.simulator.getObstacleVertex(i);
                Gizmos.DrawSphere((Vector2)o, 1);

                int last = i;

                while (true)
                {
                    int next = this.simulator.getNextObstacleVertexNo(last);

                    float2 p0 = this.simulator.getObstacleVertex(last);
                    float2 p1 = this.simulator.getObstacleVertex(next);

                    Gizmos.DrawLine((Vector2)p0, (Vector2)p1);

                    if (next == i)
                    {
                        break;
                    }

                    last = next;
                }

                i = last;
            }

            for (int i = 0; i < this.simulator.getNumAgents(); ++i)
            {
                float2 position = this.simulator.getAgentPosition(i);
                Gizmos.DrawSphere((Vector2)position, 1);
            }
        }

        private void setPreferredVelocities(float2 newGoal)
        {
            /*
             * Set the preferred velocity to be a vector of unit magnitude
             * (speed) in the direction of the goal.
             */
            for (int i = 0; i < this.simulator.getNumAgents(); ++i)
            {
                float2 goalVector = newGoal - this.simulator.getAgentPosition(i);

                if (math.lengthsq(goalVector) > 1.0f)
                {
                    goalVector = math.normalize(goalVector);
                }

                this.simulator.setAgentPrefVelocity(i, goalVector);

                /* Perturb a little to avoid deadlocks due to perfect symmetry. */
                float angle = (float)this.random.NextDouble() * 2.0f * (float)Math.PI;
                float dist = (float)this.random.NextDouble() * 0.0001f;

                this.simulator.setAgentPrefVelocity(
                    i,
                    this.simulator.getAgentPrefVelocity(i) + (dist * new float2((float)Math.Cos(angle), (float)Math.Sin(angle))));
            }
        }

        private void Start()
        {
            this.simulator = new Simulator();

            /* Set up the scenario. */
            this.setupScenario();

            this.sampler = CustomSampler.Create("RVO update", false);
        }

        private void Update()
        {
            if (!GetTouchPosition(out bool isTouchBegan, out Vector3 position))
            {
                return;
            }

            Camera mainCam = Camera.main;
            if (mainCam == null)
            {
                return;
            }

            Vector3 worldPos = mainCam.ScreenToWorldPoint(position);
            float2 worldPos2d = (Vector2)worldPos;

            switch (touchMode)
            {
                case TouchMode.Add:
                    if (isTouchBegan)
                    {
                        this.simulator.addAgent(worldPos2d);
                    }

                    break;
                case TouchMode.Move:
                    this.sampler.Begin();
                    this.setPreferredVelocities(worldPos2d);
                    JobHandle jobHandle = this.simulator.doStep();
                    jobHandle.Complete();
                    this.sampler.End();
                    break;
                case TouchMode.Remove:
                    if (isTouchBegan)
                    {
                    }

                    break;
            }
        }

        private void OnDestroy()
        {
            this.simulator.Clear();

            this.simulator.Dispose();
        }

        private enum TouchMode
        {
            Add = 0,
            Move = 1,
            Remove = 2,
            Max = 3,
        }

        private TouchMode touchMode = TouchMode.Move;

        private void OnGUI()
        {
            GUILayout.Label($"Agents:{this.simulator.getNumAgents()}");
            GUILayout.Label($"FPS:{1f / Time.deltaTime}");

            string modeText = string.Empty;
            switch (touchMode)
            {
                case TouchMode.Add:
                    modeText = "Click to create a new agent";
                    break;
                case TouchMode.Move:
                    modeText = "Press and hold to update agents";
                    break;
                case TouchMode.Remove:
                    modeText = "Click to remove an agent";
                    break;
            }

            if (GUILayout.Button(modeText))
            {
                touchMode = touchMode + 1;
                if (touchMode == TouchMode.Max)
                {
                    touchMode = TouchMode.Add;
                }
            }
        }

        private bool GetTouchPosition(out bool touchBegan, out Vector3 position)
        {
            if (Input.touchSupported)
            {
                if (Input.touchCount > 0)
                {
                    Touch touch = Input.GetTouch(0);
                    touchBegan = touch.phase == TouchPhase.Began;
                    position = Input.GetTouch(0).position;
                    return true;
                }
            }
            else
            {
                if (Input.GetMouseButton(0))
                {
                    touchBegan = Input.GetMouseButtonDown(0);
                    position = Input.mousePosition;
                    return true;
                }
            }

            touchBegan = default;
            position = default;
            return false;
        }
    }
}
