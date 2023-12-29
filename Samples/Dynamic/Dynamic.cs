// -----------------------------------------------------------------------
// <copyright file="Dynamic.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Collections.Generic;
    using Unity.Mathematics;
    using UnityEngine;
    using UnityEngine.Profiling;
    using Random = System.Random;

    internal class Dynamic : MonoBehaviour
    {
        // Random number generator.
        private readonly Random random = new Random(0);

        private Simulator simulator;

        private CustomSampler sampler;

        private List<RepeatedTask> repeatedTasks;
        private Dictionary<int, GoalAndColor> agentData;
        private List<int> obstacles;

        private void SetupScenario()
        {
            // Specify the global time step of the simulation.
            this.simulator.SetTimeStep(0.25f);

            // Specify the default parameters for agents that are subsequently added.
            this.simulator.SetAgentDefaults(15f, 10, 5f, 5f, 2f, 0.5f, new float2(0f, 0f));

            this.repeatedTasks = new List<RepeatedTask>();
            this.agentData = new Dictionary<int, GoalAndColor>();

            // Agents spawning.
            for (var i = 0; i < 3; ++i)
            {
                for (var j = 0; j < 3; ++j)
                {
                    var position = new float2(60f + (i * 10f), 60f + (j * 10f));
                    var goal = -position;
                    var color = Color.red;
                    var task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 0 };
                    this.repeatedTasks.Add(task);

                    position = new float2(-60f - (i * 10f), 60f + (j * 10f));
                    goal = -position;
                    color = Color.green;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 250 };
                    this.repeatedTasks.Add(task);

                    position = new float2(60f + (i * 10f), -60f - (j * 10f));
                    goal = -position;
                    color = Color.blue;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 500 };
                    this.repeatedTasks.Add(task);

                    position = new float2(-60f - (i * 10f), -60f - (j * 10f));
                    goal = -position;
                    color = Color.white;
                    task = new SpawnTask(position, goal, color, this.simulator, this.agentData) { interval = 1000, frameCounter = 750 };
                    this.repeatedTasks.Add(task);
                }
            }

            // Obstacles.
            {
                this.obstacles = new List<int>();

                // Add (polygonal) obstacles, specifying their vertices in counterclockwise order.
                IList<float2> obstacle1 = new List<float2>
                {
                    new float2(-35f, 35f),
                    new float2(-45f, 15f),
                    new float2(-35f, 15f),
                    new float2(-15f, 35f),
                    new float2(-15f, 45f),
                };
                var task = new ObstacleSwitchTask(obstacle1, this.simulator, this.obstacles) { interval = 600, frameCounter = -300 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle2 = new List<float2>
                {
                    new float2(35f, 35f),
                    new float2(15f, 45f),
                    new float2(15f, 35f),
                    new float2(35f, 15f),
                    new float2(45f, 15f),
                };
                task = new ObstacleSwitchTask(obstacle2, this.simulator, this.obstacles) { interval = 600, frameCounter = 0 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle3 = new List<float2>
                {
                    new float2(35f, -35f),
                    new float2(45f, -15f),
                    new float2(35f, -15f),
                    new float2(15f, -35f),
                    new float2(15f, -45f),
                };
                task = new ObstacleSwitchTask(obstacle3, this.simulator, this.obstacles) { interval = 600, frameCounter = 300 };
                this.repeatedTasks.Add(task);

                IList<float2> obstacle4 = new List<float2>
                {
                    new float2(-35f, -35f),
                    new float2(-15f, -45f),
                    new float2(-15f, -35f),
                    new float2(-35f, -15f),
                    new float2(-45f, -15f),
                };
                task = new ObstacleSwitchTask(obstacle4, this.simulator, this.obstacles) { interval = 600, frameCounter = 600 };
                this.repeatedTasks.Add(task);
            }

            // Agents destroy.
            this.repeatedTasks.Add(new AgentCleanupTask(this.simulator, this.agentData) { interval = 5, frameCounter = 0 });
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

            Color gizmosBackup = Gizmos.color;

            foreach (var pair in this.agentData)
            {
                var agentId = pair.Key;
                float2 position = this.simulator.GetAgentPosition(agentId);
                Gizmos.color = pair.Value.color;
                Gizmos.DrawSphere((Vector2)position, 2);
            }

            Gizmos.color = gizmosBackup;
        }

        private void SetPreferredVelocities()
        {
            // Set the preferred velocity to be a vector of unit magnitude
            // (speed) in the direction of the goal.
            foreach (var pair in this.agentData)
            {
                var agentId = pair.Key;
                var goal = pair.Value.goal;
                float2 goalVector = goal - this.simulator.GetAgentPosition(agentId);

                if (math.lengthsq(goalVector) > 0.01f)
                {
                    goalVector = math.normalize(goalVector);
                    goalVector *= 0.5f;
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

        private void Start()
        {
            this.simulator = new Simulator();

            // Set up the scenario.
            this.SetupScenario();

            this.sampler = CustomSampler.Create("RVO update", false);
        }

        private void Update()
        {
            foreach (var task in this.repeatedTasks)
            {
                task.frameCounter++;
                if (task.frameCounter >= task.interval)
                {
                    task.frameCounter -= task.interval;
                    task.Execute();
                }
            }

            this.SetPreferredVelocities();

            this.sampler.Begin();
            this.simulator.DoStep();
            this.sampler.End();
        }

        private void OnDestroy()
        {
            this.simulator.Clear();

            this.simulator.Dispose();
        }

        private readonly struct GoalAndColor
        {
            internal readonly float2 goal;
            internal readonly Color color;

            internal GoalAndColor(float2 goal, Color color)
            {
                this.goal = goal;
                this.color = color;
            }
        }

        private abstract class RepeatedTask
        {
            internal int frameCounter;
            internal int interval;

            internal abstract void Execute();
        }

        private class SpawnTask : RepeatedTask
        {
            private readonly float2 position;
            private readonly float2 goal;
            private readonly Color color;
            private readonly Simulator simulator;
            private readonly Dictionary<int, GoalAndColor> agentData;

            internal SpawnTask(float2 position, float2 goal, Color color, Simulator simulator, Dictionary<int, GoalAndColor> agentData)
            {
                this.position = position;
                this.goal = goal;
                this.simulator = simulator;
                this.agentData = agentData;
                this.color = color;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                var agentId = this.simulator.AddAgent(this.position);
                this.agentData.Add(agentId, new GoalAndColor(this.goal, this.color));
            }
        }

        private class AgentCleanupTask : RepeatedTask
        {
            private readonly Simulator simulator;
            private readonly Dictionary<int, GoalAndColor> agentData;
            private readonly List<int> buffer = new List<int>();

            internal AgentCleanupTask(Simulator simulator, Dictionary<int, GoalAndColor> agentData)
            {
                this.simulator = simulator;
                this.agentData = agentData;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                foreach (var pair in this.agentData)
                {
                    var agentId = pair.Key;
                    var goal = pair.Value.goal;
                    if (math.lengthsq(
                        this.simulator.GetAgentPosition(agentId) - goal)
                            <= this.simulator.GetAgentRadius(agentId) * this.simulator.GetAgentRadius(agentId))
                    {
                        this.buffer.Add(agentId);
                    }
                }

                this.simulator.RemoveAgents(this.buffer);

                this.simulator.EnsureCompleted();

                foreach (var agent in this.buffer)
                {
                    this.agentData.Remove(agent);
                }

                this.buffer.Clear();
            }
        }

        private class ObstacleSwitchTask : RepeatedTask
        {
            private readonly IList<float2> points;

            private readonly Simulator simulator;
            private readonly List<int> obstacles;
            private int obstacleId = -1;

            internal ObstacleSwitchTask(IList<float2> points, Simulator simulator, List<int> obstacles)
            {
                this.points = points;
                this.simulator = simulator;
                this.obstacles = obstacles;
            }

            internal override void Execute()
            {
                this.simulator.EnsureCompleted();

                if (this.obstacleId == -1)
                {
                    this.obstacleId = this.simulator.AddObstacle(this.points);
                    this.obstacles.Add(this.obstacleId);
                }
                else
                {
                    this.simulator.RemoveObstacle(this.obstacleId);
                    this.obstacles.Remove(this.obstacleId);
                    this.obstacleId = -1;
                }
            }
        }
    }
}
