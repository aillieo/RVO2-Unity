// -----------------------------------------------------------------------
// <copyright file="Simulator.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Simulator.cs
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

namespace RVO
{
    using System;
    using System.Collections.Generic;
    using System.Threading;
    using Unity.Burst;
    using Unity.Collections;
    using Unity.Jobs;
    using Unity.Mathematics;
    using UnityEngine;

    /**
     * <summary>Defines the simulation.</summary>
     */
    public class Simulator : IDisposable
    {
        internal NativeList<Agent> agents;
        internal NativeList<Obstacle> obstacles;
        internal KdTree kdTree;
        internal float timeStep;

        internal NativeParallelHashMap<int, int> agentNoLookup;
        internal int sid;

        private Agent defaultAgent;

        private int numWorkers;

        private JobHandle jobHandle;

        private float globalTime;
        private bool disposedValue;

        /// <summary>
        /// Initializes a new instance of the <see cref="Simulator"/> class.
        /// </summary>
        public Simulator()
        {
            this.agents = new NativeList<Agent>(8, Allocator.Persistent);
            this.obstacles = new NativeList<Obstacle>(8, Allocator.Persistent);
            this.kdTree = new KdTree(0, 0);
            this.agentNoLookup = new NativeParallelHashMap<int, int>(8, Allocator.Persistent);

            this.Clear();
        }

        ~Simulator()
        {
            this.Dispose(disposing: false);
        }

        /**
         * <summary>Adds a new agent with default properties to the simulation.
         * </summary>
         *
         * <returns>The number of the agent, or -1 when the agent defaults have
         * not been set.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         */
        public int addAgent(float2 position)
        {
            return this.addAgent(
                position,
                this.defaultAgent.neighborDist,
                this.defaultAgent.maxNeighbors,
                this.defaultAgent.timeHorizon,
                this.defaultAgent.timeHorizonObst,
                this.defaultAgent.radius,
                this.defaultAgent.maxSpeed,
                this.defaultAgent.velocity);
        }

        /**
         * <summary>Adds a new agent to the simulation.</summary>
         *
         * <returns>The number of the agent.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         * <param name="neighborDist">The maximum distance (center point to
         * center point) to other agents this agent takes into account in the
         * navigation. The larger this number, the longer the running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The maximum number of other agents this
         * agent takes into account in the navigation. The larger this number,
         * the longer the running time of the simulation. If the number is too
         * low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The minimal amount of time for which this
         * agent's velocities that are computed by the simulation are safe with
         * respect to other agents. The larger this number, the sooner this
         * agent will respond to the presence of other agents, but the less
         * freedom this agent has in choosing its velocities. Must be positive.
         * </param>
         * <param name="timeHorizonObst">The minimal amount of time for which
         * this agent's velocities that are computed by the simulation are safe
         * with respect to obstacles. The larger this number, the sooner this
         * agent will respond to the presence of obstacles, but the less freedom
         * this agent has in choosing its velocities. Must be positive.</param>
         * <param name="radius">The radius of this agent. Must be non-negative.
         * </param>
         * <param name="maxSpeed">The maximum speed of this agent. Must be
         * non-negative.</param>
         * <param name="velocity">The initial two-dimensional linear velocity of
         * this agent.</param>
         */
        public int addAgent(float2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, float2 velocity)
        {
            var agents = this.agents;

            var agentIndex = this.NewAgent();
            Agent agent = agents[agentIndex];
            agent.maxNeighbors = maxNeighbors;
            agent.maxSpeed = maxSpeed;
            agent.neighborDist = neighborDist;
            agent.position = position;
            agent.radius = radius;
            agent.timeHorizon = timeHorizon;
            agent.timeHorizonObst = timeHorizonObst;
            agent.velocity = velocity;
            agents[agentIndex] = agent;

            this.agents = agents;

            return agent.id;
        }

        /**
         * <summary>Adds a new obstacle to the simulation.</summary>
         *
         * <returns>The number of the first vertex of the obstacle, or -1 when
         * the number of vertices is less than two.</returns>
         *
         * <param name="vertices">List of the vertices of the polygonal obstacle
         * in counterclockwise order.</param>
         *
         * <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
         * the environment, the vertices should be listed in clockwise order.
         * </remarks>
         */
        public int addObstacle(IList<float2> vertices)
        {
            if (vertices.Count < 2)
            {
                return -1;
            }

            var obstacles = this.obstacles;
            var obstacleNo = obstacles.Length;

            for (var i = 0; i < vertices.Count; ++i)
            {
                var obstacleIndex = this.NewObstacle(vertices[i]);
                Obstacle obstacle = obstacles[obstacleIndex];

                if (i != 0)
                {
                    obstacle.previousIndex = obstacleIndex - 1;
                    Obstacle previous = obstacles[obstacle.previousIndex];
                    previous.nextIndex = obstacleIndex;

                    obstacles[obstacleIndex] = obstacle;
                    obstacles[obstacle.previousIndex] = previous;
                }

                if (i == vertices.Count - 1)
                {
                    obstacle.nextIndex = obstacleNo;
                    Obstacle next = obstacles[obstacleNo];
                    next.previousIndex = obstacleNo;

                    obstacles[obstacleIndex] = obstacle;
                    obstacles[obstacleNo] = next;
                }

                obstacle.direction = math.normalize(vertices[i == vertices.Count - 1 ? 0 : i + 1] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle.convex = true;
                }
                else
                {
                    obstacle.convex = RVOMath.leftOf(vertices[i == 0 ? vertices.Count - 1 : i - 1], vertices[i], vertices[i == vertices.Count - 1 ? 0 : i + 1]) >= 0f;
                }

                obstacles[obstacleIndex] = obstacle;
            }

            this.obstacles = obstacles;

            return obstacleNo;
        }

        public bool RemoveAgent(int agentNo)
        {
            if (!this.agentNoLookup.TryGetValue(agentNo, out var index))
            {
                return false;
            }

            var lastIndex = this.agents.Length - 1;
            var lastId = this.agents[lastIndex].id;
            this.agents.RemoveAtSwapBack(index);
            this.agentNoLookup.Remove(agentNo);
            this.agentNoLookup[lastId] = index;

            return true;
        }

        public bool RemoveObstacle(int obstacle)
        {
            throw new NotImplementedException();
        }

        public void CompleteImmediate()
        {
            this.jobHandle.CheckAndComplete();
        }

        /**
         * <summary>Clears the simulation.</summary>
         */
        public void Clear()
        {
            this.CompleteImmediate();

            var agents = this.agents;

            if (agents.IsCreated && agents.Length > 0)
            {
                agents.Clear();
            }

            this.agents = agents;

            this.defaultAgent = default;

            this.kdTree.Clear();

            var obstacles = this.obstacles;
            if (obstacles.IsCreated && obstacles.Length > 0)
            {
                obstacles.Clear();
            }

            this.obstacles = obstacles;

            this.globalTime = 0f;
            this.timeStep = 0.1f;

            this.SetNumWorkers(0);
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        public JobHandle doStep()
        {
            var arrayLength = this.agents.Length;

            KdTree kdTree = this.kdTree;
            ensureTreeCapacity(ref kdTree, arrayLength);
            this.kdTree = kdTree;

            // job0
            var buildJob = new BuildJob(this.kdTree, this.agents.AsParallelReader());
            JobHandle jobHandle0 = buildJob.Schedule();

            // job1
            var innerLoop = Mathf.Max(arrayLength / Mathf.Max(this.numWorkers, 1), 1);
            var agentResult = new NativeArray<float2>(this.agents.Length, Allocator.TempJob);
            var computeJob = new ComputeJob(this.agents.AsParallelReader(), this.obstacles.AsParallelReader(), this.kdTree.AsParallelReader(), this.timeStep, agentResult);
            JobHandle jobHandle1 = computeJob.Schedule(arrayLength, innerLoop, jobHandle0);

            // job2
            var updateJob = new UpdateJob(this.agents, this.timeStep, agentResult);
            JobHandle jobHandle2 = updateJob.Schedule(arrayLength, innerLoop, jobHandle1);
            agentResult.Dispose(jobHandle2);

            // job3
            var globalTime = new NativeReference<float>(this.globalTime, Allocator.TempJob);
            var updateTimeJob = new UpdateTimeJob(globalTime, this.timeStep);
            JobHandle jobHandle3 = updateTimeJob.Schedule(jobHandle2);
            jobHandle3.Complete();

            this.globalTime = globalTime.Value;
            globalTime.Dispose(jobHandle3);

            this.jobHandle = jobHandle3;
            return this.jobHandle;
        }

        /**
         * <summary>Returns the specified agent neighbor of the specified agent.
         * </summary>
         *
         * <returns>The number of the neighboring agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose agent neighbor is
         * to be retrieved.</param>
         * <param name="neighborNo">The number of the agent neighbor to be
         * retrieved.</param>
         */
        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            throw new NotImplementedException();
        }

        /**
         * <summary>Returns the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor count of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be retrieved.</param>
         */
        public int getAgentMaxNeighbors(int agentNo)
        {
            return this.agents[agentNo].maxNeighbors;
        }

        /**
         * <summary>Returns the maximum speed of a specified agent.</summary>
         *
         * <returns>The present maximum speed of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be retrieved.</param>
         */
        public float getAgentMaxSpeed(int agentNo)
        {
            return this.agents[agentNo].maxSpeed;
        }

        /**
         * <summary>Returns the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor distance of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be retrieved.</param>
         */
        public float getAgentNeighborDist(int agentNo)
        {
            return this.agents[agentNo].neighborDist;
        }

        /**
         * <summary>Returns the count of agent neighbors taken into account to
         * compute the current velocity for the specified agent.</summary>
         *
         * <returns>The count of agent neighbors taken into account to compute
         * the current velocity for the specified agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose count of agent
         * neighbors is to be retrieved.</param>
         */
        public int getAgentNumAgentNeighbors(int agentNo)
        {
            throw new NotImplementedException();
        }

        /**
         * <summary>Returns the count of obstacle neighbors taken into account
         * to compute the current velocity for the specified agent.</summary>
         *
         * <returns>The count of obstacle neighbors taken into account to
         * compute the current velocity for the specified agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose count of obstacle
         * neighbors is to be retrieved.</param>
         */
        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            throw new NotImplementedException();
        }

        /**
         * <summary>Returns the specified obstacle neighbor of the specified
         * agent.</summary>
         *
         * <returns>The number of the first vertex of the neighboring obstacle
         * edge.</returns>
         *
         * <param name="agentNo">The number of the agent whose obstacle neighbor
         * is to be retrieved.</param>
         * <param name="neighborNo">The number of the obstacle neighbor to be
         * retrieved.</param>
         */
        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            throw new NotImplementedException();
        }

        /**
         * <summary>Returns the ORCA constraints of the specified agent.
         * </summary>
         *
         * <returns>A list of lines representing the ORCA constraints.</returns>
         *
         * <param name="agentNo">The number of the agent whose ORCA constraints
         * are to be retrieved.</param>
         *
         * <remarks>The halfplane to the left of each line is the region of
         * permissible velocities with respect to that ORCA constraint.
         * </remarks>
         */
        public IEnumerable<Line> getAgentOrcaLines(int agentNo)
        {
            throw new NotImplementedException();
        }

        /**
         * <summary>Returns the two-dimensional position of a specified agent.
         * </summary>
         *
         * <returns>The present two-dimensional position of the (center of the)
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be retrieved.</param>
         */
        public float2 getAgentPosition(int agentNo)
        {
            return this.agents[agentNo].position;
        }

        /**
         * <summary>Returns the two-dimensional preferred velocity of a
         * specified agent.</summary>
         *
         * <returns>The present two-dimensional preferred velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be retrieved.</param>
         */
        public float2 getAgentPrefVelocity(int agentNo)
        {
            return this.agents[agentNo].prefVelocity;
        }

        /**
         * <summary>Returns the radius of a specified agent.</summary>
         *
         * <returns>The present radius of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * retrieved.</param>
         */
        public float getAgentRadius(int agentNo)
        {
            return this.agents[agentNo].radius;
        }

        /**
         * <summary>Returns the time horizon of a specified agent.</summary>
         *
         * <returns>The present time horizon of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be retrieved.</param>
         */
        public float getAgentTimeHorizon(int agentNo)
        {
            return this.agents[agentNo].timeHorizon;
        }

        /**
         * <summary>Returns the time horizon with respect to obstacles of a
         * specified agent.</summary>
         *
         * <returns>The present time horizon with respect to obstacles of the
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be retrieved.</param>
         */
        public float getAgentTimeHorizonObst(int agentNo)
        {
            return this.agents[agentNo].timeHorizonObst;
        }

        /**
         * <summary>Returns the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <returns>The present two-dimensional linear velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be retrieved.</param>
         */
        public float2 getAgentVelocity(int agentNo)
        {
            return this.agents[agentNo].velocity;
        }

        /**
         * <summary>Returns the global time of the simulation.</summary>
         *
         * <returns>The present global time of the simulation (zero initially).
         * </returns>
         */
        public float getGlobalTime()
        {
            return this.globalTime;
        }

        /**
         * <summary>Returns the count of agents in the simulation.</summary>
         *
         * <returns>The count of agents in the simulation.</returns>
         */
        public int getNumAgents()
        {
            return this.agents.Length;
        }

        /**
         * <summary>Returns the count of obstacle vertices in the simulation.
         * </summary>
         *
         * <returns>The count of obstacle vertices in the simulation.</returns>
         */
        public int getNumObstacleVertices()
        {
            return this.obstacles.Length;
        }

        /**
         * <summary>Returns the count of workers.</summary>
         *
         * <returns>The count of workers.</returns>
         */
        public int GetNumWorkers()
        {
            return this.numWorkers;
        }

        /**
         * <summary>Returns the two-dimensional position of a specified obstacle
         * vertex.</summary>
         *
         * <returns>The two-dimensional position of the specified obstacle
         * vertex.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex to be
         * retrieved.</param>
         */
        public float2 getObstacleVertex(int vertexNo)
        {
            return this.obstacles[vertexNo].point;
        }

        /**
         * <summary>Returns the number of the obstacle vertex succeeding the
         * specified obstacle vertex in its polygon.</summary>
         *
         * <returns>The number of the obstacle vertex succeeding the specified
         * obstacle vertex in its polygon.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex whose
         * successor is to be retrieved.</param>
         */
        public int getNextObstacleVertexNo(int vertexNo)
        {
            return this.obstacles[vertexNo].nextIndex;
        }

        /**
         * <summary>Returns the number of the obstacle vertex preceding the
         * specified obstacle vertex in its polygon.</summary>
         *
         * <returns>The number of the obstacle vertex preceding the specified
         * obstacle vertex in its polygon.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex whose
         * predecessor is to be retrieved.</param>
         */
        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return this.obstacles[vertexNo].previousIndex;
        }

        /**
         * <summary>Returns the time step of the simulation.</summary>
         *
         * <returns>The present time step of the simulation.</returns>
         */
        public float getTimeStep()
        {
            return this.timeStep;
        }

        /**
         * <summary>Processes the obstacles that have been added so that they
         * are accounted for in the simulation.</summary>
         *
         * <remarks>Obstacles added to the simulation after this function has
         * been called are not accounted for in the simulation.</remarks>
         */
        public void processObstacles()
        {
            this.buildObstacleTree();
        }

        /**
         * <summary>Performs a visibility query between the two specified points
         * with respect to the obstacles.</summary>
         *
         * <returns>A boolean specifying whether the two points are mutually
         * visible. Returns true when the obstacles have not been processed.
         * </returns>
         *
         * <param name="point1">The first point of the query.</param>
         * <param name="point2">The second point of the query.</param>
         * <param name="radius">The minimal distance between the line connecting
         * the two points and the obstacles in order for the points to be
         * mutually visible (optional). Must be non-negative.</param>
         */
        public bool queryVisibility(float2 point1, float2 point2, float radius)
        {
            var kdTree = this.kdTree;
            var obstacles = this.obstacles;
            return kdTree.AsParallelReader().queryVisibility(point1, point2, radius, obstacles);
        }

        /**
         * <summary>Sets the default properties for any new agent that is added.
         * </summary>
         *
         * <param name="neighborDist">The default maximum distance (center point
         * to center point) to other agents a new agent takes into account in
         * the navigation. The larger this number, the longer he running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The default maximum number of other agents
         * a new agent takes into account in the navigation. The larger this
         * number, the longer the running time of the simulation. If the number
         * is too low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to other agents. The larger this number, the
         * sooner an agent will respond to the presence of other agents, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="timeHorizonObst">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to obstacles. The larger this number, the
         * sooner an agent will respond to the presence of obstacles, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="radius">The default radius of a new agent. Must be
         * non-negative.</param>
         * <param name="maxSpeed">The default maximum speed of a new agent. Must
         * be non-negative.</param>
         * <param name="velocity">The default initial two-dimensional linear
         * velocity of a new agent.</param>
         */
        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, float2 velocity)
        {
            this.defaultAgent = new Agent
            {
                maxNeighbors = maxNeighbors,
                maxSpeed = maxSpeed,
                neighborDist = neighborDist,
                radius = radius,
                timeHorizon = timeHorizon,
                timeHorizonObst = timeHorizonObst,
                velocity = velocity,
            };
        }

        /**
         * <summary>Sets the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be modified.</param>
         * <param name="maxNeighbors">The replacement maximum neighbor count.
         * </param>
         */
        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.maxNeighbors = maxNeighbors;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the maximum speed of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be modified.</param>
         * <param name="maxSpeed">The replacement maximum speed. Must be
         * non-negative.</param>
         */
        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.maxSpeed = maxSpeed;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be modified.</param>
         * <param name="neighborDist">The replacement maximum neighbor distance.
         * Must be non-negative.</param>
         */
        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.neighborDist = neighborDist;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the two-dimensional position of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be modified.</param>
         * <param name="position">The replacement of the two-dimensional
         * position.</param>
         */
        public void setAgentPosition(int agentNo, float2 position)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.position = position;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the two-dimensional preferred velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be modified.</param>
         * <param name="prefVelocity">The replacement of the two-dimensional
         * preferred velocity.</param>
         */
        public void setAgentPrefVelocity(int agentNo, float2 prefVelocity)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.prefVelocity = prefVelocity;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the radius of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * modified.</param>
         * <param name="radius">The replacement radius. Must be non-negative.
         * </param>
         */
        public void setAgentRadius(int agentNo, float radius)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.radius = radius;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * other agents.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be modified.</param>
         * <param name="timeHorizon">The replacement time horizon with respect
         * to other agents. Must be positive.</param>
         */
        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.timeHorizon = timeHorizon;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * obstacles.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be modified.</param>
         * <param name="timeHorizonObst">The replacement time horizon with
         * respect to obstacles. Must be positive.</param>
         */
        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.timeHorizonObst = timeHorizonObst;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be modified.</param>
         * <param name="velocity">The replacement two-dimensional linear
         * velocity.</param>
         */
        public void setAgentVelocity(int agentNo, float2 velocity)
        {
            var agents = this.agents;
            Agent agent = agents[agentNo];
            agent.velocity = velocity;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the global time of the simulation.</summary>
         *
         * <param name="globalTime">The global time of the simulation.</param>
         */
        public void setGlobalTime(float globalTime)
        {
            this.globalTime = globalTime;
        }

        /**
         * <summary>Sets the number of workers.</summary>
         *
         * <param name="numWorkers">The number of workers.</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            this.numWorkers = numWorkers;

            if (this.numWorkers <= 0)
            {
                ThreadPool.GetMinThreads(out this.numWorkers, out _);
            }
        }

        /**
         * <summary>Sets the time step of the simulation.</summary>
         *
         * <param name="timeStep">The time step of the simulation. Must be
         * positive.</param>
         */
        public void setTimeStep(float timeStep)
        {
            this.timeStep = timeStep;
        }

        public int queryAgent(float2 point, float radius, List<int> result)
        {
            if (result == null)
            {
                throw new ArgumentNullException(nameof(result));
            }

            this.CompleteImmediate();

            var kdTree = this.kdTree;
            NativeArray<Agent> agents = this.agents;

            var buffer = new NativeList<Agent>(Allocator.Temp);
            kdTree.AsParallelReader().queryAgentTree(in point, in radius, in agents, ref buffer);

            result.Clear();
            foreach (var a in buffer)
            {
                result.Add(a.id);
            }

            buffer.Dispose();

            return result.Count;
        }

        public void Dispose()
        {
            this.Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="nodeIndex">The current agent k-D tree node index.</param>
         */
        internal static void buildAgentTreeRecursive(ref KdTree kdTree, int begin, int end, int nodeIndex, in NativeArray<Agent>.ReadOnly agents)
        {
            KdTree.AgentTreeNode node = kdTree.agentTree[nodeIndex];
            node.begin = begin;
            node.end = end;
            Agent agentBegin = agents[kdTree.agents[begin]];
            node.minX = node.maxX = agentBegin.position.x;
            node.minY = node.maxY = agentBegin.position.y;

            for (var i = begin + 1; i < end; ++i)
            {
                Agent agentI = agents[kdTree.agents[i]];
                node.maxX = math.max(node.maxX, agentI.position.x);
                node.minX = math.min(node.minX, agentI.position.x);
                node.maxY = math.max(node.maxY, agentI.position.y);
                node.minY = math.min(node.minY, agentI.position.y);
            }

            kdTree.agentTree[nodeIndex] = node;

            if (end - begin > KdTree.MaxLeafSize)
            {
                /* No leaf node. */
                var isVertical = kdTree.agentTree[nodeIndex].maxX - kdTree.agentTree[nodeIndex].minX > kdTree.agentTree[nodeIndex].maxY - kdTree.agentTree[nodeIndex].minY;
                var splitValue = 0.5f * (isVertical ? kdTree.agentTree[nodeIndex].maxX + kdTree.agentTree[nodeIndex].minX : kdTree.agentTree[nodeIndex].maxY + kdTree.agentTree[nodeIndex].minY);

                var left = begin;
                var right = end;

                while (left < right)
                {
                    while (true)
                    {
                        Agent agentLeft = agents[kdTree.agents[left]];
                        if (left < right && (isVertical ? agentLeft.position.x : agentLeft.position.y) < splitValue)
                        {
                            ++left;
                        }
                        else
                        {
                            break;
                        }
                    }

                    while (true)
                    {
                        Agent agentRight = agents[kdTree.agents[right - 1]];
                        if (right > left && (isVertical ? agentRight.position.x : agentRight.position.y) >= splitValue)
                        {
                            --right;
                        }
                        else
                        {
                            break;
                        }
                    }

                    if (left < right)
                    {
                        var tempAgentIndex = kdTree.agents[left];
                        kdTree.agents[left] = kdTree.agents[right - 1];
                        kdTree.agents[right - 1] = tempAgentIndex;
                        ++left;
                        --right;
                    }
                }

                var leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                }

                node.left = nodeIndex + 1;
                node.right = nodeIndex + (2 * leftSize);
                kdTree.agentTree[nodeIndex] = node;

                buildAgentTreeRecursive(ref kdTree, begin, left, kdTree.agentTree[nodeIndex].left, agents);
                buildAgentTreeRecursive(ref kdTree, left, end, kdTree.agentTree[nodeIndex].right, agents);
            }
        }

        internal static void ensureTreeCapacity(ref KdTree kdTree, int agentCount)
        {
            if (kdTree.agents.Length != agentCount)
            {
                kdTree.agents.Resize(agentCount, Allocator.Persistent);
                for (var i = 0; i < agentCount; ++i)
                {
                    kdTree.agents[i] = i;
                }

                var agentTreeSize = 2 * agentCount;
                kdTree.agentTree.Resize(agentTreeSize, Allocator.Persistent);
                for (var i = 0; i < agentTreeSize; ++i)
                {
                    kdTree.agentTree[i] = default;
                }
            }
        }

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal static void buildAgentTree(ref KdTree kdTree, in NativeArray<Agent>.ReadOnly agents)
        {
            if (kdTree.agents.Length != 0)
            {
                for (var i = 0; i < agents.Length; i++)
                {
                    kdTree.agents[i] = i;
                }

                buildAgentTreeRecursive(ref kdTree, 0, kdTree.agents.Length, 0, agents);
            }
        }

        internal int NewObstacle(float2 point)
        {
            var obstacles = this.obstacles;

            var newIndex = obstacles.Length;
            var obstacle = new Obstacle(newIndex, point);
            obstacles.Add(obstacle);

            this.obstacles = obstacles;

            return newIndex;
        }

        /**
         * <summary>Builds an obstacle k-D tree.</summary>
         */
        internal void buildObstacleTree()
        {
            var obstacles = this.obstacles;

            var obstacleIds = new NativeArray<int>(obstacles.Length, Allocator.Temp);

            for (var i = 0; i < obstacles.Length; ++i)
            {
                obstacleIds[i] = i;
            }

            var kdTree = this.kdTree;
            this.buildObstacleTreeRecursive(ref kdTree, in obstacleIds);
            this.kdTree = kdTree;

            obstacleIds.Dispose();
        }

        /**
         * <summary>Recursive method for building an obstacle k-D tree.
         * </summary>
         *
         * <returns>An obstacle k-D tree node.</returns>
         *
         * <param name="obstacles">A list of obstacles.</param>
         */
        internal int buildObstacleTreeRecursive(ref KdTree kdTree, in NativeArray<int> obstacleIds)
        {
            if (obstacleIds.Length == 0)
            {
                return -1;
            }

            var nodeIndex = kdTree.NewObstacleTreeNode();
            KdTree.ObstacleTreeNode node = kdTree.obstacleTreeNodes[nodeIndex];
            this.kdTree = kdTree;

            var optimalSplit = 0;
            var minLeft = obstacleIds.Length;
            var minRight = obstacleIds.Length;

            var obstacles = this.obstacles;

            for (var i = 0; i < obstacleIds.Length; ++i)
            {
                var leftSize = 0;
                var rightSize = 0;

                var obstacleI1Index = obstacleIds[i];
                Obstacle obstacleI1 = obstacles[obstacleI1Index];
                var obstacleI2Index = obstacleI1.nextIndex;
                Obstacle obstacleI2 = obstacles[obstacleI2Index];

                /* Compute optimal split node. */
                for (var j = 0; j < obstacleIds.Length; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    var obstacleJ1Index = obstacleIds[j];
                    Obstacle obstacleJ1 = obstacles[obstacleJ1Index];
                    var obstacleJ2Index = obstacleJ1.nextIndex;
                    Obstacle obstacleJ2 = obstacles[obstacleJ2Index];

                    var j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    var j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        ++leftSize;
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        ++rightSize;
                    }
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    var bound1 = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                    var bound2 = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                    if (RVOMath.greaterequal(bound1, bound2))
                    {
                        break;
                    }
                }

                var bound1f = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                var bound2f = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                if (RVOMath.less(bound1f, bound2f))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                var leftObstacles = new NativeArray<int>(minLeft, Allocator.Temp);

                for (var n = 0; n < minLeft; ++n)
                {
                    leftObstacles[n] = -1;
                }

                var rightObstacles = new NativeArray<int>(minRight, Allocator.Temp);

                for (var n = 0; n < minRight; ++n)
                {
                    rightObstacles[n] = -1;
                }

                var leftCounter = 0;
                var rightCounter = 0;
                var i = optimalSplit;

                var obstacleI1Index = obstacleIds[i];
                Obstacle obstacleI1 = obstacles[obstacleI1Index];
                var obstacleI2Index = obstacleI1.nextIndex;
                Obstacle obstacleI2 = obstacles[obstacleI2Index];

                for (var j = 0; j < obstacleIds.Length; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    var obstacleJ1Index = obstacleIds[j];
                    Obstacle obstacleJ1 = obstacles[obstacleJ1Index];
                    var obstacleJ2Index = obstacleJ1.nextIndex;
                    Obstacle obstacleJ2 = obstacles[obstacleJ2Index];

                    var j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                    var j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        leftObstacles[leftCounter++] = obstacleIds[j];
                    }
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        rightObstacles[rightCounter++] = obstacleIds[j];
                    }
                    else
                    {
                        /* Split obstacle j. */
                        var t = RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleI1.point) / RVOMath.det(obstacleI2.point - obstacleI1.point, obstacleJ1.point - obstacleJ2.point);

                        float2 splitPoint = obstacleJ1.point + (t * (obstacleJ2.point - obstacleJ1.point));

                        var newObstacleIndex = this.NewObstacle(splitPoint);
                        Obstacle newObstacle = obstacles[newObstacleIndex];
                        newObstacle.previousIndex = obstacleJ1Index;
                        newObstacle.nextIndex = obstacleJ2Index;
                        newObstacle.convex = true;
                        newObstacle.direction = obstacleJ1.direction;
                        obstacles[newObstacleIndex] = newObstacle;

                        obstacleJ1.nextIndex = newObstacleIndex;
                        obstacleJ2.previousIndex = newObstacleIndex;
                        obstacles[obstacleJ1Index] = obstacleJ1;
                        obstacles[obstacleJ2Index] = obstacleJ2;

                        if (j1LeftOfI > 0f)
                        {
                            leftObstacles[leftCounter++] = obstacleJ1Index;
                            rightObstacles[rightCounter++] = newObstacleIndex;
                        }
                        else
                        {
                            rightObstacles[rightCounter++] = obstacleJ1Index;
                            leftObstacles[leftCounter++] = newObstacleIndex;
                        }
                    }
                }

                node.obstacleIndex = obstacleI1Index;
                kdTree.obstacleTreeNodes[nodeIndex] = node;

                var leftIndex = this.buildObstacleTreeRecursive(ref kdTree, leftObstacles);
                node = kdTree.obstacleTreeNodes[nodeIndex];
                node.leftIndex = leftIndex;
                kdTree.obstacleTreeNodes[nodeIndex] = node;

                var rightIndex = this.buildObstacleTreeRecursive(ref kdTree, rightObstacles);
                node = kdTree.obstacleTreeNodes[nodeIndex];
                node.rightIndex = rightIndex;
                kdTree.obstacleTreeNodes[nodeIndex] = node;

                leftObstacles.Dispose();
                rightObstacles.Dispose();

                return nodeIndex;
            }
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposedValue)
            {
                if (disposing)
                {
                    // Managed state
                    this.Clear();
                }

                this.agents.Dispose();
                this.obstacles.Dispose();

                this.kdTree.Dispose();

                this.agentNoLookup.Dispose();

                // Unmanaged resources
                this.disposedValue = true;
            }
        }

        private int NewAgent()
        {
            var agents = this.agents;
            var newIndex = agents.Length;
            var agentId = ++this.sid;
            var agent = new Agent(agentId);

            agents.Add(agent);
            this.agentNoLookup[agent.id] = newIndex;
            this.agents = agents;

            return newIndex;
        }

        [BurstCompile]
        private struct BuildJob : IJob
        {
            private KdTree kdTree;
            private NativeArray<Agent>.ReadOnly agents;

            public BuildJob(KdTree kdTree, NativeArray<Agent>.ReadOnly agents)
                : this()
            {
                this.kdTree = kdTree;
                this.agents = agents;
            }

            public void Execute()
            {
                buildAgentTree(ref this.kdTree, in this.agents);
            }
        }

        [BurstCompile]
        private struct ComputeJob : IJobParallelFor
        {
            [WriteOnly]
            internal NativeArray<float2> agentResult;

            private readonly float timeStep;
            [ReadOnly]
            private NativeArray<Agent>.ReadOnly agents;
            [ReadOnly]
            private NativeArray<Obstacle>.ReadOnly obstacles;
            [ReadOnly]
            private KdTree.ReadOnly kdTree;

            public ComputeJob(NativeArray<Agent>.ReadOnly agents, NativeArray<Obstacle>.ReadOnly obstacles, KdTree.ReadOnly kdTree, float timeStep, NativeArray<float2> agentResult)
                : this()
            {
                this.agents = agents;
                this.obstacles = obstacles;
                this.kdTree = kdTree;
                this.timeStep = timeStep;
                this.agentResult = agentResult;
            }

            public void Execute(int index)
            {
                var agents = this.agents;
                Agent agent = agents[index];

                var agentNeighbors = new NativeList<Agent.Pair>(Allocator.Temp);
                var obstacleNeighbors = new NativeList<Agent.Pair>(Allocator.Temp);

                agent.computeNeighbors(in index, in this.kdTree, in this.agents, in this.obstacles, ref agentNeighbors, ref obstacleNeighbors);
                agent.computeNewVelocity(this.timeStep, in this.agents, in this.obstacles, ref agentNeighbors, ref obstacleNeighbors);
                this.agentResult[index] = agent.newVelocity;

                agentNeighbors.Dispose();
                obstacleNeighbors.Dispose();
            }
        }

        [BurstCompile]
        private struct UpdateJob : IJobParallelFor
        {
            [ReadOnly]
            internal NativeArray<float2> agentResult;
            private readonly float timeStep;
            private NativeArray<Agent> agents;

            public UpdateJob(NativeArray<Agent> agents, float timeStep, NativeArray<float2> agentResult)
                : this()
            {
                this.agents = agents;
                this.timeStep = timeStep;
                this.agentResult = agentResult;
            }

            public void Execute(int index)
            {
                var agentNo = index;
                var agents = this.agents;

                Agent agent = agents[agentNo];
                agent.newVelocity = this.agentResult[agentNo];
                agent.update(this.timeStep);
                agents[agentNo] = agent;
            }
        }

        private struct UpdateTimeJob : IJob
        {
            private readonly float timeStep;
            private NativeReference<float> globalTime;

            public UpdateTimeJob(NativeReference<float> globalTime, float timeStep)
                : this()
            {
                this.globalTime = globalTime;
                this.timeStep = timeStep;
            }

            public void Execute()
            {
                this.globalTime.Value += this.timeStep;
            }
        }
    }
}
