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
    using Unity.Collections;
    using Unity.Jobs;
    using Unity.Mathematics;

    /**
     * <summary>Defines the simulation.</summary>
     */
    public class Simulator : IDisposable
    {
        private struct buildJob : IJob
        {
            public void Execute()
            {

            }
        }

        private struct computeJob : IJobParallelFor
        {
            public void Execute(int index)
            {

            }
        }

        private struct updateJob : IJobParallelFor
        {
            public void Execute(int index)
            {

            }
        }

        internal struct SimulationData : IDisposable
        {
            internal List<Agent> agents_;
            internal List<Obstacle> obstacles_;
            internal KdTree kdTree_;

            // NativeParallelMultiHashMap
            internal Dictionary<int, NativeList<Agent.Pair>> agentNeighbors_;
            internal Dictionary<int, NativeList<Agent.Pair>> obstacleNeighbors_;
            internal Dictionary<int, NativeList<Line>> orcaLines_;

            public void Dispose()
            {
                this.kdTree_.Dispose();
            }
        }

        internal SimulationData data;

        internal float timeStep_;

        private Agent defaultAgent_;

        private int numWorkers_;

        private JobHandle jobHandle;

        private float globalTime_;

        /**
         * <summary>Constructs and initializes a simulation.</summary>
         */
        public Simulator()
        {
            data = new SimulationData
            {
                agents_ = new List<Agent>(),
                obstacles_ = new List<Obstacle>(),
                kdTree_ = new KdTree(0, 0),
                agentNeighbors_ = new Dictionary<int, NativeList<Agent.Pair>>(),
                obstacleNeighbors_ = new Dictionary<int, NativeList<Agent.Pair>>(),
                orcaLines_ = new Dictionary<int, NativeList<Line>>(),
            };

            this.Clear();
        }

        private int NewAgent()
        {
            var agents = this.data.agents_;
            int newIndex = agents.Count;
            Agent agent = new Agent(newIndex);

            this.data.agentNeighbors_[newIndex] = new NativeList<Agent.Pair>(8, Allocator.Persistent);
            this.data.obstacleNeighbors_[newIndex] = new NativeList<Agent.Pair>(8, Allocator.Persistent);
            this.data.orcaLines_[newIndex] = new NativeList<Line>(8, Allocator.Persistent);

            agents.Add(agent);
            this.data.agents_ = agents;

            return newIndex;
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
                this.defaultAgent_.neighborDist_,
                this.defaultAgent_.maxNeighbors_,
                this.defaultAgent_.timeHorizon_,
                this.defaultAgent_.timeHorizonObst_,
                this.defaultAgent_.radius_,
                this.defaultAgent_.maxSpeed_,
                this.defaultAgent_.velocity_);
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
            var agents = this.data.agents_;

            int agentIndex = this.NewAgent();
            Agent agent = agents[agentIndex];
            agent.maxNeighbors_ = maxNeighbors;
            agent.maxSpeed_ = maxSpeed;
            agent.neighborDist_ = neighborDist;
            agent.position_ = position;
            agent.radius_ = radius;
            agent.timeHorizon_ = timeHorizon;
            agent.timeHorizonObst_ = timeHorizonObst;
            agent.velocity_ = velocity;
            agents[agentIndex] = agent;

            this.data.agents_ = agents;

            return agentIndex;
        }

        internal int NewObstacle(float2 point)
        {
            var obstacles = this.data.obstacles_;

            int newIndex = obstacles.Count;
            Obstacle obstacle = new Obstacle(newIndex, point);
            obstacles.Add(obstacle);

            this.data.obstacles_ = obstacles;

            return newIndex;
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

            var obstacles = this.data.obstacles_;
            int obstacleNo = obstacles.Count;

            for (int i = 0; i < vertices.Count; ++i)
            {
                int obstacleIndex = this.NewObstacle(vertices[i]);
                Obstacle obstacle = obstacles[obstacleIndex];

                if (i != 0)
                {
                    obstacle.previousIndex_ = obstacleIndex - 1;
                    Obstacle previous_ = obstacles[obstacle.previousIndex_];
                    previous_.nextIndex_ = obstacleIndex;

                    obstacles[obstacleIndex] = obstacle;
                    obstacles[obstacle.previousIndex_] = previous_;
                }

                if (i == vertices.Count - 1)
                {
                    obstacle.nextIndex_ = obstacleNo;
                    Obstacle next_ = obstacles[obstacleNo];
                    next_.previousIndex_ = obstacleNo;

                    obstacles[obstacleIndex] = obstacle;
                    obstacles[obstacleNo] = next_;
                }

                obstacle.direction_ = math.normalize(vertices[i == vertices.Count - 1 ? 0 : i + 1] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle.convex_ = true;
                }
                else
                {
                    obstacle.convex_ = RVOMath.leftOf(vertices[i == 0 ? vertices.Count - 1 : i - 1], vertices[i], vertices[i == vertices.Count - 1 ? 0 : i + 1]) >= 0.0f;
                }

                obstacles[obstacleIndex] = obstacle;
            }

            this.data.obstacles_ = obstacles;

            return obstacleNo;
        }

        /**
         * <summary>Clears the simulation.</summary>
         */
        public void Clear()
        {
            var agents = this.data.agents_;

            if (agents != null && agents.Count > 0)
            {
                agents.Clear();
            }

            this.data.agents_ = agents;

            this.defaultAgent_ = default;

            this.data.kdTree_.Clear();

            var obstacles = this.data.obstacles_;
            if (obstacles != null && obstacles.Count > 0)
            {
                obstacles.Clear();
            }

            this.data.obstacles_ = obstacles;

            var agentNeighbors = this.data.agentNeighbors_;
            for (int i = 0; i < agentNeighbors.Count; ++i)
            {
                agentNeighbors[i].Dispose();
                agentNeighbors[i] = default;
            }

            agentNeighbors.Clear();

            var obstacleNeighbors = this.data.obstacleNeighbors_;
            for (int i = 0; i < obstacleNeighbors.Count; ++i)
            {
                obstacleNeighbors[i].Dispose();
                obstacleNeighbors[i] = default;
            }

            obstacleNeighbors.Clear();

            var orcaLines = this.data.orcaLines_;
            for (int i = 0; i < orcaLines.Count; ++i)
            {
                orcaLines[i].Dispose();
                orcaLines[i] = default;
            }

            orcaLines.Clear();

            this.globalTime_ = 0.0f;
            this.timeStep_ = 0.1f;

            this.SetNumWorkers(0);
        }

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal void buildAgentTree()
        {
            var kdTree = this.data.kdTree_;

            if (!kdTree.agents_.IsCreated || kdTree.agents_.Length != this.data.agents_.Count)
            {
                kdTree.agents_.Clear();
                kdTree.agents_.Capacity = this.data.agents_.Count;

                for (int i = 0; i < this.data.agents_.Count; ++i)
                {
                    kdTree.agents_.Add(i);
                }

                int agentTreeSize = 2 * kdTree.agents_.Length;
                kdTree.agentTree_.Clear();
                kdTree.agentTree_.Capacity = agentTreeSize;

                for (int i = 0; i < agentTreeSize; ++i)
                {
                    kdTree.agentTree_.Add(default(KdTree.AgentTreeNode));
                }
            }

            if (kdTree.agents_.Length != 0)
            {
                this.buildAgentTreeRecursive(0, kdTree.agents_.Length, 0, this.data.agents_);
            }

            this.data.kdTree_ = kdTree;
        }

        /**
         * <summary>Builds an obstacle k-D tree.</summary>
         */
        internal void buildObstacleTree()
        {
            var obstacles = this.data.obstacles_;

            var kdTree = this.data.kdTree_;
            kdTree.obstacleTreeNodes_.Clear();
            this.data.kdTree_ = kdTree;

            List<int> obstacleIds = new List<int>(obstacles.Count);

            for (int i = 0; i < obstacles.Count; ++i)
            {
                obstacleIds.Add(i);
            }

            this.buildObstacleTreeRecursive(obstacleIds);
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="nodeIndex">The current agent k-D tree node index.</param>
         */
        internal void buildAgentTreeRecursive(int begin, int end, int nodeIndex, List<Agent> agents)
        {
            var kdTree = this.data.kdTree_;
            KdTree.AgentTreeNode node = kdTree.agentTree_[nodeIndex];
            node.begin_ = begin;
            node.end_ = end;
            Agent agentBegin = agents[kdTree.agents_[begin]];
            node.minX_ = node.maxX_ = agentBegin.position_.x;
            node.minY_ = node.maxY_ = agentBegin.position_.y;

            for (int i = begin + 1; i < end; ++i)
            {
                Agent agentI = agents[kdTree.agents_[i]];
                node.maxX_ = math.max(node.maxX_, agentI.position_.x);
                node.minX_ = math.min(node.minX_, agentI.position_.x);
                node.maxY_ = math.max(node.maxY_, agentI.position_.y);
                node.minY_ = math.min(node.minY_, agentI.position_.y);
            }

            kdTree.agentTree_[nodeIndex] = node;

            if (end - begin > KdTree.MAX_LEAF_SIZE)
            {
                /* No leaf node. */
                bool isVertical = kdTree.agentTree_[nodeIndex].maxX_ - kdTree.agentTree_[nodeIndex].minX_ > kdTree.agentTree_[nodeIndex].maxY_ - kdTree.agentTree_[nodeIndex].minY_;
                float splitValue = 0.5f * (isVertical ? kdTree.agentTree_[nodeIndex].maxX_ + kdTree.agentTree_[nodeIndex].minX_ : kdTree.agentTree_[nodeIndex].maxY_ + kdTree.agentTree_[nodeIndex].minY_);

                int left = begin;
                int right = end;

                while (left < right)
                {
                    while (true)
                    {
                        Agent agentLeft = agents[kdTree.agents_[left]];
                        if (left < right && (isVertical ? agentLeft.position_.x : agentLeft.position_.y) < splitValue)
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
                        Agent agentRight = agents[kdTree.agents_[right - 1]];
                        if (right > left && (isVertical ? agentRight.position_.x : agentRight.position_.y) >= splitValue)
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
                        int tempAgentIndex = kdTree.agents_[left];
                        kdTree.agents_[left] = kdTree.agents_[right - 1];
                        kdTree.agents_[right - 1] = tempAgentIndex;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                }

                node.left_ = nodeIndex + 1;
                node.right_ = nodeIndex + (2 * leftSize);
                kdTree.agentTree_[nodeIndex] = node;

                this.buildAgentTreeRecursive(begin, left, kdTree.agentTree_[nodeIndex].left_, agents);
                this.buildAgentTreeRecursive(left, end, kdTree.agentTree_[nodeIndex].right_, agents);
            }
        }

        /**
         * <summary>Recursive method for building an obstacle k-D tree.
         * </summary>
         *
         * <returns>An obstacle k-D tree node.</returns>
         *
         * <param name="obstacles">A list of obstacles.</param>
         */
        internal int buildObstacleTreeRecursive(List<int> obstacleIds)
        {
            if (obstacleIds.Count == 0)
            {
                return -1;
            }

            var kdTree = this.data.kdTree_;

            int nodeIndex = kdTree.NewObstacleTreeNode();
            KdTree.ObstacleTreeNode node = kdTree.obstacleTreeNodes_[nodeIndex];

            int optimalSplit = 0;
            int minLeft = obstacleIds.Count;
            int minRight = obstacleIds.Count;

            var obstacles = this.data.obstacles_;

            for (int i = 0; i < obstacleIds.Count; ++i)
            {
                int leftSize = 0;
                int rightSize = 0;

                int obstacleI1Index = obstacleIds[i];
                Obstacle obstacleI1 = obstacles[obstacleI1Index];
                int obstacleI2Index = obstacleI1.nextIndex_;
                Obstacle obstacleI2 = obstacles[obstacleI2Index];

                /* Compute optimal split node. */
                for (int j = 0; j < obstacleIds.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    int obstacleJ1Index = obstacleIds[j];
                    Obstacle obstacleJ1 = obstacles[obstacleJ1Index];
                    int obstacleJ2Index = obstacleJ1.nextIndex_;
                    Obstacle obstacleJ2 = obstacles[obstacleJ2Index];

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

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

                    float2 bound1 = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                    float2 bound2 = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                    if (RVOMath.greaterequal(bound1, bound2))
                    {
                        break;
                    }
                }

                float2 bound1f = new float2(math.max(leftSize, rightSize), math.min(leftSize, rightSize));
                float2 bound2f = new float2(math.max(minLeft, minRight), math.min(minLeft, minRight));

                if (RVOMath.less(bound1f, bound2f))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* Build split node. */
                List<int> leftObstacles = new List<int>(minLeft);

                for (int n = 0; n < minLeft; ++n)
                {
                    leftObstacles.Add(-1);
                }

                List<int> rightObstacles = new List<int>(minRight);

                for (int n = 0; n < minRight; ++n)
                {
                    rightObstacles.Add(-1);
                }

                int leftCounter = 0;
                int rightCounter = 0;
                int i = optimalSplit;

                int obstacleI1Index = obstacleIds[i];
                Obstacle obstacleI1 = obstacles[obstacleI1Index];
                int obstacleI2Index = obstacleI1.nextIndex_;
                Obstacle obstacleI2 = obstacles[obstacleI2Index];

                for (int j = 0; j < obstacleIds.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    int obstacleJ1Index = obstacleIds[j];
                    Obstacle obstacleJ1 = obstacles[obstacleJ1Index];
                    int obstacleJ2Index = obstacleJ1.nextIndex_;
                    Obstacle obstacleJ2 = obstacles[obstacleJ2Index];

                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

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
                        float t = RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_);

                        float2 splitPoint = obstacleJ1.point_ + (t * (obstacleJ2.point_ - obstacleJ1.point_));

                        int newObstacleIndex = this.NewObstacle(splitPoint);
                        Obstacle newObstacle = obstacles[newObstacleIndex];
                        newObstacle.previousIndex_ = obstacleJ1Index;
                        newObstacle.nextIndex_ = obstacleJ2Index;
                        newObstacle.convex_ = true;
                        newObstacle.direction_ = obstacleJ1.direction_;
                        obstacles[newObstacleIndex] = newObstacle;

                        // newObstacle.id_ = this.obstacles_.Count;

                        // this.obstacles_.Add(newObstacle);

                        obstacleJ1.nextIndex_ = newObstacleIndex;
                        obstacleJ2.previousIndex_ = newObstacleIndex;
                        obstacles[obstacleJ1Index] = obstacleJ1;
                        obstacles[obstacleJ2Index] = obstacleJ2;

                        if (j1LeftOfI > 0.0f)
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

                node.obstacleIndex_ = obstacleI1Index;
                kdTree.obstacleTreeNodes_[nodeIndex] = node;

                var leftIndex = this.buildObstacleTreeRecursive(leftObstacles);
                node = kdTree.obstacleTreeNodes_[nodeIndex];
                node.leftIndex_ = leftIndex;
                kdTree.obstacleTreeNodes_[nodeIndex] = node;

                var rightIndex = this.buildObstacleTreeRecursive(rightObstacles);
                node = kdTree.obstacleTreeNodes_[nodeIndex];
                node.rightIndex_ = rightIndex;
                kdTree.obstacleTreeNodes_[nodeIndex] = node;

                return nodeIndex;
            }
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        public JobHandle doStep()
        {
            // job0
            JobHandle jobHandle0 = new buildJob().Schedule();
            this.buildAgentTree();

            // job1
            JobHandle jobHandle1 = new computeJob().Schedule(1, 1, jobHandle0);
            var agents = this.data.agents_;
            for (int agentNo = 0; agentNo < agents.Count; ++agentNo)
            {
                Agent agent = agents[agentNo];
                agent.computeNeighbors(this.data);
                agent.computeNewVelocity(this.timeStep_, this.data);
                agents[agentNo] = agent;
            }

            // job2
            JobHandle jobHandle2 = new updateJob().Schedule(1, 1, jobHandle1);
            for (int agentNo = 0; agentNo < agents.Count; ++agentNo)
            {
                Agent agent = agents[agentNo];
                agent.update(this.timeStep_);
                agents[agentNo] = agent;
            }

            this.globalTime_ += this.timeStep_;

            this.jobHandle = jobHandle2;
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
            return this.data.agentNeighbors_[agentNo][neighborNo].Value;
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
            return this.data.agents_[agentNo].maxNeighbors_;
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
            return this.data.agents_[agentNo].maxSpeed_;
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
            return this.data.agents_[agentNo].neighborDist_;
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
            return this.data.agentNeighbors_[agentNo].Length;
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
            return this.data.obstacleNeighbors_[agentNo].Length;
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
            return this.data.obstacleNeighbors_[agentNo][neighborNo].Value;
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
            return this.data.orcaLines_[agentNo];
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
            return this.data.agents_[agentNo].position_;
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
            return this.data.agents_[agentNo].prefVelocity_;
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
            return this.data.agents_[agentNo].radius_;
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
            return this.data.agents_[agentNo].timeHorizon_;
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
            return this.data.agents_[agentNo].timeHorizonObst_;
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
            return this.data.agents_[agentNo].velocity_;
        }

        /**
         * <summary>Returns the global time of the simulation.</summary>
         *
         * <returns>The present global time of the simulation (zero initially).
         * </returns>
         */
        public float getGlobalTime()
        {
            return this.globalTime_;
        }

        /**
         * <summary>Returns the count of agents in the simulation.</summary>
         *
         * <returns>The count of agents in the simulation.</returns>
         */
        public int getNumAgents()
        {
            return this.data.agents_.Count;
        }

        /**
         * <summary>Returns the count of obstacle vertices in the simulation.
         * </summary>
         *
         * <returns>The count of obstacle vertices in the simulation.</returns>
         */
        public int getNumObstacleVertices()
        {
            return this.data.obstacles_.Count;
        }

        /**
         * <summary>Returns the count of workers.</summary>
         *
         * <returns>The count of workers.</returns>
         */
        public int GetNumWorkers()
        {
            return this.numWorkers_;
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
            return this.data.obstacles_[vertexNo].point_;
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
            return this.data.obstacles_[vertexNo].nextIndex_;
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
            return this.data.obstacles_[vertexNo].previousIndex_;
        }

        /**
         * <summary>Returns the time step of the simulation.</summary>
         *
         * <returns>The present time step of the simulation.</returns>
         */
        public float getTimeStep()
        {
            return this.timeStep_;
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
            var kdTree = this.data.kdTree_;
            var obstacles = this.data.obstacles_;
            return kdTree.queryVisibility(point1, point2, radius, obstacles);
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
            this.defaultAgent_ = new Agent
            {
                maxNeighbors_ = maxNeighbors,
                maxSpeed_ = maxSpeed,
                neighborDist_ = neighborDist,
                radius_ = radius,
                timeHorizon_ = timeHorizon,
                timeHorizonObst_ = timeHorizonObst,
                velocity_ = velocity,
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.maxNeighbors_ = maxNeighbors;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.maxSpeed_ = maxSpeed;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.neighborDist_ = neighborDist;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.position_ = position;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.prefVelocity_ = prefVelocity;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.radius_ = radius;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.timeHorizon_ = timeHorizon;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.timeHorizonObst_ = timeHorizonObst;
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
            var agents = this.data.agents_;
            Agent agent = agents[agentNo];
            agent.velocity_ = velocity;
            agents[agentNo] = agent;
        }

        /**
         * <summary>Sets the global time of the simulation.</summary>
         *
         * <param name="globalTime">The global time of the simulation.</param>
         */
        public void setGlobalTime(float globalTime)
        {
            this.globalTime_ = globalTime;
        }

        /**
         * <summary>Sets the number of workers.</summary>
         *
         * <param name="numWorkers">The number of workers.</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            this.numWorkers_ = numWorkers;

            if (this.numWorkers_ <= 0)
            {
                ThreadPool.GetMinThreads(out this.numWorkers_, out _);
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
            this.timeStep_ = timeStep;
        }

        public void Dispose()
        {
            this.Clear();

            this.data.Dispose();
        }
    }
}
