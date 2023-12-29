// -----------------------------------------------------------------------
// <copyright file="KdTree.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * KdTree.cs
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
    using Unity.Collections;
    using Unity.Mathematics;

    /// <summary>
    /// Defines k-D trees for agentIds and static obstacles in the simulation.
    /// </summary>
    internal struct KdTree : IDisposable
    {
        /// <summary>
        /// The maximum size of an agent k-D tree leaf.
        /// </summary>
        internal const int MaxLeafSize = 10;

        internal NativeArray<int> agentIds;

        internal NativeArray<AgentTreeNode> agentTree;
        internal NativeArray<ObstacleTreeNode> obstacleTreeNodes;

        internal KdTree(int agentCount, int obstacleCount)
            : this()
        {
            this.agentIds = new NativeArray<int>(agentCount, Allocator.Persistent);
            this.agentTree = new NativeArray<AgentTreeNode>(agentCount * 2, Allocator.Persistent);
            this.obstacleTreeNodes = new NativeArray<ObstacleTreeNode>(obstacleCount, Allocator.Persistent);
        }

        public void Dispose()
        {
            this.Clear();

            this.agentIds.SafeDispose();
            this.agentIds = default;

            this.agentTree.SafeDispose();
            this.agentTree = default;

            this.obstacleTreeNodes.SafeDispose();
            this.obstacleTreeNodes = default;
        }

        internal ReadOnly AsParallelReader()
        {
            return new ReadOnly()
            {
                agentIds = this.agentIds.AsReadOnly(),
                agentTree = this.agentTree.AsReadOnly(),
                obstacleTreeNodes = this.obstacleTreeNodes.AsReadOnly(),
            };
        }

        internal int NewObstacleTreeNode()
        {
            var node = new ObstacleTreeNode
            {
                valid = true,
                obstacleIndex = 0,
                leftIndex = -1,
                rightIndex = -1,
            };

            var oldLen = this.obstacleTreeNodes.Length;
            this.obstacleTreeNodes.Append(node);
            return oldLen;
        }

        internal readonly void Clear()
        {
        }

        /// <summary>
        /// Defines a node of an agent k-D tree.
        /// </summary>
        internal struct AgentTreeNode
        {
            internal int begin;
            internal int end;
            internal int left;
            internal int right;
            internal float maxX;
            internal float maxY;
            internal float minX;
            internal float minY;
        }

        internal struct ReadOnly
        {
            internal NativeArray<int>.ReadOnly agentIds;
            internal NativeArray<AgentTreeNode>.ReadOnly agentTree;
            internal NativeArray<ObstacleTreeNode>.ReadOnly obstacleTreeNodes;

            /// <summary>
            /// Computes the agent neighbors of the specified agent.
            /// </summary>
            /// <param name="agentIndex">The agent for which agent neighbors are
            /// to be computed.</param>
            /// <param name="rangeSq">The squared range around the agent.</param>
            /// <param name="agents">The array that holds the agent data.</param>
            /// <param name="agentNeighbors">The list to store the neighbor data.</param>
            internal readonly void ComputeAgentNeighbors(
                int agentIndex,
                ref float rangeSq,
                in NativeArray<Agent>.ReadOnly agents,
                ref NativeList<Agent.Pair> agentNeighbors)
            {
                this.QueryAgentTreeRecursive(agentIndex, ref rangeSq, 0, in agents, ref agentNeighbors);
            }

            /// <summary>
            /// Computes the obstacle neighbors of the specified agent.
            /// </summary>
            /// <param name="agent">The agent for which obstacle neighbors are
            /// to be computed.</param>
            /// <param name="rangeSq">The squared range around the agent.</param>
            /// <param name="obstacles">The array that holds the obstacle verts.</param>
            /// <param name="obstacleNeighbors">The list to store the neighbor dara.</param>
            internal readonly void ComputeObstacleNeighbors(
                Agent agent,
                float rangeSq,
                in NativeArray<Obstacle>.ReadOnly obstacles,
                ref NativeList<Agent.Pair> obstacleNeighbors)
            {
                this.QueryObstacleTreeRecursive(agent, rangeSq, 0, in obstacles, ref obstacleNeighbors);
            }

            /// <summary>
            /// Queries the visibility between two points within a specified radius.
            /// </summary>
            /// <param name="q1">The first point between which visibility is to be tested.</param>
            /// <param name="q2">The second point between which visibility is to be tested.</param>
            /// <param name="radius">The radius within which visibility is to be tested.</param>
            /// <param name="obstacles">The array that holds the obstacle verts.</param>
            /// <returns>True if q1 and q2 are mutually visible within the radius; false otherwise.</returns>
            internal bool QueryVisibility(float2 q1, float2 q2, float radius, NativeList<Obstacle> obstacles)
            {
                return this.QueryVisibilityRecursive(q1, q2, radius, 0, obstacles);
            }

            internal void QueryAgentTree(
                in float2 position,
                in float range,
                in NativeArray<Agent> agents,
                ref NativeList<Agent> result)
            {
                if (agents.Length > 0)
                {
                    this.QueryAgentTreeRecursive(0, position, range, in agents, ref result);
                }
            }

            /// <summary>
            /// Recursive method for computing the agent neighbors of the specified agent.
            /// </summary>
            /// <param name="agentIndex">The agent for which agent neighbors are
            /// to be computed.</param>
            /// <param name="rangeSq">The squared range around the agent.</param>
            /// <param name="node">The current agent k-D tree node index.</param>
            /// <param name="agents">The array that holds the agent data.</param>
            /// <param name="agentNeighbors">The list to store the neighbor data.</param>
            private readonly void QueryAgentTreeRecursive(
                int agentIndex,
                ref float rangeSq,
                int node,
                in NativeArray<Agent>.ReadOnly agents,
                ref NativeList<Agent.Pair> agentNeighbors)
            {
                Agent agent = agents[agentIndex];
                AgentTreeNode agentTreeNode = this.agentTree[node];

                if (agentTreeNode.end - agentTreeNode.begin <= MaxLeafSize)
                {
                    for (var i = agentTreeNode.begin; i < agentTreeNode.end; ++i)
                    {
                        agent.InsertAgentNeighbor(this.agentIds[i], ref rangeSq, in agents, ref agentNeighbors);
                    }
                }
                else
                {
                    var leftChild = this.agentTree[agentTreeNode.left];
                    var rightChild = this.agentTree[agentTreeNode.right];

                    var distSqLeft = RVOMath.Square(math.max(0f, leftChild.minX - agent.position.x))
                        + RVOMath.Square(math.max(0f, agent.position.x - leftChild.maxX))
                        + RVOMath.Square(math.max(0f, leftChild.minY - agent.position.y))
                        + RVOMath.Square(math.max(0f, agent.position.y - leftChild.maxY));
                    var distSqRight = RVOMath.Square(math.max(0f, rightChild.minX - agent.position.x))
                        + RVOMath.Square(math.max(0f, agent.position.x - rightChild.maxX))
                        + RVOMath.Square(math.max(0f, rightChild.minY - agent.position.y))
                        + RVOMath.Square(math.max(0f, agent.position.y - rightChild.maxY));

                    if (distSqLeft < distSqRight)
                    {
                        if (!(distSqLeft < rangeSq))
                        {
                            return;
                        }

                        this.QueryAgentTreeRecursive(agentIndex, ref rangeSq, agentTreeNode.left, in agents, ref agentNeighbors);

                        if (distSqRight < rangeSq)
                        {
                            this.QueryAgentTreeRecursive(agentIndex, ref rangeSq, agentTreeNode.right, in agents, ref agentNeighbors);
                        }
                    }
                    else
                    {
                        if (!(distSqRight < rangeSq))
                        {
                            return;
                        }

                        this.QueryAgentTreeRecursive(agentIndex, ref rangeSq, agentTreeNode.right, in agents, ref agentNeighbors);

                        if (distSqLeft < rangeSq)
                        {
                            this.QueryAgentTreeRecursive(agentIndex, ref rangeSq, agentTreeNode.left, in agents, ref agentNeighbors);
                        }
                    }
                }
            }

            /// <summary>
            /// Recursive method for computing the obstacle neighbors of the specified agent.
            /// </summary>
            /// <param name="agent"> The agent for which obstacle neighbors are
            /// to be computed.</param>
            /// <param name="rangeSq"> The squared range around the agent.</param>
            /// <param name="nodeIndex"> The current obstacle k-D node.</param>
            /// <param name="obstacles">The array that holds the obstacle verts.</param>
            /// <param name="obstacleNeighbors">The list to store the neighbor dara.</param>
            private readonly void QueryObstacleTreeRecursive(
                Agent agent,
                float rangeSq,
                int nodeIndex,
                in NativeArray<Obstacle>.ReadOnly obstacles,
                ref NativeList<Agent.Pair> obstacleNeighbors)
            {
                ObstacleTreeNode node = default;
                if (nodeIndex >= 0 && nodeIndex < this.obstacleTreeNodes.Length)
                {
                    node = this.obstacleTreeNodes[nodeIndex];
                }

                if (!node.valid)
                {
                    return;
                }

                var obstacle1Index = node.obstacleIndex;
                Obstacle obstacle1 = obstacles[obstacle1Index];
                var obstacle2Index = obstacle1.nextIndex;
                Obstacle obstacle2 = obstacles[obstacle2Index];

                var agentLeftOfLine = RVOMath.LeftOf(obstacle1.point, obstacle2.point, agent.position);

                this.QueryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0f ? node.leftIndex : node.rightIndex, in obstacles, ref obstacleNeighbors);

                var distSqLine = RVOMath.Square(agentLeftOfLine) / math.lengthsq(obstacle2.point - obstacle1.point);

                if (!(distSqLine < rangeSq))
                {
                    return;
                }

                if (agentLeftOfLine < 0f)
                {
                    // Try obstacle at this node only if agent is on right side of
                    // obstacle (and can see obstacle).
                    agent.InsertObstacleNeighbor(node.obstacleIndex, rangeSq, in obstacles, ref obstacleNeighbors);
                }

                // Try other side of line.
                this.QueryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0f ? node.rightIndex : node.leftIndex, in obstacles, ref obstacleNeighbors);
            }

            /// <summary>
            /// Recursive method for querying the visibility between two points within a specified radius.
            /// </summary>
            /// <param name="q1">The first point between which visibility is to be tested.</param>
            /// <param name="q2">The second point between which visibility is to be tested.</param>
            /// <param name="radius">The radius within which visibility is to be tested.</param>
            /// <param name="nodeIndex">The current obstacle k-D node.</param>
            /// <param name="obstacles">The list to store the obstacle verts.</param>
            /// <returns>If q1 and q2 are mutually visible within the radius or not.</returns>
            private bool QueryVisibilityRecursive(
                float2 q1,
                float2 q2,
                float radius,
                int nodeIndex,
                NativeList<Obstacle> obstacles)
            {
                ObstacleTreeNode node = default;
                if (nodeIndex >= 0 && nodeIndex < this.obstacleTreeNodes.Length)
                {
                    node = this.obstacleTreeNodes[nodeIndex];
                }

                if (!node.valid)
                {
                    return true;
                }

                var obstacle1Index = node.obstacleIndex;
                Obstacle obstacle1 = obstacles[obstacle1Index];
                var obstacle2Index = obstacle1.nextIndex;
                Obstacle obstacle2 = obstacles[obstacle2Index];

                var q1LeftOfI = RVOMath.LeftOf(obstacle1.point, obstacle2.point, q1);
                var q2LeftOfI = RVOMath.LeftOf(obstacle1.point, obstacle2.point, q2);
                var invLengthI = 1f / math.lengthsq(obstacle2.point - obstacle1.point);
                var radiusSq = RVOMath.Square(radius);

                if (q1LeftOfI >= 0f && q2LeftOfI >= 0f)
                {
                    return this.QueryVisibilityRecursive(q1, q2, radius, node.leftIndex, obstacles)
                        && ((RVOMath.Square(q1LeftOfI) * invLengthI >= radiusSq && RVOMath.Square(q2LeftOfI) * invLengthI >= radiusSq)
                            || this.QueryVisibilityRecursive(q1, q2, radius, node.rightIndex, obstacles));
                }

                if (q1LeftOfI <= 0f && q2LeftOfI <= 0f)
                {
                    return this.QueryVisibilityRecursive(q1, q2, radius, node.rightIndex, obstacles)
                        && ((RVOMath.Square(q1LeftOfI) * invLengthI >= radiusSq && RVOMath.Square(q2LeftOfI) * invLengthI >= radiusSq)
                            || this.QueryVisibilityRecursive(q1, q2, radius, node.leftIndex, obstacles));
                }

                if (q1LeftOfI >= 0f && q2LeftOfI <= 0f)
                {
                    // One can see through obstacle from left to right.
                    return this.QueryVisibilityRecursive(q1, q2, radius, node.leftIndex, obstacles)
                        && this.QueryVisibilityRecursive(q1, q2, radius, node.rightIndex, obstacles);
                }

                var point1LeftOfQ = RVOMath.LeftOf(q1, q2, obstacle1.point);
                var point2LeftOfQ = RVOMath.LeftOf(q1, q2, obstacle2.point);
                var invLengthQ = 1f / math.lengthsq(q2 - q1);

                return point1LeftOfQ * point2LeftOfQ >= 0f
                    && RVOMath.Square(point1LeftOfQ) * invLengthQ > radiusSq
                    && RVOMath.Square(point2LeftOfQ) * invLengthQ > radiusSq
                    && this.QueryVisibilityRecursive(q1, q2, radius, node.leftIndex, obstacles)
                    && this.QueryVisibilityRecursive(q1, q2, radius, node.rightIndex, obstacles);
            }

            private void QueryAgentTreeRecursive(
                int nodeIndex,
                in float2 position,
                in float range,
                in NativeArray<Agent> agents,
                ref NativeList<Agent> result)
            {
                AgentTreeNode node = this.agentTree[nodeIndex];

                // Check if the position is within the range of the node's bounding box
                if (position.x - range > node.maxX
                    || position.x + range < node.minX
                    || position.y - range > node.maxY
                    || position.y + range < node.minY)
                {
                    return;
                }

                var rangeSq = RVOMath.Square(range);

                // Check if the node is a leaf node
                if (node.end - node.begin <= MaxLeafSize)
                {
                    // Iterate over the agentIds in the leaf node
                    for (var i = node.begin; i < node.end; ++i)
                    {
                        var agentIndex = this.agentIds[i];
                        float2 agentPosition = agents[agentIndex].position;

                        // Check if the agent is within the specified range
                        if (math.distancesq(position, agentPosition) <= rangeSq)
                        {
                            result.Add(agents[agentIndex]);
                        }
                    }
                }
                else
                {
                    // Child nodes
                    this.QueryAgentTreeRecursive(node.left, position, range, in agents, ref result);
                    this.QueryAgentTreeRecursive(node.right, position, range, in agents, ref result);
                }
            }
        }

        /// <summary>
        /// Defines a node of an obstacle k-D tree.
        /// </summary>
        internal struct ObstacleTreeNode
        {
            internal bool valid;
            internal int obstacleIndex;
            internal int leftIndex;
            internal int rightIndex;
        }
    }
}
