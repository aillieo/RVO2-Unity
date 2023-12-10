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

using Unity.Collections;

namespace RVO
{
    using System;
    using Unity.Mathematics;

    /**
     * <summary>Defines k-D trees for agents and static obstacles in the
     * simulation.</summary>
     */
    internal struct KdTree : IDisposable
    {
        internal KdTree(int agentCount, int obstacleCount)
            : this()
        {
            this.agents_ = new NativeArray<int>(agentCount, Allocator.Persistent);
            this.agentTree_ = new NativeArray<AgentTreeNode>(agentCount * 2, Allocator.Persistent);
            this.obstacleTreeNodes_ = new NativeArray<ObstacleTreeNode>(obstacleCount, Allocator.Persistent);
        }

        internal struct ReadOnly
        {
            internal NativeArray<int>.ReadOnly agents_;
            internal NativeArray<AgentTreeNode>.ReadOnly agentTree_;
            internal NativeArray<ObstacleTreeNode>.ReadOnly obstacleTreeNodes_;

            /**
         * <summary>Computes the agent neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
            internal void computeAgentNeighbors(int agentIndex, ref float rangeSq, in NativeArray<Agent> agents, ref NativeList<Agent.Pair> agentNeighbors_)
            {
                this.queryAgentTreeRecursive(agentIndex, ref rangeSq, 0, in agents, ref agentNeighbors_);
            }

            /**
             * <summary>Computes the obstacle neighbors of the specified agent.
             * </summary>
             *
             * <param name="agent">The agent for which obstacle neighbors are to be
             * computed.</param>
             * <param name="rangeSq">The squared range around the agent.</param>
             */
            internal void computeObstacleNeighbors(Agent agent, float rangeSq, in NativeArray<Obstacle> obstacles, ref NativeList<Agent.Pair> obstacleNeighbors_)
            {
                this.queryObstacleTreeRecursive(agent, rangeSq, 0, in obstacles, ref obstacleNeighbors_);
            }

            /**
             * <summary>Queries the visibility between two points within a specified
             * radius.</summary>
             *
             * <returns>True if q1 and q2 are mutually visible within the radius;
             * false otherwise.</returns>
             *
             * <param name="q1">The first point between which visibility is to be
             * tested.</param>
             * <param name="q2">The second point between which visibility is to be
             * tested.</param>
             * <param name="radius">The radius within which visibility is to be
             * tested.</param>
             */
            internal bool queryVisibility(float2 q1, float2 q2, float radius, NativeList<Obstacle> obstacles)
            {
                return this.queryVisibilityRecursive(q1, q2, radius, 0, obstacles);
            }

            /**
             * <summary>Recursive method for computing the agent neighbors of the
             * specified agent.</summary>
             *
             * <param name="agent">The agent for which agent neighbors are to be
             * computed.</param>
             * <param name="rangeSq">The squared range around the agent.</param>
             * <param name="node">The current agent k-D tree node index.</param>
             */
            private void queryAgentTreeRecursive(int agentIndex, ref float rangeSq, int node, in NativeArray<Agent> agents, ref NativeList<Agent.Pair> agentNeighbors_)
            {
                Agent agent = agents[agentIndex];
                if (this.agentTree_[node].end_ - this.agentTree_[node].begin_ <= MAX_LEAF_SIZE)
                {
                    for (int i = this.agentTree_[node].begin_; i < this.agentTree_[node].end_; ++i)
                    {
                        agent.insertAgentNeighbor(this.agents_[i], ref rangeSq, in agents, ref agentNeighbors_);
                    }
                }
                else
                {
                    float distSqLeft = RVOMath.sqr(math.max(0.0f, this.agentTree_[this.agentTree_[node].left_].minX_ - agent.position_.x))
                        + RVOMath.sqr(math.max(0.0f, agent.position_.x - this.agentTree_[this.agentTree_[node].left_].maxX_))
                        + RVOMath.sqr(math.max(0.0f, this.agentTree_[this.agentTree_[node].left_].minY_ - agent.position_.y))
                        + RVOMath.sqr(math.max(0.0f, agent.position_.y - this.agentTree_[this.agentTree_[node].left_].maxY_));
                    float distSqRight = RVOMath.sqr(math.max(0.0f, this.agentTree_[this.agentTree_[node].right_].minX_ - agent.position_.x))
                        + RVOMath.sqr(math.max(0.0f, agent.position_.x - this.agentTree_[this.agentTree_[node].right_].maxX_))
                        + RVOMath.sqr(math.max(0.0f, this.agentTree_[this.agentTree_[node].right_].minY_ - agent.position_.y))
                        + RVOMath.sqr(math.max(0.0f, agent.position_.y - this.agentTree_[this.agentTree_[node].right_].maxY_));

                    if (distSqLeft < distSqRight)
                    {
                        if (distSqLeft < rangeSq)
                        {
                            this.queryAgentTreeRecursive(agentIndex, ref rangeSq, this.agentTree_[node].left_, in agents, ref agentNeighbors_);

                            if (distSqRight < rangeSq)
                            {
                                this.queryAgentTreeRecursive(agentIndex, ref rangeSq, this.agentTree_[node].right_, in agents, ref agentNeighbors_);
                            }
                        }
                    }
                    else
                    {
                        if (distSqRight < rangeSq)
                        {
                            this.queryAgentTreeRecursive(agentIndex, ref rangeSq, this.agentTree_[node].right_, in agents, ref agentNeighbors_);

                            if (distSqLeft < rangeSq)
                            {
                                this.queryAgentTreeRecursive(agentIndex, ref rangeSq, this.agentTree_[node].left_, in agents, ref agentNeighbors_);
                            }
                        }
                    }
                }
            }

            /**
             * <summary>Recursive method for computing the obstacle neighbors of the
             * specified agent.</summary>
             *
             * <param name="agent">The agent for which obstacle neighbors are to be
             * computed.</param>
             * <param name="rangeSq">The squared range around the agent.</param>
             * <param name="nodeIndex">The current obstacle k-D node.</param>
             */
            private void queryObstacleTreeRecursive(Agent agent, float rangeSq, int nodeIndex, in NativeArray<Obstacle> obstacles, ref NativeList<Agent.Pair> obstacleNeighbors_)
            {
                ObstacleTreeNode node = default;
                if (nodeIndex >= 0 && nodeIndex < this.obstacleTreeNodes_.Length)
                {
                    node = this.obstacleTreeNodes_[nodeIndex];
                }

                if (!node.valid)
                {
                    return;
                }

                int obstacle1Index = node.obstacleIndex_;
                Obstacle obstacle1 = obstacles[obstacle1Index];
                int obstacle2Index = obstacle1.nextIndex_;
                Obstacle obstacle2 = obstacles[obstacle2Index];

                float agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

                this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.leftIndex_ : node.rightIndex_, in obstacles, ref obstacleNeighbors_);

                float distSqLine = RVOMath.sqr(agentLeftOfLine) / math.lengthsq(obstacle2.point_ - obstacle1.point_);

                if (distSqLine < rangeSq)
                {
                    if (agentLeftOfLine < 0.0f)
                    {
                        /*
                         * Try obstacle at this node only if agent is on right side of
                         * obstacle (and can see obstacle).
                         */
                        agent.insertObstacleNeighbor(node.obstacleIndex_, rangeSq, in obstacles, ref obstacleNeighbors_);
                    }

                    /* Try other side of line. */
                    this.queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.rightIndex_ : node.leftIndex_, in obstacles, ref obstacleNeighbors_);
                }
            }

            /**
             * <summary>Recursive method for querying the visibility between two
             * points within a specified radius.</summary>
             *
             * <returns>True if q1 and q2 are mutually visible within the radius;
             * false otherwise.</returns>
             *
             * <param name="q1">The first point between which visibility is to be
             * tested.</param>
             * <param name="q2">The second point between which visibility is to be
             * tested.</param>
             * <param name="radius">The radius within which visibility is to be
             * tested.</param>
             * <param name="node">The current obstacle k-D node.</param>
             */
            private bool queryVisibilityRecursive(float2 q1, float2 q2, float radius, int nodeIndex, NativeList<Obstacle> obstacles)
            {
                ObstacleTreeNode node = default;
                if (nodeIndex >= 0 && nodeIndex < this.obstacleTreeNodes_.Length)
                {
                    node = this.obstacleTreeNodes_[nodeIndex];
                }

                if (!node.valid)
                {
                    return true;
                }

                int obstacle1Index = node.obstacleIndex_;
                Obstacle obstacle1 = obstacles[obstacle1Index];
                int obstacle2Index = obstacle1.nextIndex_;
                Obstacle obstacle2 = obstacles[obstacle2Index];

                float q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
                float q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
                float invLengthI = 1.0f / math.lengthsq(obstacle2.point_ - obstacle1.point_);

                if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
                {
                    return this.queryVisibilityRecursive(q1, q2, radius, node.leftIndex_, obstacles) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.rightIndex_, obstacles));
                }

                if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
                {
                    return this.queryVisibilityRecursive(q1, q2, radius, node.rightIndex_, obstacles) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || this.queryVisibilityRecursive(q1, q2, radius, node.leftIndex_, obstacles));
                }

                if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
                {
                    /* One can see through obstacle from left to right. */
                    return this.queryVisibilityRecursive(q1, q2, radius, node.leftIndex_, obstacles) && this.queryVisibilityRecursive(q1, q2, radius, node.rightIndex_, obstacles);
                }

                float point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
                float point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
                float invLengthQ = 1.0f / math.lengthsq(q2 - q1);

                return point1LeftOfQ * point2LeftOfQ >= 0.0f && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && this.queryVisibilityRecursive(q1, q2, radius, node.leftIndex_, obstacles) && this.queryVisibilityRecursive(q1, q2, radius, node.rightIndex_, obstacles);
            }
        }

        internal ReadOnly AsParallelReader()
        {
            return new ReadOnly()
            {
                agents_ = this.agents_.AsReadOnly(),
                agentTree_ = this.agentTree_.AsReadOnly(),
                obstacleTreeNodes_ = this.obstacleTreeNodes_.AsReadOnly(),
            };
        }

        /**
         * <summary>Defines a node of an agent k-D tree.</summary>
         */
        internal struct AgentTreeNode
        {
            internal int begin_;
            internal int end_;
            internal int left_;
            internal int right_;
            internal float maxX_;
            internal float maxY_;
            internal float minX_;
            internal float minY_;
        }

        internal int NewObstacleTreeNode()
        {
            ObstacleTreeNode node = new ObstacleTreeNode
            {
                valid = true,
                obstacleIndex_ = 0,
                leftIndex_ = -1,
                rightIndex_ = -1,
            };

            int oldLen = this.obstacleTreeNodes_.Length;
            this.obstacleTreeNodes_.Append(node);
            return oldLen;
        }

        /**
         * <summary>Defines a node of an obstacle k-D tree.</summary>
         */
        internal struct ObstacleTreeNode
        {
            internal bool valid;
            internal int obstacleIndex_;
            internal int leftIndex_;
            internal int rightIndex_;
        }

        /**
         * <summary>The maximum size of an agent k-D tree leaf.</summary>
         */
        internal const int MAX_LEAF_SIZE = 10;

        internal NativeArray<int> agents_;

        internal NativeArray<AgentTreeNode> agentTree_;
        internal NativeArray<ObstacleTreeNode> obstacleTreeNodes_;

        

        internal readonly void Clear()
        {
            //if (this.agents_.IsCreated && !this.agents_.IsEmpty)
            //{
            //    this.agents_.Clear();
            //}

            //if (this.agentTree_.IsCreated && !this.agentTree_.IsEmpty)
            //{
            //    this.agentTree_.Clear();
            //}

            //if (this.obstacleTreeNodes_.IsCreated && !this.obstacleTreeNodes_.IsEmpty)
            //{
            //    this.obstacleTreeNodes_.Clear();
            //}
        }

        public void Dispose()
        {
            this.Clear();
            //if (this.agents_.IsCreated)
            {
                this.agents_.SafeDispose();
                this.agents_ = default;
            }

            //if (this.agentTree_.IsCreated)
            {
                this.agentTree_.SafeDispose();
                this.agentTree_ = default;
            }

            //if (this.obstacleTreeNodes_.IsCreated)
            {
                this.obstacleTreeNodes_.SafeDispose();
                this.obstacleTreeNodes_ = default;
            }
        }
    }
}
