// -----------------------------------------------------------------------
// <copyright file="Agent.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Agent.cs
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
    using Unity.Collections.LowLevel.Unsafe;
    using Unity.Mathematics;

    /// <summary>
    /// Defines an agent in the simulation.
    /// </summary>
    internal struct Agent
    {
        internal readonly int id;

        internal float2 position;
        internal float2 prefVelocity;
        internal float2 velocity;
        internal int maxNeighbors;
        internal float maxSpeed;
        internal float neighborDist;
        internal float radius;
        internal float timeHorizon;
        internal float timeHorizonObst;

        internal float2 newVelocity;

        internal Agent(int id)
            : this()
        {
            this.id = id;
        }

        /// <summary>
        /// Computes the neighbors of this agent.
        /// </summary>
        internal unsafe void ComputeNeighbors(
            in int index,
            in KdTree.ReadOnly kdTree,
            Agent* agents,
            int agentsLength,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> agentNeighbors,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            var rangeSq = RVOMath.Square((this.timeHorizonObst * this.maxSpeed) + this.radius);
            fixed (Agent* thisPtr = &this)
            {
                kdTree.ComputeObstacleNeighbors(
                    thisPtr,
                    rangeSq,
                    obstacles,
                    obstaclesLength,
                    ref obstacleNeighbors);
            }

            if (this.maxNeighbors > 0)
            {
                rangeSq = RVOMath.Square(this.neighborDist);
                fixed (Agent* thisPtr = &this)
                {
                    kdTree.ComputeAgentNeighbors(
                        thisPtr,
                        ref rangeSq,
                        agents,
                        agentsLength,
                        ref agentNeighbors);
                }
            }
        }

        /// <summary>
        /// Computes the new velocity of this agent.
        /// </summary>
        internal unsafe void ComputeNewVelocity(
            float timeStep,
            Agent* agents,
            int agentsLength,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> agentNeighbors,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            var orcaLines = new UnsafeList<Line>(8, Allocator.Temp);

            var invTimeHorizonObst = 1f / this.timeHorizonObst;

            // Create obstacle ORCA lines.
            for (var i = 0; i < obstacleNeighbors.Length; ++i)
            {
                var obstacle1Index = obstacleNeighbors[i].index;
                Obstacle* obstacle1 = obstacles + obstacle1Index;
                var obstacle2Index = obstacle1->nextIndex;
                Obstacle* obstacle2 = obstacles + obstacle2Index;

                float2 relativePosition1 = obstacle1->point - this.position;
                float2 relativePosition2 = obstacle2->point - this.position;

                // Check if velocity obstacle of obstacle is already taken care
                // of by previously constructed obstacle ORCA lines.
                var alreadyCovered = false;

                var relativeMove1 = invTimeHorizonObst * relativePosition1;
                var relativeMove2 = invTimeHorizonObst * relativePosition2;
                var dist = invTimeHorizonObst * this.radius;

                for (var j = 0; j < orcaLines.Length; ++j)
                {
                    Line lineJ = orcaLines[j];
                    if (RVOMath.Det(relativeMove1 - lineJ.point, lineJ.direction) - dist >= -RVOMath.RVO_EPSILON &&
                        RVOMath.Det(relativeMove2 - lineJ.point, lineJ.direction) - dist >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                // Not yet covered. Check for collisions.
                var distSq1 = math.lengthsq(relativePosition1);
                var distSq2 = math.lengthsq(relativePosition2);

                var radiusSq = RVOMath.Square(this.radius);

                float2 obstacleVector = obstacle2->point - obstacle1->point;
                var s = math.dot(-relativePosition1, obstacleVector) / math.lengthsq(obstacleVector);
                var distSqLine = math.lengthsq(-relativePosition1 - (s * obstacleVector));

                if (s < 0f && distSq1 <= radiusSq)
                {
                    // Collision with left vertex. Ignore if non-convex.
                    if (obstacle1->convex)
                    {
                        Line line;
                        line.point = new float2(0f, 0f);
                        line.direction = math.normalize(new float2(-relativePosition1.y, relativePosition1.x));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s > 1f && distSq2 <= radiusSq)
                {
                    // Collision with right vertex. Ignore if non-convex or if
                    // it will be taken care of by neighboring obstacle.
                    if (obstacle2->convex && RVOMath.Det(relativePosition2, obstacle2->direction) >= 0f)
                    {
                        Line line;
                        line.point = new float2(0f, 0f);
                        line.direction = math.normalize(new float2(-relativePosition2.y, relativePosition2.x));
                        orcaLines.Add(line);
                    }

                    continue;
                }
                else if (s >= 0f && s <= 1f && distSqLine <= radiusSq)
                {
                    // Collision with obstacle segment.
                    Line line;
                    line.point = new float2(0f, 0f);
                    line.direction = -obstacle1->direction;
                    orcaLines.Add(line);

                    continue;
                }

                // No collision. Compute legs. When obliquely viewed, both legs
                // can come from a single vertex. Legs extend cut-off line when
                // non-convex vertex.
                float2 leftLegDirection, rightLegDirection;

                if (s < 0f && distSqLine <= radiusSq)
                {
                    // Obstacle viewed obliquely so that left vertex
                    // defines velocity obstacle.
                    if (!obstacle1->convex)
                    {
                        // Ignore obstacle.
                        continue;
                    }

                    obstacle2 = obstacle1;

                    var leg1 = math.sqrt(distSq1 - radiusSq);

                    leftLegDirection = new float2(
                        (relativePosition1.x * leg1) - (relativePosition1.y * this.radius),
                        (relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                        / distSq1;
                    rightLegDirection = new float2(
                        (relativePosition1.x * leg1) + (relativePosition1.y * this.radius),
                        (-relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                        / distSq1;
                }
                else if (s > 1f && distSqLine <= radiusSq)
                {
                    // Obstacle viewed obliquely so that
                    // right vertex defines velocity obstacle.
                    if (!obstacle2->convex)
                    {
                        // Ignore obstacle.
                        continue;
                    }

                    obstacle1 = obstacle2;

                    var leg2 = math.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new float2(
                        (relativePosition2.x * leg2) - (relativePosition2.y * this.radius),
                        (relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                        / distSq2;
                    rightLegDirection = new float2(
                        (relativePosition2.x * leg2) + (relativePosition2.y * this.radius),
                        (-relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                        / distSq2;
                }
                else
                {
                    // Usual situation.
                    if (obstacle1->convex)
                    {
                        var leg1 = math.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new float2(
                            (relativePosition1.x * leg1) - (relativePosition1.y * this.radius),
                            (relativePosition1.x * this.radius) + (relativePosition1.y * leg1))
                            / distSq1;
                    }
                    else
                    {
                        // Left vertex non-convex; left leg extends cut-off line.
                        leftLegDirection = -obstacle1->direction;
                    }

                    if (obstacle2->convex)
                    {
                        var leg2 = math.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new float2(
                            (relativePosition2.x * leg2) + (relativePosition2.y * this.radius),
                            (-relativePosition2.x * this.radius) + (relativePosition2.y * leg2))
                            / distSq2;
                    }
                    else
                    {
                        // Right vertex non-convex; right leg extends cut-off line.
                        rightLegDirection = obstacle1->direction;
                    }
                }

                // Legs can never point into neighboring edge when convex
                // vertex, take cutoff-line of neighboring edge instead. If
                // velocity projected on "foreign" leg, no constraint is added.
                var leftNeighborIndex = obstacle1->previousIndex;
                Obstacle* leftNeighbor = obstacles + leftNeighborIndex;

                var isLeftLegForeign = false;
                var isRightLegForeign = false;

                if (obstacle1->convex && RVOMath.Det(leftLegDirection, -leftNeighbor->direction) >= 0f)
                {
                    // Left leg points into obstacle.
                    leftLegDirection = -leftNeighbor->direction;
                    isLeftLegForeign = true;
                }

                if (obstacle2->convex && RVOMath.Det(rightLegDirection, obstacle2->direction) <= 0f)
                {
                    // Right leg points into obstacle.
                    rightLegDirection = obstacle2->direction;
                    isRightLegForeign = true;
                }

                // Compute cut-off centers.
                float2 leftCutOff = invTimeHorizonObst * (obstacle1->point - this.position);
                float2 rightCutOff = invTimeHorizonObst * (obstacle2->point - this.position);
                float2 cutOffVector = rightCutOff - leftCutOff;

                // Project current velocity on velocity obstacle.

                // Check if current velocity is projected on cutoff circles.
                var same = obstacle1->id == obstacle2->id;
                var t = same ? 0.5f : math.dot(this.velocity - leftCutOff, cutOffVector)
                    / math.lengthsq(cutOffVector);
                var tLeft = math.dot(this.velocity - leftCutOff, leftLegDirection);
                var tRight = math.dot(this.velocity - rightCutOff, rightLegDirection);

                if ((t < 0f && tLeft < 0f) || (same && tLeft < 0f && tRight < 0f))
                {
                    // Project on left cut-off circle.
                    float2 unitW = math.normalize(this.velocity - leftCutOff);

                    Line line;
                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = leftCutOff + (dist * unitW);
                    orcaLines.Add(line);

                    continue;
                }
                else if (t > 1f && tRight < 0f)
                {
                    // Project on right cut-off circle.
                    float2 unitW = math.normalize(this.velocity - rightCutOff);

                    Line line;
                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = rightCutOff + (dist * unitW);
                    orcaLines.Add(line);

                    continue;
                }

                // Project on left leg, right leg, or cut-off line, whichever is
                // closest to velocity.
                var distSqCutoff = (t < 0f || t > 1f || obstacle1->id == obstacle2->id)
                    ? float.PositiveInfinity
                    : math.lengthsq(this.velocity - (leftCutOff + (t * cutOffVector)));
                var distSqLeft = tLeft < 0f ? float.PositiveInfinity
                    : math.lengthsq(this.velocity - (leftCutOff + (tLeft * leftLegDirection)));
                var distSqRight = tRight < 0f ? float.PositiveInfinity
                    : math.lengthsq(this.velocity - (rightCutOff + (tRight * rightLegDirection)));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    // Project on cut-off line.
                    Line line;
                    line.direction = -obstacle1->direction;
                    line.point = leftCutOff + (dist * new float2(-line.direction.y, line.direction.x));
                    orcaLines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    // Project on left leg.
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    Line line;
                    line.direction = leftLegDirection;
                    line.point = leftCutOff + (dist * new float2(-line.direction.y, line.direction.x));
                    orcaLines.Add(line);

                    continue;
                }

                // Project on right leg.
                if (isRightLegForeign)
                {
                    continue;
                }

                Line line0;
                line0.direction = -rightLegDirection;
                line0.point = rightCutOff + (dist * new float2(-line0.direction.y, line0.direction.x));
                orcaLines.Add(line0);
            }

            var numObstLines = orcaLines.Length;

            var invTimeHorizon = 1f / this.timeHorizon;

            // Create agent ORCA lines.
            for (var i = 0; i < agentNeighbors.Length; ++i)
            {
                var otherIndex = agentNeighbors[i].index;
                Agent* other = agents + otherIndex;

                float2 relativePosition = other->position - this.position;
                float2 relativeVelocity = this.velocity - other->velocity;
                var distSq = math.lengthsq(relativePosition);
                var combinedRadius = this.radius + other->radius;
                var combinedRadiusSq = RVOMath.Square(combinedRadius);

                Line line;
                float2 u;

                if (distSq > combinedRadiusSq)
                {
                    // No collision.
                    float2 w = relativeVelocity - (invTimeHorizon * relativePosition);

                    // Vector from cutoff center to relative velocity.
                    var wLengthSq = math.lengthsq(w);
                    var dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0f && RVOMath.Square(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        // Project on cut-off circle.
                        var wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.direction = new float2(unitW.y, -unitW.x);
                        u = ((combinedRadius * invTimeHorizon) - wLength) * unitW;
                    }
                    else
                    {
                        // Project on legs.
                        var leg = math.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.Det(relativePosition, w) > 0f)
                        {
                            // Project on left leg.
                            line.direction = new float2(
                                (relativePosition.x * leg) - (relativePosition.y * combinedRadius),
                                (relativePosition.x * combinedRadius) + (relativePosition.y * leg))
                                / distSq;
                        }
                        else
                        {
                            // Project on right leg.
                            line.direction = -new float2(
                                (relativePosition.x * leg) + (relativePosition.y * combinedRadius),
                                (-relativePosition.x * combinedRadius) + (relativePosition.y * leg))
                                / distSq;
                        }

                        var dotProduct2 = math.dot(relativeVelocity, line.direction);
                        u = (dotProduct2 * line.direction) - relativeVelocity;
                    }
                }
                else
                {
                    // Collision. Project on cut-off circle of time timeStep.
                    var invTimeStep = 1f / timeStep;

                    // Vector from cutoff center to relative velocity.
                    float2 w = relativeVelocity - (invTimeStep * relativePosition);

                    var wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.direction = new float2(unitW.y, -unitW.x);
                    u = ((combinedRadius * invTimeStep) - wLength) * unitW;
                }

                line.point = this.velocity + (0.5f * u);
                orcaLines.Add(line);
            }

            var lineFail = this.LinearProgram2(
                orcaLines.Ptr,
                orcaLines.Length,
                this.maxSpeed,
                this.prefVelocity,
                false,
                ref this.newVelocity);

            if (lineFail < orcaLines.Length)
            {
                this.LinearProgram3(
                    orcaLines.Ptr,
                    orcaLines.Length,
                    numObstLines,
                    lineFail,
                    this.maxSpeed,
                    ref this.newVelocity);
            }

            orcaLines.Dispose();
        }

        /// <summary>
        /// Inserts an agent neighbor into the set of neighbors of this agent.
        /// </summary>
        /// <param name="agentIndex">A pointer to the agent to be inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        /// <param name="agents">The array that holds the agent data.</param>
        /// <param name="agentsLength">The length for array <paramref name="agents"/>.</param>
        /// <param name="agentNeighbors">The list to store the neighbor data.</param>
        internal unsafe void InsertAgentNeighbor(
            int agentIndex,
            ref float rangeSq,
            Agent* agents,
            int agentsLength,
            ref UnsafeList<Pair> agentNeighbors)
        {
            Agent* agent = agents + agentIndex;
            if (this.id == agent->id)
            {
                return;
            }

            var distSq = math.lengthsq(this.position - agent->position);

            if (distSq < rangeSq)
            {
                if (agentNeighbors.Length < this.maxNeighbors)
                {
                    agentNeighbors.Add(new Pair(distSq, agentIndex));
                }

                var i = agentNeighbors.Length - 1;

                while (i != 0 && distSq < agentNeighbors[i - 1].dist)
                {
                    agentNeighbors[i] = agentNeighbors[i - 1];
                    --i;
                }

                agentNeighbors[i] = new Pair(distSq, agentIndex);

                if (agentNeighbors.Length == this.maxNeighbors)
                {
                    rangeSq = agentNeighbors[agentNeighbors.Length - 1].dist;
                }
            }
        }

        /// <summary>
        /// Inserts a static obstacle neighbor into the set of neighbors of this agent.
        /// </summary>
        /// <param name="obstacleIndex">The number of the static obstacle to be inserted.</param>
        /// <param name="rangeSq">The squared range around this agent.</param>
        /// <param name="obstacles">The array that holds the obstacle verts.</param>
        /// <param name="obstaclesLength">The length for array <paramref name="obstacles"/>.</param>
        /// <param name="obstacleNeighbors">The list to store the neighbor dara.</param>
        internal unsafe void InsertObstacleNeighbor(
            int obstacleIndex,
            float rangeSq,
            Obstacle* obstacles,
            int obstaclesLength,
            ref UnsafeList<Pair> obstacleNeighbors)
        {
            Obstacle* obstacle = obstacles + obstacleIndex;
            var nextObstacleIndex = obstacle->nextIndex;
            Obstacle* nextObstacle = obstacles + nextObstacleIndex;

            var distSq = RVOMath.DistSqPointLineSegment(obstacle->point, nextObstacle->point, this.position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new Pair(distSq, obstacleIndex));

                var i = obstacleNeighbors.Length - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].dist)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }

                obstacleNeighbors[i] = new Pair(distSq, obstacleIndex);
            }
        }

        /// <summary>
        /// Updates the two-dimensional position and two-dimensional velocity of this agent.
        /// </summary>
        internal void Update(float timeStep)
        {
            this.velocity = this.newVelocity;
            this.position += this.velocity * timeStep;
        }

        /// <summary>
        /// Solves a one-dimensional linear program on a specified line subject to
        /// linear constraints defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="lineNo">The specified line constraint.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        /// <returns>True if successful.</returns>
        private unsafe bool LinearProgram1(
            Line* lines,
            int linesLength,
            int lineNo,
            float radius,
            float2 optVelocity,
            bool directionOpt,
            ref float2 result)
        {
            Line* lineNoPtr = lines + lineNo;
            var dotProduct = math.dot(lineNoPtr->point, lineNoPtr->direction);
            var discriminant = RVOMath.Square(dotProduct) + RVOMath.Square(radius) - math.lengthsq(lineNoPtr->point);

            if (discriminant < 0f)
            {
                // Max speed circle fully invalidates line lineNo.
                return false;
            }

            var sqrtDiscriminant = math.sqrt(discriminant);
            var tLeft = -dotProduct - sqrtDiscriminant;
            var tRight = -dotProduct + sqrtDiscriminant;

            for (var i = 0; i < lineNo; ++i)
            {
                Line* lineI = lines + i;
                var denominator = RVOMath.Det(lineNoPtr->direction, lineI->direction);
                var numerator = RVOMath.Det(lineI->direction, lineNoPtr->point - lineI->point);

                if (math.abs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    // Lines lineNo and i are (almost) parallel.
                    if (numerator < 0f)
                    {
                        return false;
                    }

                    continue;
                }

                var t = numerator / denominator;

                if (denominator >= 0f)
                {
                    // Line i bounds line lineNo on the right.
                    tRight = math.min(tRight, t);
                }
                else
                {
                    // Line i bounds line lineNo on the left.
                    tLeft = math.max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                // Optimize direction.
                if (math.dot(optVelocity, lineNoPtr->direction) > 0f)
                {
                    // Take right extreme.
                    result = lineNoPtr->point + (tRight * lineNoPtr->direction);
                }
                else
                {
                    // Take left extreme.
                    result = lineNoPtr->point + (tLeft * lineNoPtr->direction);
                }
            }
            else
            {
                // Optimize closest point.
                var t = math.dot(lineNoPtr->direction, optVelocity - lineNoPtr->point);

                if (t < tLeft)
                {
                    result = lineNoPtr->point + (tLeft * lineNoPtr->direction);
                }
                else if (t > tRight)
                {
                    result = lineNoPtr->point + (tRight * lineNoPtr->direction);
                }
                else
                {
                    result = lineNoPtr->point + (t * lineNoPtr->direction);
                }
            }

            return true;
        }

        /// <summary>
        /// Solves a two-dimensional linear program subject to linear constraints
        /// defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="optVelocity">The optimization velocity.</param>
        /// <param name="directionOpt">True if the direction should be optimized.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        /// <returns>The number of the line it fails on, and the number of lines if successful.</returns>
        private unsafe int LinearProgram2(
            Line* lines,
            int linesLength,
            float radius,
            float2 optVelocity,
            bool directionOpt,
            ref float2 result)
        {
            if (directionOpt)
            {
                // Optimize direction. Note that the optimization velocity is of unit length in this case.
                result = optVelocity * radius;
            }
            else if (math.lengthsq(optVelocity) > RVOMath.Square(radius))
            {
                // Optimize closest point and outside circle.
                result = math.normalize(optVelocity) * radius;
            }
            else
            {
                // Optimize closest point and inside circle.
                result = optVelocity;
            }

            for (var i = 0; i < linesLength; ++i)
            {
                Line* lineI = lines + i;
                if (RVOMath.Det(lineI->direction, lineI->point - result) > 0f)
                {
                    // Result does not satisfy constraint i. Compute new optimal result.
                    float2 tempResult = result;
                    if (!this.LinearProgram1(
                        lines,
                        linesLength,
                        i,
                        radius,
                        optVelocity,
                        directionOpt,
                        ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return linesLength;
        }

        /// <summary>
        /// Solves a two-dimensional linear program subject to linear constraints
        /// defined by lines and a circular constraint.
        /// </summary>
        /// <param name="lines">Lines defining the linear constraints.</param>
        /// <param name="linesLength">The length for array <paramref name="lines"/>.</param>
        /// <param name="numObstLines">Count of obstacle lines.</param>
        /// <param name="beginLine">The line on which the 2-d linear program failed.</param>
        /// <param name="radius">The radius of the circular constraint.</param>
        /// <param name="result">A reference to the result of the linear program.</param>
        private unsafe void LinearProgram3(
            Line* lines,
            int linesLength,
            int numObstLines,
            int beginLine,
            float radius,
            ref float2 result)
        {
            var distance = 0f;

            for (var i = beginLine; i < linesLength; ++i)
            {
                Line* lineI = lines + i;
                if (RVOMath.Det(lineI->direction, lineI->point - result) <= distance)
                {
                    continue;
                }

                // Result does not satisfy constraint of line i.
                var projLines = new UnsafeList<Line>(numObstLines, Allocator.Temp);
                for (var ii = 0; ii < numObstLines; ++ii)
                {
                    projLines.Add(lines[ii]);
                }

                for (var j = numObstLines; j < i; ++j)
                {
                    Line* lineJ = lines + j;
                    Line line;

                    var determinant = RVOMath.Det(lineI->direction, lineJ->direction);

                    if (math.abs(determinant) <= RVOMath.RVO_EPSILON)
                    {
                        // Line i and line j are parallel.
                        if (math.dot(lineI->direction, lineJ->direction) > 0f)
                        {
                            // Line i and line j point in the same direction.
                            continue;
                        }
                        else
                        {
                            // Line i and line j point in opposite direction.
                            line.point = 0.5f * (lineI->point + lineJ->point);
                        }
                    }
                    else
                    {
                        line.point = lineI->point
                            + (RVOMath.Det(lineJ->direction, lineI->point - lineJ->point) / determinant * lineI->direction);
                    }

                    line.direction = math.normalize(lineJ->direction - lineI->direction);
                    projLines.Add(line);
                }

                float2 tempResult = result;
                if (this.LinearProgram2(
                    projLines.Ptr,
                    projLines.Length,
                    radius,
                    new float2(-lineI->direction.y, lineI->direction.x),
                    true,
                    ref result)
                    < projLines.Length)
                {
                    // This should in principle not happen. The result is by
                    // definition already in the feasible region of this
                    // linear program. If it fails, it is due to small
                    // floating point error, and the current result is kept.
                    result = tempResult;
                }

                distance = RVOMath.Det(lineI->direction, lineI->point - result);
            }
        }

        internal readonly struct Pair : IEquatable<Pair>
        {
            public readonly float dist;
            public readonly int index;

            public Pair(float dist, int index)
            {
                this.dist = dist;
                this.index = index;
            }

            public bool Equals(Pair other)
            {
                return this.dist.Equals(other.dist) && this.index == other.index;
            }

            public override bool Equals(object obj)
            {
                return obj is Pair other && this.Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    return (this.dist.GetHashCode() * 397) ^ this.index;
                }
            }
        }
    }
}
