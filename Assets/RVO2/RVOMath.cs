// -----------------------------------------------------------------------
// <copyright file="RVOMath.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * RVOMath.cs
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
    using Unity.Burst;
    using Unity.Mathematics;

    /**
     * <summary>Contains functions and constants used in multiple classes.
     * </summary>
     */
    internal static class RVOMath
    {
        /**
         * <summary>A sufficiently small positive number.</summary>
         */
        internal const float RVO_EPSILON = 0.00001f;

        /**
         * <summary>Computes the determinant of a two-dimensional square matrix
         * with rows consisting of the specified two-dimensional vectors.
         * </summary>
         *
         * <returns>The determinant of the two-dimensional square matrix.
         * </returns>
         *
         * <param name="vector1">The top row of the two-dimensional square
         * matrix.</param>
         * <param name="vector2">The bottom row of the two-dimensional square
         * matrix.</param>
         */
        [BurstCompile]
        internal static float det(float2 vector1, float2 vector2)
        {
            return (vector1.x * vector2.y) - (vector1.y * vector2.x);
        }

        /**
         * <summary>Computes the squared distance from a line segment with the
         * specified endpoints to a specified point.</summary>
         *
         * <returns>The squared distance from the line segment to the point.
         * </returns>
         *
         * <param name="vector1">The first endpoint of the line segment.</param>
         * <param name="vector2">The second endpoint of the line segment.
         * </param>
         * <param name="vector3">The point to which the squared distance is to
         * be calculated.</param>
         */
        [BurstCompile]
        internal static float distSqPointLineSegment(float2 vector1, float2 vector2, float2 vector3)
        {
            float r = math.dot(vector3 - vector1, vector2 - vector1) / math.lengthsq(vector2 - vector1);

            if (r < 0.0f)
            {
                return math.lengthsq(vector3 - vector1);
            }

            if (r > 1.0f)
            {
                return math.lengthsq(vector3 - vector2);
            }

            return math.lengthsq(vector3 - (vector1 + (r * (vector2 - vector1))));
        }

        /**
         * <summary>Computes the signed distance from a line connecting the
         * specified points to a specified point.</summary>
         *
         * <returns>Positive when the point c lies to the left of the line ab.
         * </returns>
         *
         * <param name="a">The first point on the line.</param>
         * <param name="b">The second point on the line.</param>
         * <param name="c">The point to which the signed distance is to be
         * calculated.</param>
         */
        [BurstCompile]
        internal static float leftOf(float2 a, float2 b, float2 c)
        {
            return det(a - c, b - a);
        }

        /**
         * <summary>Computes the square of a float.</summary>
         *
         * <returns>The square of the float.</returns>
         *
         * <param name="scalar">The float to be squared.</param>
         */
        [BurstCompile]
        internal static float sqr(float scalar)
        {
            return scalar * scalar;
        }

        /**
         * <summary>Returns true if the first pair of scalar values is less
         * than the second pair of scalar values.</summary>
         *
         * <returns>True if the first pair of scalar values is less than the
         * second pair of scalar values.</returns>
         *
         * <param name="a">The first pair of scalar values.</param>
         * <param name="b">The second pair of scalar values.</param>
         */
        [BurstCompile]
        internal static bool less(float2 a, float2 b)
        {
            return a.x < b.x || (!(b.x < a.x) && a.y < b.y);
        }

        /**
         * <summary>Returns true if the first pair of scalar values is less
         * than or equal to the second pair of scalar values.</summary>
         *
         * <returns>True if the first pair of scalar values is less than or
         * equal to the second pair of scalar values.</returns>
         *
         * <param name="a">The first pair of scalar values.</param>
         * <param name="b">The second pair of scalar values.</param>
         */
        [BurstCompile]
        internal static bool lessequal(float2 a, float2 b)
        {
            return (a.x == b.x && a.y == b.y) || less(a, b);
        }

        /**
         * <summary>Returns true if the first pair of scalar values is
         * greater than the second pair of scalar values.</summary>
         *
         * <returns>True if the first pair of scalar values is greater than
         * the second pair of scalar values.</returns>
         *
         * <param name="a">The first pair of scalar values.</param>
         * <param name="b">The second pair of scalar values.</param>
         */
        [BurstCompile]
        internal static bool greater(float2 a, float2 b)
        {
            return !lessequal(a, b);
        }

        /**
         * <summary>Returns true if the first pair of scalar values is
         * greater than or equal to the second pair of scalar values.
         * </summary>
         *
         * <returns>True if the first pair of scalar values is greater than
         * or equal to the second pair of scalar values.</returns>
         *
         * <param name="a">The first pair of scalar values.</param>
         * <param name="b">The second pair of scalar values.</param>
         */
        [BurstCompile]
        internal static bool greaterequal(float2 a, float2 b)
        {
            return !less(a, b);
        }
    }
}
