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
#if RVO2_DETERMINISTIC
    using RvoReal = RVO.FixedPoint;
    using RvoVector2 = RVO.FixedPoint2;
#else
    using RvoReal = System.Single;
    using RvoVector2 = Unity.Mathematics.float2;
#endif

    /// <summary>
    /// Contains functions and constants used in multiple classes.
    /// </summary>
    internal static class RVOMath
    {
        /// <summary>
        /// A sufficiently small positive number.
        /// </summary>
        internal static readonly RvoReal RVO_EPSILON =
#if RVO2_DETERMINISTIC
            FixedPoint.FromRaw(42950L);
#else
            0.00001f;
#endif

        internal static readonly RvoReal Zero =
#if RVO2_DETERMINISTIC
            FixedPoint.Zero;
#else
            0f;
#endif

        internal static readonly RvoReal One =
#if RVO2_DETERMINISTIC
            FixedPoint.One;
#else
            1f;
#endif

        internal static readonly RvoReal Half =
#if RVO2_DETERMINISTIC
            FixedPoint.Half;
#else
            0.5f;
#endif

        internal static readonly RvoReal MaxValue =
#if RVO2_DETERMINISTIC
            FixedPoint.MaxValue;
#else
            float.PositiveInfinity;
#endif

        internal static readonly RvoVector2 ZeroVector =
#if RVO2_DETERMINISTIC
            FixedPoint2.Zero;
#else
            float2.zero;
#endif

        /// <summary>
        /// Computes the determinant of a two-dimensional square matrix
        /// with rows consisting of the specified two-dimensional vectors.
        /// </summary>
        /// <param name="vector1">The top row of the two-dimensional square matrix.</param>
        /// <param name="point2">The bottom row of the two-dimensional square matrix.</param>
        /// <returns>The determinant of the two-dimensional square matrix.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Det(RvoVector2 vector1, RvoVector2 point2)
        {
#if RVO2_DETERMINISTIC
            return FixedPoint2.Cross(vector1, point2);
#else
            return (vector1.x * point2.y) - (vector1.y * point2.x);
#endif
        }

        /// <summary>
        /// Computes the squared distance from a line segment with the
        /// specified endpoints to a specified point.
        /// </summary>
        /// <param name="vector1">The first endpoint of the line segment.</param>
        /// <param name="point2">The second endpoint of the line segment.</param>
        /// <param name="vector3">The point to which the squared distance is to be calculated.</param>
        /// <returns>The squared distance from the line segment to the point.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal DistSqPointLineSegment(
            RvoVector2 vector1,
            RvoVector2 point2,
            RvoVector2 vector3)
        {
            var segment = point2 - vector1;
            var r = Dot(vector3 - vector1, segment) / LengthSq(segment);

            if (r < Zero)
            {
                return LengthSq(vector3 - vector1);
            }

            if (r > One)
            {
                return LengthSq(vector3 - point2);
            }

            return LengthSq(vector3 - (vector1 + (r * segment)));
        }

        /// <summary>
        /// Computes the signed distance from a line connecting
        /// the specified points to a specified point.
        /// </summary>
        /// <param name="a">The first point on the line.</param>
        /// <param name="b">The second point on the line.</param>
        /// <param name="c">The point to which the signed distance is to be calculated.</param>
        /// <returns>Positive when the point c lies to the left of the line ab.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal LeftOf(RvoVector2 a, RvoVector2 b, RvoVector2 c)
        {
            return Det(a - c, b - a);
        }

        /// <summary>
        /// Computes the square of a fixed-point value.
        /// </summary>
        /// <param name="scalar">The fixed-point value to be squared.</param>
        /// <returns>The square of the fixed-point value.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Square(RvoReal scalar)
        {
            return scalar * scalar;
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Dot(RvoVector2 vector1, RvoVector2 point2)
        {
#if RVO2_DETERMINISTIC
            return FixedPoint2.Dot(vector1, point2);
#else
            return math.dot(vector1, point2);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal LengthSq(RvoVector2 vector)
        {
#if RVO2_DETERMINISTIC
            return vector.SqrMagnitude;
#else
            return math.lengthsq(vector);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Length(RvoVector2 vector)
        {
#if RVO2_DETERMINISTIC
            return vector.Magnitude;
#else
            return math.length(vector);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoVector2 Normalize(RvoVector2 vector)
        {
#if RVO2_DETERMINISTIC
            return vector.Normalized;
#else
            return math.normalizesafe(vector);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoVector2 ToReal(float2 value)
        {
#if RVO2_DETERMINISTIC
            return new FixedPoint2((FixedPoint)value.x, (FixedPoint)value.y);
#else
            return value;
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal ToReal(float value)
        {
#if RVO2_DETERMINISTIC
            return (FixedPoint)value;
#else
            return value;
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static float2 ToFloat(RvoVector2 value)
        {
#if RVO2_DETERMINISTIC
            return new float2((float)value.x, (float)value.y);
#else
            return value;
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static float ToFloat(RvoReal value)
        {
#if RVO2_DETERMINISTIC
            return (float)value;
#else
            return value;
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal FromRatio(int numerator, int denominator)
        {
#if RVO2_DETERMINISTIC
            return FixedPoint.FromRatio(numerator, denominator);
#else
            return (float)numerator / denominator;
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Sqrt(RvoReal value)
        {
#if RVO2_DETERMINISTIC
            return FixedPointMath.Sqrt(value);
#else
            return math.sqrt(value);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Abs(RvoReal value)
        {
#if RVO2_DETERMINISTIC
            return FixedPointMath.Abs(value);
#else
            return math.abs(value);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Min(RvoReal a, RvoReal b)
        {
#if RVO2_DETERMINISTIC
            return FixedPointMath.Min(a, b);
#else
            return math.min(a, b);
#endif
        }

        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static RvoReal Max(RvoReal a, RvoReal b)
        {
#if RVO2_DETERMINISTIC
            return FixedPointMath.Max(a, b);
#else
            return math.max(a, b);
#endif
        }

        /// <summary>
        /// Returns true if the first pair of scalar values is less
        /// than the second pair of scalar values.
        /// </summary>
        /// <param name="a">The first pair of scalar values.</param>
        /// <param name="b">The second pair of scalar values.</param>
        /// <returns>True if the first pair of scalar values is less
        /// than the second pair of scalar values.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static bool Less(RvoVector2 a, RvoVector2 b)
        {
            return a.x < b.x || (!(b.x < a.x) && a.y < b.y);
        }

        /// <summary>
        /// Returns true if the first pair of scalar values is less
        /// than or equal to the second pair of scalar values.
        /// </summary>
        /// <param name="a">The first pair of scalar values.</param>
        /// <param name="b">The second pair of scalar values.</param>
        /// <returns>True if the first pair of scalar values is less
        /// than or equal to the second pair of scalar values.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static bool LessEqual(RvoVector2 a, RvoVector2 b)
        {
            return (a.x == b.x && a.y == b.y) || Less(a, b);
        }

        /// <summary>
        /// Returns true if the first pair of scalar values is greater
        /// than the second pair of scalar values.
        /// </summary>
        /// <param name="a">The first pair of scalar values.</param>
        /// <param name="b">The second pair of scalar values.</param>
        /// <returns>True if the first pair of scalar values is greater
        /// than the second pair of scalar values.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static bool Greater(RvoVector2 a, RvoVector2 b)
        {
            return !LessEqual(a, b);
        }

        /// <summary>
        /// Returns true if the first pair of scalar values is greater
        /// than or equal to the second pair of scalar values.
        /// </summary>
        /// <param name="a">The first pair of scalar values.</param>
        /// <param name="b">The second pair of scalar values.</param>
        /// <returns>True if the first pair of scalar values is greater
        /// than or equal to the second pair of scalar values.</returns>
        [BurstCompile(RvoBurstSettings.FloatPrecision, RvoBurstSettings.FloatMode, OptimizeFor = RvoBurstSettings.OptimizeFor)]
        internal static bool GreaterEqual(RvoVector2 a, RvoVector2 b)
        {
            return !Less(a, b);
        }
    }
}
