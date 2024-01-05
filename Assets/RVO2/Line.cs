// -----------------------------------------------------------------------
// <copyright file="Line.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Line.cs
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
    using Unity.Mathematics;

    /// <summary>
    /// Defines a directed line.
    /// </summary>
    internal readonly struct Line : IEquatable<Line>
    {
        internal readonly float2 direction;
        internal readonly float2 point;

        /// <summary>
        /// Initializes a new instance of the <see cref="Line"/> struct.
        /// </summary>
        /// <param name="point">Origin of the Line.</param>
        /// <param name="direction">Direction of the Line.</param>
        public Line(float2 point, float2 direction)
        {
            this.point = point;
            this.direction = direction;
        }

        /// <inheritdoc/>
        public bool Equals(Line other)
        {
            return this.direction.Equals(other.direction) && this.point.Equals(other.point);
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            return obj is Line other && this.Equals(other);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return (this.direction.GetHashCode() * 397) ^ this.point.GetHashCode();
            }
        }
    }
}
