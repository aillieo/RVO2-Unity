// -----------------------------------------------------------------------
// <copyright file="Obstacle.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// NOTICE: THIS FILE HAS BEEN MODIFIED BY AillieoTech UNDER COMPLIANCE WITH THE APACHE 2.0 LICENCE FROM THE ORIGINAL WORK.
// THE FOLLOWING IS THE COPYRIGHT OF THE ORIGINAL DOCUMENT:

/*
 * Obstacle.cs
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
#if RVO2_DETERMINISTIC
    using RvoVector2 = RVO.FixedPoint2;
#else
    using RvoVector2 = Unity.Mathematics.float2;
#endif

    /// <summary>
    /// Defines static obstacles in the simulation.
    /// </summary>
    internal struct Obstacle
    {
        internal readonly int id;
        internal readonly RvoVector2 point;
        internal readonly int obstacle;

        internal int nextIndex;
        internal int previousIndex;
        internal bool convex;
        internal RvoVector2 direction;

        internal Obstacle(int id, RvoVector2 point, int obstacle)
            : this()
        {
            this.id = id;
            this.point = point;
            this.obstacle = obstacle;
        }
    }
}
