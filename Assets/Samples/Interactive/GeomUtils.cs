// -----------------------------------------------------------------------
// <copyright file="GeomUtils.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System.Collections.Generic;
    using Unity.Mathematics;

    /// <summary>
    /// Provides utility methods for geometric calculations.
    /// </summary>
    public static class GeomUtils
    {
        /// <summary>
        /// Determines whether a point is inside a polygon.
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <param name="polygon">The polygon represented as a list of points.</param>
        /// <returns>If the point is inside the polygon or not.</returns>
        public static bool PointInPolygon(float2 point, IList<float2> polygon)
        {
            var len = polygon.Count;
            var crossTimes = 0;
            for (var i = 0; i < len; ++i)
            {
                float2 p1 = polygon[i];
                float2 p2 = polygon[(i + 1) % len];
                if (p1.y == p2.y)
                {
                    continue;
                }

                if (p1.y < p2.y)
                {
                    if (point.y < p1.y)
                    {
                        continue;
                    }

                    if (point.y > p2.y)
                    {
                        continue;
                    }
                }
                else
                {
                    if (point.y < p2.y)
                    {
                        continue;
                    }

                    if (point.y > p1.y)
                    {
                        continue;
                    }
                }

                // Now p.y must be in range p1 and p2
                // Lerp
                var x = ((point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y)) + p1.x;
                if (x > point.x)
                {
                    crossTimes++;
                }
            }

            return crossTimes % 2 == 1;
        }
    }
}
