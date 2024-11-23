// -----------------------------------------------------------------------
// <copyright file="ObstacleHelper.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System.Collections.Generic;
    using System.Linq;
    using System.Runtime.CompilerServices;
    using Unity.Mathematics;
    using UnityEngine;

    internal static class ObstacleHelper
    {
        private static readonly List<Vector2> buffer0 = new List<Vector2>();
        private static readonly List<Vector2> buffer1 = new List<Vector2>();

        public static List<float2> CalculateBoundingPolygon(Mesh mesh, Matrix4x4 localToWorldMatrix)
        {
            Vector3[] vertices = mesh.vertices;

            var points2d = vertices
                .Select(v => localToWorldMatrix.MultiplyPoint(v))
                .Select(v => new float2(v.x, v.z));

            List<float2> convexHull = CalculateConvexHull(points2d);

            Simplify(convexHull, 0.01f);
            return convexHull;
        }

        public static List<float2> CalculateBoundingPolygon(MeshFilter meshFilter)
        {
            return CalculateBoundingPolygon(meshFilter.mesh, meshFilter.transform.localToWorldMatrix);
        }

        private static List<float2> CalculateConvexHull(IEnumerable<float2> points)
        {
            // 先按y 再按x
            var sortedPoints = points.OrderBy(p => p.y).ThenBy(p => p.x).ToList();

            if (sortedPoints.Count <= 3)
            {
                return sortedPoints;
            }

            var hull = new List<float2>();

            // 上半圈
            foreach (float2 point in sortedPoints)
            {
                while (hull.Count >= 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], point) <= 0)
                {
                    hull.RemoveAt(hull.Count - 1);
                }

                hull.Add(point);
            }

            // 下半圈
            var lowerHullCount = hull.Count;
            for (var i = sortedPoints.Count - 2; i >= 0; i--)
            {
                float2 point = sortedPoints[i];
                while (hull.Count > lowerHullCount && Cross(hull[hull.Count - 2], hull[hull.Count - 1], point) <= 0)
                {
                    hull.RemoveAt(hull.Count - 1);
                }

                hull.Add(point);
            }

            hull.RemoveAt(hull.Count - 1);
            return hull;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Cross(in float2 o, in float2 a, in float2 b)
        {
            return ((a.x - o.x) * (b.y - o.y)) - ((a.y - o.y) * (b.x - o.x));
        }

        private static void Simplify(List<float2> points, float tolerance)
        {
            try
            {
                buffer0.AddRange(points.Select(f2 => (Vector2)f2));
                LineUtility.Simplify(buffer0, tolerance, buffer1);
                points.Clear();
                points.AddRange(buffer1.Select(v2 => (float2)v2));
            }
            finally
            {
                buffer0.Clear();
                buffer1.Clear();
            }
        }
    }
}
