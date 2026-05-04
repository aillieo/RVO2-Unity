// -----------------------------------------------------------------------
// <copyright file="FixedPoint2.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Runtime.CompilerServices;
    using Unity.Burst;

    /// <summary>
    /// A 2D vector using <see cref="FixedPoint"/> components for deterministic calculations.
    /// </summary>
    [BurstCompile(FixedBurstSettings.FloatPrecision, FixedBurstSettings.FloatMode, OptimizeFor = FixedBurstSettings.OptimizeFor)]
    [Serializable]
    public struct FixedPoint2 : IEquatable<FixedPoint2>
    {
        /// <summary>Shorthand for (0, 0).</summary>
        public static readonly FixedPoint2 Zero = new FixedPoint2(FixedPoint.Zero, FixedPoint.Zero);

        /// <summary>Shorthand for (1, 1).</summary>
        public static readonly FixedPoint2 One = new FixedPoint2(FixedPoint.One, FixedPoint.One);

        /// <summary>Shorthand for (0, 1).</summary>
        public static readonly FixedPoint2 Up = new FixedPoint2(FixedPoint.Zero, FixedPoint.One);

        /// <summary>Shorthand for (0, -1).</summary>
        public static readonly FixedPoint2 Down = new FixedPoint2(FixedPoint.Zero, -FixedPoint.One);

        /// <summary>Shorthand for (-1, 0).</summary>
        public static readonly FixedPoint2 Left = new FixedPoint2(-FixedPoint.One, FixedPoint.Zero);

        /// <summary>Shorthand for (1, 0).</summary>
        public static readonly FixedPoint2 Right = new FixedPoint2(FixedPoint.One, FixedPoint.Zero);

        /// <summary>The x component.</summary>
        public FixedPoint x;

        /// <summary>The y component.</summary>
        public FixedPoint y;

        /// <summary>Initializes a new instance of the <see cref="FixedPoint2"/> struct.</summary>
        /// <param name="x">The x component.</param>
        /// <param name="y">The y component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedPoint2(FixedPoint x, FixedPoint y)
        {
            this.x = x;
            this.y = y;
        }

        /// <summary>Gets the squared magnitude of the vector, suitable for distance comparisons without sqrt.</summary>
        public FixedPoint SqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (this.x * this.x) + (this.y * this.y);
        }

        /// <summary>Gets the magnitude (length) of the vector, requires sqrt.</summary>
        public FixedPoint Magnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => FixedPointMath.Sqrt((this.x * this.x) + (this.y * this.y));
        }

        /// <summary>Gets the normalized vector. Returns zero vector if magnitude is zero.</summary>
        public FixedPoint2 Normalized
        {
            get
            {
                var sqr = this.SqrMagnitude;

                if (sqr.RawValue == 0)
                {
                    return Zero;
                }

                var mag = FixedPointMath.Sqrt(sqr);
                return new FixedPoint2(this.x / mag, this.y / mag);
            }
        }

        /// <summary>Adds two vectors component-wise.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>The component-wise sum.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator +(FixedPoint2 a, FixedPoint2 b) =>
            new FixedPoint2(a.x + b.x, a.y + b.y);

        /// <summary>Subtracts one vector from another component-wise.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>The component-wise difference.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator -(FixedPoint2 a, FixedPoint2 b) =>
            new FixedPoint2(a.x - b.x, a.y - b.y);

        /// <summary>Negates a vector.</summary>
        /// <param name="a">The vector to negate.</param>
        /// <returns>The negated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator -(FixedPoint2 a) =>
            new FixedPoint2(-a.x, -a.y);

        /// <summary>Multiplies two vectors component-wise (scaling).</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>The component-wise product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator *(FixedPoint2 a, FixedPoint2 b) =>
            new FixedPoint2(a.x * b.x, a.y * b.y);

        /// <summary>Multiplies a vector by a scalar.</summary>
        /// <param name="a">The vector.</param>
        /// <param name="s">The scalar.</param>
        /// <returns>The scaled vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator *(FixedPoint2 a, FixedPoint s) =>
            new FixedPoint2(a.x * s, a.y * s);

        /// <summary>Multiplies a scalar by a vector.</summary>
        /// <param name="s">The scalar.</param>
        /// <param name="a">The vector.</param>
        /// <returns>The scaled vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator *(FixedPoint s, FixedPoint2 a) =>
            new FixedPoint2(a.x * s, a.y * s);

        /// <summary>Divides a vector by a scalar.</summary>
        /// <param name="a">The vector.</param>
        /// <param name="s">The scalar.</param>
        /// <returns>The divided vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint2 operator /(FixedPoint2 a, FixedPoint s) =>
            new FixedPoint2(a.x / s, a.y / s);

        /// <summary>Determines whether two vectors are equal.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns><c>true</c> if the vectors are equal; otherwise, <c>false</c>.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedPoint2 a, FixedPoint2 b) => a.x == b.x && a.y == b.y;

        /// <summary>Determines whether two vectors are not equal.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns><c>true</c> if the vectors are not equal; otherwise, <c>false</c>.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedPoint2 a, FixedPoint2 b) => a.x != b.x || a.y != b.y;

        /// <summary>Computes the dot product of two vectors.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>The dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Dot(FixedPoint2 a, FixedPoint2 b) => (a.x * b.x) + (a.y * b.y);

        /// <summary>Computes the 2D cross product (scalar) of two vectors.</summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>The 2D cross product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Cross(FixedPoint2 a, FixedPoint2 b) => (a.x * b.y) - (a.y * b.x);

        /// <summary>Returns the squared distance between two points.</summary>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        /// <returns>The squared distance.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint SqrDistance(FixedPoint2 a, FixedPoint2 b)
        {
            var dx = a.x - b.x;
            var dy = a.y - b.y;
            return (dx * dx) + (dy * dy);
        }

        /// <summary>Returns the distance between two points.</summary>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        /// <returns>The distance.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Distance(FixedPoint2 a, FixedPoint2 b) =>
            FixedPointMath.Sqrt(SqrDistance(a, b));

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedPoint2 other) => this.x == other.x && this.y == other.y;

        /// <inheritdoc/>
        public override bool Equals(object obj) => obj is FixedPoint2 v && this.Equals(v);

        /// <inheritdoc/>
        public override int GetHashCode() => (this.x.GetHashCode() * 397) ^ this.y.GetHashCode();

        /// <inheritdoc/>
        public override string ToString() => $"({this.x}, {this.y})";
    }
}
