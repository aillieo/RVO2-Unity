// -----------------------------------------------------------------------
// <copyright file="FixedPoint.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Globalization;
    using System.Runtime.CompilerServices;
    using Unity.Burst;
    using UnityEngine;

    /// <summary>
    /// A fixed-point numeric type using Q31.32 representation (1 sign bit, 31 integer bits, 32 fraction bits).
    /// Provides deterministic arithmetic across all platforms.
    /// </summary>
    [BurstCompile(FixedBurstSettings.FloatPrecision, FixedBurstSettings.FloatMode, OptimizeFor = FixedBurstSettings.OptimizeFor)]
    [Serializable]
    public readonly struct FixedPoint : IEquatable<FixedPoint>, IComparable<FixedPoint>
    {
        /// <summary>Number of fractional bits in the Q31.32 format.</summary>
        public const int FractionalBits = 32;

        /// <summary>The raw long value representing 1.0 (2^32).</summary>
        public const long OneRaw = 1L << FractionalBits;

        /// <summary>Bitmask isolating the fractional part (low 32 bits).</summary>
        public const long FractionMask = OneRaw - 1L;

        /// <summary>FixedPoint constant representing zero.</summary>
        public static readonly FixedPoint Zero = new FixedPoint(0L);

        /// <summary>FixedPoint constant representing one.</summary>
        public static readonly FixedPoint One = new FixedPoint(OneRaw);

        /// <summary>FixedPoint constant representing one half (0.5).</summary>
        public static readonly FixedPoint Half = new FixedPoint(HalfRaw);

        /// <summary>Smallest representable FixedPoint value.</summary>
        public static readonly FixedPoint MinValue = new FixedPoint(long.MinValue);

        /// <summary>Largest representable FixedPoint value.</summary>
        public static readonly FixedPoint MaxValue = new FixedPoint(long.MaxValue);

        /// <summary>Smallest positive representable FixedPoint value (1 ULP).</summary>
        public static readonly FixedPoint Epsilon = new FixedPoint(1L);

        private const long HalfRaw = OneRaw >> 1;

        [SerializeField]
        private readonly long raw;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private FixedPoint(long raw)
        {
            this.raw = raw;
        }

        /// <summary>Gets the raw 64-bit integer value underlying this FixedPoint.</summary>
        public long RawValue
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => this.raw;
        }

        /// <summary>Explicitly converts an <see cref="int"/> to a <see cref="FixedPoint"/>.</summary>
        /// <param name="v">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator FixedPoint(int v) => new FixedPoint((long)v << FractionalBits);

        /// <summary>Explicitly converts a <see cref="short"/> to a <see cref="FixedPoint"/>.</summary>
        /// <param name="v">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator FixedPoint(short v) => new FixedPoint((long)v << FractionalBits);

        /// <summary>
        /// Explicitly converts a <see cref="long"/> to a <see cref="FixedPoint"/>.
        /// Throws <see cref="OverflowException"/> if the value exceeds the int range.
        /// </summary>
        /// <param name="v">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator FixedPoint(long v)
        {
            if (v > int.MaxValue || v < int.MinValue)
            {
                throw new OverflowException($"FixedPoint (Q31.32) cannot represent long value {v}.");
            }

            return new FixedPoint(v << FractionalBits);
        }

        /// <summary>
        /// Explicitly converts a <see cref="float"/> to a <see cref="FixedPoint"/>.
        /// Uses double intermediation to reduce precision loss. Not recommended for hot paths due to floating-point non-determinism.
        /// </summary>
        /// <param name="v">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator FixedPoint(float v)
        {
            var scaled = (double)v * OneRaw;

            if (scaled > long.MaxValue || scaled < long.MinValue)
            {
                throw new OverflowException($"FixedPoint (Q31.32) cannot represent float value {v}.");
            }

            return new FixedPoint((long)scaled);
        }

        /// <summary>Explicitly converts a <see cref="FixedPoint"/> to a <see cref="long"/>, truncating toward zero.</summary>
        /// <param name="f">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator long(FixedPoint f)
        {
            var raw = f.raw;

            if (raw < 0 && (raw & FractionMask) != 0)
            {
                return (raw >> FractionalBits) + 1;
            }

            return raw >> FractionalBits;
        }

        /// <summary>Explicitly converts a <see cref="FixedPoint"/> to an <see cref="int"/>, truncating toward zero.</summary>
        /// <param name="f">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator int(FixedPoint f) => (int)(long)f;

        /// <summary>Explicitly converts a <see cref="FixedPoint"/> to a <see cref="short"/>, truncating toward zero.</summary>
        /// <param name="f">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator short(FixedPoint f) => (short)(long)f;

        /// <summary>Explicitly converts a <see cref="FixedPoint"/> to a <see cref="float"/>.</summary>
        /// <param name="f">The value to convert.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float(FixedPoint f) => (float)(double)f.raw / OneRaw;

        /// <summary>Adds two <see cref="FixedPoint"/> values.</summary>
        /// <param name="a">The first operand.</param>
        /// <param name="b">The second operand.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint operator +(FixedPoint a, FixedPoint b) => new FixedPoint(a.raw + b.raw);

        /// <summary>Subtracts one <see cref="FixedPoint"/> from another.</summary>
        /// <param name="a">The first operand.</param>
        /// <param name="b">The second operand.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint operator -(FixedPoint a, FixedPoint b) => new FixedPoint(a.raw - b.raw);

        /// <summary>Negates a <see cref="FixedPoint"/> value.</summary>
        /// <param name="a">The value to negate.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint operator -(FixedPoint a) => new FixedPoint(-a.raw);

        /// <summary>Multiplies two <see cref="FixedPoint"/> values using 128-bit intermediate arithmetic with banker's rounding.</summary>
        /// <param name="a">The first operand.</param>
        /// <param name="b">The second operand.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint operator *(FixedPoint a, FixedPoint b)
        {
            var ar = a.raw;
            var br = b.raw;

            var signMask = (ar ^ br) >> 63;
            var ua = (ulong)(ar < 0 ? -ar : ar);
            var ub = (ulong)(br < 0 ? -br : br);

            var aLo = (uint)ua;
            var aHi = ua >> 32;
            var bLo = (uint)ub;
            var bHi = ub >> 32;

            var lolo = (ulong)aLo * bLo;
            var lohi = aLo * bHi;
            var hilo = aHi * bLo;
            var hihi = aHi * bHi;

            var mid = (lolo >> 32) + (uint)lohi + (uint)hilo;
            var carry = (mid >> 32) + (lohi >> 32) + (hilo >> 32);

            var roundCarry = (lolo & 0xFFFFFFFFu) >= 0x80000000u ? 1u : 0u;
            var lowResult = mid & 0xFFFFFFFFu | carry << 32;
            var magnitude = lowResult + roundCarry;

            magnitude += hihi << 32;

            var result = (long)magnitude;
            return new FixedPoint(signMask == 0 ? result : -result);
        }

        /// <summary>Divides one <see cref="FixedPoint"/> by another using 128-bit intermediate dividend.</summary>
        /// <param name="a">The dividend.</param>
        /// <param name="b">The divisor.</param>
        public static FixedPoint operator /(FixedPoint a, FixedPoint b)
        {
            if (b.raw == 0)
            {
                throw new DivideByZeroException("FixedPoint division by zero.");
            }

            var ar = a.raw;
            var br = b.raw;
            var signMask = (ar ^ br) >> 63;
            var ua = (ulong)(ar < 0 ? -ar : ar);
            var ub = (ulong)(br < 0 ? -br : br);

            var hi = ua >> 32;
            var lo = ua << 32;

            var q = DivU128By64(hi, lo, ub);

            var result = (long)q;
            return new FixedPoint(signMask == 0 ? result : -result);
        }

        /// <summary>Computes the remainder of dividing one <see cref="FixedPoint"/> by another.</summary>
        /// <param name="a">The dividend.</param>
        /// <param name="b">The divisor.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint operator %(FixedPoint a, FixedPoint b)
        {
            if (b.raw == 0)
            {
                throw new DivideByZeroException("FixedPoint mod by zero.");
            }

            return new FixedPoint(a.raw % b.raw);
        }

        /// <summary>Determines whether two FixedPoint values are equal.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(FixedPoint a, FixedPoint b) => a.raw == b.raw;

        /// <summary>Determines whether two FixedPoint values are not equal.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(FixedPoint a, FixedPoint b) => a.raw != b.raw;

        /// <summary>Determines whether the first FixedPoint is less than the second.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(FixedPoint a, FixedPoint b) => a.raw < b.raw;

        /// <summary>Determines whether the first FixedPoint is greater than the second.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(FixedPoint a, FixedPoint b) => a.raw > b.raw;

        /// <summary>Determines whether the first FixedPoint is less than or equal to the second.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(FixedPoint a, FixedPoint b) => a.raw <= b.raw;

        /// <summary>Determines whether the first FixedPoint is greater than or equal to the second.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(FixedPoint a, FixedPoint b) => a.raw >= b.raw;

        /// <summary>Constructs a FixedPoint from a raw long bit pattern, primarily for serialization and advanced usage.</summary>
        /// <param name="raw">The raw Q31.32 long value.</param>
        /// <returns>A new FixedPoint with the specified raw value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint FromRaw(long raw) => new FixedPoint(raw);

        /// <summary>Constructs a FixedPoint from an integer numerator and denominator ratio.</summary>
        /// <param name="numerator">The numerator of the ratio.</param>
        /// <param name="denominator">The denominator of the ratio (must not be zero).</param>
        /// <returns>A FixedPoint representing <paramref name="numerator"/> / <paramref name="denominator"/>.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint FromRatio(int numerator, int denominator)
        {
            if (denominator == 0)
            {
                throw new DivideByZeroException("FixedPoint ratio with zero denominator.");
            }

            var raw = ((long)numerator << FractionalBits) / denominator;
            return new FixedPoint(raw);
        }

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(FixedPoint other) => this.raw == other.raw;

        /// <inheritdoc/>
        public override bool Equals(object obj) => obj is FixedPoint f && this.raw == f.raw;

        /// <inheritdoc/>
        public override int GetHashCode() => this.raw.GetHashCode();

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CompareTo(FixedPoint other) => this.raw.CompareTo(other.raw);

        /// <inheritdoc/>
        public override string ToString() => ((float)this).ToString("F8", CultureInfo.InvariantCulture);

        /// <summary>Converts the value to its string representation using the specified format.</summary>
        /// <param name="format">A numeric format string.</param>
        /// <returns>The formatted string representation.</returns>
        public string ToString(string format) => ((float)this).ToString(format, CultureInfo.InvariantCulture);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static ulong DivU128By64(ulong hi, ulong lo, ulong d)
        {
            if (hi == 0)
            {
                return lo / d;
            }

            var q = 0UL;

            for (var i = 0; i < 64; i++)
            {
                var newHi = (hi << 1) | (lo >> 63);
                lo <<= 1;
                hi = newHi;
                q <<= 1;

                if (hi >= d)
                {
                    hi -= d;
                    q |= 1;
                }
            }

            return q;
        }
    }
}
