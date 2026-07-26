// -----------------------------------------------------------------------
// <copyright file="FixedPointMath.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Runtime.CompilerServices;
    using Unity.Burst;

    /// <summary>
    /// Provides common math functions for <see cref="FixedPoint"/> values.
    /// </summary>
    [BurstCompile(FixedBurstSettings.FloatPrecision, FixedBurstSettings.FloatMode, OptimizeFor = FixedBurstSettings.OptimizeFor)]
    public static class FixedPointMath
    {
        /// <summary>Returns the smaller of two FixedPoint values.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        /// <returns>The smaller of the two values.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Min(FixedPoint a, FixedPoint b) => a.RawValue < b.RawValue ? a : b;

        /// <summary>Returns the larger of two FixedPoint values.</summary>
        /// <param name="a">The first value.</param>
        /// <param name="b">The second value.</param>
        /// <returns>The larger of the two values.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Max(FixedPoint a, FixedPoint b) => a.RawValue > b.RawValue ? a : b;

        /// <summary>Clamps a value to the inclusive range [min, max].</summary>
        /// <param name="value">The value to clamp.</param>
        /// <param name="min">The lower bound.</param>
        /// <param name="max">The upper bound.</param>
        /// <returns>The clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Clamp(FixedPoint value, FixedPoint min, FixedPoint max)
        {
            if (value.RawValue < min.RawValue)
            {
                return min;
            }

            if (value.RawValue > max.RawValue)
            {
                return max;
            }

            return value;
        }

        /// <summary>Clamps a value to the inclusive range [0, 1].</summary>
        /// <param name="value">The value to clamp.</param>
        /// <returns>The clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Clamp01(FixedPoint value)
        {
            if (value.RawValue < 0L)
            {
                return FixedPoint.Zero;
            }

            if (value.RawValue > FixedPoint.OneRaw)
            {
                return FixedPoint.One;
            }

            return value;
        }

        /// <summary>Returns the absolute value.</summary>
        /// <param name="value">The value.</param>
        /// <returns>The absolute value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Abs(FixedPoint value)
        {
            var r = value.RawValue;
            var mask = r >> 63;
            return FixedPoint.FromRaw((r ^ mask) - mask);
        }

        /// <summary>Returns an integer indicating the sign of a FixedPoint value (-1, 0, or 1).</summary>
        /// <param name="value">The value.</param>
        /// <returns>-1, 0, or 1.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Sign(FixedPoint value)
        {
            var r = value.RawValue;
            return (r > 0L ? 1 : 0) - (r < 0L ? 1 : 0);
        }

        /// <summary>Returns the largest integer less than or equal to the value.</summary>
        /// <param name="value">The value.</param>
        /// <returns>The floor value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Floor(FixedPoint value)
        {
            return FixedPoint.FromRaw(value.RawValue & ~FixedPoint.FractionMask);
        }

        /// <summary>Returns the smallest integer greater than or equal to the value.</summary>
        /// <param name="value">The value.</param>
        /// <returns>The ceiling value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Ceil(FixedPoint value)
        {
            var r = value.RawValue;
            var frac = r & FixedPoint.FractionMask;
            var floored = r & ~FixedPoint.FractionMask;
            return FixedPoint.FromRaw(frac == 0L ? floored : floored + FixedPoint.OneRaw);
        }

        /// <summary>Rounds the value to the nearest integer.</summary>
        /// <param name="value">The value.</param>
        /// <returns>The rounded value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Round(FixedPoint value)
        {
            var r = value.RawValue;
            var half = FixedPoint.OneRaw >> 1;
            if (r >= 0L)
            {
                return FixedPoint.FromRaw((r + half) & ~FixedPoint.FractionMask);
            }
            else
            {
                return FixedPoint.FromRaw(-(((-r) + half) & ~FixedPoint.FractionMask));
            }
        }

        /// <summary>
        /// Computes the square root of a FixedPoint value using 128-bit intermediate arithmetic.
        /// Negative input throws <see cref="ArgumentException"/>.
        /// </summary>
        /// <param name="value">The value to compute the square root of (must be non-negative).</param>
        /// <returns>The square root.</returns>
        public static FixedPoint Sqrt(FixedPoint value)
        {
            var raw = value.RawValue;
            if (raw < 0L)
            {
                throw new ArgumentException("FixedPoint.Sqrt: negative input is not supported.", nameof(value));
            }

            if (raw == 0L)
            {
                return FixedPoint.Zero;
            }

            // 128 位被开方数:hi = raw >> 32,lo = raw << 32
            var nHi = (ulong)raw >> 32;
            var nLo = (ulong)raw << 32;

            // 128 位整数开方,按位法
            // 算法:从最高偶数位开始,每次尝试 (result + bit)^2 <= n
            // 这里把 128 位运算拆为高/低两部分
            ulong resHi = 0, resLo = 0;

            // bit 从 2^126 起 (偶数位最高位),由于实际 n <= 2^95,可以从 2^94 起
            // 直接从高位开始扫即可,正确性更稳;迭代约 64 次
            var bitHi = 1UL << 62;
            var bitLo = 0UL;

            // 跳过比 n 高的位
            while (bitHi > nHi || (bitHi == nHi && bitLo > nLo))
            {
                // bit >>= 2
                bitLo = (bitLo >> 2) | (bitHi << 62);
                bitHi >>= 2;
                if (bitHi == 0 && bitLo == 0)
                {
                    break;
                }
            }

            while (!(bitHi == 0 && bitLo == 0))
            {
                // t = result + bit  (128 位加)
                var tLo = resLo + bitLo;
                var tHi = resHi + bitHi + (tLo < resLo ? 1UL : 0UL);

                // result >>= 1
                resLo = (resLo >> 1) | (resHi << 63);
                resHi >>= 1;

                // if (n >= t) { n -= t; result += bit; }
                if (nHi > tHi || (nHi == tHi && nLo >= tLo))
                {
                    // n -= t
                    var newLo = nLo - tLo;
                    var borrow = nLo < tLo ? 1UL : 0UL;
                    nHi = nHi - tHi - borrow;
                    nLo = newLo;

                    // result += bit
                    var rLo = resLo + bitLo;
                    var rHi = resHi + bitHi + (rLo < resLo ? 1UL : 0UL);
                    resLo = rLo;
                    resHi = rHi;
                }

                // bit >>= 2
                bitLo = (bitLo >> 2) | (bitHi << 62);
                bitHi >>= 2;
            }

            // result 此时即 sqrt(n),最多 64 位 (因 n < 2^96,sqrt < 2^48)
            return FixedPoint.FromRaw((long)resLo);
        }

        /// <summary>Linearly interpolates between a and b by t (unclamped).</summary>
        /// <param name="a">The start value.</param>
        /// <param name="b">The end value.</param>
        /// <param name="t">The interpolation factor.</param>
        /// <returns>The interpolated value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint Lerp(FixedPoint a, FixedPoint b, FixedPoint t)
        {
            return a + ((b - a) * t);
        }

        /// <summary>Linearly interpolates between a and b by t, clamping t to [0, 1].</summary>
        /// <param name="a">The start value.</param>
        /// <param name="b">The end value.</param>
        /// <param name="t">The interpolation factor (clamped to [0, 1]).</param>
        /// <returns>The interpolated value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPoint LerpClamped(FixedPoint a, FixedPoint b, FixedPoint t)
        {
            return a + ((b - a) * Clamp01(t));
        }
    }
}
