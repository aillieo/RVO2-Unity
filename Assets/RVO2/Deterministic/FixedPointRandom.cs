// -----------------------------------------------------------------------
// <copyright file="FixedPointRandom.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Runtime.CompilerServices;
    using Unity.Burst;

    /// <summary>
    /// A deterministic pseudo-random number generator using xorshift32,
    /// producing <see cref="FixedPoint"/> and <see cref="FixedPoint2"/> values.
    /// </summary>
    [BurstCompile(FixedBurstSettings.FloatPrecision, FixedBurstSettings.FloatMode, OptimizeFor = FixedBurstSettings.OptimizeFor)]
    [Serializable]
    public struct FixedPointRandom
    {
        private uint state;

        /// <summary>
        /// Initializes a new instance of the <see cref="FixedPointRandom"/> struct.
        /// A seed of 0 is replaced with 1 to prevent xorshift degeneration.
        /// </summary>
        /// <param name="seed">The initial seed value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedPointRandom(uint seed)
        {
            this.state = seed == 0 ? 1u : seed;
        }

        /// <summary>Gets or sets the current internal state, useful for serialization and replay.</summary>
        public uint State
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => this.state;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => this.state = value == 0 ? 1u : value;
        }

        /// <summary>Constructs a <see cref="FixedPointRandom"/> from an int seed, negative values are bitwise-cast to uint.</summary>
        /// <param name="seed">The seed value.</param>
        /// <returns>A new <see cref="FixedPointRandom"/> instance.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedPointRandom FromSeed(int seed) => new FixedPointRandom(unchecked((uint)seed));

        /// <summary>Returns the next pseudo-random <see cref="uint"/> in the sequence.</summary>
        /// <returns>A 32-bit unsigned integer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public uint NextUInt()
        {
            // xorshift32
            uint x = this.state;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            this.state = x;
            return x;
        }

        /// <summary>Returns a pseudo-random int in [0, int.MaxValue].</summary>
        /// <returns>A non-negative pseudo-random integer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int NextInt() => (int)(this.NextUInt() & 0x7FFFFFFFu);

        /// <summary>Returns a uniformly distributed pseudo-random int in [min, max).</summary>
        /// <param name="min">The inclusive lower bound.</param>
        /// <param name="max">The exclusive upper bound.</param>
        /// <returns>A pseudo-random integer in [min, max).</returns>
        public int NextInt(int min, int max)
        {
            if (min >= max)
            {
                if (min == max)
                {
                    return min;
                }

                throw new ArgumentException("min must be <= max");
            }

            // 用 uint 范围避免 int 溢出
            uint range = (uint)(max - min);
            uint r = this.NextUInt();

            // 非二次幂 range 使用拒绝采样消除取模偏差
            if ((range & (range - 1)) != 0)
            {
                ulong bound = (0x100000000UL / range) * range;
                while (r >= bound)
                {
                    r = this.NextUInt();
                }
            }

            return (int)(r % range) + min;
        }

        /// <summary>
        /// Returns a FixedPoint in [0, 1). The Q31.32 fraction uses all 32 bits from xorshift32,
        /// yielding 2^32 equally probable values.
        /// </summary>
        /// <returns>A FixedPoint in [0, 1).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedPoint NextFixed()
        {
            // xorshift32 输出 32 位,正好等于 Q31.32 的小数位宽,raw ∈ [0, 2^32-1] = [0, 1)
            return FixedPoint.FromRaw((long)this.NextUInt() & FixedPoint.FractionMask);
        }

        /// <summary>Returns a FixedPoint in [min, max).</summary>
        /// <param name="min">The inclusive lower bound.</param>
        /// <param name="max">The exclusive upper bound.</param>
        /// <returns>A FixedPoint in [min, max).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedPoint NextFixed(FixedPoint min, FixedPoint max)
        {
            // (max - min) * NextFixed() + min
            return min + ((max - min) * this.NextFixed());
        }

        /// <summary>Returns a pseudo-random boolean value.</summary>
        /// <returns>True or false with equal probability.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool NextBool() => (this.NextUInt() & 1u) != 0u;

        /// <summary>
        /// Returns a uniformly distributed random point inside the unit circle.
        /// Uses rejection sampling in [-1,1)^2 with a polar fallback to guarantee a valid result.
        /// </summary>
        /// <returns>A random point inside the unit circle.</returns>
        public FixedPoint2 InsideUnitCircle()
        {
            // 在 [-1, 1)^2 方块内拒绝采样
            for (int i = 0; i < 32; i++)
            {
                var x = (this.NextFixed() * (FixedPoint)2) - FixedPoint.One;
                var y = (this.NextFixed() * (FixedPoint)2) - FixedPoint.One;
                var s = (x * x) + (y * y);
                if (s > FixedPoint.Zero && s <= FixedPoint.One)
                {
                    return new FixedPoint2(x, y);
                }
            }

            // 兜底:将随机向量归一化到单位圆上再乘以 sqrt(uniform) 得到圆内均匀分布
            var bx = (this.NextFixed() * (FixedPoint)2) - FixedPoint.One;
            var by = (this.NextFixed() * (FixedPoint)2) - FixedPoint.One;
            var bm = FixedPointMath.Sqrt((bx * bx) + (by * by));
            if (bm <= FixedPoint.Zero)
            {
                return FixedPoint2.Zero;
            }

            var r = FixedPointMath.Sqrt(this.NextFixed());
            return new FixedPoint2((bx / bm) * r, (by / bm) * r);
        }
    }
}
