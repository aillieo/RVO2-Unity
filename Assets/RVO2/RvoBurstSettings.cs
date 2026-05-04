// -----------------------------------------------------------------------
// <copyright file="RvoBurstSettings.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Burst;

    internal static class RvoBurstSettings
    {
        // Define RVO_BURST_FAST_MATH for float builds that prefer throughput over strict IEEE behavior.
        // RVO2_DETERMINISTIC always stays strict so fixed-point results remain stable across platforms.
#if RVO2_DETERMINISTIC
        internal const FloatPrecision FloatPrecision = Unity.Burst.FloatPrecision.Standard;
        internal const FloatMode FloatMode = Unity.Burst.FloatMode.Strict;
#elif RVO_BURST_FAST_MATH
        internal const FloatPrecision FloatPrecision = Unity.Burst.FloatPrecision.Low;
        internal const FloatMode FloatMode = Unity.Burst.FloatMode.Fast;
#else
        internal const FloatPrecision FloatPrecision = Unity.Burst.FloatPrecision.Standard;
        internal const FloatMode FloatMode = Unity.Burst.FloatMode.Strict;
#endif

        internal const OptimizeFor OptimizeFor = Unity.Burst.OptimizeFor.Performance;
    }
}
