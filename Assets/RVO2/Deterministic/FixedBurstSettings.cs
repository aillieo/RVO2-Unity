// -----------------------------------------------------------------------
// <copyright file="FixedBurstSettings.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Burst;

    internal static class FixedBurstSettings
    {
        internal const FloatPrecision FloatPrecision = Unity.Burst.FloatPrecision.Standard;
        internal const FloatMode FloatMode = Unity.Burst.FloatMode.Strict;
        internal const OptimizeFor OptimizeFor = Unity.Burst.OptimizeFor.Performance;
    }
}
