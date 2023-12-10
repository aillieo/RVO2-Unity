// -----------------------------------------------------------------------
// <copyright file="Simulator.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Jobs;

    public static class JobHandleExtensions
    {
        public static void CheckAndComplete(ref this JobHandle jobHandle)
        {
            if (jobHandle.IsCompleted)
            {
                jobHandle.Complete();
            }
        }
    }
}
