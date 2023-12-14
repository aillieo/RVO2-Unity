// -----------------------------------------------------------------------
// <copyright file="JobHandleExtensions.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using Unity.Jobs;

    public static class JobHandleExtensions
    {
        /// <summary>
        /// Checks if the JobHandle is completed and completes it if necessary.
        /// </summary>
        /// <param name="jobHandle">The JobHandle to check and complete.</param>
        public static void CheckAndComplete(ref this JobHandle jobHandle)
        {
            if (jobHandle.IsCompleted)
            {
                jobHandle.Complete();
            }
        }
    }
}
