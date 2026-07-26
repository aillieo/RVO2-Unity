// -----------------------------------------------------------------------
// <copyright file="SimulatorFloatTests.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

#if UNITY_INCLUDE_TESTS
namespace RVO.Tests
{
    using System.Collections.Generic;
    using NUnit.Framework;
    using RVO;
    using Unity.Mathematics;

    public sealed class SimulatorFloatTests
    {
        [Test]
        public void FloatApi_SingleAgent_AdvancesByPreferredVelocity()
        {
            using (var simulator = new Simulator())
            {
                simulator.SetTimeStep(0.25f);
                simulator.SetAgentDefaults(10f, 10, 5f, 5f, 1f, 2f, float2.zero);

                var agentId = simulator.AddAgent(float2.zero);
                simulator.SetAgentPrefVelocity(agentId, new float2(1f, 0f));

                simulator.DoStep();
                simulator.EnsureCompleted();

                var position = simulator.GetAgentPosition(agentId);
                Assert.AreEqual(0.25f, simulator.GetGlobalTime(), 0.0001f);
                Assert.AreEqual(0.25f, position.x, 0.0001f);
                Assert.AreEqual(0f, position.y, 0.0001f);
            }
        }

        [Test]
        public void FloatApi_QueryAgent_ReturnsNearbyAgent()
        {
            using (var simulator = new Simulator())
            {
                simulator.SetAgentDefaults(10f, 10, 5f, 5f, 1f, 2f, float2.zero);

                var nearAgentId = simulator.AddAgent(float2.zero);
                var farAgentId = simulator.AddAgent(new float2(10f, 0f));

                var result = new List<int>();
                var count = simulator.QueryAgent(float2.zero, 2f, result);

                Assert.AreEqual(1, count);
                Assert.Contains(nearAgentId, result);
                Assert.IsFalse(result.Contains(farAgentId));
            }
        }
    }
}
#endif
