// -----------------------------------------------------------------------
// <copyright file="SimulatorDeterministicTests.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

#if UNITY_INCLUDE_TESTS && RVO2_DETERMINISTIC
namespace RVO.Tests
{
    using System.Collections.Generic;
    using NUnit.Framework;
    using RVO;

    public sealed class SimulatorDeterministicTests
    {
        [Test]
        public void FixedApi_SingleAgent_AdvancesByPreferredVelocity()
        {
            using (var simulator = new Simulator())
            {
                simulator.SetTimeStep(FixedPoint.FromRatio(1, 4));
                simulator.SetAgentDefaults(
                    (FixedPoint)10,
                    10,
                    (FixedPoint)5,
                    (FixedPoint)5,
                    FixedPoint.One,
                    (FixedPoint)2,
                    FixedPoint2.Zero);

                var agentId = simulator.AddAgent(FixedPoint2.Zero);
                simulator.SetAgentPrefVelocity(agentId, FixedPoint2.Right);

                simulator.DoStep();
                simulator.EnsureCompleted();

                var position = simulator.GetAgentPositionFixed(agentId);
                Assert.AreEqual(FixedPoint.FromRatio(1, 4), simulator.GetGlobalTimeFixed());
                Assert.AreEqual(FixedPoint.FromRatio(1, 4), position.x);
                Assert.AreEqual(FixedPoint.Zero, position.y);
            }
        }

        [Test]
        public void FixedApi_QueryAgent_ReturnsNearbyAgent()
        {
            using (var simulator = new Simulator())
            {
                simulator.SetAgentDefaults(
                    (FixedPoint)10,
                    10,
                    (FixedPoint)5,
                    (FixedPoint)5,
                    FixedPoint.One,
                    (FixedPoint)2,
                    FixedPoint2.Zero);

                var nearAgentId = simulator.AddAgent(FixedPoint2.Zero);
                var farAgentId = simulator.AddAgent(new FixedPoint2((FixedPoint)10, FixedPoint.Zero));

                var result = new List<int>();
                var count = simulator.QueryAgent(FixedPoint2.Zero, (FixedPoint)2, result);

                Assert.AreEqual(1, count);
                Assert.Contains(nearAgentId, result);
                Assert.IsFalse(result.Contains(farAgentId));
            }
        }
    }
}
#endif
