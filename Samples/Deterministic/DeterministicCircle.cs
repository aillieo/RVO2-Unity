// -----------------------------------------------------------------------
// <copyright file="DeterministicCircle.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
#if RVO2_DETERMINISTIC
    using System;
    using System.Collections;
    using System.Collections.Generic;
#endif
    using UnityEngine;

    internal sealed class DeterministicCircle : MonoBehaviour
    {
        private const string InactiveMessage =
            "DeterministicCircle requires RVO2_DETERMINISTIC mode.\nUse Tools/RVO2/Deterministic/Enable RVO2_DETERMINISTIC, then wait for Unity to recompile.";

#if RVO2_DETERMINISTIC
        private const int AgentCount = 250;
        private const int CircleRadius = 200;

        private readonly Dictionary<int, FixedPoint2> goals = new Dictionary<int, FixedPoint2>();

        private FixedPointRandom pointRandom;
        private Simulator simulator;
#else
        private bool inactiveMessageLogged;
#endif

        private void Start()
        {
#if RVO2_DETERMINISTIC
            this.simulator = new Simulator();
            this.StartCoroutine(this.Main());
#else
            this.LogInactiveMessage();
#endif
        }

        private void OnDestroy()
        {
#if RVO2_DETERMINISTIC
            this.simulator?.Clear();
            this.simulator?.Dispose();
#endif
        }

        private void OnDrawGizmos()
        {
#if RVO2_DETERMINISTIC
            if (this.simulator == null)
            {
                return;
            }

            this.simulator.EnsureCompleted();

            foreach (var pair in this.goals)
            {
                var position = this.simulator.GetAgentPositionFixed(pair.Key);
                Gizmos.DrawSphere(new Vector2((float)position.x, (float)position.y), 1.5f);
            }
#endif
        }

#if !RVO2_DETERMINISTIC
        private void OnGUI()
        {
            this.LogInactiveMessage();

            var style = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 18,
                wordWrap = true,
            };
            style.normal.textColor = Color.white;

            GUI.Label(new Rect(0f, 0f, Screen.width, Screen.height), InactiveMessage, style);
        }

        private void LogInactiveMessage()
        {
            if (this.inactiveMessageLogged)
            {
                return;
            }

            this.inactiveMessageLogged = true;
            Debug.LogWarning(InactiveMessage, this);
        }
#endif

#if RVO2_DETERMINISTIC
        private IEnumerator Main()
        {
            while (true)
            {
                this.SetupScenario();

                do
                {
                    this.SetPreferredVelocities();
                    this.simulator.DoStep();
                    yield return null;
                    this.simulator.EnsureCompleted();
                }
                while (!this.ReachedGoal());

                yield return new WaitForSeconds(1);
                this.simulator.Clear();
            }
        }

        private void SetupScenario()
        {
            this.goals.Clear();
            this.pointRandom = FixedPointRandom.FromSeed(0);

            this.simulator.SetTimeStep(FixedPoint.FromRatio(1, 4));
            this.simulator.SetAgentDefaults(
                (FixedPoint)15,
                10,
                (FixedPoint)10,
                (FixedPoint)10,
                FixedPoint.FromRatio(3, 2),
                (FixedPoint)2,
                FixedPoint2.Zero);

            for (var i = 0; i < AgentCount; ++i)
            {
                var angle = i * 2.0 * Math.PI / AgentCount;
                var startPosition = new FixedPoint2(
                    (FixedPoint)((float)(CircleRadius * Math.Cos(angle))),
                    (FixedPoint)((float)(CircleRadius * Math.Sin(angle))));
                var agentId = this.simulator.AddAgent(startPosition);
                this.goals.Add(agentId, -startPosition);
            }
        }

        private void SetPreferredVelocities()
        {
            foreach (var pair in this.goals)
            {
                var agentId = pair.Key;
                var goalVector = pair.Value - this.simulator.GetAgentPositionFixed(agentId);

                if (goalVector.SqrMagnitude > FixedPoint.One)
                {
                    goalVector = goalVector.Normalized;
                }

                var perturb = this.pointRandom.InsideUnitCircle();
                goalVector += perturb * FixedPoint.FromRatio(1, 10000);

                this.simulator.SetAgentPrefVelocity(agentId, goalVector);
            }
        }

        private bool ReachedGoal()
        {
            foreach (var pair in this.goals)
            {
                var position = this.simulator.GetAgentPositionFixed(pair.Key);
                var radius = this.simulator.GetAgentRadiusFixed(pair.Key);
                if (FixedPoint2.SqrDistance(position, pair.Value) > radius * radius)
                {
                    return false;
                }
            }

            return true;
        }
#endif
    }
}
