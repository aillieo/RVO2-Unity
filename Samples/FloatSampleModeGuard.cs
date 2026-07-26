// -----------------------------------------------------------------------
// <copyright file="FloatSampleModeGuard.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using UnityEngine;

    internal abstract class FloatSampleModeGuard : MonoBehaviour
    {
        private const string InactiveMessage =
            "This sample requires float mode.\nUse Tools/RVO2/Deterministic/Disable RVO2_DETERMINISTIC, then wait for Unity to recompile.";

        private bool inactiveMessageLogged;

        protected virtual void Start()
        {
            this.LogInactiveMessage();
        }

        protected virtual void OnGUI()
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
    }
}
