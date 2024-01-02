// -----------------------------------------------------------------------
// <copyright file="EditorUtils.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using System.Reflection;
#if UNITY_EDITOR
    using UnityEditor;
#endif

    internal static class EditorUtils
    {
        public static void DrawGizmosSceneView(bool value)
        {
#if UNITY_EDITOR
            try
            {
                var typeofGameView = typeof(EditorWindow).Assembly.GetType("UnityEditor.GameView");
                var gameViewInstance = EditorWindow.GetWindow(typeofGameView);
                var drawGizmos = typeofGameView.GetProperty(
                    "drawGizmos",
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
                drawGizmos.SetValue(gameViewInstance, value);
            }
            catch (Exception e)
            {
                UnityEngine.Debug.LogException(e);
            }
#endif
        }
    }
}
