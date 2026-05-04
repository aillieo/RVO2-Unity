// -----------------------------------------------------------------------
// <copyright file="DeterministicDefineMenu.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO.Editor
{
    using System;
    using System.Collections.Generic;
    using UnityEditor;
    using UnityEditor.Build;
    using UnityEngine;

    internal static class DeterministicDefineMenu
    {
        private const string Symbol = "RVO2_DETERMINISTIC";
        private const string EnableMenu = "Tools/RVO2/Deterministic/Enable RVO2_DETERMINISTIC";
        private const string DisableMenu = "Tools/RVO2/Deterministic/Disable RVO2_DETERMINISTIC";
        private const string StatusMenu = "Tools/RVO2/Deterministic/Print Current Define State";

        [MenuItem(EnableMenu)]
        private static void Enable()
        {
            SetForActiveBuildTarget(true);
        }

        [MenuItem(EnableMenu, true)]
        private static bool ValidateEnable()
        {
            Menu.SetChecked(EnableMenu, IsEnabledForActiveBuildTarget());
            return true;
        }

        [MenuItem(DisableMenu)]
        private static void Disable()
        {
            SetForActiveBuildTarget(false);
        }

        [MenuItem(DisableMenu, true)]
        private static bool ValidateDisable()
        {
            Menu.SetChecked(DisableMenu, !IsEnabledForActiveBuildTarget());
            return true;
        }

        [MenuItem(StatusMenu)]
        private static void PrintStatus()
        {
            var target = GetActiveNamedBuildTarget();
            var defines = PlayerSettings.GetScriptingDefineSymbols(target);
            Debug.Log($"RVO2 {Symbol} is {(ContainsSymbol(defines) ? "enabled" : "disabled")} for {target.TargetName}. Defines: {defines}");
        }

        private static bool IsEnabledForActiveBuildTarget()
        {
            return ContainsSymbol(PlayerSettings.GetScriptingDefineSymbols(GetActiveNamedBuildTarget()));
        }

        private static void SetForActiveBuildTarget(bool enabled)
        {
            var target = GetActiveNamedBuildTarget();
            var defines = PlayerSettings.GetScriptingDefineSymbols(target);
            var symbols = ParseSymbols(defines);
            var changed = false;

            if (enabled)
            {
                if (!symbols.Contains(Symbol))
                {
                    symbols.Add(Symbol);
                    changed = true;
                }
            }
            else
            {
                changed = symbols.RemoveAll(s => string.Equals(s, Symbol, StringComparison.Ordinal)) > 0;
            }

            if (!changed)
            {
                Debug.Log($"RVO2 {Symbol} is already {(enabled ? "enabled" : "disabled")} for {target.TargetName}.");
                return;
            }

            PlayerSettings.SetScriptingDefineSymbols(target, symbols.ToArray());
            AssetDatabase.SaveAssets();
            Debug.Log($"RVO2 {Symbol} {(enabled ? "enabled" : "disabled")} for {target.TargetName}. Unity will recompile scripts.");
        }

        private static NamedBuildTarget GetActiveNamedBuildTarget()
        {
            var group = EditorUserBuildSettings.selectedBuildTargetGroup;
            if (group == BuildTargetGroup.Unknown)
            {
                group = BuildPipeline.GetBuildTargetGroup(EditorUserBuildSettings.activeBuildTarget);
            }

            return NamedBuildTarget.FromBuildTargetGroup(group);
        }

        private static bool ContainsSymbol(string defines)
        {
            return ParseSymbols(defines).Contains(Symbol);
        }

        private static List<string> ParseSymbols(string defines)
        {
            var result = new List<string>();
            if (string.IsNullOrEmpty(defines))
            {
                return result;
            }

            var symbols = defines.Split(';');
            for (var i = 0; i < symbols.Length; ++i)
            {
                var symbol = symbols[i].Trim();
                if (symbol.Length > 0 && !result.Contains(symbol))
                {
                    result.Add(symbol);
                }
            }

            return result;
        }
    }
}
