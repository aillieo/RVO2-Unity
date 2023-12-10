// -----------------------------------------------------------------------
// <copyright file="Simulator.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using Unity.Collections;
    using Unity.Mathematics;

    public static class NativeArrayExtensions
    {
        public static void SafeDispose<T>(this ref NativeArray<T> array) where T : struct
        {
            if (array.IsCreated)
            {
                array.Dispose();
            }
        }

        public static void Append<T>(this ref NativeArray<T> array, T item, Allocator allocator = Allocator.Persistent) where T : struct
        {
            Resize(ref array, array.Length + 1, allocator);
            array[array.Length - 1] = item;
        }

        public static void Resize<T>(this ref NativeArray<T> array, int newSize, Allocator allocator = Allocator.Persistent) where T : struct
        {
            if (array.Length == newSize)
            {
                return;
            }

            NativeArray<T> newArray = new NativeArray<T>(newSize, allocator);

            if (array.IsCreated)
            {
                int min = math.min(array.Length, newSize);
                NativeArray<T>.Copy(array, newArray, min);
                array.Dispose();
            }

            array = newArray;
        }
    }
}
