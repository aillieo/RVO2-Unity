// -----------------------------------------------------------------------
// <copyright file="NativeArrayExtensions.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using Unity.Collections;
    using Unity.Mathematics;

    /// <summary>
    /// Extension methods for <see cref="NativeArray"/>.
    /// </summary>
    public static class NativeArrayExtensions
    {
        /// <summary>
        /// Safely disposes the NativeArray if it is created.
        /// </summary>
        /// <typeparam name="T">The type of the struct stored in the NativeArray.</typeparam>
        /// <param name="array">The NativeArray to dispose.</param>
        public static void SafeDispose<T>(this ref NativeArray<T> array)
            where T : struct
        {
            if (array.IsCreated)
            {
                array.Dispose();
            }
        }

        /// <summary>
        /// Appends an item to the NativeArray.
        /// </summary>
        /// <typeparam name="T">The type of the struct stored in the NativeArray.</typeparam>
        /// <param name="array">The NativeArray to append to.</param>
        /// <param name="item">The item to append.</param>
        /// <param name="allocator">The allocator used for resizing the NativeArray (default: Allocator.Persistent).</param>
        public static void Append<T>(this ref NativeArray<T> array, T item, Allocator allocator = Allocator.Persistent)
            where T : struct
        {
            Resize(ref array, array.Length + 1, allocator);
            array[array.Length - 1] = item;
        }

        /// <summary>
        /// Resizes the NativeArray to the specified size.
        /// </summary>
        /// <typeparam name="T">The type of the struct stored in the NativeArray.</typeparam>
        /// <param name="array">The NativeArray to resize.</param>
        /// <param name="newSize">The new size of the NativeArray.</param>
        /// <param name="allocator">The allocator used for resizing the NativeArray (default: Allocator.Persistent).</param>
        public static void Resize<T>(this ref NativeArray<T> array, int newSize, Allocator allocator = Allocator.Persistent)
            where T : struct
        {
            if (newSize < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(newSize), "New size cannot be negative.");
            }

            if (array.Length == newSize)
            {
                return;
            }

            var newArray = new NativeArray<T>(newSize, allocator);

            if (array.IsCreated)
            {
                var min = math.min(array.Length, newSize);
                NativeArray<T>.Copy(array, newArray, min);
                array.Dispose();
            }

            array = newArray;
        }
    }
}
