// -----------------------------------------------------------------------
// <copyright file="NativeParallelMultiHashMapExtensions.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

namespace RVO
{
    using System;
    using Unity.Collections;

    /// <summary>
    /// Extension methods for <see cref="NativeParallelHashMap{TKey, TValue}"/>.
    /// </summary>
    public static class NativeParallelMultiHashMapExtensions
    {
        /// <summary>
        /// Removes a key-value pair from the NativeParallelHashMap if it exists.
        /// </summary>
        /// <typeparam name="TKey">The type of the keys in the map.</typeparam>
        /// <typeparam name="TValue">The type of the values in the map.</typeparam>
        /// <param name="hashMap">The NativeParallelHashMap.</param>
        /// <param name="key">The key to search.</param>
        /// <param name="value">The value to remove.</param>
        /// <returns>If the key-value pair was found and removed or not.</returns>
        public static bool RemoveOnce<TKey, TValue>(
            this ref NativeParallelMultiHashMap<TKey, TValue> hashMap,
            in TKey key,
            in TValue value)
            where TKey : struct, IEquatable<TKey>
            where TValue : struct, IEquatable<TValue>
        {
            if (hashMap.TryGetFirstValue(key, out TValue result, out NativeParallelMultiHashMapIterator<TKey> it))
            {
                if (result.Equals(value))
                {
                    hashMap.Remove(it);
                    return true;
                }

                while (hashMap.TryGetNextValue(out result, ref it))
                {
                    if (result.Equals(value))
                    {
                        hashMap.Remove(it);
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Removes all occurrences of a key-value pair from the NativeParallelHashMap if they exist.
        /// </summary>
        /// <typeparam name="TKey">The type of the keys in the map.</typeparam>
        /// <typeparam name="TValue">The type of the values in the map.</typeparam>
        /// <param name="hashMap">The NativeParallelHashMap.</param>
        /// <param name="key">The key to search.</param>
        /// <param name="value">The value to remove.</param>
        /// <returns>The number of key-value pairs that were found and removed.</returns>
        public static int Remove<TKey, TValue>(
            this ref NativeParallelMultiHashMap<TKey, TValue> hashMap,
            in TKey key,
            in TValue value)
            where TKey : struct, IEquatable<TKey>
            where TValue : struct, IEquatable<TValue>
        {
            var remove = 0;

            if (hashMap.TryGetFirstValue(key, out TValue result, out NativeParallelMultiHashMapIterator<TKey> it))
            {
                if (result.Equals(value))
                {
                    hashMap.Remove(it);
                    remove++;
                }

                while (hashMap.TryGetNextValue(out result, ref it))
                {
                    if (result.Equals(value))
                    {
                        hashMap.Remove(it);
                        remove++;
                    }
                }
            }

            return remove;
        }

        /// <summary>
        /// Replaces the first occurrence of a key-value pair in the NativeParallelHashMap with a new value if it exists.
        /// </summary>
        /// <typeparam name="TKey">The type of the keys in the map.</typeparam>
        /// <typeparam name="TValue">The type of the values in the map.</typeparam>
        /// <param name="hashMap">The NativeParallelHashMap.</param>
        /// <param name="key">The key to search.</param>
        /// <param name="value">The value to replace.</param>
        /// <param name="newValue">The new value.</param>
        /// <returns>If the key-value pair was found and replaced or not.</returns>
        public static bool ReplaceOnce<TKey, TValue>(
            this ref NativeParallelMultiHashMap<TKey, TValue> hashMap,
            in TKey key,
            in TValue value,
            in TValue newValue)
            where TKey : struct, IEquatable<TKey>
            where TValue : struct, IEquatable<TValue>
        {
            if (hashMap.TryGetFirstValue(key, out TValue result, out NativeParallelMultiHashMapIterator<TKey> it))
            {
                if (result.Equals(value))
                {
                    hashMap.SetValue(newValue, it);
                    return true;
                }

                while (hashMap.TryGetNextValue(out result, ref it))
                {
                    if (result.Equals(value))
                    {
                        hashMap.SetValue(newValue, it);
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Replaces all occurrences of a key-value pair in the NativeParallelHashMap with a new value if they exist.
        /// </summary>
        /// <typeparam name="TKey">The type of keys in the map.</typeparam>
        /// <typeparam name="TValue">The type of the values in the map.</typeparam>
        /// <param name="hashMap">The NativeParallelHashMap.</param>
        /// <param name="key">The key to search.</param>
        /// <param name="value">The value to replace.</param>
        /// <param name="newValue">The new value.</param>
        /// <returns>The number of key-value pairs that were found and replaced.</returns>
        public static int Replace<TKey, TValue>(
            this ref NativeParallelMultiHashMap<TKey, TValue> hashMap,
            in TKey key,
            in TValue value,
            in TValue newValue)
            where TKey : struct, IEquatable<TKey>
            where TValue : struct, IEquatable<TValue>
        {
            var remove = 0;

            if (hashMap.TryGetFirstValue(key, out TValue result, out NativeParallelMultiHashMapIterator<TKey> it))
            {
                if (result.Equals(value))
                {
                    hashMap.SetValue(newValue, it);
                    remove++;
                }

                while (hashMap.TryGetNextValue(out result, ref it))
                {
                    if (result.Equals(value))
                    {
                        hashMap.SetValue(newValue, it);
                        remove++;
                    }
                }
            }

            return remove;
        }

        /// <summary>
        /// Retrieves all values associated with a specific key in the NativeParallelHashMap.
        /// </summary>
        /// <typeparam name="TKey">The type of the keys in the map.</typeparam>
        /// <typeparam name="TValue">The type of the values in the map.</typeparam>
        /// <param name="hashMap">The NativeParallelHashMap.</param>
        /// <param name="key">The key to get the values for.</param>
        /// <param name="toFill">The NativeList to fill with the values.</param>
        public static void GetValuesForKey<TKey, TValue>(
            this ref NativeParallelMultiHashMap<TKey, TValue> hashMap,
            in TKey key,
            ref NativeList<TValue> toFill)
            where TKey : struct, IEquatable<TKey>
            where TValue : unmanaged, IEquatable<TValue>
        {
            if (!toFill.IsCreated)
            {
                throw new InvalidOperationException();
            }

            if (hashMap.TryGetFirstValue(key, out TValue result, out NativeParallelMultiHashMapIterator<TKey> it))
            {
                toFill.Add(result);
                while (hashMap.TryGetNextValue(out result, ref it))
                {
                    toFill.Add(result);
                }
            }
        }
    }
}
