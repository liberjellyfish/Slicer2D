using System.Collections.Generic;
using UnityEngine;

public static class SliceMeshArrayPool
{
    private static readonly Dictionary<int, Stack<Mesh[]>> Pool = new Dictionary<int, Stack<Mesh[]>>();

    public static Mesh[] Rent(int size)
    {
        if (size <= 0)
        {
            return System.Array.Empty<Mesh>();
        }

        if (Pool.TryGetValue(size, out Stack<Mesh[]> bucket) && bucket.Count > 0)
        {
            return bucket.Pop();
        }

        return new Mesh[size];
    }

    public static void Return(Mesh[] array)
    {
        if (array == null || array.Length == 0)
        {
            return;
        }

        System.Array.Clear(array, 0, array.Length);

        if (!Pool.TryGetValue(array.Length, out Stack<Mesh[]> bucket))
        {
            bucket = new Stack<Mesh[]>();
            Pool[array.Length] = bucket;
        }

        bucket.Push(array);
    }
}
