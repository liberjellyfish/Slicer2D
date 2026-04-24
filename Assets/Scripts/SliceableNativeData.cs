using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

[RequireComponent(typeof(PolygonCollider2D))]
public class SliceableNativeData : MonoBehaviour
{
    public NativeArray<float2> CachedVertices;
    public NativeArray<int2> CachedPathRanges; // x = start index, y = length

    private void Awake()
    {
        if (!CachedVertices.IsCreated)
        {
            InitializeFromCollider();
        }
    }

    private void InitializeFromCollider()
    {
        PolygonCollider2D col = GetComponent<PolygonCollider2D>();
        if (col == null || col.pathCount == 0)
        {
            return;
        }

        int totalPoints = 0;
        for (int i = 0; i < col.pathCount; i++)
        {
            totalPoints += col.GetPath(i).Length;
        }

        CachedVertices = new NativeArray<float2>(totalPoints, Allocator.Persistent);
        CachedPathRanges = new NativeArray<int2>(col.pathCount, Allocator.Persistent);

        int offset = 0;
        for (int i = 0; i < col.pathCount; i++)
        {
            Vector2[] path = col.GetPath(i);
            CachedPathRanges[i] = new int2(offset, path.Length);
            for (int k = 0; k < path.Length; k++)
            {
                CachedVertices[offset + k] = new float2(path[k].x, path[k].y);
            }

            offset += path.Length;
        }
    }

    public void InitFromNative(NativeArray<float2> persistentVertices, NativeArray<int2> persistentPathRanges)
    {
        // Do not dispose on pool despawn. Old buffers stay alive until this piece is
        // actually assigned a new shape again or the object is destroyed.
        DisposeCachedData();

        CachedVertices = persistentVertices;
        CachedPathRanges = persistentPathRanges;
    }

    private void OnDestroy()
    {
        DisposeCachedData();
    }

    private void DisposeCachedData()
    {
        if (CachedVertices.IsCreated)
        {
            CachedVertices.Dispose();
        }

        if (CachedPathRanges.IsCreated)
        {
            CachedPathRanges.Dispose();
        }
    }
}
