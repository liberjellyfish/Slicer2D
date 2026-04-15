using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

/// <summary>
/// 切割数据的只读缓存容器。
/// 在游戏初始化阶段消化所有的物理碰撞数据，从而消灭运行时的 PolygonCollider2D 调用 GC。
/// </summary>
[RequireComponent(typeof(PolygonCollider2D))]
public class SliceableNativeData : MonoBehaviour
{
    public NativeArray<float2> CachedVertices;
    public NativeArray<int2> CachedPathRanges; // x = start index, y = length

    private void Awake()
    {
        // 只有当尚未被 Native 初始化时，才自动抓取托管数组 (适用于初始加载的预制体)
        if (!CachedVertices.IsCreated)
        {
            InitializeFromCollider();
        }
    }

    private void InitializeFromCollider()
    {
        var col = GetComponent<PolygonCollider2D>();
        if (col == null || col.pathCount == 0) return;

        int totalPoints = 0;
        // 获取 path 产生少量 GC，但只发生在 Awake 初始化时，切割全流程里消除了此步骤
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

    /// <summary>
    /// 提供给切割管线后处理碎片的无 GC 初始化接口。
    /// 接管来自底层生成的 Native 数据，跳过中间托管的 Vector2[]
    /// </summary>
    public void InitFromNative(NativeArray<float2> vertices, NativeArray<int2> pathRanges)
    {
        // 彻底切断与托管的关联。进行一次 Native 间的深拷贝，保证生命周期独立。
        if (CachedVertices.IsCreated) CachedVertices.Dispose();
        if (CachedPathRanges.IsCreated) CachedPathRanges.Dispose();

        this.CachedVertices = new NativeArray<float2>(vertices, Allocator.Persistent);
        this.CachedPathRanges = new NativeArray<int2>(pathRanges, Allocator.Persistent);

        // TODO: Phase 4 这里将对接 PhysicsShapeGroup2D 底层构建碰撞体，彻底越过 SetPath() 
    }

    private void OnDestroy()
    {
        if (CachedVertices.IsCreated) CachedVertices.Dispose();
        if (CachedPathRanges.IsCreated) CachedPathRanges.Dispose();
    }
}
