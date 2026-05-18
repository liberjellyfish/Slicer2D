using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// 直线与曲线切割生命周期的上下文数据载体 (跨帧零 GC)
/// </summary>
public class SliceContext : IDisposable
{
    public NativeList<float2> RawEdges;
    public NativeList<float2> UniqueVertices;
    public NativeList<int> AliasMap;
    public NativeParallelMultiHashMap<int, int> NativeGraph;

    public NativeList<float2> FlattenedLoops;
    public NativeList<int2> LoopRanges; // start_idx, count

    // ---- Phase 5 预计算分类结果 (由 SimplifyLoopsJob + ClassifyLoopsJob 写入) ----
    public NativeList<int> LoopTypes;       // 0=discard, 1=solid(CCW), -1=hole(CW)
    public NativeList<float> LoopAreas;     // 绝对面积
    public NativeList<float4> LoopBounds;   // (minX, minY, maxX, maxY)
    public NativeList<int> HoleParents;     // 孔洞→父级solid的环索引，-1=非孔洞或无归属

    // ---- Phase 6 搭桥合并 + 三角剖分 Job 输出 (由 BuildSolidHoleMapJob + MergeTriangulateJob 写入) ----
    public NativeList<int2> SolidHoleMap;   // 等长于 LoopRanges: solid→(holeStart, holeCount)，非solid=(-1,0)
    public NativeList<int2> HoleRangeBuffer; // 扁平化存储所有 solid 对应的孔洞范围
    public NativeStream MeshDataStream;      // MergeTriangulateJob 的输出流。会跨帧保留到 Phase 6 完成，因此必须使用 Persistent。
    public float4 UVRect;                    // (minX, minY, width, height) UV 参照矩形
    public UnityEngine.Mesh.MeshDataArray MeshDataArray; // 分配的 WritableMeshData
    public NativeArray<SlicerCore.FragmentPhysicsData> LoopPhysicsData; // 最终碎片级物理几何数据

    // ---- 拓扑重建托管容器 (用于 Phase 3 还原为 Unity Mesh) ----
    public Stack<List<Vector2>> ListPool;

    // 容量控制阈值
    private const int INITIAL_CAPACITY = 2048;
    private const int TRIM_THRESHOLD = 8192;

    public SliceContext()
    {
        InitializeCollections(INITIAL_CAPACITY);
    }

    private void InitializeCollections(int capacity)
    {
        RawEdges = new NativeList<float2>(capacity * 2, Allocator.Persistent);
        UniqueVertices = new NativeList<float2>(capacity, Allocator.Persistent);
        AliasMap = new NativeList<int>(capacity * 2, Allocator.Persistent);
        NativeGraph = new NativeParallelMultiHashMap<int, int>(capacity, Allocator.Persistent);
        FlattenedLoops = new NativeList<float2>(capacity * 2, Allocator.Persistent);
        LoopRanges = new NativeList<int2>(128, Allocator.Persistent);
        LoopTypes = new NativeList<int>(128, Allocator.Persistent);
        LoopAreas = new NativeList<float>(128, Allocator.Persistent);
        LoopBounds = new NativeList<float4>(128, Allocator.Persistent);
        HoleParents = new NativeList<int>(128, Allocator.Persistent);
        SolidHoleMap = new NativeList<int2>(128, Allocator.Persistent);
        HoleRangeBuffer = new NativeList<int2>(64, Allocator.Persistent);

        ListPool = new Stack<List<Vector2>>();
    }

    /// <summary>
    /// 重置结构以供下一个切割对象使用。
    /// 包含防潮汐内存膨胀检测：若容量超标则安全抛弃重建，防止内存高水位。
    /// </summary>
    public void ClearForReuse()
    {
        if (RawEdges.Capacity > TRIM_THRESHOLD || NativeGraph.Capacity > TRIM_THRESHOLD)
        {
            Dispose();
            InitializeCollections(INITIAL_CAPACITY);
        }
        else
        {
            RawEdges.Clear();
            UniqueVertices.Clear();
            AliasMap.Clear();
            NativeGraph.Clear();
            FlattenedLoops.Clear();
            LoopRanges.Clear();
            LoopTypes.Clear();
            LoopAreas.Clear();
            LoopBounds.Clear();
            HoleParents.Clear();
            SolidHoleMap.Clear();
            HoleRangeBuffer.Clear();
            if (LoopPhysicsData.IsCreated) { LoopPhysicsData.Dispose(); LoopPhysicsData = default; }
            // Phase 6 的 MeshDataStream 允许跨帧存活，统一在 Context 回收点释放。
            if (MeshDataStream.IsCreated) { MeshDataStream.Dispose(); MeshDataStream = default; }
            // Dispose MeshDataArray if it was not consumed by ApplyAndDisposeWritableMeshData
            if (MeshDataArray.Length > 0) { MeshDataArray.Dispose(); MeshDataArray = default; }
        }
    }

    public List<Vector2> GetList()
    {
        return ListPool.Count > 0 ? ListPool.Pop() : new List<Vector2>();
    }

    public void ReturnList(List<Vector2> list)
    {
        if (list == null) return;
        list.Clear();
        ListPool.Push(list);
    }

    public void Dispose()
    {
        if (RawEdges.IsCreated) RawEdges.Dispose();
        if (UniqueVertices.IsCreated) UniqueVertices.Dispose();
        if (AliasMap.IsCreated) AliasMap.Dispose();
        if (NativeGraph.IsCreated) NativeGraph.Dispose();
        if (FlattenedLoops.IsCreated) FlattenedLoops.Dispose();
        if (LoopRanges.IsCreated) LoopRanges.Dispose();
        if (LoopTypes.IsCreated) LoopTypes.Dispose();
        if (LoopAreas.IsCreated) LoopAreas.Dispose();
        if (LoopBounds.IsCreated) LoopBounds.Dispose();
        if (HoleParents.IsCreated) HoleParents.Dispose();
        if (SolidHoleMap.IsCreated) SolidHoleMap.Dispose();
        if (HoleRangeBuffer.IsCreated) HoleRangeBuffer.Dispose();
        if (LoopPhysicsData.IsCreated) LoopPhysicsData.Dispose();
        if (MeshDataStream.IsCreated) MeshDataStream.Dispose();
        if (MeshDataArray.Length > 0) MeshDataArray.Dispose();
    }
}

/// <summary>
/// 管理所有 SliceContext，提供借取并负责全局生命周期释放 (防止泄漏)。
/// </summary>
public static class SliceContextPool
{
    private static readonly Queue<SliceContext> Pool = new Queue<SliceContext>();
    private static readonly List<SliceContext> AllCreatedContexts = new List<SliceContext>();

    public static SliceContext Get()
    {
        if (Pool.Count > 0)
        {
            SliceContext ctx = Pool.Dequeue();
            ctx.ClearForReuse(); // 保险起见
            return ctx;
        }

        SliceContext newCtx = new SliceContext();
        AllCreatedContexts.Add(newCtx);
        return newCtx;
    }

    public static void Return(SliceContext ctx)
    {
        if (ctx == null) return;
        ctx.ClearForReuse();
        Pool.Enqueue(ctx);
    }

    /// <summary>
    /// 强制清理所有的常驻内存分配。
    /// </summary>
    public static void DisposeAll()
    {
        foreach (var ctx in AllCreatedContexts)
        {
            ctx.Dispose();
        }
        AllCreatedContexts.Clear();
        Pool.Clear();
    }

#if UNITY_EDITOR
    [UnityEditor.InitializeOnLoadMethod]
    private static void EditorInit()
    {
        UnityEditor.EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        UnityEditor.EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(UnityEditor.PlayModeStateChange state)
    {
        if (state == UnityEditor.PlayModeStateChange.ExitingPlayMode)
        {
            DisposeAll();
        }
    }
#endif

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void RuntimeInit()
    {
        DisposeAll(); // 处理 Domain Reload Disabled 下的初始化
        Application.quitting -= DisposeAll;
        Application.quitting += DisposeAll;
    }
}
