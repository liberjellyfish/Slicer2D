using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

public static partial class CurveSlicerCore
{
    /// <summary>
    /// Phase 1: 异步改造，返回 JobHandle。原托管层代码移交 ResolveCutResult。
    /// </summary>
    public static JobHandle ScheduleCurveSliceJob(NativeArray<float2> pathVerts, NativeArray<int2> pathRanges, NativeArray<float2> cutPath, SliceContext sys)
    {
        sys.ClearForReuse();

        int cutSegCount = cutPath.Length - 1;
        int pathsCount = pathRanges.Length;
        if (cutSegCount < 1 || pathsCount == 0) return default;

        // 为单条切路设立极速反向 AABB 树加速搜捕，以及数据流
        NativeAABBTree cutTree = new NativeAABBTree();
        cutTree.Build(cutPath);
        
        NativeStream edgeStream = new NativeStream(pathsCount, Allocator.TempJob);
        NativeStream cutHitStream = new NativeStream(pathsCount, Allocator.TempJob);

        var rebuildJob = new SlicerCore.CurveRebuildPathJob
        {
            PathVerts = pathVerts,
            PathRanges = pathRanges,
            CutPath = cutPath,
            CutTree = cutTree,
            EdgeStreamWriter = edgeStream.AsWriter(),
            CutHitStreamWriter = cutHitStream.AsWriter()
        };
        var rebuildHandle = rebuildJob.Schedule(pathsCount, 1);

        var flattenJob = new SlicerCore.CurveFlattenAndSewJob
        {
            EdgeStreamReader = edgeStream.AsReader(),
            CutHitStreamReader = cutHitStream.AsReader(),
            PathCount = pathsCount,
            CutPath = cutPath,
            RawEdges = sys.RawEdges
        };
        var flattenHandle = flattenJob.Schedule(rebuildHandle);

        var graphHandle = SlicerCore.ScheduleNativeGraphPipeline(sys, flattenHandle);

        // 绑定异步依赖回收，交给底层生命周期
        cutTree.Dispose(graphHandle);
        edgeStream.Dispose(graphHandle);
        cutHitStream.Dispose(graphHandle);

        return graphHandle;
    }
}
