using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

public static partial class CurveSlicerCore
{
    /// <summary>
    /// Phase 3: 全抛弃托管端，曲线贯穿切割全流 Job 化核心。
    /// </summary>
    public static List<SlicerCore.PolygonData> CalculateCurve(NativeArray<float2> pathVerts, NativeArray<int2> pathRanges, NativeArray<float2> cutPath, SliceContext sys)
    {
        sys.ClearForReuse();

        int cutSegCount = cutPath.Length - 1;
        int pathsCount = pathRanges.Length;
        if (cutSegCount < 1 || pathsCount == 0) return null;

        // Step 1: 为单条切路设立极速反向 AABB 树加速搜捕
        NativeAABBTree cutTree = new NativeAABBTree();
        NativeStream edgeStream = default;
        NativeStream cutHitStream = default;

        try
        {
            cutTree.Build(cutPath);

            // Step 2: 建立流向汇聚中心
            edgeStream = new NativeStream(pathsCount, Allocator.TempJob);
            cutHitStream = new NativeStream(pathsCount, Allocator.TempJob);

            // Step 3: 并行下发曲线截面碰撞检索
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

            // Step 4: 合并，抹除共用极点，拼装实体内部的双向墙壁
            var flattenJob = new SlicerCore.CurveFlattenAndSewJob
            {
                EdgeStreamReader = edgeStream.AsReader(),
                CutHitStreamReader = cutHitStream.AsReader(),
                PathCount = pathsCount,
                CutPath = cutPath,
                RawEdges = sys.RawEdges
            };
            var flattenHandle = flattenJob.Schedule(rebuildHandle);

            // 沿着原定计划汇聚入图层管线，传递依赖链
            // 内部会执行 extractHandle.Complete()，强制完成并同步主线程
            SlicerCore.RunNativeGraphPipeline(sys, flattenHandle, out List<SlicerCore.PolygonData> solids, out List<List<Vector2>> holesForTree);

            // --- Phase 4: 孔洞归属分配 ---
            NativePolyTree tree = new NativePolyTree();
            tree.Build(solids);

            for (int i = 0; i < holesForTree.Count; i++)
            {
                List<Vector2> hole = holesForTree[i];
                if (hole.Count < 3)
                {
                    sys.ReturnList(hole);
                    continue;
                }
                Vector2 testPoint = (hole[0] + hole[1]) * 0.5f;
                float holeAreaAbs = Mathf.Abs(SlicerCore.SignedArea(hole));

                SlicerCore.PolygonData bestParent = tree.QueryBestParent(testPoint, holeAreaAbs);
                if (bestParent != null)
                {
                    bestParent.Holes.Add(hole);
                }
                else
                {
                    sys.ReturnList(hole);
                }
            }

            tree.Dispose();
            return solids;
        }
        finally
        {
            // 所有操作完成后，不管发生异常还是正常结束，安全回收 TempJob 内存
            if (cutTree.IsCreated) cutTree.Dispose();
            if (edgeStream.IsCreated) edgeStream.Dispose();
            if (cutHitStream.IsCreated) cutHitStream.Dispose();
        }
    }
}
