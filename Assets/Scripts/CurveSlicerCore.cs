using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

public static partial class CurveSlicerCore
{
    /// <summary>
    /// 曲线切割专用交点信息结构。
    /// </summary>
    public struct CurveIntersectionInfo
    {
        public Vector2 Point;
        public float GlobalT;        // 沿曲线的全局排序键
        public int SegmentIndex;     // 多边形边索引
        public float LocalTOnEdge;   // 在多边形边上的进度
        public int PathId;           // 所属路径 ID
        public bool IsEntry;         // 是否为进入实体的交叉点（由 cross(edgeDir, cutDir) 判定）
    }

    /// <summary>
    /// 曲线贯穿切割核心算法。
    /// cutPath 是经过 RDP 抽稀后的局部空间折线路径（已延长头尾）。
    /// </summary>
    public static List<SlicerCore.PolygonData> CalculateCurve(List<List<Vector2>> originalPaths, List<Vector2> cutPath)
    {
        var sys = SlicerSystem.Instance;
        sys.ClearAll();

        var cutIntersections = sys.CutIntersections;

        int cutSegCount = cutPath.Count - 1;
        if (cutSegCount < 1) return null;

        // --- Phase 1: 多段线 vs 多边形边界的全量碰撞 (使用 AABB 树预过滤) ---
        NativeList<CurveIntersectionInfo> allHits = new NativeList<CurveIntersectionInfo>(Mathf.Max(64, cutSegCount * 2), Allocator.Temp);
        NativeList<CurvePair> candidatePairs = new NativeList<CurvePair>(Mathf.Max(64, cutSegCount * 2), Allocator.Temp);
        NativeList<CurveIntersectionInfo> pathHits = new NativeList<CurveIntersectionInfo>(64, Allocator.Temp);
        
        try
        {
        NativeAABBTree envTree = new NativeAABBTree();
        List<List<Vector2>> holes = null;
        if (originalPaths.Count > 1) 
        {
            holes = new List<List<Vector2>>(originalPaths.Count - 1);
            for(int i = 1; i < originalPaths.Count; i++) holes.Add(originalPaths[i]);
        }
        envTree.Build(originalPaths[0], holes);

        List<NativeAABBTree.Segment> aabbResults = new List<NativeAABBTree.Segment>(16);

        float padding = 0.005f;

        for (int cutIdx = 0; cutIdx < cutSegCount; cutIdx++)
        {
            Vector2 cutA = cutPath[cutIdx];
            Vector2 cutB = cutPath[cutIdx + 1];
            
            aabbResults.Clear();
            // AABB 预查
            envTree.QueryOverlap(cutA, cutB, padding, aabbResults);

            for (int i = 0; i < aabbResults.Count; i++)
            {
                var s = aabbResults[i];
                candidatePairs.Add(new CurvePair
                {
                    EdgeA = s.P1,
                    EdgeB = s.P2,
                    CutA = cutA,
                    CutB = cutB,
                    PathId = s.PathId,
                    EdgeIdx = s.EdgeIdx,
                    CutIdx = cutIdx
                });
            }
        }
        
        envTree.Dispose();

        int totalPairs = candidatePairs.Length;
        bool useCurveJob = totalPairs > 64;
        NativeArray<CurvePair> jobPairs = default;
        NativeArray<CurveHitResult> jobResults = default;

        try
        {
            if (useCurveJob)
            {
                jobPairs = new NativeArray<CurvePair>(totalPairs, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                jobResults = new NativeArray<CurveHitResult>(totalPairs, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                for (int i = 0; i < totalPairs; i++)
                {
                    jobPairs[i] = candidatePairs[i];
                }

                new CurveIntersectionJob { Pairs = jobPairs, Results = jobResults }
                    .Schedule(totalPairs, 64).Complete();
            }

            CurvePathHitsComparer pathHitsComparer = new CurvePathHitsComparer();

            for (int pId = 0; pId < originalPaths.Count; pId++)
            {
                var path = originalPaths[pId];
                int pCount = path.Count;

                pathHits.Clear();

                if (useCurveJob)
                {
                    // 从 Job 结果中收集本路径的命中
                    for (int i = 0; i < totalPairs; i++)
                    {
                        CurveHitResult r = jobResults[i];
                        if (r.Hit && r.PathId == pId)
                        {
                            pathHits.Add(new CurveIntersectionInfo
                            {
                                Point = r.Point,
                                GlobalT = r.GlobalT,
                                SegmentIndex = r.EdgeIdx,
                                LocalTOnEdge = r.LocalTOnEdge,
                                PathId = r.PathId,
                                IsEntry = r.IsEntry
                            });
                        }
                    }
                }
                else
                {
                    // 小规模回退：主线程串行检测
                    for (int i = 0; i < totalPairs; i++)
                    {
                        var pair = candidatePairs[i];
                        if (pair.PathId == pId)
                        {
                            if (SlicerMath.SegmentSegmentIntersect(pair.EdgeA, pair.EdgeB, pair.CutA, pair.CutB,
                                out Vector2 intersection, out float tEdge, out float tCut))
                            {
                                float globalT = pair.CutIdx + tCut;
                                Vector2 edir = pair.EdgeB - pair.EdgeA;
                                Vector2 cdir = pair.CutB - pair.CutA;
                                float crossVal = edir.x * cdir.y - edir.y * cdir.x;
                                bool isEntry = crossVal > 0;

                                pathHits.Add(new CurveIntersectionInfo
                                {
                                    Point = intersection,
                                    GlobalT = globalT,
                                    SegmentIndex = pair.EdgeIdx,
                                    LocalTOnEdge = tEdge,
                                    PathId = pair.PathId,
                                    IsEntry = isEntry
                                });
                            }
                        }
                    }
                }

                // 按照 SegmentIndex 排序（主键），同 SegmentIndex 下按 LocalTOnEdge 排序（副键）
                pathHits.AsArray().Sort(pathHitsComparer);

                // 重建多边形路径（插入交点）
                sys.TempNewPath.Clear();
                var newPathVertices = sys.TempNewPath;

                int hitIdx = 0;
                for (int i = 0; i < pCount; i++)
                {
                    Vector2 currentVert = path[i];
                    if (newPathVertices.Count == 0 || SlicerCore.SqrDist(newPathVertices[newPathVertices.Count - 1], currentVert) > 0.0001f)
                    {
                        newPathVertices.Add(currentVert);
                    }

                    while (hitIdx < pathHits.Length && pathHits[hitIdx].SegmentIndex == i)
                    {
                        Vector2 p = pathHits[hitIdx].Point;
                        if (SlicerCore.SqrDist(newPathVertices[newPathVertices.Count - 1], p) <= 0.0001f)
                        {
                            // 极近时用已有顶点代替，但仍记录交点
                            cutIntersections.Add(newPathVertices[newPathVertices.Count - 1]);
                            allHits.Add(new CurveIntersectionInfo
                            {
                                Point = newPathVertices[newPathVertices.Count - 1],
                                GlobalT = pathHits[hitIdx].GlobalT,
                                SegmentIndex = pathHits[hitIdx].SegmentIndex,
                                LocalTOnEdge = pathHits[hitIdx].LocalTOnEdge,
                                PathId = pId,
                                IsEntry = pathHits[hitIdx].IsEntry
                            });
                        }
                        else
                        {
                            newPathVertices.Add(p);
                            cutIntersections.Add(p);
                            allHits.Add(pathHits[hitIdx]);
                        }
                        hitIdx++;
                    }
                }

                // 闭合检查
                if (newPathVertices.Count > 1 && SlicerCore.SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Count - 1]) < 0.0001f)
                    newPathVertices.RemoveAt(newPathVertices.Count - 1);

                // 将重建后的多边形路径加入图汇聚流 RawEdges
                for (int i = 0; i < newPathVertices.Count; i++)
                {
                    Vector2 u = newPathVertices[i];
                    Vector2 v = newPathVertices[(i + 1) % newPathVertices.Count];
                    if (SlicerCore.SqrDist(u, v) > 0.0001f) {
                        sys.RawEdges.Add(u);
                        sys.RawEdges.Add(v);
                    }
                }
            } // end per-path loop

        } // end inner try
        finally
        {
            if (jobPairs.IsCreated) jobPairs.Dispose();
            if (jobResults.IsCreated) jobResults.Dispose();
        }

        // --- Phase 2: 按全局 T 排序，使用 Entry/Exit 智能配对缝合曲线内壁 ---
        if (cutIntersections.Count < 2) return null;

        // 按全局 T 排序 allHits（零 GC Native 排列）
        allHits.AsArray().Sort(new CurveGlobalTComparer());

        // 深度追踪配对：depth=0 表示在空气中，depth>0 表示在实体中
        int depth = 0;
        int entryHitIdx = -1;

        for (int i = 0; i < allHits.Length; i++)
        {
            if (allHits[i].IsEntry)
            {
                depth++;
                if (depth == 1) entryHitIdx = i; // 刚进入实体，记录入口
            }
            else
            {
                if (depth > 0)
                {
                    depth--;
                    if (depth == 0 && entryHitIdx >= 0)
                    {
                        // 配对完成：从 entryHitIdx 到 i 的切割线段在实体内部，需要缝合内壁
                        Vector2 entry = allHits[entryHitIdx].Point;
                        Vector2 exit = allHits[i].Point;

                        if (SlicerCore.SqrDist(entry, exit) >= 0.0001f)
                        {
                            float entryT = allHits[entryHitIdx].GlobalT;
                            float exitT = allHits[i].GlobalT;

                            int startCutSeg = Mathf.CeilToInt(entryT);
                            int endCutSeg = Mathf.FloorToInt(exitT);

                            // 正向缝合（Entry → 内部曲线节点 → Exit）
                            List<Vector2> forwardWall = new List<Vector2>();
                            forwardWall.Add(entry);
                            for (int k = startCutSeg; k <= endCutSeg && k < cutPath.Count; k++)
                            {
                                if (SlicerCore.SqrDist(forwardWall[forwardWall.Count - 1], cutPath[k]) > 0.0001f &&
                                    SlicerCore.SqrDist(cutPath[k], exit) > 0.0001f)
                                {
                                    forwardWall.Add(cutPath[k]);
                                }
                            }
                            forwardWall.Add(exit);

                            // 正向边
                            for (int k = 0; k < forwardWall.Count - 1; k++)
                            {
                                sys.RawEdges.Add(forwardWall[k]);
                                sys.RawEdges.Add(forwardWall[k + 1]);
                            }

                            // 反向边 (Exit → 内部曲线节点逆序 → Entry)
                            for (int k = forwardWall.Count - 1; k > 0; k--)
                            {
                                sys.RawEdges.Add(forwardWall[k]);
                                sys.RawEdges.Add(forwardWall[k - 1]);
                            }
                        }
                        entryHitIdx = -1;
                    }
                }
                // depth < 0 说明有多余的 Exit（浮点边缘情况），安全钳位
                if (depth < 0) depth = 0;
            }
        }

        // --- Phase 3: 提取回路 (交由 SlicerSystem 原生层) ---
        SlicerCore.RunNativeGraphPipeline(out List<SlicerCore.PolygonData> solids, out List<List<Vector2>> holesForTree);

        // --- Phase 4: 孔洞归属分配 (复用现有 AABB 树) ---
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
        } // end outer try
        finally
        {
            if (allHits.IsCreated) allHits.Dispose();
            if (candidatePairs.IsCreated) candidatePairs.Dispose();
            if (pathHits.IsCreated) pathHits.Dispose();
        }
    }
}
