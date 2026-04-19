using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// SlicerCore 的 Job/Burst 子模块——所有并行化的碰撞检测 Job 集中在此文件管理。
/// 使用 partial class 保持与 SlicerCore 的同一命名空间。
/// </summary>
public static partial class SlicerCore
{
    // =================================================================================
    //                    Phase 2 直线切割并行重建与缝合 Job
    // =================================================================================

    public struct NativeIntersectionInfo
    {
        public float2 Point;
        public float T;
        public int SegmentIndex;
    }

    public struct NativeIntersectionComparer : IComparer<NativeIntersectionInfo>
    {
        public NativeArray<float2> Vertices;
        public int Offset;
        public int Compare(NativeIntersectionInfo a, NativeIntersectionInfo b)
        {
            if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
            float2 segStart = Vertices[Offset + a.SegmentIndex];
            float distA = (a.Point.x - segStart.x) * (a.Point.x - segStart.x) + (a.Point.y - segStart.y) * (a.Point.y - segStart.y);
            float distB = (b.Point.x - segStart.x) * (b.Point.x - segStart.x) + (b.Point.y - segStart.y) * (b.Point.y - segStart.y);
            return distA.CompareTo(distB);
        }
    }

    public struct NativeCutIntersectionComparer : IComparer<float2>
    {
        public float2 Start, End;
        public int Compare(float2 a, float2 b)
        {
            float distA = (a.x - Start.x) * (End.x - Start.x) + (a.y - Start.y) * (End.y - Start.y);
            float distB = (b.x - Start.x) * (End.x - Start.x) + (b.y - Start.y) * (End.y - Start.y);
            return distA.CompareTo(distB);
        }
    }

    [BurstCompile]
    public struct RebuildPathJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> PathVerts;
        [ReadOnly] public NativeArray<int2> PathRanges;
        public float2 SliceStart;
        public float2 SliceEnd;

        public NativeStream.Writer EdgeStreamWriter;
        public NativeStream.Writer CutHitStreamWriter;

        private float SqrDist(float2 a, float2 b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        private bool GetLineIntersection(float2 p1, float2 p2, out float2 intersection, out float t)
        {
            intersection = float2.zero;
            t = 0f;

            float2 dir = SliceEnd - SliceStart;
            float lenSq = dir.x * dir.x + dir.y * dir.y;
            if (lenSq < 1e-8f) return false;

            float2 normal = new float2(-dir.y, dir.x);

            float dist1 = math.dot(normal, p1 - SliceStart);
            float dist2 = math.dot(normal, p2 - SliceStart);

            int sign1 = dist1 > 0f ? 1 : -1;
            int sign2 = dist2 > 0f ? 1 : -1;

            if (sign1 != sign2)
            {
                float u = dist1 / (dist1 - dist2);
                intersection = p1 + u * (p2 - p1);
                t = math.dot(intersection - SliceStart, dir) / lenSq;

                if (t >= -1e-5f && t <= 1f + 1e-5f) return true;
            }
            return false;
        }

        public void Execute(int index)
        {
            EdgeStreamWriter.BeginForEachIndex(index);
            CutHitStreamWriter.BeginForEachIndex(index);

            int2 range = PathRanges[index];
            int pCount = range.y;
            int offset = range.x;

            NativeList<NativeIntersectionInfo> tempHits = new NativeList<NativeIntersectionInfo>(16, Allocator.Temp);
            NativeList<float2> newPathVertices = new NativeList<float2>(pCount + 16, Allocator.Temp);

            for (int i = 0; i < pCount; i++)
            {
                float2 p1 = PathVerts[offset + i];
                float2 p2 = PathVerts[offset + ((i + 1) % pCount)];

                if (GetLineIntersection(p1, p2, out float2 intersection, out float t))
                {
                    tempHits.Add(new NativeIntersectionInfo { Point = intersection, T = t, SegmentIndex = i });
                }
            }

            if (tempHits.Length > 1) {
                tempHits.Sort(new NativeIntersectionComparer { Vertices = PathVerts, Offset = offset });
            }

            int hitIndex = 0;
            for (int i = 0; i < pCount; i++)
            {
                float2 currentVert = PathVerts[offset + i];
                if (newPathVertices.Length == 0 || SqrDist(newPathVertices[newPathVertices.Length - 1], currentVert) > 0.0001f)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < tempHits.Length && tempHits[hitIndex].SegmentIndex == i)
                {
                    float2 p = tempHits[hitIndex].Point;
                    if (SqrDist(newPathVertices[newPathVertices.Length - 1], p) <= 0.0001f)
                    {
                        CutHitStreamWriter.Write(newPathVertices[newPathVertices.Length - 1]);
                    }
                    else
                    {
                        newPathVertices.Add(p);
                        CutHitStreamWriter.Write(p);
                    }
                    hitIndex++;
                }
            }

            if (newPathVertices.Length > 1 && SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Length - 1]) < 0.0001f)
            {
                newPathVertices.Length = newPathVertices.Length - 1; 
            }

            for (int i = 0; i < newPathVertices.Length; i++)
            {
                float2 u = newPathVertices[i];
                float2 v = newPathVertices[(i + 1) % newPathVertices.Length];
                if (SqrDist(u, v) > 0.0001f)
                {
                    EdgeStreamWriter.Write(u);
                    EdgeStreamWriter.Write(v);
                }
            }

            tempHits.Dispose();
            newPathVertices.Dispose();

            EdgeStreamWriter.EndForEachIndex();
            CutHitStreamWriter.EndForEachIndex();
        }
    }

    [BurstCompile]
    public struct FlattenAndSewJob : IJob
    {
        public NativeStream.Reader EdgeStreamReader;
        public NativeStream.Reader CutHitStreamReader;
        public int PathCount;
        public float2 SliceStart;
        public float2 SliceEnd;

        public NativeList<float2> RawEdges;

        public void Execute()
        {
            NativeList<float2> cutIntersections = new NativeList<float2>(64, Allocator.Temp);

            for (int i = 0; i < PathCount; i++)
            {
                int edgeCount = EdgeStreamReader.BeginForEachIndex(i);
                for (int e = 0; e < edgeCount; e++) {
                    RawEdges.Add(EdgeStreamReader.Read<float2>());
                }
                EdgeStreamReader.EndForEachIndex();

                int cutCount = CutHitStreamReader.BeginForEachIndex(i);
                for (int c = 0; c < cutCount; c++) {
                    cutIntersections.Add(CutHitStreamReader.Read<float2>());
                }
                CutHitStreamReader.EndForEachIndex();
            }

            if (cutIntersections.Length < 2) 
            {
                cutIntersections.Dispose();
                return;
            }

            cutIntersections.Sort(new NativeCutIntersectionComparer { Start = SliceStart, End = SliceEnd });

            int validCount = (cutIntersections.Length % 2 == 0) ? cutIntersections.Length : cutIntersections.Length - 1;
            for (int i = 0; i < validCount; i += 2)
            {
                float2 pA = cutIntersections[i];
                float2 pB = cutIntersections[i + 1];
                float dx = pA.x - pB.x; float dy = pA.y - pB.y;
                if (dx * dx + dy * dy > 0.0001f)
                {
                    RawEdges.Add(pA);
                    RawEdges.Add(pB);
                    RawEdges.Add(pB);
                    RawEdges.Add(pA);
                }
            }

            cutIntersections.Dispose();
        }
    }

    // =================================================================================
    //                    Phase 3 曲线切割并行重建与缝合 Job
    // =================================================================================

    public struct NativeCurveIntersectionInfo
    {
        public float2 Point;
        public float GlobalT;        
        public int SegmentIndex;     
        public float LocalTOnEdge;   
        public int PathId;           
        public bool IsEntry;         
    }

    // 针对每个 Path 局部重排 (防 S 型曲线重叠灾难)
    public struct NativeCurveIntersectionComparer : IComparer<NativeCurveIntersectionInfo>
    {
        public int Compare(NativeCurveIntersectionInfo a, NativeCurveIntersectionInfo b)
        {
            int segCmp = a.SegmentIndex.CompareTo(b.SegmentIndex);
            if (segCmp != 0) return segCmp;
            return a.LocalTOnEdge.CompareTo(b.LocalTOnEdge);
        }
    }

    // 全局 T 时序整理
    public struct CurveGlobalTComparer : IComparer<NativeCurveIntersectionInfo>
    {
        public int Compare(NativeCurveIntersectionInfo a, NativeCurveIntersectionInfo b)
        {
            return a.GlobalT.CompareTo(b.GlobalT);
        }
    }

    [BurstCompile]
    public struct CurveRebuildPathJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> PathVerts;
        [ReadOnly] public NativeArray<int2> PathRanges;
        [ReadOnly] public NativeArray<float2> CutPath;
        [ReadOnly] public NativeAABBTree CutTree; // 极速逆向 AABB 树

        public NativeStream.Writer EdgeStreamWriter;
        public NativeStream.Writer CutHitStreamWriter;

        private float SqrDist(float2 a, float2 b) { float dx=a.x-b.x; float dy=a.y-b.y; return dx*dx+dy*dy; }

        private bool SegmentIntersect(float2 a, float2 b, float2 c, float2 d, out float2 intersection, out float tEdge, out float tCut)
        {
            intersection = float2.zero; tEdge = 0; tCut = 0;
            float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
            if (den == 0f) return false;
            
            tEdge = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
            tCut = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
            
            if (tEdge >= -1e-5f && tEdge <= 1f + 1e-5f && tCut >= -1e-5f && tCut <= 1f + 1e-5f)
            {
                intersection = a + tEdge * (b - a);
                return true;
            }
            return false;
        }

        public void Execute(int index)
        {
            EdgeStreamWriter.BeginForEachIndex(index);
            CutHitStreamWriter.BeginForEachIndex(index);

            int2 range = PathRanges[index];
            int pCount = range.y;
            int offset = range.x;

            NativeList<NativeCurveIntersectionInfo> localHits = new NativeList<NativeCurveIntersectionInfo>(16, Allocator.Temp);
            NativeList<int> candidateIndices = new NativeList<int>(16, Allocator.Temp);

            for (int i = 0; i < pCount; i++)
            {
                float2 e1 = PathVerts[offset + i];
                float2 e2 = PathVerts[offset + ((i + 1) % pCount)];

                float minX = e1.x < e2.x ? e1.x : e2.x;
                float minY = e1.y < e2.y ? e1.y : e2.y;
                float maxX = e1.x > e2.x ? e1.x : e2.x;
                float maxY = e1.y > e2.y ? e1.y : e2.y;
                float padding = 0.005f;

                candidateIndices.Clear();
                CutTree.QueryOverlapBurst(minX - padding, minY - padding, maxX + padding, maxY + padding, ref candidateIndices);

                for (int c = 0; c < candidateIndices.Length; c++)
                {
                    int cutIdx = candidateIndices[c];
                    float2 c1 = CutPath[cutIdx];
                    float2 c2 = CutPath[cutIdx + 1];

                    if (SegmentIntersect(e1, e2, c1, c2, out float2 hitPoint, out float tEdge, out float tCut))
                    {
                        float2 edir = e2 - e1;
                        float2 cdir = c2 - c1;
                        float crossVal = edir.x * cdir.y - edir.y * cdir.x;
                        bool isEntry = crossVal > 0;

                        localHits.Add(new NativeCurveIntersectionInfo {
                            Point = hitPoint,
                            GlobalT = cutIdx + tCut,
                            SegmentIndex = i,
                            LocalTOnEdge = tEdge,
                            PathId = index, // index 是当前环在 PathRanges 的序号
                            IsEntry = isEntry
                        });
                    }
                }
            }

            // 局部排序对抗 S 型曲线切割交叉
            if (localHits.Length > 1) {
                localHits.Sort(new NativeCurveIntersectionComparer());
            }

            NativeList<float2> newPathVertices = new NativeList<float2>(pCount + 16, Allocator.Temp);

            int hitIndex = 0;
            for (int i = 0; i < pCount; i++)
            {
                float2 currentVert = PathVerts[offset + i];
                if (newPathVertices.Length == 0 || SqrDist(newPathVertices[newPathVertices.Length - 1], currentVert) > 0.0001f)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < localHits.Length && localHits[hitIndex].SegmentIndex == i)
                {
                    float2 p = localHits[hitIndex].Point;
                    if (SqrDist(newPathVertices[newPathVertices.Length - 1], p) <= 0.0001f)
                    {
                        // 极近点不作为物理端点更新拓扑路径，但保留交点实体身份抛向 CutHitStreamWriter
                        CutHitStreamWriter.Write(localHits[hitIndex]);
                    }
                    else
                    {
                        newPathVertices.Add(p);
                        CutHitStreamWriter.Write(localHits[hitIndex]);
                    }
                    hitIndex++;
                }
            }

            if (newPathVertices.Length > 1 && SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Length - 1]) < 0.0001f)
            {
                newPathVertices.Length = newPathVertices.Length - 1; 
            }

            for (int i = 0; i < newPathVertices.Length; i++)
            {
                float2 u = newPathVertices[i];
                float2 v = newPathVertices[(i + 1) % newPathVertices.Length];
                if (SqrDist(u, v) > 0.0001f)
                {
                    EdgeStreamWriter.Write(u);
                    EdgeStreamWriter.Write(v);
                }
            }

            localHits.Dispose();
            candidateIndices.Dispose();
            newPathVertices.Dispose();

            EdgeStreamWriter.EndForEachIndex();
            CutHitStreamWriter.EndForEachIndex();
        }
    }

    [BurstCompile]
    public struct CurveFlattenAndSewJob : IJob
    {
        public NativeStream.Reader EdgeStreamReader;
        public NativeStream.Reader CutHitStreamReader;
        public int PathCount;
        [ReadOnly] public NativeArray<float2> CutPath;

        public NativeList<float2> RawEdges;

        private float SqrDist(float2 a, float2 b) { float dx=a.x-b.x; float dy=a.y-b.y; return dx*dx+dy*dy; }

        public void Execute()
        {
            NativeList<NativeCurveIntersectionInfo> allHits = new NativeList<NativeCurveIntersectionInfo>(128, Allocator.Temp);

            for (int i = 0; i < PathCount; i++)
            {
                int edgeCount = EdgeStreamReader.BeginForEachIndex(i);
                for (int e = 0; e < edgeCount; e++) {
                    RawEdges.Add(EdgeStreamReader.Read<float2>());
                }
                EdgeStreamReader.EndForEachIndex();

                int cutCount = CutHitStreamReader.BeginForEachIndex(i);
                for (int c = 0; c < cutCount; c++) {
                    allHits.Add(CutHitStreamReader.Read<NativeCurveIntersectionInfo>());
                }
                CutHitStreamReader.EndForEachIndex();
            }

            if (allHits.Length < 2) 
            {
                allHits.Dispose();
                return;
            }

            allHits.Sort(new CurveGlobalTComparer());

            // Grazing(边缘极点波掠) 与顶点穿透的极终防错机制：按物理空间进行聚类合并！
            // 彻底根除因为截面浮点精度导致的幻象交点分离和同一顶点的出入奇偶性翻转（导致原本凹口生成实体块的恶性Bug）
            NativeList<NativeCurveIntersectionInfo> validHits = new NativeList<NativeCurveIntersectionInfo>(allHits.Length, Allocator.Temp);
            
            int c_idx = 0;
            while (c_idx < allHits.Length)
            {
                var baseHit = allHits[c_idx];
                bool hasEntry = baseHit.IsEntry;
                bool hasExit = !baseHit.IsEntry;
                
                int next_idx = c_idx + 1;
                while (next_idx < allHits.Length)
                {
                    var nextHit = allHits[next_idx];
                    // 放宽 T 容错到 0.05 (避开除法精度灾难)，主要信赖物理距离 0.0001f 判定归属同一顶点簇
                    if (math.abs(nextHit.GlobalT - baseHit.GlobalT) < 0.05f && SqrDist(nextHit.Point, baseHit.Point) < 0.0001f)
                    {
                        if (nextHit.IsEntry) hasEntry = true;
                        else hasExit = true;
                        next_idx++;
                    }
                    else
                    {
                        break;
                    }
                }
                
                if (hasEntry && hasExit)
                {
                    // 存在一进一出，证明是经过边缘的波掠 (Graze) 或是穿透了非流形触点 -> 宏观净贡献为 0，互抵消失！
                }
                else
                {
                    // 净结果为纯粹的 Entry 或纯粹的 Exit，保留其中之一作为该顶点的代表交点
                    baseHit.IsEntry = hasEntry; // 确保属性映射正确
                    validHits.Add(baseHit);
                }
                
                c_idx = next_idx;
            }

            int depth = 0;
            int entryHitIdx = -1;

            for (int i = 0; i < validHits.Length; i++)
            {
                if (validHits[i].IsEntry)
                {
                    depth++;
                    if (depth == 1) entryHitIdx = i; 
                }
                else
                {
                    if (depth > 0)
                    {
                        depth--;
                        if (depth == 0 && entryHitIdx >= 0)
                        {
                            float2 entry = validHits[entryHitIdx].Point;
                            float2 exit = validHits[i].Point;

                            if (SqrDist(entry, exit) >= 0.0001f)
                            {
                                float entryT = validHits[entryHitIdx].GlobalT;
                                float exitT = validHits[i].GlobalT;

                                int startCutSeg = (int)math.ceil(entryT);
                                int endCutSeg = (int)math.floor(exitT);

                                NativeList<float2> forwardWall = new NativeList<float2>(16, Allocator.Temp);
                                forwardWall.Add(entry);

                                for (int k = startCutSeg; k <= endCutSeg && k < CutPath.Length; k++)
                                {
                                    if (SqrDist(forwardWall[forwardWall.Length - 1], CutPath[k]) > 0.0001f &&
                                        SqrDist(CutPath[k], exit) > 0.0001f)
                                    {
                                        forwardWall.Add(CutPath[k]);
                                    }
                                }
                                forwardWall.Add(exit);

                                // 正向连结
                                for (int k = 0; k < forwardWall.Length - 1; k++)
                                {
                                    RawEdges.Add(forwardWall[k]);
                                    RawEdges.Add(forwardWall[k + 1]);
                                }

                                // 逆向连结（双层贴图厚面背夹）
                                for (int k = forwardWall.Length - 1; k > 0; k--)
                                {
                                    RawEdges.Add(forwardWall[k]);
                                    RawEdges.Add(forwardWall[k - 1]);
                                }
                                forwardWall.Dispose();
                            }
                            entryHitIdx = -1;
                        }
                    }
                    if (depth < 0) depth = 0;
                }
            }

            allHits.Dispose();
            validHits.Dispose();
        }
    }


    // =================================================================================
    //                    Phase 4 原生图拓扑重构与提环 Job (Migrated from SlicerSystem)
    // =================================================================================

    public struct PointWithIndex : System.IComparable<PointWithIndex>
    {
        public float2 P;
        public int OrigIndex;

        public int CompareTo(PointWithIndex other)
        {
            return P.x.CompareTo(other.P.x); // 按 X 单调排序
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct WeldingJob : IJob
    {
        [ReadOnly] public NativeList<float2> RawEdges;
        public NativeList<float2> UniqueVertices;
        public NativeList<int> AliasMap;
        public float ToleranceSq;
        public float ToleranceX;

        public void Execute()
        {
            int N = RawEdges.Length;
            AliasMap.Length = N;
            if (N == 0) return;

            NativeArray<PointWithIndex> sorted = new NativeArray<PointWithIndex>(N, Allocator.Temp);
            for (int i = 0; i < N; i++) {
                sorted[i] = new PointWithIndex { P = RawEdges[i], OrigIndex = i };
            }
            
            sorted.Sort();

            int currentUniqueId = 0;
            
            for (int i = 0; i < N; i++)
            {
                if (i == 0) {
                    UniqueVertices.Add(sorted[i].P);
                    AliasMap[sorted[i].OrigIndex] = currentUniqueId;
                } else {
                    int matchedId = -1;
                    // 一维 Sweep-line 向后扫掠，剔除超限者
                    for (int j = i - 1; j >= 0; j--) {
                        if (sorted[i].P.x - sorted[j].P.x > ToleranceX) break; // 超过 X 容差中断，缓存高命中

                        if (math.distancesq(sorted[i].P, sorted[j].P) < ToleranceSq) {
                            matchedId = AliasMap[sorted[j].OrigIndex];
                            break;
                        }
                    }

                    if (matchedId != -1) {
                        AliasMap[sorted[i].OrigIndex] = matchedId;
                    } else {
                        currentUniqueId++;
                        UniqueVertices.Add(sorted[i].P);
                        AliasMap[sorted[i].OrigIndex] = currentUniqueId;
                    }
                }
            }
            sorted.Dispose();
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct BuildGraphJob : IJob
    {
        [ReadOnly] public NativeList<int> AliasMap;
        public NativeParallelMultiHashMap<int, int> Graph;

        public void Execute()
        {
            for (int i = 0; i < AliasMap.Length; i += 2)
            {
                int u = AliasMap[i];
                int v = AliasMap[i + 1];

                if (u == v) continue; 

                // 查重过滤
                bool exists = false;
                if (Graph.TryGetFirstValue(u, out int neighbor, out var iterator))
                {
                    do {
                        if (neighbor == v) { exists = true; break; }
                    } while (Graph.TryGetNextValue(out neighbor, ref iterator));
                }

                if (!exists) Graph.Add(u, v);
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct ExtractLoopsJob : IJob
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, int> Graph;
        [ReadOnly] public NativeList<float2> UniqueVertices;
        
        public NativeList<float2> FlattenedLoops;
        public NativeList<int2> LoopRanges;

        public void Execute()
        {
            NativeHashSet<long> visitedEdges = new NativeHashSet<long>(Graph.Capacity, Allocator.Temp);
            var (keys, keysCount) = Graph.GetUniqueKeyArray(Allocator.Temp);

            for (int k = 0; k < keysCount; k++)
            {
                int startNode = keys[k];
                
                if (Graph.TryGetFirstValue(startNode, out int nextNodeInitial, out var iterInitial))
                {
                    do
                    {
                        long edgeKey = ((long)startNode << 32) | (uint)nextNodeInitial;
                        if (visitedEdges.Contains(edgeKey)) continue;

                        int startIndex = FlattenedLoops.Length;
                        int curr = startNode;
                        int next = nextNodeInitial;

                        FlattenedLoops.Add(UniqueVertices[curr]);

                        int watchdog = 0;
                        int maxIter = keys.Length * 2 + 100;
                        bool loopClosed = false;

                        while (watchdog++ < maxIter)
                        {
                            visitedEdges.Add(((long)curr << 32) | (uint)next);
                            FlattenedLoops.Add(UniqueVertices[next]);

                            if (next == startNode)
                            {
                                loopClosed = true;
                                break;
                            }

                            int prev = curr;
                            curr = next;

                            next = GetLeftMostNeighbor(prev, curr);
                            if (next == -1) break; 
                        }

                        int count = FlattenedLoops.Length - startIndex;
                        if (loopClosed && count > 2)
                        {
                            FlattenedLoops.Length -= 1; // 裁掉强行封口的重合尾点
                            LoopRanges.Add(new int2(startIndex, count - 1));
                        }
                        else
                        {
                            // Rollback 失效回路
                            FlattenedLoops.Length = startIndex;
                        }

                    } while (Graph.TryGetNextValue(out nextNodeInitial, ref iterInitial));
                }
            }
            
            visitedEdges.Dispose();
            keys.Dispose();
        }

        private int GetLeftMostNeighbor(int prev, int curr)
        {
            float2 prevP = UniqueVertices[prev];
            float2 currP = UniqueVertices[curr];
            
            float2 inDir = currP - prevP;
            float lenSq = inDir.x * inDir.x + inDir.y * inDir.y;
            if (lenSq < 1e-10f) inDir = new float2(1, 0); 
            
            FixedList64Bytes<int> neighbors = new FixedList64Bytes<int>();
            if (Graph.TryGetFirstValue(curr, out int n, out var it))
            {
                do {
                    if (n == prev && Graph.CountValuesForKey(curr) > 1) continue; 
                    if (neighbors.Length < 15) neighbors.Add(n);
                } while (Graph.TryGetNextValue(out n, ref it));
            }

            if (neighbors.Length == 0) return -1;
            if (neighbors.Length == 1) return neighbors[0];

            int bestNeighbor = neighbors[0];
            for (int i = 1; i < neighbors.Length; i++)
            {
                int cand = neighbors[i];
                float2 OutCand = UniqueVertices[cand] - currP;
                float2 OutBest = UniqueVertices[bestNeighbor] - currP;
                
                int cmp = CompareLeftMost(OutBest, OutCand, inDir);
                if (cmp < 0) { // Cand is more left than Best
                    bestNeighbor = cand;
                }
            }
            return bestNeighbor;
        }

        private int CompareLeftMost(float2 OutA, float2 OutB, float2 inDir)
        {
            int catA = GetCategory(OutA, inDir);
            int catB = GetCategory(OutB, inDir);

            if (catA != catB) return catA > catB ? 1 : -1;

            if (catA == 0 || catA == 2) return 0;

            float cross = OutA.x * OutB.y - OutA.y * OutB.x;
            
            if (cross > 1e-5f) return -1; // B is CCW over A (OutA < OutB)
            if (cross < -1e-5f) return 1; // A is CCW over B (OutA > OutB)
            return 0;
        }

        private int GetCategory(float2 V, float2 inDir)
        {
            float c = inDir.x * V.y - inDir.y * V.x; 
            float d = inDir.x * V.x + inDir.y * V.y; 

            if (c > 1e-5f) return 3; // Left Plane
            if (c < -1e-5f) return 1; // Right Plane
            if (d > 0) return 2; // Forward Line
            return 0; // Backward Line
        }
    }
}
