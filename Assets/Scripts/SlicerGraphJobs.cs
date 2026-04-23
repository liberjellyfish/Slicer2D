using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// SlicerCore 的 Phase 4 子模块——原生图拓扑重构与提环 Job。
/// </summary>
public static partial class SlicerCore
{
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
            for (int i = 0; i < N; i++)
            {
                sorted[i] = new PointWithIndex { P = RawEdges[i], OrigIndex = i };
            }

            sorted.Sort();

            int currentUniqueId = 0;

            for (int i = 0; i < N; i++)
            {
                if (i == 0)
                {
                    UniqueVertices.Add(sorted[i].P);
                    AliasMap[sorted[i].OrigIndex] = currentUniqueId;
                }
                else
                {
                    int matchedId = -1;
                    // 一维 Sweep-line 向后扫掠，剔除超限者
                    for (int j = i - 1; j >= 0; j--)
                    {
                        if (sorted[i].P.x - sorted[j].P.x > ToleranceX) break; // 超过 X 容差中断，缓存高命中

                        if (math.distancesq(sorted[i].P, sorted[j].P) < ToleranceSq)
                        {
                            matchedId = AliasMap[sorted[j].OrigIndex];
                            break;
                        }
                    }

                    if (matchedId != -1)
                    {
                        AliasMap[sorted[i].OrigIndex] = matchedId;
                    }
                    else
                    {
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
            // O(1) 哈希去重替代 O(degree) 线性扫描
            NativeHashSet<long> edgeSet = new NativeHashSet<long>(AliasMap.Length, Allocator.Temp);

            for (int i = 0; i < AliasMap.Length; i += 2)
            {
                int u = AliasMap[i];
                int v = AliasMap[i + 1];

                if (u == v) continue;

                long edgeKey = ((long)u << 32) | (uint)v;
                if (edgeSet.Add(edgeKey))
                {
                    Graph.Add(u, v);
                }
            }

            edgeSet.Dispose();
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
            int degree = Graph.CountValuesForKey(curr); // 循环外缓存度数，避免 O(D²)
            if (Graph.TryGetFirstValue(curr, out int n, out var it))
            {
                do
                {
                    if (n == prev && degree > 1) continue;
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
                if (cmp < 0)
                { // Cand is more left than Best
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
