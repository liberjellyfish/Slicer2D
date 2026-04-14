using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

/// <summary>
/// 多边形空洞合并器 (Optimized Hole Merger)
/// <para>
/// 功能：使用"搭桥法" (Bridge Building) 将带洞多边形转换为简单多边形，以便进行三角剖分。
/// 优化策略：
/// 1. 绕序规范化：强制外圈 CCW，孔洞 CW。
/// 2. 静态树加速：使用 NativeAABBTree 将几何查询从 O(N) 降至 O(log N)。
/// 3. NativeArray内存展平：将原本频繁装箱 GC 的双向链表，压平成数组内连续寻址的零开销模式。
/// </para>
/// </summary>
public class PolygonHoleMerger
{
    // 双向链表节点 (NativeArray扁平版)，用于 O(1) 插入与查询
    private struct NativeListNode
    {
        public Vector2 Position;
        public int Next;
        public int Prev;
    }

    // 孔洞元数据
    private struct HoleData
    {
        public int Head;
        public int Count;
        public float MaxX;      // 关键：用于从最右侧开始合并
        public int MaxXNode;
    }

    // 动态生成的"桥"记录
    public struct BridgeSegment
    {
        public Vector2 A;
        public Vector2 B;
    }

    /// <summary>
    /// 合并核心入口
    /// </summary>
    public static List<Vector2> Merge(List<Vector2> outRing, List<List<Vector2>> holes)
    {
        if (holes == null || holes.Count == 0) return new List<Vector2>(outRing);

        // 0. 规范化绕序
        EnsureWinding(outRing, true);
        for (int i = 0; i < holes.Count; i++) EnsureWinding(holes[i], false);

        NativeAABBTree staticWallTree = new NativeAABBTree();
        staticWallTree.Build(outRing, holes);

        int totalNodes = outRing.Count;
        for (int i = 0; i < holes.Count; i++) totalNodes += holes[i].Count;
        int maxNodes = totalNodes + holes.Count * 2;
        
        NativeArray<NativeListNode> nodes = new NativeArray<NativeListNode>(maxNodes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        int nextFreeIndex = totalNodes;

        try
        {
            // 2. 链表化外圈
            int outerHead = CreateLoop(outRing, ref nodes, 0);

            // 3. 预处理孔洞
            List<HoleData> holeDatas = new List<HoleData>(holes.Count);
            int currentOffset = outRing.Count;
            for (int i = 0; i < holes.Count; i++)
            {
                var holePoints = holes[i];
                if (holePoints.Count < 3) continue;

                int head = CreateLoop(holePoints, ref nodes, currentOffset);
                currentOffset += holePoints.Count;

                // 寻找 X 坐标最大的点 (MaxX)
                int curr = head;
                int maxNode = head;
                float maxX = -float.MaxValue;
                int count = 0;
                
                int watchdog = 0;
                int watchdogLimit = maxNodes * 2;

                do
                {
                    if (nodes[curr].Position.x > maxX)
                    {
                        maxX = nodes[curr].Position.x;
                        maxNode = curr;
                    }
                    curr = nodes[curr].Next;
                    count++;
                    watchdog++;
                    if (watchdog > watchdogLimit) { Debug.LogError("[PolygonHoleMerger] Init Watchdog Timeout."); break; }
                } while (curr != head);

                holeDatas.Add(new HoleData { Head = head, Count = count, MaxX = maxX, MaxXNode = maxNode });
            }

            // 4. 排序：优先处理最右边的洞 (O(H log H))
            holeDatas.Sort((a, b) => b.MaxX.CompareTo(a.MaxX));

            List<BridgeSegment> dynamicBridges = new List<BridgeSegment>(holes.Count);

            // 5. 逐个合并
            foreach (var hole in holeDatas)
            {
                Vector2 M = nodes[hole.MaxXNode].Position;

                // 寻找最佳连接点 P (O(N_outer * log N_total))
                int bestP = FindBestBridgePoint(M, outerHead, ref nodes, staticWallTree, dynamicBridges, maxNodes);

                if (bestP != -1)
                {
                    Vector2 P = nodes[bestP].Position;
                    // 记录新桥，防止后续的洞穿过这条线
                    dynamicBridges.Add(new BridgeSegment { A = M, B = P });

                    // 执行指针缝合 (Surgery)
                    StitchLists(bestP, hole.MaxXNode, ref nodes, ref nextFreeIndex);
                }
                else
                {
                    Debug.LogWarning($"[PolygonHoleMerger] 无法为孔洞找到合法的桥! M点: {M}");
                }
            }

            // 6. 还原为 List (O(N))
            return FlattenList(outerHead, ref nodes, maxNodes);
        }
        finally
        {
            if (nodes.IsCreated) nodes.Dispose();
            staticWallTree.Dispose();
        }
    }

    private static int FindBestBridgePoint(
        Vector2 M,
        int outerLoop,
        ref NativeArray<NativeListNode> nodes,
        NativeAABBTree tree,
        List<BridgeSegment> bridges,
        int limitNodesCount)
    {
        NativeList<Vector2> candidates = new NativeList<Vector2>(limitNodesCount, Allocator.TempJob);
        NativeList<int> candidateIndices = new NativeList<int>(limitNodesCount, Allocator.TempJob);
        
        try
        {
            int curr = outerLoop;
            int watchdog = 0;
            do
            {
                if (curr < 0 || curr >= nodes.Length) { Debug.LogError("[PolygonHoleMerger] OOB inside FindBestBridgePoint."); break; }
                candidates.Add(nodes[curr].Position);
                candidateIndices.Add(curr);
                curr = nodes[curr].Next;
                watchdog++;
                if (watchdog > limitNodesCount * 2) { Debug.LogError("[PolygonHoleMerger] Watchdog Timeout FindBestBridgePoint."); break; }
            } while (curr != outerLoop);

            int pointCount = candidates.Length;

            if (pointCount <= 128)
            {
                return FindBestBridgePointSequential(M, candidates, candidateIndices, tree, bridges);
            }

            NativeArray<float> distances = new NativeArray<float>(pointCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<BridgeSegment> dynamicBridges = new NativeArray<BridgeSegment>(bridges.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            try
            {
                for (int i = 0; i < bridges.Count; i++) dynamicBridges[i] = bridges[i];

                BridgeScanJob job = new BridgeScanJob
                {
                    M = M,
                    Points = candidates.AsArray(),
                    Tree = tree,
                    Bridges = dynamicBridges,
                    OutputDistances = distances
                };

                job.Schedule(pointCount, 32).Complete();

                float minDist = float.MaxValue;
                int bestIndex = -1;

                for (int i = 0; i < pointCount; i++)
                {
                    if (distances[i] < minDist)
                    {
                        minDist = distances[i];
                        bestIndex = i;
                    }
                }

                if (bestIndex == -1) return -1;
                return candidateIndices[bestIndex];
            }
            finally
            {
                distances.Dispose();
                dynamicBridges.Dispose();
            }
        }
        finally
        {
            candidates.Dispose();
            candidateIndices.Dispose();
        }
    }

    private static int FindBestBridgePointSequential(
        Vector2 M,
        NativeList<Vector2> candidates,
        NativeList<int> candidateIndices,
        NativeAABBTree tree,
        List<BridgeSegment> bridges)
    {
        int bestNode = -1;
        float minDistSq = float.MaxValue;

        for (int i = 0; i < candidates.Length; i++)
        {
            Vector2 P = candidates[i];
            float distSq = (P - M).sqrMagnitude;

            if (P.x <= M.x) distSq += 1000000f;

            if (distSq < minDistSq)
            {
                if (IsBridgeValid(M, P, tree, bridges))
                {
                    minDistSq = distSq;
                    bestNode = candidateIndices[i];
                }
            }
        }
        return bestNode;
    }

    private static bool IsBridgeValid(Vector2 start, Vector2 end, NativeAABBTree tree, List<BridgeSegment> bridges)
    {
        if (tree.Intersects(start, end)) return false;

        int bridgeCount = bridges.Count;
        for (int i = 0; i < bridgeCount; i++)
        {
            BridgeSegment b = bridges[i];
            if (IsSamePoint(start, b.A) || IsSamePoint(start, b.B) ||
                IsSamePoint(end, b.A) || IsSamePoint(end, b.B)) return false;

            if (SegmentsIntersect(start, end, b.A, b.B)) return false;
        }
        return true;
    }

    private static void StitchLists(int nodeP, int nodeM, ref NativeArray<NativeListNode> nodes, ref int nextFreeIndex)
    {
        int pNext = nodes[nodeP].Next;
        int mPrev = nodes[nodeM].Prev;

        int copyM = nextFreeIndex++;
        int copyP = nextFreeIndex++;

        NativeListNode mPrime = new NativeListNode { Position = nodes[nodeM].Position };
        NativeListNode pPrime = new NativeListNode { Position = nodes[nodeP].Position };

        {
            var pNode = nodes[nodeP]; pNode.Next = nodeM; nodes[nodeP] = pNode;
            var mNode = nodes[nodeM]; mNode.Prev = nodeP; nodes[nodeM] = mNode;
        }

        {
            var mPrevNode = nodes[mPrev]; mPrevNode.Next = copyM; nodes[mPrev] = mPrevNode;
            mPrime.Prev = mPrev;
        }

        {
            mPrime.Next = copyP;
            pPrime.Prev = copyM;
        }

        {
            pPrime.Next = pNext;
            nodes[copyM] = mPrime;
            nodes[copyP] = pPrime;

            var pNextNode = nodes[pNext]; pNextNode.Prev = copyP; nodes[pNext] = pNextNode;
        }
    }

    private static void EnsureWinding(List<Vector2> points, bool targetCCW)
    {
        if (points == null || points.Count < 3) return;

        double area = 0; 
        for (int i = 0; i < points.Count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % points.Count];
            area += (p2.x - p1.x) * (p2.y + p1.y);
        }

        bool isCCW = area < 0;
        if (isCCW != targetCCW) points.Reverse();
    }

    private static int CreateLoop(List<Vector2> points, ref NativeArray<NativeListNode> nodes, int startOffset)
    {
        if (points.Count == 0) return -1;
        int count = points.Count;
        for (int i = 0; i < count; i++)
        {
            NativeListNode n = new NativeListNode();
            n.Position = points[i];
            n.Prev = startOffset + (i - 1 + count) % count;
            n.Next = startOffset + (i + 1) % count;
            nodes[startOffset + i] = n;
        }
        return startOffset;
    }

    private static List<Vector2> FlattenList(int head, ref NativeArray<NativeListNode> nodes, int limitNodesCount)
    {
        List<Vector2> result = new List<Vector2>();
        if (head == -1) return result;

        int curr = head;
        int watchdog = 0;
        int watchdogLimit = limitNodesCount * 2;
        do
        {
            if (curr < 0 || curr >= nodes.Length)
            {
                Debug.LogError($"[PolygonHoleMerger] FlattenList OOB limits! Cursor={curr}");
                break;
            }
            result.Add(nodes[curr].Position);
            curr = nodes[curr].Next;
            watchdog++;
            if (watchdog > watchdogLimit) 
            {
                Debug.LogError("[PolygonHoleMerger] FlattenList 遇到极限大环 WatchDog 死循环保护触发，强制跳出！");
                break;
            }
        } while (curr != head);

        return result;
    }

    private static bool IsSamePoint(Vector2 a, Vector2 b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return (dx * dx + dy * dy) < 1e-7f;
    }

    private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (den == 0) return false;
        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
        return (u > 1e-5f && u < 1f - 1e-5f && v > 1e-5f && v < 1f - 1e-5f);
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct BridgeScanJob : IJobParallelFor
    {
        public Vector2 M;
        [ReadOnly] public NativeArray<Vector2> Points;
        [ReadOnly] public NativeAABBTree Tree;
        [ReadOnly] public NativeArray<BridgeSegment> Bridges;

        [WriteOnly] public NativeArray<float> OutputDistances;

        public void Execute(int index)
        {
            Vector2 P = Points[index];
            float distSq = (P - M).sqrMagnitude;

            if (P.x <= M.x) distSq += 1000000f; 

            if (Tree.Intersects(M, P))
            {
                OutputDistances[index] = float.MaxValue;
                return;
            }

            for (int i = 0; i < Bridges.Length; i++)
            {
                BridgeSegment b = Bridges[i];
                if (IsSamePoint(M, b.A) || IsSamePoint(M, b.B) ||
                    IsSamePoint(P, b.A) || IsSamePoint(P, b.B))
                {
                    OutputDistances[index] = float.MaxValue;
                    return;
                }

                if (SegmentsIntersect(M, P, b.A, b.B))
                {
                    OutputDistances[index] = float.MaxValue;
                    return;
                }
            }

            OutputDistances[index] = distSq;
        }

        private static bool IsSamePoint(Vector2 a, Vector2 b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return (dx * dx + dy * dy) < 1e-7f;
        }

        private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
            if (den == 0) return false;
            float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
            float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
            return (u > 1e-5f && u < 1f - 1e-5f && v > 1e-5f && v < 1f - 1e-5f);
        }
    }
}