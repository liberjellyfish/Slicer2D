using UnityEngine;
using System.Collections.Generic;

public class PolygonHoleMerger
{
    private struct HoleData
    {
        public List<Vector2> Points;
        public int Index;
        public float MaxX;
        public int MaxXIndex;
    }

    // 缓存动态生成的桥，用于防止后续桥穿插
    private struct BridgeSegment
    {
        public Vector2 A;
        public Vector2 B;
    }

    public static List<Vector2> Merge(List<Vector2> outRing, List<List<Vector2>> holes)
    {
        if (holes == null || holes.Count == 0) return new List<Vector2>(outRing);

        // 1. 准备数据 & 排序 (O(H log H))
        List<HoleData> holeDatas = new List<HoleData>(holes.Count);
        for (int i = 0; i < holes.Count; i++)
        {
            var h = holes[i];
            float max = -float.MaxValue;
            int idx = 0;
            for (int j = 0; j < h.Count; j++)
            {
                if (h[j].x > max) { max = h[j].x; idx = j; }
            }
            holeDatas.Add(new HoleData { Points = h, Index = i, MaxX = max, MaxXIndex = idx });
        }
        holeDatas.Sort((a, b) => b.MaxX.CompareTo(a.MaxX)); // 从右向左合并

        // 2. 构建静态 AABB 树 (O(TotalV log TotalV))
        // 包含所有“不可穿越”的原始墙壁
        SegmentAABBTree staticWallTree = new SegmentAABBTree(outRing, holes);

        // 3. 动态桥列表 (用于记录新生成的切割线)
        List<BridgeSegment> dynamicBridges = new List<BridgeSegment>();

        List<Vector2> merged = new List<Vector2>(outRing);

        // 4. 逐个合并
        // 总循环次数 H
        for (int h = 0; h < holeDatas.Count; h++)
        {
            HoleData currentHoleData = holeDatas[h];
            List<Vector2> currentHole = currentHoleData.Points;
            Vector2 M = currentHole[currentHoleData.MaxXIndex];

            // === 寻找最佳连接点 P ===
            // 这里我们遍历 merged 列表 (O(N))
            // 但内部的 Check 变成了 O(log N)
            // 总复杂度: O(N log N)
            int bestConnectIndex = FindBestBridgePoint(M, merged, staticWallTree, dynamicBridges);

            if (bestConnectIndex != -1)
            {
                Vector2 P = merged[bestConnectIndex];

                // 记录新桥 (M <-> P)
                // 这保证了下一个洞不会穿过这条线
                dynamicBridges.Add(new BridgeSegment { A = M, B = P });

                // 执行几何缝合
                List<Vector2> insertion = new List<Vector2>();
                int holeCount = currentHole.Count;
                for (int i = 0; i < holeCount; i++)
                {
                    insertion.Add(currentHole[(currentHoleData.MaxXIndex + i) % holeCount]);
                }
                insertion.Add(M); // 回到起点的 M
                insertion.Add(P); // 回到 merged 上的 P

                merged.InsertRange(bestConnectIndex + 1, insertion);
            }
            else
            {
                Debug.LogWarning($"[Merge] Failed to merge hole {h}. Skipping.");
            }
        }

        return merged;
    }

    // 优化后的搜索函数
    private static int FindBestBridgePoint(
        Vector2 M,
        List<Vector2> mergedPoly,
        SegmentAABBTree tree,
        List<BridgeSegment> existingBridges)
    {
        int bestIndex = -1;
        float minDistSq = float.MaxValue;
        int count = mergedPoly.Count;

        // 遍历 merged 上所有可能的点 P
        for (int i = 0; i < count; i++)
        {
            Vector2 P = mergedPoly[i];

            // 1. 基础剪枝：只找右侧的点 (配合 MaxX 策略)
            if (P.x <= M.x) continue;

            float distSq = (P - M).sqrMagnitude;

            // 2. 距离剪枝
            if (distSq >= minDistSq) continue;

            // 3. 核心验证 (Expensive Check)
            // 只有当距离更近时，才进行昂贵的几何检测
            if (IsBridgeValid(M, P, tree, existingBridges))
            {
                minDistSq = distSq;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    // 验证逻辑：检查是否撞墙 (Tree) 或 撞桥 (List)
    private static bool IsBridgeValid(
        Vector2 start,
        Vector2 end,
        SegmentAABBTree tree,
        List<BridgeSegment> bridges)
    {
        // 1. 检查原始几何体 (O(log N))
        if (tree.Intersects(start, end)) return false;

        // 2. 检查动态生成的桥 (O(Number of Holes))
        // 桥的数量通常很少 (<50)，线性遍历极快
        foreach (var bridge in bridges)
        {
            // 排除共享顶点
            if ((start - bridge.A).sqrMagnitude < 1e-9f || (start - bridge.B).sqrMagnitude < 1e-9f ||
                (end - bridge.A).sqrMagnitude < 1e-9f || (end - bridge.B).sqrMagnitude < 1e-9f)
                continue;

            if (SegmentsIntersect(start, end, bridge.A, bridge.B)) return false;
        }

        // 3. 自交检查 (Edge Case)
        // 理论上 Tree 已经包含了所有可能的墙，但如果 merged 产生了复杂的自缠绕（虽然不该发生），
        // 这里的逻辑通常已经足够安全。

        return true;
    }

    private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (den == 0) return false;
        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
        return (u > 1e-5f && u < 0.99999f && v > 1e-5f && v < 0.99999f);
    }
}