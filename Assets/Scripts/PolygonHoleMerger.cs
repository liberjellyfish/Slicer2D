using UnityEngine;
using System.Collections.Generic;
using System;

public class PolygonHoleMerger
{
    // 简单的缓存结构，避免排序时重复遍历
    private struct HoleData
    {
        public List<Vector2> Points;
        public int Index;
        public float MaxX;
        public int MaxXIndex;
    }

    public static List<Vector2> Merge(List<Vector2> outRing, List<List<Vector2>> holes)
    {
        if (holes == null || holes.Count == 0) return new List<Vector2>(outRing);

        // 1. 预处理数据 (优化排序性能)
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

        // 2. 排序：O(H log H) - 现在的比较是 O(1)
        holeDatas.Sort((a, b) => b.MaxX.CompareTo(a.MaxX));

        List<Vector2> merged = new List<Vector2>(outRing);

        // 缓存 merged 的包围盒，用于快速剔除
        // (在复杂场景下可进一步维护动态AABB树，此处暂用简单逻辑)

        // 3. 逐个合并
        // 总复杂度优化目标：接近 O(H * N)
        for (int h = 0; h < holeDatas.Count; h++)
        {
            HoleData currentHoleData = holeDatas[h];
            List<Vector2> currentHole = currentHoleData.Points;
            Vector2 M = currentHole[currentHoleData.MaxXIndex];

            // === 优化策略：射线投射法 ===
            // 不再盲目遍历所有点，而是寻找“M”向右射线的最近交点边
            // 该边的两个端点是极高概率的最佳候选点 P

            int bestConnectIndex = -1;

            // 尝试通过射线法快速找到候选点 (O(N))
            int raycastCandidateIndex = FindBridgeByRaycast(M, merged);

            if (raycastCandidateIndex != -1)
            {
                // 如果射线法找到了点，验证其合法性
                if (IsBridgeValid(M, merged[raycastCandidateIndex], merged, holeDatas, h))
                {
                    bestConnectIndex = raycastCandidateIndex;
                }
            }

            // 如果射线法失败（例如被其他洞挡住，虽然极少发生），回退到全局搜索 (Fallback)
            // 这保证了算法的鲁棒性，同时在 99% 的情况下享受 O(N) 的性能
            if (bestConnectIndex == -1)
            {
                bestConnectIndex = FindBridgeByGlobalSearch(M, merged, holeDatas, h);
            }

            if (bestConnectIndex != -1)
            {
                List<Vector2> insertion = new List<Vector2>();
                int holeCount = currentHole.Count;
                for (int i = 0; i < holeCount; i++)
                {
                    insertion.Add(currentHole[(currentHoleData.MaxXIndex + i) % holeCount]);
                }
                insertion.Add(M); // M -> ... -> M
                insertion.Add(merged[bestConnectIndex]); // 回程桥 P

                merged.InsertRange(bestConnectIndex + 1, insertion);
            }
            else
            {
                Debug.LogWarning("Merge Failed: 无法找到合法的桥接点。");
            }
        }

        return merged;
    }

    // 优化 1: 射线法寻找候选边 (O(N))
    private static int FindBridgeByRaycast(Vector2 M, List<Vector2> poly)
    {
        float minRayDist = float.MaxValue;
        int bestIndex = -1;
        Vector2 rayDir = Vector2.right;

        int count = poly.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 p1 = poly[i];
            Vector2 p2 = poly[(i + 1) % count];

            // 快速 AABB 剔除：如果边的 Y 范围不包含 M.y，或都在 M 左侧，跳过
            if ((p1.y < M.y && p2.y < M.y) || (p1.y > M.y && p2.y > M.y)) continue;
            if (p1.x < M.x && p2.x < M.x) continue;

            // 射线相交检测
            if (RayIntersectsSegment(M, rayDir, p1, p2, out float distance))
            {
                if (distance < minRayDist)
                {
                    minRayDist = distance;
                    // 选择该边中 X 较大的那个点作为候选点 (通常视角更好)
                    // 或者选择距离 M 最近的点
                    bestIndex = (p1.x > p2.x) ? i : (i + 1) % count;
                }
            }
        }
        return bestIndex;
    }

    // 优化 2: 全局搜索 (带距离剪枝)
    private static int FindBridgeByGlobalSearch(Vector2 M, List<Vector2> merged, List<HoleData> allHoles, int currentHoleIdx)
    {
        int bestIndex = -1;
        float minDistSq = float.MaxValue;
        int count = merged.Count;

        for (int i = 0; i < count; i++)
        {
            Vector2 P = merged[i];
            // 只考虑右侧的点 (简化)
            if (P.x <= M.x) continue;

            float distSq = (P - M).sqrMagnitude;
            if (distSq < minDistSq)
            {
                // 在调用昂贵的 IsBridgeValid 前，先做距离判断
                // 如果当前距离已经大于找到的最小距离，根本不用检测

                if (IsBridgeValid(M, P, merged, allHoles, currentHoleIdx))
                {
                    minDistSq = distSq;
                    bestIndex = i;
                }
            }
        }
        return bestIndex;
    }

    private static bool IsBridgeValid(Vector2 start, Vector2 end, List<Vector2> mergedPoly, List<HoleData> allHoles, int currentHoleIndex)
    {
        // AABB 预检查：构建桥的包围盒
        float minX = Mathf.Min(start.x, end.x);
        float maxX = Mathf.Max(start.x, end.x);
        float minY = Mathf.Min(start.y, end.y);
        float maxY = Mathf.Max(start.y, end.y);

        // 1. 检查主体
        if (LineIntersectsPolygon(start, end, mergedPoly, minX, maxX, minY, maxY)) return false;

        // 2. 检查其他未合并的洞
        for (int h = currentHoleIndex + 1; h < allHoles.Count; h++)
        {
            // 简单的 AABB 剔除：如果洞的 MaxX 都比桥的 MinX 小，肯定不相交 (因为洞排过序，这步其实很有用)
            if (allHoles[h].MaxX < minX) continue;

            if (LineIntersectsPolygon(start, end, allHoles[h].Points, minX, maxX, minY, maxY)) return false;
        }

        // 3. 检查自身
        if (LineIntersectsPolygon(start, end, allHoles[currentHoleIndex].Points, minX, maxX, minY, maxY)) return false;

        return true;
    }

    // 带 AABB 优化的线段与多边形相交检测
    private static bool LineIntersectsPolygon(Vector2 start, Vector2 end, List<Vector2> poly, float bMinX, float bMaxX, float bMinY, float bMaxY)
    {
        int count = poly.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 p1 = poly[i];
            Vector2 p2 = poly[(i + 1) % count];

            // 边 AABB 剔除
            if (Mathf.Max(p1.x, p2.x) < bMinX || Mathf.Min(p1.x, p2.x) > bMaxX ||
                Mathf.Max(p1.y, p2.y) < bMinY || Mathf.Min(p1.y, p2.y) > bMaxY)
                continue;

            if (p1 == start || p1 == end || p2 == start || p2 == end) continue;

            if (SegmentsIntersect(start, end, p1, p2)) return true;
        }
        return false;
    }

    private static bool RayIntersectsSegment(Vector2 origin, Vector2 direction, Vector2 p1, Vector2 p2, out float distance)
    {
        distance = 0f;
        Vector2 v1 = origin - p1;
        Vector2 v2 = p2 - p1;
        Vector2 v3 = new Vector2(-direction.y, direction.x);

        float dot = Vector2.Dot(v2, v3);
        if (Mathf.Abs(dot) < 0.000001f) return false;

        float t1 = (v2.x * v1.y - v2.y * v1.x) / dot;
        float t2 = Vector2.Dot(v1, v3) / dot;

        if (t1 >= 0.0f && (t2 >= 0.0f && t2 <= 1.0f))
        {
            distance = t1;
            return true;
        }
        return false;
    }

    private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float denominator = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (denominator == 0) return false;

        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / denominator;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / denominator;

        return (u > 0 && u < 1 && v > 0 && v < 1);
    }
}