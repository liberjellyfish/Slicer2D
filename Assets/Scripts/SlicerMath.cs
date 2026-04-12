using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

/// <summary>
/// 纯静态数学工具库，为切割系统提供：
/// 1. RDP (Ramer-Douglas-Peucker) 曲线抽稀
/// 2. 折线自交检测与回路提取
/// 3. 线段-线段精确相交检测
/// 4. Point-in-Polygon 射线法
/// </summary>
public static class SlicerMath
{
    // =========================================================================
    //                         常量
    // =========================================================================
    private const float EPSILON_SQ = 1e-8f;   // 距离平方阈值
    private const float EPSILON = 1e-4f;       // 线性阈值

    // =========================================================================
    //              RDP (Ramer-Douglas-Peucker) 曲线抽稀
    // =========================================================================

    /// <summary>
    /// 对输入的世界坐标路径点执行 RDP 抽稀。
    /// 在保持视觉弧度不变的前提下，大幅减少点数量。
    /// </summary>
    /// <param name="points">原始路径点序列</param>
    /// <param name="tolerance">容差阈值（弦高），推荐 0.02~0.05</param>
    /// <returns>精简后的路径点列表</returns>
    public static List<Vector2> SimplifyRDP(List<Vector2> points, float tolerance)
    {
        if (points == null || points.Count < 3)
        {
            return new List<Vector2>(points ?? new List<Vector2>());
        }

        bool[] keep = new bool[points.Count];
        keep[0] = true;
        keep[points.Count - 1] = true;

        RDPRecursive(points, 0, points.Count - 1, tolerance * tolerance, keep);

        List<Vector2> result = new List<Vector2>();
        for (int i = 0; i < points.Count; i++)
        {
            if (keep[i]) result.Add(points[i]);
        }
        return result;
    }

    private static void RDPRecursive(List<Vector2> points, int startIdx, int endIdx, float toleranceSq, bool[] keep)
    {
        if (endIdx - startIdx < 2) return;

        Vector2 lineStart = points[startIdx];
        Vector2 lineEnd = points[endIdx];

        float maxDistSq = 0f;
        int maxIdx = startIdx;

        for (int i = startIdx + 1; i < endIdx; i++)
        {
            float distSq = PointToSegmentDistanceSq(points[i], lineStart, lineEnd);
            if (distSq > maxDistSq)
            {
                maxDistSq = distSq;
                maxIdx = i;
            }
        }

        if (maxDistSq > toleranceSq)
        {
            keep[maxIdx] = true;
            RDPRecursive(points, startIdx, maxIdx, toleranceSq, keep);
            RDPRecursive(points, maxIdx, endIdx, toleranceSq, keep);
        }
    }

    // =========================================================================
    //                   折线自交检测与回路提取
    // =========================================================================

    /// <summary>
    /// 检测折线是否存在自交。如果存在，提取最大闭合环并截断脐带。
    /// </summary>
    /// <param name="path">输入路径（会被修改）</param>
    /// <param name="extractedLoop">如果发现自交，输出提取出的纯净闭合环；否则为 null</param>
    /// <returns>true = 发现了自交并已处理</returns>
    public static bool DetectAndResolveSelfIntersection(List<Vector2> path, out List<Vector2> extractedLoop)
    {
        extractedLoop = null;
        if (path == null || path.Count < 4) return false;

        // 从后往前扫描，优先发现最晚出现的（最大的）回路
        // 对每一对不相邻的线段进行碰撞测试
        for (int i = path.Count - 2; i >= 1; i--)
        {
            Vector2 c = path[i];
            Vector2 d = path[i + 1];

            // 与更早的线段比较（跳过相邻线段）
            for (int j = 0; j < i - 1; j++)
            {
                Vector2 a = path[j];
                Vector2 b = path[j + 1];

                if (SegmentSegmentIntersect(a, b, c, d, out Vector2 intersection))
                {
                    // 发现自交！提取从 j+1 到 i 的闭合环
                    extractedLoop = new List<Vector2>();
                    extractedLoop.Add(intersection);
                    for (int k = j + 1; k <= i; k++)
                    {
                        extractedLoop.Add(path[k]);
                    }
                    // 闭合环（回到交点）

                    // 截断原始路径：删除环区段以及环之后的部分（脐带尾），也删除环之前的废线头
                    // 最终只保留纯净闭合环
                    path.Clear();
                    path.AddRange(extractedLoop);

                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// 判断折线的首末端是否形成闭合环（距离极近）。
    /// </summary>
    public static bool IsClosedLoop(List<Vector2> path, float threshold = 0.15f)
    {
        if (path == null || path.Count < 3) return false;
        return (path[0] - path[path.Count - 1]).sqrMagnitude < threshold * threshold;
    }

    // =========================================================================
    //               线段-线段精确相交检测
    // =========================================================================

    /// <summary>
    /// 判断线段 AB 与线段 CD 是否相交，如果相交返回交点。
    /// 使用 SDF 符号距离法的变体，保持与 SlicerCore 一致的鲁棒性。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool SegmentSegmentIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d, out Vector2 intersection)
    {
        intersection = Vector2.zero;

        // 以 CD 为基准平面
        Vector2 dirCD = d - c;
        float lenCDSq = dirCD.sqrMagnitude;
        if (lenCDSq < EPSILON_SQ) return false;

        Vector2 normalCD = new Vector2(-dirCD.y, dirCD.x);
        float distA = Vector2.Dot(normalCD, a - c);
        float distB = Vector2.Dot(normalCD, b - c);

        int signA = distA > 0f ? 1 : -1;
        int signB = distB > 0f ? 1 : -1;
        if (signA == signB) return false;

        // A 和 B 分别在 CD 平面两侧，算交点
        float u = distA / (distA - distB);
        Vector2 p = a + u * (b - a);

        // 验证交点是否落在 CD 线段的范围内
        float t = Vector2.Dot(p - c, dirCD) / lenCDSq;
        if (t < -EPSILON || t > 1f + EPSILON) return false;

        // 还要验证交点是否落在 AB 线段范围内
        Vector2 dirAB = b - a;
        float lenABSq = dirAB.sqrMagnitude;
        if (lenABSq < EPSILON_SQ) return false;
        float s = Vector2.Dot(p - a, dirAB) / lenABSq;
        if (s < -EPSILON || s > 1f + EPSILON) return false;

        intersection = p;
        return true;
    }

    /// <summary>
    /// 线段 AB 与线段 CD 相交检测，同时返回交点在两条线段上的参数 t 和 s。
    /// t: 交点在 AB 上的进度 [0,1]
    /// s: 交点在 CD 上的进度 [0,1]
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool SegmentSegmentIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d,
        out Vector2 intersection, out float tAB, out float sCD)
    {
        intersection = Vector2.zero;
        tAB = 0f;
        sCD = 0f;

        Vector2 dirCD = d - c;
        float lenCDSq = dirCD.sqrMagnitude;
        if (lenCDSq < EPSILON_SQ) return false;

        Vector2 normalCD = new Vector2(-dirCD.y, dirCD.x);
        float distA = Vector2.Dot(normalCD, a - c);
        float distB = Vector2.Dot(normalCD, b - c);

        int signA = distA > 0f ? 1 : -1;
        int signB = distB > 0f ? 1 : -1;
        if (signA == signB) return false;

        float u = distA / (distA - distB);
        Vector2 p = a + u * (b - a);

        float t = Vector2.Dot(p - c, dirCD) / lenCDSq;
        if (t < -EPSILON || t > 1f + EPSILON) return false;

        Vector2 dirAB = b - a;
        float lenABSq = dirAB.sqrMagnitude;
        if (lenABSq < EPSILON_SQ) return false;
        float s = Vector2.Dot(p - a, dirAB) / lenABSq;
        if (s < -EPSILON || s > 1f + EPSILON) return false;

        intersection = p;
        tAB = Mathf.Clamp01(s);
        sCD = Mathf.Clamp01(t);
        return true;
    }

    // =========================================================================
    //                   Point-in-Polygon 射线法
    // =========================================================================

    /// <summary>
    /// 使用射线投射法判定一个点是否在多边形内部。
    /// </summary>
    public static bool PointInPolygon(Vector2 point, List<Vector2> polygon)
    {
        if (polygon == null || polygon.Count < 3) return false;

        bool inside = false;
        int count = polygon.Count;
        for (int i = 0, j = count - 1; i < count; j = i++)
        {
            Vector2 pi = polygon[i];
            Vector2 pj = polygon[j];

            if ((pi.y > point.y) != (pj.y > point.y) &&
                point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
            {
                inside = !inside;
            }
        }
        return inside;
    }

    // =========================================================================
    //                   辅助几何函数
    // =========================================================================

    /// <summary>
    /// 点到线段的最短距离的平方。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PointToSegmentDistanceSq(Vector2 point, Vector2 segA, Vector2 segB)
    {
        Vector2 ab = segB - segA;
        float abLenSq = ab.sqrMagnitude;

        if (abLenSq < EPSILON_SQ)
        {
            return (point - segA).sqrMagnitude;
        }

        float t = Mathf.Clamp01(Vector2.Dot(point - segA, ab) / abLenSq);
        Vector2 projection = segA + t * ab;
        return (point - projection).sqrMagnitude;
    }

    /// <summary>
    /// 计算折线路径的总弧长。
    /// </summary>
    public static float PolylineLength(List<Vector2> path)
    {
        if (path == null || path.Count < 2) return 0f;
        float length = 0f;
        for (int i = 1; i < path.Count; i++)
        {
            length += Vector2.Distance(path[i - 1], path[i]);
        }
        return length;
    }

    /// <summary>
    /// 确保路径点之间的最小间距。移除距离过近的冗余点。
    /// </summary>
    public static void EnforceMinimumSpacing(List<Vector2> path, float minDistSq = 0.0001f)
    {
        if (path == null || path.Count < 2) return;

        int writeIdx = 1;
        for (int i = 1; i < path.Count; i++)
        {
            if ((path[i] - path[writeIdx - 1]).sqrMagnitude > minDistSq)
            {
                path[writeIdx++] = path[i];
            }
        }
        // 保留最后一个点（即使它和倒数第二个很近）
        if (writeIdx > 1 && writeIdx < path.Count)
        {
            path[writeIdx - 1] = path[path.Count - 1];
        }
        if (writeIdx < path.Count)
        {
            path.RemoveRange(writeIdx, path.Count - writeIdx);
        }
    }
}
