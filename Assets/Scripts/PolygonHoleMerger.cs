using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// 多边形空洞合并器 (Optimized Hole Merger)
/// <para>
/// 功能：使用"搭桥法" (Bridge Building) 将带洞多边形转换为简单多边形，以便进行三角剖分。
/// 优化策略：
/// 1. 绕序规范化：强制外圈 CCW，孔洞 CW。
/// 2. 静态树加速：使用 NativeAABBTree 将几何查询从 O(N) 降至 O(log N)。
/// 3. 双向链表：将缝合操作从 O(N) 内存搬运降至 O(1) 指针操作。
/// </para>
/// </summary>
public class PolygonHoleMerger
{
    // 双向链表节点，用于 O(1) 插入
    private class ListNode
    {
        public Vector2 Position;
        public ListNode Next;
        public ListNode Prev;
        public ListNode(Vector2 p) { Position = p; }
    }

    // 孔洞元数据
    private struct HoleData
    {
        public ListNode Head;
        public int Count;
        public float MaxX;      // 关键：用于从最右侧开始合并
        public ListNode MaxXNode;
    }

    // 动态生成的"桥"记录
    private struct BridgeSegment
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

        // 0. 规范化绕序 (Winding Order Normalization)
        // 图形学铁律：外实内虚 -> 外圈逆时针(CCW), 孔洞顺时针(CW)
        EnsureWinding(outRing, true);
        for (int i = 0; i < holes.Count; i++) EnsureWinding(holes[i], false);

        // 1. 构建 AABB 树 (包含所有原始边，作为静态阻挡层)
        NativeAABBTree staticWallTree = new NativeAABBTree();
        staticWallTree.Build(outRing, holes);

        // 2. 链表化外圈
        ListNode outerHead = CreateLoop(outRing);

        // 3. 预处理孔洞
        List<HoleData> holeDatas = new List<HoleData>(holes.Count);
        for (int i = 0; i < holes.Count; i++)
        {
            var holePoints = holes[i];
            if (holePoints.Count < 3) continue;

            ListNode head = CreateLoop(holePoints);

            // 寻找 X 坐标最大的点 (MaxX)
            // 策略：往最右边搭桥，通常被阻挡的概率最小
            ListNode curr = head;
            ListNode maxNode = head;
            float maxX = -float.MaxValue;
            int count = 0;
            do
            {
                if (curr.Position.x > maxX)
                {
                    maxX = curr.Position.x;
                    maxNode = curr;
                }
                curr = curr.Next;
                count++;
            } while (curr != head);

            holeDatas.Add(new HoleData { Head = head, Count = count, MaxX = maxX, MaxXNode = maxNode });
        }

        // 4. 排序：优先处理最右边的洞 (O(H log H))
        holeDatas.Sort((a, b) => b.MaxX.CompareTo(a.MaxX));

        List<BridgeSegment> dynamicBridges = new List<BridgeSegment>(holes.Count);

        // 5. 逐个合并
        foreach (var hole in holeDatas)
        {
            Vector2 M = hole.MaxXNode.Position;

            // 寻找最佳连接点 P (O(N_outer * log N_total))
            ListNode bestP = FindBestBridgePoint(M, outerHead, staticWallTree, dynamicBridges);

            if (bestP != null)
            {
                Vector2 P = bestP.Position;
                // 记录新桥，防止后续的洞穿过这条线
                dynamicBridges.Add(new BridgeSegment { A = M, B = P });

                // 执行指针缝合 (Surgery)
                StitchLists(bestP, hole.MaxXNode);
            }
            else
            {
                Debug.LogWarning($"[PolygonHoleMerger] 无法为孔洞找到合法的桥! M点: {M}");
            }
        }

        // 6. 还原为 List (O(N))
        return FlattenList(outerHead);
    }

    /// <summary>
    /// 寻找最佳搭桥点 P
    /// </summary>
    private static ListNode FindBestBridgePoint(
        Vector2 M,
        ListNode outerLoop,
        NativeAABBTree tree,
        List<BridgeSegment> bridges)
    {
        ListNode bestNode = null;
        float minDistSq = float.MaxValue;

        ListNode curr = outerLoop;
        do
        {
            Vector2 P = curr.Position;

            // 1. 几何方向剪枝：只找右侧点 (配合 MaxX 策略)
            if (P.x > M.x)
            {
                float distSq = (P - M).sqrMagnitude;

                // 2. 距离剪枝
                if (distSq < minDistSq)
                {
                    // 3. 昂贵的可见性验证 (Query Tree)
                    if (IsBridgeValid(M, P, tree, bridges))
                    {
                        minDistSq = distSq;
                        bestNode = curr;
                    }
                }
            }
            curr = curr.Next;
        } while (curr != outerLoop);

        return bestNode;
    }

    /// <summary>
    /// 验证桥是否合法 (不撞墙、不撞其他桥)
    /// </summary>
    private static bool IsBridgeValid(Vector2 start, Vector2 end, NativeAABBTree tree, List<BridgeSegment> bridges)
    {
        // 1. 检查静态几何体 (O(log N))
        if (tree.Intersects(start, end)) return false;

        // 2. 检查动态生成的桥 (O(H) - 极小常数)
        int bridgeCount = bridges.Count;
        for (int i = 0; i < bridgeCount; i++)
        {
            BridgeSegment b = bridges[i];
            // 排除共享顶点
            if (IsSamePoint(start, b.A) || IsSamePoint(start, b.B) ||
                IsSamePoint(end, b.A) || IsSamePoint(end, b.B)) continue;

            if (SegmentsIntersect(start, end, b.A, b.B)) return false;
        }
        return true;
    }

    /// <summary>
    /// 核心缝合逻辑：将孔洞链表接入外圈链表
    /// </summary>
    /// <param name="nodeP">外圈上的连接点 P</param>
    /// <param name="nodeM">孔洞上的起始点 M</param>
    private static void StitchLists(ListNode nodeP, ListNode nodeM)
    {
        // 目标拓扑： ... -> P -> M -> (Hole Loop) -> M' -> P' -> P_Next ...
        // P -> M 是进洞，M' -> P' 是出洞

        // 1. 缓存关键连接点
        ListNode pNext = nodeP.Next;
        ListNode mPrev = nodeM.Prev; // 孔洞的逻辑尾部

        // 2. 创建回程节点副本 (模拟几何重合但拓扑不同的边)
        ListNode copyM = new ListNode(nodeM.Position); // M'
        ListNode copyP = new ListNode(nodeP.Position); // P'

        // 3. 指针重连

        // Step A: 外圈 P -> 孔洞 M
        nodeP.Next = nodeM;
        nodeM.Prev = nodeP;

        // Step B: 孔洞尾 Z -> M' (将闭环断开并指向副本)
        mPrev.Next = copyM;
        copyM.Prev = mPrev;

        // Step C: M' -> P' (出洞桥)
        copyM.Next = copyP;
        copyP.Prev = copyM;

        // Step D: P' -> 外圈 P_Next (回归主路)
        copyP.Next = pNext;
        pNext.Prev = copyP;
    }

    // 强制修正绕序 (面积法)
    private static void EnsureWinding(List<Vector2> points, bool targetCCW)
    {
        if (points == null || points.Count < 3) return;

        double area = 0; // double 防止累加溢出
        for (int i = 0; i < points.Count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % points.Count];
            area += (p2.x - p1.x) * (p2.y + p1.y);
        }

        // 梯形公式：Area < 0 为 CCW
        bool isCCW = area < 0;
        if (isCCW != targetCCW)
        {
            points.Reverse();
        }
    }

    private static ListNode CreateLoop(List<Vector2> points)
    {
        if (points.Count == 0) return null;
        ListNode head = new ListNode(points[0]);
        ListNode curr = head;
        for (int i = 1; i < points.Count; i++)
        {
            ListNode n = new ListNode(points[i]);
            curr.Next = n;
            n.Prev = curr;
            curr = n;
        }
        curr.Next = head;
        head.Prev = curr;
        return head;
    }

    private static List<Vector2> FlattenList(ListNode head)
    {
        List<Vector2> result = new List<Vector2>();
        if (head == null) return result;

        ListNode curr = head;
        int safety = 0;
        do
        {
            result.Add(curr.Position);
            curr = curr.Next;
            safety++;
            if (safety > 100000) // 死循环保护
            {
                Debug.LogError("[PolygonHoleMerger] FlattenList 检测到无限循环，强制中断。");
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
}