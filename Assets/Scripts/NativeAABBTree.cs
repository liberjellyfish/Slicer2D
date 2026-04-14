using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using System;

/// <summary>
/// 高性能静态 AABB 树 (High-Performance Static AABB Tree)
/// <para>
/// 设计目标：
/// 1. 零 GC (Zero Garbage Collection)：使用 NativeArray 替代托管数组，构建过程中不产生堆内存分配。
/// 2. 缓存友好 (Cache Friendly)：节点在内存中连续存放，提高 CPU 缓存命中率。
/// 3. SIMD 友好：叶子节点批量存储 4 条边，便于后续扩展 SIMD 指令集优化。
/// </para>
/// </summary>
public struct NativeAABBTree : IDisposable
{
    /// <summary>
    /// 扁平化树节点 (32 bytes)
    /// 不使用 Class，而是 Struct，直接存放在大数组中。
    /// </summary>
    public struct FlatNode
    {
        // AABB 包围盒数据
        public float minX, minY, maxX, maxY;

        // 子节点索引 (替代传统树的 Left/Right 指针)
        public int leftChildIndex;
        public int rightChildIndex;

        // 叶子节点数据 (仅叶子节点有效)
        // 指向 segments 数组中的起始位置和长度
        public int segmentStartIndex;
        public int segmentCount;

        /// <summary>
        /// 快速检测 AABB 重叠 (内联优化)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IntersectsBox(Vector2 p1, Vector2 p2)
        {
            // 计算线段的 AABB
            float sMinX = p1.x < p2.x ? p1.x : p2.x;
            float sMaxX = p1.x > p2.x ? p1.x : p2.x;

            // 分离轴定理 (SAT) 的简化版：AABB 不重叠
            if (sMinX > maxX || sMaxX < minX) return false;

            float sMinY = p1.y < p2.y ? p1.y : p2.y;
            float sMaxY = p1.y > p2.y ? p1.y : p2.y;
            if (sMinY > maxY || sMaxY < minY) return false;

            return true;
        }
    }

    /// <summary>
    /// 原始线段数据
    /// 包含预计算的包围盒，避免运行时重复计算
    /// </summary>
    public struct Segment
    {
        public Vector2 P1, P2;
        public float minX, minY, maxX, maxY;
        public int PathId;
        public int EdgeIdx;
    }

    // === 核心数据存储 ===
    // 使用 Allocator.Temp 极速分配的多线程安全内存池
    private NativeArray<FlatNode> nodes;
    private NativeArray<Segment> segments; 
    private int nodesUsed;

    // 叶子容量阈值：小于此数量不再分裂
    // 4 是经验值，平衡树深度与线性遍历开销
    private const int MAX_LEAF_SIZE = 4;

    /// <summary>
    /// 构建 AABB 树 (O(N log N))
    /// </summary>
    /// <param name="outerLoop">外圈顶点</param>
    /// <param name="holes">孔洞列表</param>
    public void Build(List<Vector2> outerLoop, List<List<Vector2>> holes)
    {
        // 1. 统计总边数，一次性分配内存，避免 Resize
        int totalEdges = outerLoop.Count;
        if (holes != null)
        {
            for (int i = 0; i < holes.Count; i++) totalEdges += holes[i].Count;
        }

        // 分配物理内存池 (单帧结束前必须 Dispose)
        segments = new NativeArray<Segment>(totalEdges, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        nodes = new NativeArray<FlatNode>(totalEdges * 2, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

        // 2. 填充线段数组 (此时是乱序的)
        int ptr = 0;
        AddSegmentsFromLoop(outerLoop, ref ptr, 0);
        if (holes != null)
        {
            for (int i = 0; i < holes.Count; i++) AddSegmentsFromLoop(holes[i], ref ptr, i + 1);
        }

        // 3. 递归建树
        nodesUsed = 0;
        if (totalEdges > 0)
        {
            BuildRecursive(0, totalEdges);
        }
    }

    // 将环路顶点转换为线段存入数组
    private void AddSegmentsFromLoop(List<Vector2> loop, ref int ptr, int pathId)
    {
        int count = loop.Count;
        if (count < 2) return;

        for (int i = 0; i < count; i++)
        {
            if (ptr >= segments.Length) break;

            Vector2 p1 = loop[i];
            Vector2 p2 = loop[(i + 1) % count];

            // 存入数据
            Segment s = new Segment();
            s.P1 = p1;
            s.P2 = p2;

            // 预计算 Min/Max，加速后续的 Partition 过程
            s.minX = p1.x < p2.x ? p1.x : p2.x;
            s.minY = p1.y < p2.y ? p1.y : p2.y;
            s.maxX = p1.x > p2.x ? p1.x : p2.x;
            s.maxY = p1.y > p2.y ? p1.y : p2.y;
            s.PathId = pathId;
            s.EdgeIdx = i;
            
            segments[ptr] = s;
            ptr++;
        }
    }

    /// <summary>
    /// 递归构建函数 (In-Place Partitioning)
    /// 核心优化：直接在 segments 数组上进行交换排序，不使用任何临时 List
    /// </summary>
    private int BuildRecursive(int start, int count)
    {
        // 分配节点 ID
        int nodeIndex = nodesUsed++;

        // 1. 计算当前集合的总包围盒
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;

        for (int i = 0; i < count; i++)
        {
            Segment s = segments[start + i];
            if (s.minX < minX) minX = s.minX;
            if (s.minY < minY) minY = s.minY;
            if (s.maxX > maxX) maxX = s.maxX;
            if (s.maxY > maxY) maxY = s.maxY;
        }

        FlatNode node = new FlatNode();
        node.minX = minX;
        node.minY = minY;
        node.maxX = maxX;
        node.maxY = maxY;

        // 2. 叶子节点判定 (Base Case)
        if (count <= MAX_LEAF_SIZE)
        {
            node.segmentStartIndex = start;
            node.segmentCount = count;
            node.leftChildIndex = -1;
            node.rightChildIndex = -1;
            nodes[nodeIndex] = node;
            return nodeIndex;
        }

        node.segmentCount = 0; // 标记为非叶子
        nodes[nodeIndex] = node; // 占位保存，后续再更新 left/right

        // 3. 空间划分 (Spatial Partitioning)
        // 选择最长轴进行分割，使树更平衡
        bool splitX = (maxX - minX) > (maxY - minY);
        float mid = splitX ? (minX + maxX) * 0.5f : (minY + maxY) * 0.5f;

        // 4. 原地划分算法 (类似 QuickSort 的 Partition)
        // 将小于中点的线段换到左边，大于的换到右边
        int left = start;
        int right = start + count - 1;
        while (left <= right)
        {
            float center = splitX
                ? (segments[left].minX + segments[left].maxX) * 0.5f
                : (segments[left].minY + segments[left].maxY) * 0.5f;

            if (center < mid)
            {
                left++;
            }
            else
            {
                // Swap
                Segment temp = segments[left];
                segments[left] = segments[right];
                segments[right] = temp;
                right--;
            }
        }

        int leftCount = left - start;
        // 边界保护：如果所有线段中心点重合，强制对半分，防止死循环
        if (leftCount == 0 || leftCount == count) leftCount = count / 2;

        // 5. 递归
        int leftChild = BuildRecursive(start, leftCount);
        int rightChild = BuildRecursive(start + leftCount, count - leftCount);

        node = nodes[nodeIndex];
        node.leftChildIndex = leftChild;
        node.rightChildIndex = rightChild;
        nodes[nodeIndex] = node;

        return nodeIndex;
    }

    /// <summary>
    /// 查询线段是否与树中任意线段相交 (O(log N))
    /// </summary>
    public bool Intersects(Vector2 p1, Vector2 p2)
    {
        if (nodesUsed == 0) return false;
        return IntersectsRecursive(0, p1, p2);
    }

    private bool IntersectsRecursive(int nodeIdx, Vector2 p1, Vector2 p2)
    {
        // 1. AABB 剔除 (Pruning)
        if (!nodes[nodeIdx].IntersectsBox(p1, p2)) return false;

        // 2. 叶子节点处理
        if (nodes[nodeIdx].segmentCount > 0)
        {
            int start = nodes[nodeIdx].segmentStartIndex;
            int end = start + nodes[nodeIdx].segmentCount;
            // 暴力遍历叶子内的几条边
            for (int i = start; i < end; i++)
            {
                Segment s = segments[i];

                // 排除共享顶点 (如果连接点本身就在墙上，不算撞墙)
                if (IsSamePoint(s.P1, p1) || IsSamePoint(s.P1, p2) ||
                    IsSamePoint(s.P2, p1) || IsSamePoint(s.P2, p2)) continue;

                if (SegmentsIntersect(p1, p2, s.P1, s.P2)) return true;
            }
            return false;
        }

        // 3. 递归查询
        if (IntersectsRecursive(nodes[nodeIdx].leftChildIndex, p1, p2)) return true;
        if (IntersectsRecursive(nodes[nodeIdx].rightChildIndex, p1, p2)) return true;

        return false;
    }

    /// <summary>
    /// 查询给定 AABB 内可能重叠的所有线段 (用于预过滤)
    /// </summary>
    public void QueryOverlap(Vector2 p1, Vector2 p2, float padding, List<Segment> results)
    {
        if (nodesUsed == 0) return;
        
        float sMinX = p1.x < p2.x ? p1.x : p2.x;
        float sMaxX = p1.x > p2.x ? p1.x : p2.x;
        float sMinY = p1.y < p2.y ? p1.y : p2.y;
        float sMaxY = p1.y > p2.y ? p1.y : p2.y;

        sMinX -= padding; sMaxX += padding;
        sMinY -= padding; sMaxY += padding;

        QueryOverlapRecursive(0, sMinX, sMinY, sMaxX, sMaxY, results);
    }

    private void QueryOverlapRecursive(int nodeIdx, float sMinX, float sMinY, float sMaxX, float sMaxY, List<Segment> results)
    {
        FlatNode node = nodes[nodeIdx];
        if (sMinX > node.maxX || sMaxX < node.minX || sMinY > node.maxY || sMaxY < node.minY) return;

        if (node.segmentCount > 0)
        {
            int start = node.segmentStartIndex;
            int end = start + node.segmentCount;
            for (int i = start; i < end; i++)
            {
                Segment s = segments[i];
                if (!(sMinX > s.maxX || sMaxX < s.minX || sMinY > s.maxY || sMaxY < s.minY))
                {
                    results.Add(s);
                }
            }
            return;
        }

        if (node.leftChildIndex != -1) QueryOverlapRecursive(node.leftChildIndex, sMinX, sMinY, sMaxX, sMaxY, results);
        if (node.rightChildIndex != -1) QueryOverlapRecursive(node.rightChildIndex, sMinX, sMinY, sMaxX, sMaxY, results);
    }

    // 增加容差到 1e-7f，处理 float 精度问题
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSamePoint(Vector2 a, Vector2 b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return (dx * dx + dy * dy) < 1e-7f;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (den == 0) return false;

        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;

        // 严格内部相交 (不包含端点)，防止误判邻边
        return (u > 1e-5f && u < 1f - 1e-5f && v > 1e-5f && v < 1f - 1e-5f);
    }

    public void Dispose()
    {
        if (nodes.IsCreated) nodes.Dispose();
        if (segments.IsCreated) segments.Dispose();
    }
}