using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

/// <summary>
/// 高性能三角剖分器 (Grid-Accelerated & Candidate-Cached Ear Clipping)
/// <para>
/// 核心优化策略：
/// 1. 空间哈希 (Uniform Grid)：将 IsEar 检测从 O(N) 降至 O(1)。
/// 2. 候选列表 (Ear Candidates)：维护一个凸点列表，将寻找下一个耳朵的复杂度从 O(N) 降至 O(1)。
///    - 初始化时：识别所有凸点加入列表。
///    - 运行时：直接从列表中取点判断。
///    - 更新时：切掉顶点后，只检查其邻居是否变成了新的候选耳。
/// 3. 总时间复杂度：稳定在 O(N)。
/// </para>
/// </summary>
public static class Triangulator
{
    // =================================================================================
    //                                  内部数据结构
    // =================================================================================

    /// <summary>
    /// 双向链表节点
    /// </summary>
    private class VertexNode
    {
        public Vector2 position;
        public int index;        // 原始索引

        // 几何属性
        public bool isReflex;    // 是否为凹点
        public bool isCandidate; // 是否已在耳朵候选列表中 (防止重复添加)

        // 拓扑指针
        public VertexNode prev;
        public VertexNode next;

        // 空间索引指针 (Grid Bucket)
        public VertexNode nextInGrid;

        public VertexNode(Vector2 pos, int idx)
        {
            position = pos;
            index = idx;
            isReflex = false;
            isCandidate = false;
        }
    }

    /// <summary>
    /// 均匀网格索引 (Uniform Grid)
    /// 用于快速查询三角形内部是否包含凹点
    /// </summary>
    private class UniformGrid
    {
        public VertexNode[] cells;
        public int cols;
        public int rows;
        public float minX, minY;
        public float invCellSize;

        public void Initialize(List<VertexNode> nodes, int count)
        {
            // 1. 计算包围盒
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            for (int i = 0; i < count; i++)
            {
                Vector2 p = nodes[i].position;
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            // 2. 自适应网格尺寸
            float width = maxX - minX;
            float height = maxY - minY;

            if (width < 0.001f) width = 0.001f;
            if (height < 0.001f) height = 0.001f;

            float area = width * height;
            // 目标：GridCount ≈ NodeCount (Density ≈ 1)
            float cellSize = Mathf.Sqrt(area / (count + 1));
            if (cellSize < 0.0001f) cellSize = 0.0001f;

            this.invCellSize = 1.0f / cellSize;
            this.cols = Mathf.CeilToInt(width * invCellSize) + 1;
            this.rows = Mathf.CeilToInt(height * invCellSize) + 1;

            // 内存熔断保护 (20万格子 ≈ 1.6MB)
            if (cols * rows > 200000)
            {
                float ratio = Mathf.Sqrt(200000f / (cols * rows));
                cellSize /= ratio;
                this.invCellSize = 1.0f / cellSize;
                this.cols = Mathf.CeilToInt(width * invCellSize) + 1;
                this.rows = Mathf.CeilToInt(height * invCellSize) + 1;
            }

            this.cells = new VertexNode[cols * rows];
            this.minX = minX;
            this.minY = minY;

            // 3. 填充网格
            for (int i = 0; i < count; i++)
            {
                VertexNode node = nodes[i];
                int idx = GetCellIndex(node.position);
                if (idx >= 0 && idx < cells.Length)
                {
                    node.nextInGrid = cells[idx];
                    cells[idx] = node;
                }
            }
        }

        public void Remove(VertexNode node)
        {
            int idx = GetCellIndex(node.position);
            if (idx < 0 || idx >= cells.Length) return;

            VertexNode curr = cells[idx];
            VertexNode prev = null;

            while (curr != null)
            {
                if (curr == node)
                {
                    if (prev == null) cells[idx] = curr.nextInGrid;
                    else prev.nextInGrid = curr.nextInGrid;
                    return;
                }
                prev = curr;
                curr = curr.nextInGrid;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetCellIndex(Vector2 pos)
        {
            int x = (int)((pos.x - minX) * invCellSize);
            int y = (int)((pos.y - minY) * invCellSize);

            if (x < 0) x = 0; else if (x >= cols) x = cols - 1;
            if (y < 0) y = 0; else if (y >= rows) y = rows - 1;
            return y * cols + x;
        }
    }

    // =================================================================================
    //                                  核心算法入口
    // =================================================================================

    public static int[] Triangulate(Vector2[] vertices)
    {
        int n = vertices.Length;
        if (n < 3) return new int[0];

        // 1. 构建链表 (O(N))
        List<VertexNode> nodeList = new List<VertexNode>(n);
        for (int i = 0; i < n; i++)
        {
            nodeList.Add(new VertexNode(vertices[i], i));
        }
        for (int i = 0; i < n; i++)
        {
            nodeList[i].prev = nodeList[(i - 1 + n) % n];
            nodeList[i].next = nodeList[(i + 1) % n];
        }

        // 2. 绕序修正 (Winding Order)
        // 保证多边形为逆时针 (CCW)
        float area = 0;
        for (int i = 0; i < n; i++)
        {
            Vector2 p1 = nodeList[i].position;
            Vector2 p2 = nodeList[i].next.position;
            area += (p2.x - p1.x) * (p2.y + p1.y);
        }
        if (area > 0) // Area > 0 为顺时针(CW)，需要翻转
        {
            for (int i = 0; i < n; i++)
            {
                VertexNode node = nodeList[i];
                VertexNode temp = node.prev;
                node.prev = node.next;
                node.next = temp;
            }
        }

        // 3. 识别凹凸性并构建初始列表 (O(N))
        List<VertexNode> reflexVertices = new List<VertexNode>(n / 2);
        List<VertexNode> earCandidates = new List<VertexNode>(n); // 耳朵候选列表 (Convex Vertices)

        VertexNode current = nodeList[0];
        VertexNode start = current;
        do
        {
            if (IsReflex(current))
            {
                current.isReflex = true;
                reflexVertices.Add(current);
            }
            else
            {
                // 凸点是潜在的耳朵
                current.isReflex = false;
                current.isCandidate = true;
                earCandidates.Add(current);
            }
            current = current.next;
        } while (current != start);

        // 4. 构建空间加速网格 (O(R))
        UniformGrid grid = new UniformGrid();
        grid.Initialize(reflexVertices, reflexVertices.Count);

        // 5. 耳切主循环 (O(N))
        // 优化后：不再遍历链表寻找耳朵，而是直接从 candidates 列表中取
        List<int> triangles = new List<int>((n - 2) * 3);
        int pointCount = n;

        // 双指针用于高效遍历 List 充当 Queue/Stack
        // 我们从列表末尾取，这样 RemoveAt 也是 O(1)
        while (pointCount > 3 && earCandidates.Count > 0)
        {
            // 从候选列表中取出一个点 (LIFO 策略，通常能保持较好的局部性)
            int candidateIdx = earCandidates.Count - 1;
            VertexNode candidate = earCandidates[candidateIdx];
            earCandidates.RemoveAt(candidateIdx);

            candidate.isCandidate = false; // 移除标记

            // 验证是否真的是耳朵 (Grid Check)
            // 注意：候选列表里的点只是凸点，不一定是耳朵（可能包含凹点），所以必须 Check
            if (IsEar(candidate, grid))
            {
                // --- 切耳朵 ---
                VertexNode prev = candidate.prev;
                VertexNode next = candidate.next;

                triangles.Add(prev.index);
                triangles.Add(candidate.index);
                triangles.Add(next.index);

                // 拓扑移除
                prev.next = next;
                next.prev = prev;

                pointCount--;

                // 如果被切掉的点本身在网格里（理论上凸点不在，但为了安全...）
                if (candidate.isReflex) grid.Remove(candidate);

                // --- 邻居更新 (Critical Step) ---
                // 切掉中间点后，prev 和 next 的角度会变大 (变得更凸)
                // 我们需要重新检查它们的凹凸性

                // 检查 Prev
                bool wasReflex = prev.isReflex;
                if (IsReflex(prev))
                {
                    prev.isReflex = true; // 依然是凹点，或者从凸变凹(极少见但理论可能)
                }
                else
                {
                    // 变成了凸点 (Convex)
                    prev.isReflex = false;
                    // 如果之前是凹点，现在变凸了，从网格移除
                    if (wasReflex) grid.Remove(prev);

                    // 如果不在候选列表中，加入列表
                    if (!prev.isCandidate)
                    {
                        prev.isCandidate = true;
                        earCandidates.Add(prev);
                    }
                }

                // 检查 Next
                wasReflex = next.isReflex;
                if (IsReflex(next))
                {
                    next.isReflex = true;
                }
                else
                {
                    next.isReflex = false;
                    if (wasReflex) grid.Remove(next);

                    if (!next.isCandidate)
                    {
                        next.isCandidate = true;
                        earCandidates.Add(next);
                    }
                }
            }
            else
            {
                // 如果是凸点但不是耳朵（被凹点阻挡），它依然有可能在未来成为耳朵。
                // 但为了避免死循环和重复检查，我们暂时把它移出列表。
                // 只有当它的邻居被切掉时，它才会被重新加入列表进行检查。
                // 这就是 O(N) 的奥义：失败的检查不会立即重试。
            }
        }

        // 处理最后剩下的 3 个点
        if (pointCount == 3)
        {
            // 剩下的三个点必然构成最后一个三角形
            // 此时链表中任意取一个点即可，比如 current.prev
            VertexNode survivor = null;
            survivor = earCandidates[0];
            if (survivor != null)
            {
                triangles.Add(survivor.prev.index);
                triangles.Add(survivor.index);
                triangles.Add(survivor.next.index);
            }
        }

        return triangles.ToArray();
    }

    // =================================================================================
    //                                  几何判定函数
    // =================================================================================

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsReflex(VertexNode v)
    {
        Vector2 a = v.prev.position;
        Vector2 b = v.position;
        Vector2 c = v.next.position;
        // 叉积 <= 0 表示右转或共线 (CCW下为凹点)
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) <= 0;
    }

    private static bool IsEar(VertexNode v, UniformGrid grid)
    {
        // 1. 只有凸点才能是耳尖
        if (v.isReflex) return false;

        Vector2 a = v.prev.position;
        Vector2 b = v.position;
        Vector2 c = v.next.position;

        // 2. 网格查询
        float minX = a.x; if (b.x < minX) minX = b.x; if (c.x < minX) minX = c.x;
        float maxX = a.x; if (b.x > maxX) maxX = b.x; if (c.x > maxX) maxX = c.x;
        float minY = a.y; if (b.y < minY) minY = b.y; if (c.y < minY) minY = c.y;
        float maxY = a.y; if (b.y > maxY) maxY = b.y; if (c.y > maxY) maxY = c.y;

        int startX = (int)((minX - grid.minX) * grid.invCellSize);
        int endX = (int)((maxX - grid.minX) * grid.invCellSize);
        int startY = (int)((minY - grid.minY) * grid.invCellSize);
        int endY = (int)((maxY - grid.minY) * grid.invCellSize);

        if (startX < 0) startX = 0; if (endX >= grid.cols) endX = grid.cols - 1;
        if (startY < 0) startY = 0; if (endY >= grid.rows) endY = grid.rows - 1;

        for (int y = startY; y <= endY; y++)
        {
            int offset = y * grid.cols;
            for (int x = startX; x <= endX; x++)
            {
                VertexNode node = grid.cells[offset + x];
                while (node != null)
                {
                    // 排除三角形自身顶点
                    if (node == v.prev || node == v.next)
                    {
                        node = node.nextInGrid;
                        continue;
                    }

                    // 鲁棒性：排除重合点 (搭桥法产生的缝合点)
                    float d2a = (node.position - a).sqrMagnitude;
                    float d2b = (node.position - b).sqrMagnitude;
                    float d2c = (node.position - c).sqrMagnitude;

                    // 1e-6f 是经验阈值
                    if (d2a < 1e-6f || d2b < 1e-6f || d2c < 1e-6f)
                    {
                        node = node.nextInGrid;
                        continue;
                    }

                    // 点在三角形内测试
                    if (IsPointInTriangle(a, b, c, node.position)) return false;

                    node = node.nextInGrid;
                }
            }
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointInTriangle(Vector2 a, Vector2 b, Vector2 c, Vector2 p)
    {
        bool check1 = ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) >= 0;
        bool check2 = ((c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x)) >= 0;
        bool check3 = ((a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x)) >= 0;
        return check1 && check2 && check3;
    }
}