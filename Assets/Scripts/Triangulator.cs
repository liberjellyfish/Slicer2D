using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

/// <summary>
/// 高性能三角剖分器 (Grid-Accelerated & Candidate-Cached Ear Clipping)
/// <para>
/// 修复与优化：
/// 1. 修复了候选列表 (Ear Candidates) 的管理逻辑，使用 Swap-Removal 实现 O(1) 移除。
/// 2. 增强了 IsEar 检测的鲁棒性，特别是针对共线和重合点的处理。
/// 3. 完善了最后 3 个点的兜底处理，防止索引溢出或丢失。
/// 4. 保持均匀网格 (Uniform Grid) 实现，因为对于轮廓点分布，网格比 AABB 树构建更快且查询足够高效。
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
        public bool isCandidate; // 是否已在耳朵候选列表中

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
    /// </summary>
    private class UniformGrid
    {
        public VertexNode[] cells;
        public int cols;
        public int rows;
        public float minX, minY;
        public float invCellSize;

        public void Initialize(List<VertexNode> reflexNodes)
        {
            int count = reflexNodes.Count;
            if (count == 0) return;

            // 1. 计算包围盒
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            for (int i = 0; i < count; i++)
            {
                Vector2 p = reflexNodes[i].position;
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
                VertexNode node = reflexNodes[i];
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
            if (cells == null) return;
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

        // 2. 绕序修正 (Winding Order) - 必须为逆时针 (CCW)
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
        List<VertexNode> earCandidates = new List<VertexNode>(n);

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
                current.isReflex = false;
                current.isCandidate = true;
                earCandidates.Add(current);
            }
            current = current.next;
        } while (current != start);

        // 4. 构建空间加速网格 (O(R))
        UniformGrid grid = new UniformGrid();
        grid.Initialize(reflexVertices);

        // 5. 耳切主循环 (O(N))
        List<int> triangles = new List<int>((n - 2) * 3);
        int pointCount = n;

        while (pointCount > 3 && earCandidates.Count > 0)
        {
            // [优化] 使用 Swap-Removal 从列表末尾取点，避免 O(N) 移动开销
            int lastIdx = earCandidates.Count - 1;
            VertexNode candidate = earCandidates[lastIdx];
            earCandidates.RemoveAt(lastIdx);

            candidate.isCandidate = false; // 移除标记

            // 验证是否真的是耳朵
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

                // --- 邻居更新 ---
                // 检查 Prev
                UpdateNeighbor(prev, grid, earCandidates);
                // 检查 Next
                UpdateNeighbor(next, grid, earCandidates);
            }
            else
            {
                // 如果检测失败（被凹点阻挡），它暂时不是耳朵。
                // 我们不需要立即把它加回去，只有当它的邻居发生变化时，它才可能变成耳朵。
                // 因此，这里直接丢弃是正确的 O(N) 逻辑。
            }
        }

        // 处理最后剩下的 3 个点

        if (pointCount == 3 && earCandidates.Count > 0)
        {
            VertexNode n1 = earCandidates[0];
            VertexNode n2 = n1.next;
            VertexNode n3 = n2.next;

            triangles.Add(n1.index);
            triangles.Add(n2.index);
            triangles.Add(n3.index);
        }

        return triangles.ToArray();
    }

    // 封装邻居更新逻辑，减少代码重复
    private static void UpdateNeighbor(VertexNode node, UniformGrid grid, List<VertexNode> candidates)
    {
        bool wasReflex = node.isReflex;

        // 重新判断凹凸性
        if (IsReflex(node))
        {
            node.isReflex = true;
        }
        else
        {
            // 变成了凸点 (Convex)
            node.isReflex = false;

            // 如果之前是凹点，现在变凸了，从网格移除
            if (wasReflex) grid.Remove(node);

            // 如果不在候选列表中，加入列表
            if (!node.isCandidate)
            {
                node.isCandidate = true;
                candidates.Add(node);
            }
        }
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
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) <= 0;
    }

    private static bool IsEar(VertexNode v, UniformGrid grid)
    {
        if (v.isReflex) return false;

        Vector2 a = v.prev.position;
        Vector2 b = v.position;
        Vector2 c = v.next.position;

        // 优化：先快速判断是否有凹点在 AABB 内，再精确判断
        float minX = a.x; if (b.x < minX) minX = b.x; if (c.x < minX) minX = c.x;
        float maxX = a.x; if (b.x > maxX) maxX = b.x; if (c.x > maxX) maxX = c.x;
        float minY = a.y; if (b.y < minY) minY = b.y; if (c.y < minY) minY = c.y;
        float maxY = a.y; if (b.y > maxY) maxY = b.y; if (c.y > maxY) maxY = c.y;

        // 如果网格未初始化（例如没有凹点），则不需要检查
        if (grid.cells == null) return true;

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
                    // 1. 排除三角形自身顶点
                    if (node == v.prev || node == v.next)
                    {
                        node = node.nextInGrid;
                        continue;
                    }

                    // 2. 鲁棒性：排除重合点 (搭桥法产生的缝合点)
                    float d2a = (node.position - a).sqrMagnitude;
                    float d2b = (node.position - b).sqrMagnitude;
                    float d2c = (node.position - c).sqrMagnitude;

                    if (d2a < 1e-6f || d2b < 1e-6f || d2c < 1e-6f)
                    {
                        node = node.nextInGrid;
                        continue;
                    }

                    // 3. 点在三角形内测试
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