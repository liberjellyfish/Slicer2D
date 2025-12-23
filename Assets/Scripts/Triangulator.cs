using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

/// <summary>
/// 高性能三角剖分器 (Grid-Accelerated Ear Clipping)
/// <para>
/// 算法原理：
/// 标准的耳切法 (Ear Clipping) 时间复杂度为 O(N^2)，瓶颈在于判断一个凸点是否为"耳朵"时，
/// 需要遍历所有凹点来检查是否包含。
/// 本脚本通过引入"均匀网格 (Uniform Grid)"作为空间加速结构：
/// 1. 将空间划分为若干格子 (Spatial Hashing)。
/// 2. 在几何查询时，只检索三角形包围盒覆盖的格子内的凹点。
/// 3. 将平均查询复杂度从 O(N) 降至 O(1)，总体算法复杂度优化至接近 O(N)。
/// </para>
/// </summary>
public static class Triangulator
{
    // =================================================================================
    //                                  内部数据结构
    // =================================================================================

    /// <summary>
    /// 双向循环链表节点
    /// <para>使用链表是为了支持 O(1) 的节点移除操作，这是耳切法拓扑更新的基础。</para>
    /// </summary>
    private class VertexNode
    {
        // 几何数据
        public Vector2 position; // 顶点坐标
        public int index;        // 原始顶点数组中的索引 (用于输出)

        // 拓扑数据
        public bool isReflex;    // 是否为凹点 (Reflex Vertex)
        public VertexNode prev;  // 前驱节点
        public VertexNode next;  // 后继节点

        // 空间索引数据
        public VertexNode nextInGrid; // 网格桶内的链表指针 (解决哈希冲突)

        public VertexNode(Vector2 pos, int idx)
        {
            position = pos;
            index = idx;
            isReflex = false;
        }
    }

    /// <summary>
    /// 均匀网格索引 (Spatial Acceleration Structure)
    /// </summary>
    private class UniformGrid
    {
        private VertexNode[] cells; // 扁平化网格数组
        private int cols;           // 列数
        private int rows;           // 行数
        private float minX, minY;  // 网格原点
        private float cellSize;     // 格子边长
        private float invCellSize;  // 1 / 边长 (乘法比除法快)

        /// <summary>
        /// 初始化网格并构建空间哈希
        /// </summary>
        /// <param name="nodes">待索引的凹点列表</param>
        /// <param name="count">凹点数量</param>
        public void Initialize(List<VertexNode> nodes, int count)
        {
            // 1. 计算包围盒 (AABB)
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            foreach (var node in nodes)
            {
                if (node.position.x < minX) minX = node.position.x;
                if (node.position.x > maxX) maxX = node.position.x;
                if (node.position.y < minY) minY = node.position.y;
                if (node.position.y > maxY) maxY = node.position.y;
            }

            // 2. [核心算法] 确定最佳网格尺寸 (自适应分块策略)
            float width = maxX - minX;
            float height = maxY - minY;

            // 容错：防止共线或单点导致的零面积
            if (width < 0.001f) width = 0.001f;
            if (height < 0.001f) height = 0.001f;

            float area = width * height;

            // === 黄金公式 ===
            // 目标：让网格单元总数 (Cells) 大致等于 凹点总数 (count)
            // 推导：Density = Count / (Cols * Rows) ≈ 1
            // 这样既避免了格子太少退化成 O(N)，也避免了格子太多导致遍历空格子开销大
            // CellSize = Sqrt(TotalArea / TargetCellCount)
            cellSize = Mathf.Sqrt(area / (count + 1));

            // 限制最小尺寸，防止浮点下溢
            if (cellSize < 0.0001f) cellSize = 0.0001f;

            invCellSize = 1.0f / cellSize;

            // 计算行列数
            cols = Mathf.CeilToInt(width * invCellSize) + 1;
            rows = Mathf.CeilToInt(height * invCellSize) + 1;

            // === 工程保护：内存熔断 ===
            // 某些极端几何体（如极其稀疏的巨大空框）会导致计算出上百万个空格子
            // 我们设置一个硬上限（20万个格子），这大约占用 800KB-1.5MB 内存，非常安全
            const int MAX_GRID_CELLS = 200000;
            if (cols * rows > MAX_GRID_CELLS)
            {
                // 降低分辨率，强行压缩到 MAX_GRID_CELLS 以内
                float ratio = Mathf.Sqrt((float)MAX_GRID_CELLS / (cols * rows));
                cellSize /= ratio; // 增大格子尺寸
                invCellSize = 1.0f / cellSize;

                cols = Mathf.CeilToInt(width * invCellSize) + 1;
                rows = Mathf.CeilToInt(height * invCellSize) + 1;
            }

            cells = new VertexNode[cols * rows];
            this.minX = minX;
            this.minY = minY;

            // 3. 填入数据 (Spatial Hashing)
            foreach (var node in nodes)
            {
                int idx = GetCellIndex(node.position);
                if (idx >= 0 && idx < cells.Length)
                {
                    // 使用"头插法"构建桶内链表，解决哈希冲突 (O(1))
                    node.nextInGrid = cells[idx];
                    cells[idx] = node;
                }
            }
        }

        /// <summary>
        /// 从网格中移除节点 (当凹点变成凸点时调用)
        /// </summary>
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

        /// <summary>
        /// 查询三角形包围盒覆盖的所有网格中的点
        /// <para>这是一个迭代器，避免了分配 List 带来的 GC</para>
        /// </summary>
        public IEnumerable<VertexNode> QueryTriangle(Vector2 a, Vector2 b, Vector2 c)
        {
            // 快速 AABB 裁剪
            float minX = a.x; if (b.x < minX) minX = b.x; if (c.x < minX) minX = c.x;
            float maxX = a.x; if (b.x > maxX) maxX = b.x; if (c.x > maxX) maxX = c.x;
            float minY = a.y; if (b.y < minY) minY = b.y; if (c.y < minY) minY = c.y;
            float maxY = a.y; if (b.y > maxY) maxY = b.y; if (c.y > maxY) maxY = c.y;

            int startX = (int)((minX - this.minX) * invCellSize);
            int endX = (int)((maxX - this.minX) * invCellSize);
            int startY = (int)((minY - this.minY) * invCellSize);
            int endY = (int)((maxY - this.minY) * invCellSize);

            // 边界截断
            if (startX < 0) startX = 0; if (endX >= cols) endX = cols - 1;
            if (startY < 0) startY = 0; if (endY >= rows) endY = rows - 1;

            for (int y = startY; y <= endY; y++)
            {
                int offset = y * cols;
                for (int x = startX; x <= endX; x++)
                {
                    VertexNode node = cells[offset + x];
                    while (node != null)
                    {
                        yield return node;
                        node = node.nextInGrid;
                    }
                }
            }
        }

        private int GetCellIndex(Vector2 pos)
        {
            int x = (int)((pos.x - minX) * invCellSize);
            int y = (int)((pos.y - minY) * invCellSize);

            // 增加一点容错，处理边界上的点
            if (x < 0) x = 0; else if (x >= cols) x = cols - 1;
            if (y < 0) y = 0; else if (y >= rows) y = rows - 1;
            return y * cols + x;
        }
    }

    // =================================================================================
    //                                  核心剖分逻辑
    // =================================================================================

    /// <summary>
    /// 执行三角剖分
    /// </summary>
    /// <param name="vertices">有序的多边形顶点数组</param>
    /// <returns>三角形索引数组</returns>
    public static int[] Triangulate(Vector2[] vertices)
    {
        int n = vertices.Length;
        if (n < 3) return new int[0];

        // 1. 构建双向循环链表 (O(N))
        List<VertexNode> nodes = new List<VertexNode>(n);
        for (int i = 0; i < n; i++) nodes.Add(new VertexNode(vertices[i], i));
        for (int i = 0; i < n; i++)
        {
            nodes[i].prev = nodes[(i - 1 + n) % n];
            nodes[i].next = nodes[(i + 1) % n];
        }

        // 2. 检查并修正绕序 (Winding Order) (O(N))
        // 耳切法要求多边形必须是逆时针 (CCW)
        float area = 0;
        for (int i = 0; i < n; i++)
        {
            Vector2 p1 = nodes[i].position;
            Vector2 p2 = nodes[i].next.position;
            area += (p2.x - p1.x) * (p2.y + p1.y);
        }

        // 梯形公式：Area > 0 表示顺时针 (CW)，需要翻转链表
        if (area > 0)
        {
            for (int i = 0; i < n; i++)
            {
                VertexNode node = nodes[i];
                VertexNode temp = node.prev;
                node.prev = node.next;
                node.next = temp;
            }
        }

        // 3. 识别并缓存凹点 (Reflex Vertices) (O(N))
        // 几何定理：只有凹点才可能位于三角形内部，因此只需索引凹点
        List<VertexNode> reflexVertices = new List<VertexNode>(n / 2);
        VertexNode current = nodes[0];
        VertexNode start = current;
        do
        {
            if (IsReflex(current))
            {
                current.isReflex = true;
                reflexVertices.Add(current);
            }
            current = current.next;
        } while (current != start);

        // 4. 初始化网格加速结构 (O(R))
        UniformGrid grid = new UniformGrid();
        grid.Initialize(reflexVertices, reflexVertices.Count);

        // 5. 耳切主循环 (近似 O(N))
        List<int> triangles = new List<int>((n - 2) * 3);
        int pointCount = n;
        current = nodes[0];

        int iteration = 0;
        int maxIteration = n * 2; // 死循环熔断器 (防止自交或数值精度问题)

        while (pointCount > 3)
        {
            bool earFound = false;
            VertexNode startNode = current;

            // 在当前链表中搜索耳朵
            do
            {
                // 使用网格加速查询
                if (IsEar(current, grid))
                {
                    earFound = true;
                    // --- 核心操作：切下耳朵 ---
                    VertexNode prev = current.prev;
                    VertexNode next = current.next;

                    // 记录三角形索引
                    triangles.Add(prev.index);
                    triangles.Add(current.index);
                    triangles.Add(next.index);

                    // 拓扑移除 (O(1))
                    prev.next = next;
                    next.prev = prev;

                    // 从网格中移除被切掉的点 (如果是凹点)
                    if (current.isReflex) grid.Remove(current);

                    pointCount--;

                    // --- 关键：更新邻居的凹凸性 ---
                    // 切掉中间点后，左右邻居的角度发生变化，可能从凹点变成凸点
                    if (prev.isReflex && !IsReflex(prev))
                    {
                        prev.isReflex = false;
                        grid.Remove(prev); // 必须移除，否则会阻挡合法的耳朵检测
                    }
                    if (next.isReflex && !IsReflex(next))
                    {
                        next.isReflex = false;
                        grid.Remove(next);
                    }

                    // 贪心策略：切掉耳朵后，优先检查邻居，利用数据局部性
                    current = next;
                    break;
                }
                current = current.next;
                iteration++;
            } while (current != startNode && iteration < maxIteration);

            if (!earFound)
            {
                Debug.LogWarning("[Triangulator] 耳切失败。可能原因：多边形自交、退化或浮点精度不足。");
                break;
            }
        }

        // 处理最后剩下的 3 个点 (必然构成最后一个三角形)
        if (pointCount == 3)
        {
            triangles.Add(current.prev.index);
            triangles.Add(current.index);
            triangles.Add(current.next.index);
        }

        return triangles.ToArray();
    }

    // =================================================================================
    //                                  几何判定函数
    // =================================================================================

    /// <summary>
    /// 判断是否为凹点 (Reflex Vertex)
    /// </summary>
    private static bool IsReflex(VertexNode v)
    {
        Vector2 a = v.prev.position;
        Vector2 b = v.position;
        Vector2 c = v.next.position;

        // 2D 叉积 (Cross Product) 判断转向
        // 在 CCW 绕序下：
        // 结果 > 0 : 左转 (凸点 Convex)
        // 结果 <= 0 : 右转或共线 (凹点 Reflex)
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) <= 0;
    }

    /// <summary>
    /// 判断是否为"耳朵" (Ear Tip)
    /// </summary>
    private static bool IsEar(VertexNode v, UniformGrid grid)
    {
        // 1. 只有凸点才能是耳尖
        if (v.isReflex) return false;

        Vector2 a = v.prev.position;
        Vector2 b = v.position;
        Vector2 c = v.next.position;

        // 2. 加速查询：只检索三角形包围盒覆盖的网格单元
        foreach (var r in grid.QueryTriangle(a, b, c))
        {
            // 排除三角形自身的顶点
            if (r == v.prev || r == v || r == v.next) continue;

            // [鲁棒性修复]：处理重合顶点 (Coincident Vertices)
            // 在"搭桥法"中，进洞和出洞的桥接点位置是完全重合的。
            // 必须排除这些位置重合的点，否则会误判为包含凹点。
            float d2a = (r.position - a).sqrMagnitude;
            float d2b = (r.position - b).sqrMagnitude;
            float d2c = (r.position - c).sqrMagnitude;
            if (d2a < 1e-6f || d2b < 1e-6f || d2c < 1e-6f) continue;

            // 只要有一个凹点在三角形内部，它就不是耳朵
            if (IsPointInTriangle(a, b, c, r.position)) return false;
        }

        return true;
    }

    /// <summary>
    /// 判断点 P 是否在三角形 ABC 内部 (内联优化)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointInTriangle(Vector2 a, Vector2 b, Vector2 c, Vector2 p)
    {
        // 使用边测试法 (Edge Side Test)
        // 点 P 必须在 AB, BC, CA 三条边的同一侧 (内侧)
        // 这里的叉积 >= 0 表示在左侧或线上
        bool check1 = ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) >= 0;
        bool check2 = ((c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x)) >= 0;
        bool check3 = ((a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x)) >= 0;
        return check1 && check2 && check3;
    }
}