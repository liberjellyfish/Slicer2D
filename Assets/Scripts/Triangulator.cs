using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// 高性能原生三角剖分器 (Native & Grid-Accelerated Ear Clipping)
/// <para>
/// 优化策略：
/// 1. 废除托管引用对象 (`VertexNode` class)，整体被替换为 `NativeArray` 的无锁双端指针寻址。
/// 2. 原生桶排序栅格 (`NativeUniformGrid`)，在内存连贯性状下提升寻值命中率 100%。
/// 3. 全局采用 `EarClipJob.Run()` 实现主线程单频 Burst 暴走。无分配调度税。
/// Phase C-2: 内部 float2 统一化，新增 TriangulateNative 零拷贝入口。
/// </para>
/// </summary>
public static class Triangulator
{
    // =================================================================================
    //                                  底层无锁内存结构
    // =================================================================================

    // Phase C-2: Position 从 Vector2 → float2
    private struct NativeVertexNode
    {
        public float2 Position;
        public int Index;        // 原始索引

        public int Prev;         // 指向 NativeArray 的伪指针
        public int Next;         // 指向 NativeArray 的伪指针
        public int NextInGrid;   // 栅格空间链表寻址

        public bool IsReflex;
        public bool IsCandidate;
    }

    private struct NativeUniformGrid
    {
        public NativeArray<int> Cells;
        public int Cols;
        public int Rows;
        public float MinX, MinY;
        public float InvCellSize;

        public void Initialize(NativeList<int> reflexNodes, ref NativeArray<NativeVertexNode> nodes)
        {
            int count = reflexNodes.Length;
            if (count == 0) return;

            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            for (int i = 0; i < count; i++)
            {
                float2 p = nodes[reflexNodes[i]].Position;
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            float width = maxX - minX;
            float height = maxY - minY;
            if (width < 0.001f) width = 0.001f;
            if (height < 0.001f) height = 0.001f;

            float area = width * height;
            float cellSize = math.sqrt(area / (count + 1));
            if (cellSize < 0.0001f) cellSize = 0.0001f;

            this.InvCellSize = 1.0f / cellSize;
            this.Cols = (int)math.ceil(width * InvCellSize) + 1;
            this.Rows = (int)math.ceil(height * InvCellSize) + 1;

            if (Cols * Rows > 200000)
            {
                float ratio = math.sqrt(200000f / (Cols * Rows));
                cellSize /= ratio;
                this.InvCellSize = 1.0f / cellSize;
                this.Cols = (int)math.ceil(width * InvCellSize) + 1;
                this.Rows = (int)math.ceil(height * InvCellSize) + 1;
            }

            int length = Cols * Rows;
            Cells = new NativeArray<int>(length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < length; i++) Cells[i] = -1;

            this.MinX = minX;
            this.MinY = minY;

            for (int i = 0; i < count; i++)
            {
                int nodeIdx = reflexNodes[i];
                NativeVertexNode node = nodes[nodeIdx];
                int idx = GetCellIndex(node.Position);
                if (idx >= 0 && idx < length)
                {
                    node.NextInGrid = Cells[idx];
                    Cells[idx] = nodeIdx;
                    nodes[nodeIdx] = node;
                }
            }
        }

        public void Remove(int nodeIdx, ref NativeArray<NativeVertexNode> nodes)
        {
            if (!Cells.IsCreated) return;
            float2 pos = nodes[nodeIdx].Position;
            int idx = GetCellIndex(pos);
            if (idx < 0 || idx >= Cells.Length) return;

            int currIdx = Cells[idx];
            int prevIdx = -1;

            while (currIdx != -1)
            {
                if (currIdx == nodeIdx)
                {
                    if (prevIdx == -1) Cells[idx] = nodes[currIdx].NextInGrid;
                    else
                    {
                        var prevNode = nodes[prevIdx];
                        prevNode.NextInGrid = nodes[currIdx].NextInGrid;
                        nodes[prevIdx] = prevNode;
                    }
                    return;
                }
                prevIdx = currIdx;
                currIdx = nodes[currIdx].NextInGrid;
            }
        }

        // Phase C-2: float2 参数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetCellIndex(float2 pos)
        {
            int x = (int)((pos.x - MinX) * InvCellSize);
            int y = (int)((pos.y - MinY) * InvCellSize);

            if (x < 0) x = 0; else if (x >= Cols) x = Cols - 1;
            if (y < 0) y = 0; else if (y >= Rows) y = Rows - 1;
            return y * Cols + x;
        }

        public void Dispose()
        {
            if (Cells.IsCreated) Cells.Dispose();
        }
    }

    // =================================================================================
    //                                  Job 并发加速核心
    // =================================================================================

    // Phase C-2: Vertices 从 NativeArray<Vector2> → NativeArray<float2>
    [BurstCompile(FloatMode = FloatMode.Fast)]
    private struct EarClipJob : IJob
    {
        [ReadOnly] public NativeArray<float2> Vertices;
        public NativeList<int> Triangles;

        public void Execute()
        {
            int n = Vertices.Length;
            if (n < 3) return;

            // 1. 构建完全扁平化的链表阵列
            NativeArray<NativeVertexNode> nodes = new NativeArray<NativeVertexNode>(n, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < n; i++)
            {
                nodes[i] = new NativeVertexNode
                {
                    Position = Vertices[i],
                    Index = i,
                    Prev = (i - 1 + n) % n,
                    Next = (i + 1) % n,
                    NextInGrid = -1,
                    IsReflex = false,
                    IsCandidate = false
                };
            }

            // 2. 绕序修正 (Winding Order)
            float area = 0;
            for (int i = 0; i < n; i++)
            {
                float2 p1 = nodes[i].Position;
                float2 p2 = nodes[nodes[i].Next].Position;
                area += (p2.x - p1.x) * (p2.y + p1.y);
            }

            if (area > 0) // Area > 0 为顺时针, 翻转链表索引
            {
                for (int i = 0; i < n; i++)
                {
                    NativeVertexNode node = nodes[i];
                    int temp = node.Prev;
                    node.Prev = node.Next;
                    node.Next = temp;
                    nodes[i] = node;
                }
            }

            // 3. 极速分类检测
            NativeList<int> reflexVertices = new NativeList<int>(n, Allocator.Temp);
            NativeList<int> earCandidates = new NativeList<int>(n, Allocator.Temp);

            int currentIdx = 0;
            int startIdx = currentIdx;
            do
            {
                NativeVertexNode current = nodes[currentIdx];
                if (IsReflex(currentIdx, ref nodes))
                {
                    current.IsReflex = true;
                    reflexVertices.Add(currentIdx);
                }
                else
                {
                    current.IsReflex = false;
                    current.IsCandidate = true;
                    earCandidates.Add(currentIdx);
                }
                nodes[currentIdx] = current;
                currentIdx = current.Next;
            } while (currentIdx != startIdx);

            // 4. 空间化加载避障
            NativeUniformGrid grid = new NativeUniformGrid();
            grid.Initialize(reflexVertices, ref nodes);

            int pointCount = n;

            // 5. 三角割耳主循环
            while (pointCount > 3 && earCandidates.Length > 0)
            {
                // [O(1) Pop末端] 即时裁剪无需 GC 整理
                int lastIdx = earCandidates.Length - 1;
                int candidateIdx = earCandidates[lastIdx];
                earCandidates.Length = lastIdx;

                NativeVertexNode candidate = nodes[candidateIdx];
                candidate.IsCandidate = false;
                nodes[candidateIdx] = candidate;

                // 核心安全验证
                if (IsEar(candidateIdx, ref nodes, ref grid))
                {
                    int prevIdx = candidate.Prev;
                    int nextIdx = candidate.Next;


                    Triangles.Add(nodes[prevIdx].Index);
                    Triangles.Add(nodes[nextIdx].Index); // 与上一行互换位置
                    Triangles.Add(candidate.Index);      // 与上一行互换位置

                    NativeVertexNode prev = nodes[prevIdx];
                    NativeVertexNode next = nodes[nextIdx];
                    prev.Next = nextIdx;
                    next.Prev = prevIdx;

                    nodes[prevIdx] = prev;
                    nodes[nextIdx] = next;

                    pointCount--;

                    if (candidate.IsReflex) grid.Remove(candidateIdx, ref nodes);

                    UpdateNeighbor(prevIdx, ref nodes, ref grid, ref earCandidates);
                    UpdateNeighbor(nextIdx, ref nodes, ref grid, ref earCandidates);
                }
            }

            // [兜底] 残阵收尾
            if (pointCount == 3 && earCandidates.Length > 0)
            {
                int n1 = earCandidates[0];
                int n2 = nodes[n1].Next;
                int n3 = nodes[n2].Next;

                Triangles.Add(nodes[n1].Index);
                Triangles.Add(nodes[n3].Index);
                Triangles.Add(nodes[n2].Index);
            }

            // 释放堆栈内存池
            grid.Dispose();
            nodes.Dispose();
            reflexVertices.Dispose();
            earCandidates.Dispose();
        }

        private void UpdateNeighbor(int nodeIdx, ref NativeArray<NativeVertexNode> nodes, ref NativeUniformGrid grid, ref NativeList<int> candidates)
        {
            NativeVertexNode node = nodes[nodeIdx];
            bool wasReflex = node.IsReflex;

            if (IsReflex(nodeIdx, ref nodes))
            {
                node.IsReflex = true;
            }
            else
            {
                node.IsReflex = false;
                if (wasReflex) grid.Remove(nodeIdx, ref nodes);
                if (!node.IsCandidate)
                {
                    node.IsCandidate = true;
                    candidates.Add(nodeIdx);
                }
            }
            nodes[nodeIdx] = node;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsReflex(int nodeIdx, ref NativeArray<NativeVertexNode> nodes)
        {
            NativeVertexNode v = nodes[nodeIdx];
            float2 a = nodes[v.Prev].Position;
            float2 b = v.Position;
            float2 c = nodes[v.Next].Position;
            return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) <= 0;
        }

        private bool IsEar(int nodeIdx, ref NativeArray<NativeVertexNode> nodes, ref NativeUniformGrid grid)
        {
            NativeVertexNode v = nodes[nodeIdx];
            if (v.IsReflex) return false;

            float2 a = nodes[v.Prev].Position;
            float2 b = v.Position;
            float2 c = nodes[v.Next].Position;

            float minX = a.x; if (b.x < minX) minX = b.x; if (c.x < minX) minX = c.x;
            float maxX = a.x; if (b.x > maxX) maxX = b.x; if (c.x > maxX) maxX = c.x;
            float minY = a.y; if (b.y < minY) minY = b.y; if (c.y < minY) minY = c.y;
            float maxY = a.y; if (b.y > maxY) maxY = b.y; if (c.y > maxY) maxY = c.y;

            if (!grid.Cells.IsCreated) return true;

            int startX = (int)((minX - grid.MinX) * grid.InvCellSize);
            int endX = (int)((maxX - grid.MinX) * grid.InvCellSize);
            int startY = (int)((minY - grid.MinY) * grid.InvCellSize);
            int endY = (int)((maxY - grid.MinY) * grid.InvCellSize);

            if (startX < 0) startX = 0; if (endX >= grid.Cols) endX = grid.Cols - 1;
            if (startY < 0) startY = 0; if (endY >= grid.Rows) endY = grid.Rows - 1;

            for (int y = startY; y <= endY; y++)
            {
                int offset = y * grid.Cols;
                for (int x = startX; x <= endX; x++)
                {
                    int currIdx = grid.Cells[offset + x];
                    while (currIdx != -1)
                    {
                        NativeVertexNode node = nodes[currIdx];

                        if (currIdx == v.Prev || currIdx == v.Next)
                        {
                            currIdx = node.NextInGrid;
                            continue;
                        }

                        // 鲁棒性防抖排除重合点
                        // Phase C-2: 使用 math.distancesq 替代 .sqrMagnitude (Burst SIMD 友好)
                        float d2a = math.distancesq(node.Position, a);
                        float d2b = math.distancesq(node.Position, b);
                        float d2c = math.distancesq(node.Position, c);

                        if (d2a < 1e-6f || d2b < 1e-6f || d2c < 1e-6f)
                        {
                            currIdx = node.NextInGrid;
                            continue;
                        }

                        if (IsPointInTriangle(a, b, c, node.Position)) return false;

                        currIdx = node.NextInGrid;
                    }
                }
            }
            return true;
        }

        // Phase C-2: float2 参数
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsPointInTriangle(float2 a, float2 b, float2 c, float2 p)
        {
            bool check1 = ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) >= 0;
            bool check2 = ((c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x)) >= 0;
            bool check3 = ((a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x)) >= 0;
            return check1 && check2 && check3;
        }
    }

    /// <summary>
    /// Phase C-2: Native 零拷贝三角剖分入口 — 直接消费 MergeNative 的 NativeList&lt;float2&gt; 输出。
    /// 调用者拥有返回的 NativeList 并负责 Dispose。
    /// </summary>
    public static NativeList<int> TriangulateNative(NativeList<float2> vertices, Allocator outputAllocator = Allocator.TempJob)
    {
        int n = vertices.Length;
        NativeList<int> nativeTriangles = new NativeList<int>(math.max((n - 2) * 3, 0), outputAllocator);
        if (n < 3) return nativeTriangles;

        EarClipJob job = new EarClipJob
        {
            Vertices = vertices.AsArray(),
            Triangles = nativeTriangles
        };

        // 零拷贝：输入 NativeList 直接 AsArray() 传入，无中间 NativeArray 分配
        job.Run();
        return nativeTriangles;
    }

    /// <summary>
    /// 三角网格构建主入口（托管路径），已彻底对接 Burst 底层。
    /// 保留供 CurveSlicer.PerformHolePunch 等无 Native 上下文的调用。
    /// </summary>
    public static int[] Triangulate(Vector2[] vertices)
    {
        int n = vertices.Length;
        if (n < 3) return new int[0];

        // I/O 边界：Vector2[] → NativeArray<float2>（单次批量转换）
        NativeArray<float2> nativeVerts = new NativeArray<float2>(n, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < n; i++)
        {
            nativeVerts[i] = new float2(vertices[i].x, vertices[i].y);
        }

        NativeList<int> nativeTriangles = new NativeList<int>((n - 2) * 3, Allocator.TempJob);

        try
        {
            EarClipJob job = new EarClipJob
            {
                Vertices = nativeVerts,
                Triangles = nativeTriangles
            };

            // 直接呼叫 Burst 执行机立刻接管主线程，跳过调度开销！
            job.Run();

            // 平滑传回给原有的 Unity Mesh API
            return nativeTriangles.AsArray().ToArray();
        }
        finally
        {
            nativeVerts.Dispose();
            nativeTriangles.Dispose();
        }
    }
}