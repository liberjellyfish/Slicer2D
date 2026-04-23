using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// SlicerCore 的 Phase 5+ 子模块——环分类、孔洞归属、搭桥合并与三角剖分 Job。
/// </summary>
public static partial class SlicerCore
{
    // =================================================================================
    //                    Phase 5 原生路径简化与环分类 Job
    // =================================================================================

    /// <summary>
    /// 在 FlattenedLoops 上对每个环做原地顶点去重压缩。
    /// 替代主线程的 SimplifyPath，消灭中间 List&lt;Vector2&gt; 分配。
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct SimplifyLoopsJob : IJob
    {
        public NativeList<float2> FlattenedLoops;
        public NativeList<int2> LoopRanges;
        public float MinVertDistSq;

        public void Execute()
        {
            for (int loopIdx = 0; loopIdx < LoopRanges.Length; loopIdx++)
            {
                int2 range = LoopRanges[loopIdx];
                int start = range.x;
                int count = range.y;

                if (count < 3) continue;

                // 保留第一个顶点，从第二个开始扫掠压缩
                int writeIdx = 1;

                for (int i = 1; i < count; i++)
                {
                    float2 prev = FlattenedLoops[start + writeIdx - 1];
                    float2 curr = FlattenedLoops[start + i];
                    float dx = curr.x - prev.x;
                    float dy = curr.y - prev.y;
                    if (dx * dx + dy * dy > MinVertDistSq)
                    {
                        FlattenedLoops[start + writeIdx] = curr;
                        writeIdx++;
                    }
                }

                // 检查首尾是否过近，如果是则去掉末尾点
                if (writeIdx > 2)
                {
                    float2 first = FlattenedLoops[start];
                    float2 last = FlattenedLoops[start + writeIdx - 1];
                    float dx = first.x - last.x;
                    float dy = first.y - last.y;
                    if (dx * dx + dy * dy < MinVertDistSq)
                    {
                        writeIdx--;
                    }
                }

                // 原地更新范围长度（起始位置不变，间隙留空无害）
                LoopRanges[loopIdx] = new int2(start, writeIdx);
            }
        }
    }

    /// <summary>
    /// 为每个环计算有向面积 (SignedArea)、AABB 包围盒，并分类为：
    ///   1 = Solid (CCW, area &gt; 0)
    ///  -1 = Hole  (CW,  area &lt; 0)
    ///   0 = Discard (面积过小或顶点不足)
    /// 替代主线程的 SignedArea + CalculateBounds + 面积判定。
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct ClassifyLoopsJob : IJob
    {
        [ReadOnly] public NativeList<float2> FlattenedLoops;
        [ReadOnly] public NativeList<int2> LoopRanges;
        public NativeList<int> LoopTypes;
        public NativeList<float> LoopAreas;
        public NativeList<float4> LoopBounds;
        public float AreaThreshold;

        public void Execute()
        {
            int loopCount = LoopRanges.Length;
            LoopTypes.Length = loopCount;
            LoopAreas.Length = loopCount;
            LoopBounds.Length = loopCount;

            for (int loopIdx = 0; loopIdx < loopCount; loopIdx++)
            {
                int2 range = LoopRanges[loopIdx];
                int start = range.x;
                int count = range.y;

                if (count < 3)
                {
                    LoopTypes[loopIdx] = 0;
                    LoopAreas[loopIdx] = 0;
                    LoopBounds[loopIdx] = float4.zero;
                    continue;
                }

                // 单次遍历同时计算有向面积与包围盒
                float area = 0;
                float minX = float.MaxValue, minY = float.MaxValue;
                float maxX = float.MinValue, maxY = float.MinValue;

                for (int i = 0; i < count; i++)
                {
                    float2 p1 = FlattenedLoops[start + i];
                    float2 p2 = FlattenedLoops[start + ((i + 1) % count)];
                    area += (p1.x * p2.y) - (p2.x * p1.y);

                    if (p1.x < minX) minX = p1.x;
                    if (p1.x > maxX) maxX = p1.x;
                    if (p1.y < minY) minY = p1.y;
                    if (p1.y > maxY) maxY = p1.y;
                }
                area *= 0.5f;

                float absArea = math.abs(area);
                if (absArea < AreaThreshold)
                {
                    LoopTypes[loopIdx] = 0;
                    LoopAreas[loopIdx] = 0;
                    LoopBounds[loopIdx] = float4.zero;
                }
                else if (area > 0)
                {
                    LoopTypes[loopIdx] = 1;
                    LoopAreas[loopIdx] = absArea;
                    LoopBounds[loopIdx] = new float4(minX, minY, maxX, maxY);
                }
                else
                {
                    LoopTypes[loopIdx] = -1;
                    LoopAreas[loopIdx] = absArea;
                    LoopBounds[loopIdx] = new float4(minX, minY, maxX, maxY);
                }
            }
        }
    }

    /// <summary>
    /// 孔洞归属分配 Job：对每个 hole 环找到包含它的最小面积 solid 环。
    /// 使用 NativePolyTree (BVH) 加速空间查询，复杂度 O(H * logS * V)。
    /// 完全在工作线程执行，替代主线程的 NativePolyTree 构建与查询。
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct AssignHolesJob : IJob
    {
        [ReadOnly] public NativeList<float2> FlattenedLoops;
        [ReadOnly] public NativeList<int2> LoopRanges;
        [ReadOnly] public NativeList<int> LoopTypes;
        [ReadOnly] public NativeList<float> LoopAreas;
        [ReadOnly] public NativeList<float4> LoopBounds;

        public NativeList<int> HoleParents; // 输出：每个环的父级 solid 环索引，-1 = 非孔洞或无归属

        public void Execute()
        {
            int loopCount = LoopRanges.Length;
            HoleParents.Length = loopCount;

            for (int i = 0; i < loopCount; i++) HoleParents[i] = -1;

            // 构建 BVH（Allocator.Temp，Execute 结束后自动释放）
            NativePolyTree tree = new NativePolyTree();
            tree.Build(FlattenedLoops, LoopRanges, LoopTypes, LoopAreas, LoopBounds);

            // 对每个孔洞执行 BVH 加速查询
            for (int h = 0; h < loopCount; h++)
            {
                if (LoopTypes[h] != -1) continue;

                int2 hRange = LoopRanges[h];
                if (hRange.y < 3) continue;

                // 测试点 = 前两个顶点的中点
                float2 p0 = FlattenedLoops[hRange.x];
                float2 p1 = FlattenedLoops[hRange.x + 1];
                float2 testPoint = (p0 + p1) * 0.5f;
                float holeArea = LoopAreas[h];

                HoleParents[h] = tree.QueryBestParent(testPoint, holeArea);
            }

            tree.Dispose();
        }
    }

    // =================================================================================
    //                    Phase 6 孔洞映射构建 + 搭桥合并 + 三角剖分 Job
    // =================================================================================

    /// <summary>
    /// 构建 per-loop 的 Solid→Hole 映射表。
    /// 输出等长于 LoopRanges 的 SolidHoleMap 数组：
    ///   solid 环: (holeStartInBuffer, holeCount)
    ///   非 solid 环: (-1, 0)
    /// 以及扁平化的 HoleRangeBuffer 存储所有 solid 对应的孔洞范围。
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct BuildSolidHoleMapJob : IJob
    {
        [ReadOnly] public NativeList<int2> LoopRanges;
        [ReadOnly] public NativeList<int> LoopTypes;
        [ReadOnly] public NativeList<int> HoleParents;

        public NativeList<int2> SolidHoleMap;     // 等长于 LoopRanges
        public NativeList<int2> HoleRangeBuffer;  // 扁平化存储所有孔洞的 (start, count)

        public void Execute()
        {
            int loopCount = LoopRanges.Length;
            SolidHoleMap.Length = loopCount;

            // 初始化全部为无效
            for (int i = 0; i < loopCount; i++)
                SolidHoleMap[i] = new int2(-1, 0);

            // 对每个 solid，扫描 HoleParents 收集归属孔洞
            for (int solidIdx = 0; solidIdx < loopCount; solidIdx++)
            {
                if (LoopTypes[solidIdx] != 1) continue;

                int holeStart = HoleRangeBuffer.Length;
                int holeCount = 0;

                for (int h = 0; h < loopCount; h++)
                {
                    if (HoleParents[h] != solidIdx) continue;
                    if (LoopRanges[h].y < 3) continue;

                    HoleRangeBuffer.Add(LoopRanges[h]);
                    holeCount++;
                }

                SolidHoleMap[solidIdx] = new int2(holeStart, holeCount);
            }
        }
    }

    /// <summary>
    /// 并行搭桥合并 + 三角剖分 Job (IJobParallelFor)。
    /// 按 LoopRanges.Length 并发，非 Solid 环直接跳过（空槽位）。
    /// 每个 Solid 独立执行：MergeBurst → EarClipBurst → 写入 NativeStream。
    /// Stream 格式：[int vertCount] [float3 pos * N] [float2 uv * N] [int triCount] [int idx * M]
    /// </summary>
    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct MergeTriangulateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> FlattenedLoops;
        [ReadOnly] public NativeArray<int2> LoopRanges;
        [ReadOnly] public NativeArray<int> LoopTypes;
        [ReadOnly] public NativeArray<int2> SolidHoleMap;
        [ReadOnly] public NativeArray<int2> HoleRangeBuffer;

        public float4 UVRect; // (minX, minY, width, height)

        public NativeStream.Writer MeshDataWriter;

        public void Execute(int index)
        {
            MeshDataWriter.BeginForEachIndex(index);

            if (LoopTypes[index] != 1)
            {
                MeshDataWriter.EndForEachIndex();
                return;
            }

            int2 outerRange = LoopRanges[index];
            int2 holeData = SolidHoleMap[index];

            // Phase 6a: Burst 搭桥合并
            NativeList<float2> merged = PolygonHoleMerger.MergeBurst(
                FlattenedLoops, outerRange, HoleRangeBuffer, holeData.x, holeData.y);

            // Phase 6b: Burst 三角剖分
            NativeList<int> triangles = new NativeList<int>(math.max((merged.Length - 2) * 3, 0), Allocator.Temp);
            Triangulator.EarClipBurst(merged.AsArray(), ref triangles);

            // Phase 6c: 写入 Stream
            int vertCount = merged.Length;
            int triCount = triangles.Length;

            float uMin = UVRect.x;
            float vMin = UVRect.y;
            float width = UVRect.z < 0.0001f ? 1f : UVRect.z;
            float height = UVRect.w < 0.0001f ? 1f : UVRect.w;

            MeshDataWriter.Write(vertCount);
            for (int i = 0; i < vertCount; i++)
            {
                float2 v = merged[i];
                MeshDataWriter.Write(new float3(v.x, v.y, 0f));
                MeshDataWriter.Write(new float2((v.x - uMin) / width, (v.y - vMin) / height));
            }

            MeshDataWriter.Write(triCount);
            for (int i = 0; i < triCount; i++)
            {
                MeshDataWriter.Write(triangles[i]);
            }

            merged.Dispose();
            triangles.Dispose();

            MeshDataWriter.EndForEachIndex();
        }
    }
}
