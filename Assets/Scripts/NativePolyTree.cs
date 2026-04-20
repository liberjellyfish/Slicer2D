using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using System;

/// <summary>
/// 多边形层级查询树 (全 Native / Burst-Ready)
/// 用于快速判断一个孔洞该归属给哪一个外围碎片多边形。
/// 
/// 已重构为完全脱离托管数据源 (List/class)，直接从 SliceContext 的
/// NativeList 数据读取，可在 Burst Job 内部调用。
/// 
/// 数据源：FlattenedLoops, LoopRanges, LoopTypes, LoopAreas, LoopBounds
/// 这些由 ClassifyLoopsJob 在工作线程预计算完成。
/// </summary>
public struct NativePolyTree : IDisposable
{
    public struct FlatNode
    {
        public float4 Box;      // (minX, minY, maxX, maxY)
        public int LoopIndex;   // 叶子节点：对应原始环索引；内部节点：-1
        public int Left;
        public int Right;
    }

    private NativeArray<FlatNode> nodes;
    private NativeArray<int> indices;       // solid 环索引的工作数组（划分排序用）
    private int nodesUsed;
    private int solidCount;

    // 外部数据源引用（由调用方保证生命周期覆盖查询阶段）
    [ReadOnly] private NativeArray<float2> flattenedLoops;
    [ReadOnly] private NativeArray<int2> loopRanges;
    [ReadOnly] private NativeArray<float> loopAreas;
    [ReadOnly] private NativeArray<float4> loopBounds;

    /// <summary>
    /// 从 ClassifyLoopsJob 的产出中构建 BVH。
    /// 仅索引 LoopTypes[i] == 1 (solid) 的环。
    /// </summary>
    public void Build(
        NativeList<float2> srcFlattenedLoops,
        NativeList<int2> srcLoopRanges,
        NativeList<int> srcLoopTypes,
        NativeList<float> srcLoopAreas,
        NativeList<float4> srcLoopBounds)
    {
        // 缓存外部数据源的只读视图
        flattenedLoops = srcFlattenedLoops.AsArray();
        loopRanges = srcLoopRanges.AsArray();
        loopAreas = srcLoopAreas.AsArray();
        loopBounds = srcLoopBounds.AsArray();

        int totalLoops = srcLoopRanges.Length;

        // 第一遍：收集所有 solid 的原始环索引
        NativeList<int> solidIndices = new NativeList<int>(totalLoops, Allocator.Temp);
        for (int i = 0; i < totalLoops; i++)
        {
            if (srcLoopTypes[i] == 1) solidIndices.Add(i);
        }

        solidCount = solidIndices.Length;
        if (solidCount == 0)
        {
            solidIndices.Dispose();
            return;
        }

        // 分配 BVH 节点和工作索引数组
        nodes = new NativeArray<FlatNode>(solidCount * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        indices = new NativeArray<int>(solidCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

        for (int i = 0; i < solidCount; i++) indices[i] = solidIndices[i];
        solidIndices.Dispose();

        nodesUsed = 0;
        BuildRecursive(0, solidCount);
    }

    private int BuildRecursive(int start, int count)
    {
        int nodeIndex = nodesUsed++;

        // 计算所有子项的合并 AABB
        float4 total = loopBounds[indices[start]];
        for (int i = 1; i < count; i++)
        {
            float4 b = loopBounds[indices[start + i]];
            total.x = math.min(total.x, b.x); // minX
            total.y = math.min(total.y, b.y); // minY
            total.z = math.max(total.z, b.z); // maxX
            total.w = math.max(total.w, b.w); // maxY
        }

        FlatNode node = new FlatNode { Box = total };

        if (count == 1)
        {
            node.LoopIndex = indices[start];
            node.Left = -1;
            node.Right = -1;
            nodes[nodeIndex] = node;
            return nodeIndex;
        }

        node.LoopIndex = -1;
        nodes[nodeIndex] = node; // 先占位

        // 中轴划分
        float sizeX = total.z - total.x;
        float sizeY = total.w - total.y;
        bool splitX = sizeX > sizeY;
        float mid = splitX ? (total.x + total.z) * 0.5f : (total.y + total.w) * 0.5f;

        int left = start, right = start + count - 1;
        while (left <= right)
        {
            float4 b = loopBounds[indices[left]];
            float center = splitX ? (b.x + b.z) * 0.5f : (b.y + b.w) * 0.5f;
            if (center < mid)
                left++;
            else
            {
                int temp = indices[left]; indices[left] = indices[right]; indices[right] = temp;
                right--;
            }
        }

        int leftCount = left - start;
        if (leftCount == 0 || leftCount == count) leftCount = count / 2;

        int leftChild = BuildRecursive(start, leftCount);
        int rightChild = BuildRecursive(start + leftCount, count - leftCount);

        node = nodes[nodeIndex];
        node.Left = leftChild;
        node.Right = rightChild;
        nodes[nodeIndex] = node;

        return nodeIndex;
    }

    /// <summary>
    /// 查询包含指定点的最小面积 solid 环索引。
    /// 返回 LoopRanges 中的原始索引，-1 表示未找到。
    /// </summary>
    public int QueryBestParent(float2 point, float holeArea)
    {
        if (solidCount == 0) return -1;
        return QueryRecursive(0, point, holeArea);
    }

    private int QueryRecursive(int nodeIdx, float2 point, float holeArea)
    {
        FlatNode node = nodes[nodeIdx];

        // AABB 排除
        if (point.x < node.Box.x || point.x > node.Box.z ||
            point.y < node.Box.y || point.y > node.Box.w)
            return -1;

        // 叶子节点：精确 PointInPolygon 判定
        if (node.LoopIndex != -1)
        {
            int loopIdx = node.LoopIndex;
            if (loopAreas[loopIdx] > holeArea)
            {
                int2 range = loopRanges[loopIdx];
                if (PointInPolygon(point, range.x, range.y))
                    return loopIdx;
            }
            return -1;
        }

        // 内部节点：递归两个子树，取面积更小的那个
        int l = QueryRecursive(node.Left, point, holeArea);
        int r = QueryRecursive(node.Right, point, holeArea);

        if (l != -1 && r != -1)
            return loopAreas[l] < loopAreas[r] ? l : r;
        return l != -1 ? l : r;
    }

    private bool PointInPolygon(float2 point, int start, int count)
    {
        bool inside = false;
        for (int i = 0, j = count - 1; i < count; j = i++)
        {
            float2 pi = flattenedLoops[start + i];
            float2 pj = flattenedLoops[start + j];

            if ((pi.y > point.y) != (pj.y > point.y) &&
                point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
            {
                inside = !inside;
            }
        }
        return inside;
    }

    public void Dispose()
    {
        if (nodes.IsCreated) nodes.Dispose();
        if (indices.IsCreated) indices.Dispose();
        solidCount = 0;
    }
}
