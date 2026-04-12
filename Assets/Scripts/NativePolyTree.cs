using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using System;

/// <summary>
/// 多边形层级查询树 (Native)
/// 用于快速判断一个孔洞该归属给哪一个外围生成的碎片多边形。
/// 已重构为无静态锁、无 GC 的 Burst-ready 多线程安全结构。
/// </summary>
public struct NativePolyTree : IDisposable
{
    public struct FlatNode
    {
        public Bounds Box;
        public int PolygonIndex;
        public int Left;
        public int Right;
    }

    private NativeArray<FlatNode> nodes;
    private NativeArray<int> indices;
    private int nodesUsed;
    private List<SlicerCore.PolygonData> srcData;

    public void Build(List<SlicerCore.PolygonData> solids)
    {
        if (solids == null || solids.Count == 0) return;
        srcData = solids;
        int count = solids.Count;

        // 使用 Allocator.Temp 获取超高速堆栈式分配，离开作用域前必须 Dispose
        nodes = new NativeArray<FlatNode>(count * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        indices = new NativeArray<int>(count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

        for (int i = 0; i < count; i++) indices[i] = i;
        nodesUsed = 0;
        BuildRecursive(0, count);
    }

    private int BuildRecursive(int start, int count)
    {
        int nodeIndex = nodesUsed++;
        Bounds total = srcData[indices[start]].Bounds;
        for (int i = 1; i < count; i++) total.Encapsulate(srcData[indices[start + i]].Bounds);

        FlatNode node = new FlatNode { Box = total };

        if (count == 1)
        {
            node.PolygonIndex = indices[start];
            node.Left = -1;
            node.Right = -1;
            nodes[nodeIndex] = node;
            return nodeIndex;
        }

        node.PolygonIndex = -1;
        nodes[nodeIndex] = node; // 先写入占据空间，后续更新左右子节点

        bool splitX = total.size.x > total.size.y;
        float mid = splitX ? total.center.x : total.center.y;
        int left = start, right = start + count - 1;
        
        while (left <= right)
        {
            float center = splitX ? srcData[indices[left]].Bounds.center.x : srcData[indices[left]].Bounds.center.y;
            if (center < mid) left++;
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

        // 取出节点，赋予子节点指针，再装回去
        node = nodes[nodeIndex];
        node.Left = leftChild;
        node.Right = rightChild;
        nodes[nodeIndex] = node;

        return nodeIndex;
    }

    public SlicerCore.PolygonData QueryBestParent(Vector2 point, float holeArea)
    {
        if (srcData == null || srcData.Count == 0) return null;
        return QueryRecursive(0, point, holeArea);
    }

    private SlicerCore.PolygonData QueryRecursive(int nodeIdx, Vector2 point, float holeArea)
    {
        FlatNode node = nodes[nodeIdx];
        if (!node.Box.Contains(new Vector3(point.x, point.y, 0))) return null;

        if (node.PolygonIndex != -1)
        {
            SlicerCore.PolygonData candidate = srcData[node.PolygonIndex];
            if (candidate.Area > holeArea && IsPointInPolygon(point, candidate.OuterLoop)) return candidate;
            return null;
        }
        SlicerCore.PolygonData l = QueryRecursive(node.Left, point, holeArea);
        SlicerCore.PolygonData r = QueryRecursive(node.Right, point, holeArea);
        if (l != null && r != null) return l.Area < r.Area ? l : r;
        return l != null ? l : r;
    }

    private static bool IsPointInPolygon(Vector2 p, List<Vector2> polygon)
    {
        bool inside = false;
        int count = polygon.Count;
        for (int i = 0, j = count - 1; i < count; j = i++)
        {
            if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
                (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    public void Dispose()
    {
        srcData = null;
        if (nodes.IsCreated) nodes.Dispose();
        if (indices.IsCreated) indices.Dispose();
    }
}
