using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

// 这是一个纯静态的计算核心，不依赖 GameObject 实例化
public static class SlicerCore
{
    // =================================================================================
    //                                  内存池与上下文
    // =================================================================================

    // 用于避免 Vector2 在 Dictionary 中装箱和哈希冲突
    public struct Vector2Comparer : IEqualityComparer<Vector2>
    {
        public bool Equals(Vector2 x, Vector2 y)
        {
            return Mathf.Abs(x.x - y.x) < 1e-5f && Mathf.Abs(x.y - y.y) < 1e-5f;
        }

        public int GetHashCode(Vector2 obj)
        {
            // 简单的空间哈希，乘大质数
            return ((int)(obj.x * 1000) * 397) ^ (int)(obj.y * 1000);
        }
    }

    // 避免 Lambda 排序产生的 GC
    private struct IntersectionComparer : IComparer<IntersectionInfo>
    {
        public List<Vector2> Path;
        public int Compare(IntersectionInfo a, IntersectionInfo b)
        {
            if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
            // 缓存的 Path 访问
            float distA = (a.Point.x - Path[a.SegmentIndex].x) * (a.Point.x - Path[a.SegmentIndex].x) + (a.Point.y - Path[a.SegmentIndex].y) * (a.Point.y - Path[a.SegmentIndex].y);
            float distB = (b.Point.x - Path[b.SegmentIndex].x) * (b.Point.x - Path[b.SegmentIndex].x) + (b.Point.y - Path[b.SegmentIndex].y) * (b.Point.y - Path[b.SegmentIndex].y);
            return distA.CompareTo(distB);
        }
    }

    // 切割点排序
    private struct CutIntersectionComparer : IComparer<Vector2>
    {
        public Vector2 Start, End;
        public int Compare(Vector2 a, Vector2 b)
        {
            float distA = (a.x - Start.x) * (End.x - Start.x) + (a.y - Start.y) * (End.y - Start.y);
            float distB = (b.x - Start.x) * (End.x - Start.x) + (b.y - Start.y) * (End.y - Start.y);
            return distA.CompareTo(distB);
        }
    }

    // 核心数据结构：PolygonData (改为可回收的类)
    public class PolygonData
    {
        public List<Vector2> OuterLoop;
        public List<List<Vector2>> Holes;
        public float Area;
        public Bounds Bounds;

        public PolygonData() { } // 由池管理初始化

        public void Init()
        {
            if (Holes == null) Holes = new List<List<Vector2>>();
            else Holes.Clear();
            OuterLoop = null; // 由外部赋值
            Area = 0;
            Bounds = default;
        }
    }

    // 上下文：一次切割操作中复用的所有内存
    private class SliceContext
    {
        public Dictionary<Vector2, List<Vector2>> Graph;
        public List<Vector2> CutIntersections;
        public HashSet<EdgeKey> VisitedEdges;

        // 各种临时列表，避免反复 new
        public List<IntersectionInfo> TempHits;
        public List<Vector2> TempNewPath;
        public List<List<Vector2>> TempRawLoops;

        // 对象池
        public Stack<List<Vector2>> ListPool;
        public Stack<PolygonData> PolyPool;

        public SliceContext()
        {
            Graph = new Dictionary<Vector2, List<Vector2>>(256, new Vector2Comparer());
            CutIntersections = new List<Vector2>(32);
            VisitedEdges = new HashSet<EdgeKey>();
            TempHits = new List<IntersectionInfo>(32);
            TempNewPath = new List<Vector2>(128);
            TempRawLoops = new List<List<Vector2>>(16);
            ListPool = new Stack<List<Vector2>>();
            PolyPool = new Stack<PolygonData>();
        }

        public List<Vector2> GetList()
        {
            return ListPool.Count > 0 ? ListPool.Pop() : new List<Vector2>();
        }

        public void ReturnList(List<Vector2> list)
        {
            if (list == null) return;
            list.Clear();
            ListPool.Push(list);
        }

        public PolygonData GetPoly()
        {
            PolygonData p = PolyPool.Count > 0 ? PolyPool.Pop() : new PolygonData();
            p.Init();
            return p;
        }

        public void ReturnPoly(PolygonData p)
        {
            if (p == null) return;
            // 归还它持有的 Lists
            if (p.OuterLoop != null) ReturnList(p.OuterLoop);
            if (p.Holes != null)
            {
                foreach (var hole in p.Holes) ReturnList(hole);
                p.Holes.Clear();
            }
            PolyPool.Push(p);
        }

        public void ClearAll()
        {
            foreach (var kvp in Graph) ReturnList(kvp.Value);
            Graph.Clear();
            CutIntersections.Clear();
            VisitedEdges.Clear();
            TempHits.Clear();
            TempNewPath.Clear();
            // TempRawLoops 的元素也是 List，需要单独处理归还逻辑，在 ExtractLoops 内部处理
        }
    }

    // 静态单例上下文，单线程下安全 (Unity 协程需要在主线程跑，所以通常是安全的)
    // 如果有多线程需求，需改为 [ThreadStatic]
    private static SliceContext context = new SliceContext();

    // =================================================================================
    //                                  配置常量 & 结构
    // =================================================================================
    private const float MIN_VERT_DIST_SQ = 0.0001f; // 0.01 * 0.01
    private const float AREA_THRESHOLD = 0.01f;

    private readonly struct EdgeKey : System.IEquatable<EdgeKey>
    {
        private readonly long id; // 将坐标压缩成一个 long
        public EdgeKey(Vector2 u, Vector2 v)
        {
            // 简单量化处理
            int x1 = (int)(u.x * 1000); int y1 = (int)(u.y * 1000);
            int x2 = (int)(v.x * 1000); int y2 = (int)(v.y * 1000);
            // 混合 hash
            id = ((long)x1 << 48) ^ ((long)y1 << 32) ^ ((long)x2 << 16) ^ (long)y2;
        }
        public bool Equals(EdgeKey other) => id == other.id;
        public override int GetHashCode() => id.GetHashCode();
    }

    private struct IntersectionInfo
    {
        public Vector2 Point;
        public float T;
        public int SegmentIndex;
    }

    // =================================================================================
    //                                  对外计算接口
    // =================================================================================

    /// <summary>
    /// 执行核心切割计算。
    /// 调用方在使用完数据生成 Mesh 后，调用 FreeResult() 来归还内存。
    /// </summary>
    public static List<PolygonData> Calculate(List<List<Vector2>> originalPaths, Vector2 start, Vector2 end)
    {
        // 1. 清理并准备上下文
        context.ClearAll();

        var graph = context.Graph;
        var cutIntersections = context.CutIntersections;

        // --- Phase 1: 构建拓扑图 ---
        IntersectionComparer hitComparer = new IntersectionComparer();

        foreach (var path in originalPaths)
        {
            context.TempHits.Clear();
            hitComparer.Path = path; // 设置 Comparer 上下文

            for (int i = 0; i < path.Count; i++)
            {
                Vector2 p1 = path[i];
                Vector2 p2 = path[(i + 1) % path.Count];

                if (GetLineIntersection(p1, p2, start, end, out Vector2 intersection, out float t))
                {
                    context.TempHits.Add(new IntersectionInfo { Point = intersection, T = t, SegmentIndex = i });
                }
            }

            // 无 GC 排序
            context.TempHits.Sort(hitComparer);

            // 重建路径
            context.TempNewPath.Clear();
            var newPathVertices = context.TempNewPath;

            int hitIndex = 0;
            for (int i = 0; i < path.Count; i++)
            {
                Vector2 currentVert = path[i];
                if (newPathVertices.Count == 0 || SqrDist(newPathVertices[newPathVertices.Count - 1], currentVert) > MIN_VERT_DIST_SQ)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < context.TempHits.Count && context.TempHits[hitIndex].SegmentIndex == i)
                {
                    Vector2 p = context.TempHits[hitIndex].Point;
                    if (SqrDist(newPathVertices[newPathVertices.Count - 1], p) > MIN_VERT_DIST_SQ)
                    {
                        newPathVertices.Add(p);
                        cutIntersections.Add(p);
                    }
                    hitIndex++;
                }
            }
            // 闭合检查
            if (newPathVertices.Count > 1 && SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Count - 1]) < MIN_VERT_DIST_SQ)
                newPathVertices.RemoveAt(newPathVertices.Count - 1);

            for (int i = 0; i < newPathVertices.Count; i++)
            {
                AddEdge(graph, newPathVertices[i], newPathVertices[(i + 1) % newPathVertices.Count]);
            }
        }

        // --- Phase 2: 处理切割缝 ---
        // 去重
        for (int i = cutIntersections.Count - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (SqrDist(cutIntersections[i], cutIntersections[j]) < MIN_VERT_DIST_SQ)
                {
                    cutIntersections.RemoveAt(i);
                    break;
                }
            }
        }

        if (cutIntersections.Count < 2) return null;

        // 排序
        cutIntersections.Sort(new CutIntersectionComparer { Start = start, End = end });

        int validCount = (cutIntersections.Count % 2 == 0) ? cutIntersections.Count : cutIntersections.Count - 1;
        for (int i = 0; i < validCount; i += 2)
        {
            Vector2 pA = cutIntersections[i];
            Vector2 pB = cutIntersections[i + 1];
            if (SqrDist(pA, pB) > MIN_VERT_DIST_SQ)
            {
                AddEdge(graph, pA, pB);
                AddEdge(graph, pB, pA);
            }
        }

        // --- Phase 3: 提取回路 ---
        ExtractLoops(graph); // 结果存入 context.TempRawLoops

        List<PolygonData> solids = new List<PolygonData>(); // 这个 List 需要返回给外部，所以 new 它是合理的，或者也可以池化
        List<List<Vector2>> holes = new List<List<Vector2>>();

        foreach (var rawLoop in context.TempRawLoops)
        {
            List<Vector2> loop = SimplifyPath(rawLoop);
            // rawLoop 归还池
            context.ReturnList(rawLoop);

            float area = SignedArea(loop);
            if (Mathf.Abs(area) < AREA_THRESHOLD)
            {
                context.ReturnList(loop); // 无效 loop，归还
                continue;
            }

            if (area > 0)
            {
                PolygonData poly = context.GetPoly(); // 从池中取
                poly.OuterLoop = loop;
                poly.Area = area;
                poly.Bounds = CalculateBounds(loop); // 内联优化
                solids.Add(poly);
            }
            else
            {
                holes.Add(loop);
            }
        }
        context.TempRawLoops.Clear(); // 列表本身清空，内容已转移或归还

        // --- Phase 4: AABB 树归属权分配 ---
        NativePolyTree tree = new NativePolyTree();
        tree.Build(solids); // 注意：tree 内部引用了 solids 列表

        for (int i = 0; i < holes.Count; i++)
        {
            List<Vector2> hole = holes[i];
            if (hole.Count < 3)
            {
                context.ReturnList(hole);
                continue;
            }
            Vector2 testPoint = (hole[0] + hole[1]) * 0.5f;
            float holeAreaAbs = Mathf.Abs(SignedArea(hole));

            PolygonData bestParent = tree.QueryBestParent(testPoint, holeAreaAbs);

            if (bestParent != null)
            {
                bestParent.Holes.Add(hole);
            }
            else
            {
                // 孤儿孔洞，为了内存安全需要归还
                context.ReturnList(hole);
            }
        }

        tree.Dispose(); // 释放 tree 内部的临时数组缓存

        return solids;
    }

    public static void ReturnResultToPool(List<PolygonData> results)
    {
        if (results == null) return;
        foreach (var poly in results)
        {
            context.ReturnPoly(poly);
        }
    }

    // =================================================================================
    //                                  内部逻辑 (图论 & 几何)
    // =================================================================================

    private static void AddEdge(Dictionary<Vector2, List<Vector2>> graph, Vector2 u, Vector2 v)
    {
        if (SqrDist(u, v) < 1e-6f) return;

        if (!graph.TryGetValue(u, out List<Vector2> neighbors))
        {
            neighbors = context.GetList();
            graph[u] = neighbors;
        }

        // 避免重复边
        bool exists = false;
        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            if (SqrDist(neighbors[i], v) < 1e-6f) { exists = true; break; }
        }
        if (!exists) neighbors.Add(v);
    }

    private static void ExtractLoops(Dictionary<Vector2, List<Vector2>> graph)
    {
        context.TempRawLoops.Clear();
        context.VisitedEdges.Clear();
        // 直接遍历 KVP
        foreach (var kvp in graph)
        {
            Vector2 startNode = kvp.Key;
            List<Vector2> neighbors = kvp.Value;

            for (int i = 0; i < neighbors.Count; i++)
            {
                Vector2 nextNode = neighbors[i];
                EdgeKey edgeKey = new EdgeKey(startNode, nextNode);
                if (context.VisitedEdges.Contains(edgeKey)) continue;

                List<Vector2> currentLoop = context.GetList(); // 池化
                Vector2 curr = startNode;
                Vector2 next = nextNode;
                currentLoop.Add(curr);

                int watchdog = 0;
                int maxIterations = graph.Count * 2 + 100;
                bool loopClosed = false;

                while (watchdog++ < maxIterations)
                {
                    context.VisitedEdges.Add(new EdgeKey(curr, next));
                    currentLoop.Add(next);

                    if (SqrDist(next, startNode) < MIN_VERT_DIST_SQ)
                    {
                        loopClosed = true;
                        break;
                    }

                    Vector2 prev = curr;
                    curr = next;

                    if (!graph.TryGetValue(curr, out List<Vector2> currNeighbors) || currNeighbors.Count == 0) break;

                    next = GetLeftMostNeighbor(prev, curr, currNeighbors);

                    if (next == Vector2.zero) break;
                }

                if (loopClosed && currentLoop.Count > 2)
                {
                    // 移除闭合点
                    currentLoop.RemoveAt(currentLoop.Count - 1);
                    context.TempRawLoops.Add(currentLoop);
                }
                else
                {
                    // 失败的循环，归还内存
                    context.ReturnList(currentLoop);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SqrDist(Vector2 a, Vector2 b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 GetLeftMostNeighbor(Vector2 prev, Vector2 curr, List<Vector2> neighbors)
    {
        Vector2 incomingDir = (curr - prev).normalized;
        if (incomingDir == Vector2.zero) incomingDir = Vector2.right;

        float bestAngle = -9999f;
        Vector2 bestNext = Vector2.zero;
        Vector2 backDir = -incomingDir;

        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 neighbor = neighbors[i];
            if (SqrDist(neighbor, prev) < MIN_VERT_DIST_SQ && count > 1) continue;

            Vector2 outgoingDir = (neighbor - curr).normalized;
            if (outgoingDir == Vector2.zero) continue;

            float angle = Vector2.SignedAngle(backDir, outgoingDir);
            if (angle < 0) angle += 360f;

            if (angle > bestAngle)
            {
                bestAngle = angle;
                bestNext = neighbor;
            }
        }

        if (bestNext == Vector2.zero && count > 0) return neighbors[0];
        return bestNext;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool GetLineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 intersection, out float t)
    {
        intersection = Vector2.zero;
        t = 0;
        float d = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
        if (Mathf.Abs(d) < 1e-6f) return false;

        float u = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / d;
        float v = ((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / d;

        if (u >= -1e-4f && u <= 1.0001f && v >= -1e-4f && v <= 1.0001f)
        {
            t = Mathf.Clamp01(v);
            intersection = p1 + Mathf.Clamp01(u) * (p2 - p1);
            return true;
        }
        return false;
    }

    private static List<Vector2> SimplifyPath(List<Vector2> path)
    {
        if (path.Count < 3)
        {
            var copy = context.GetList();
            copy.AddRange(path);
            return copy;
        }

        List<Vector2> simplified = context.GetList();
        simplified.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            if (SqrDist(path[i], simplified[simplified.Count - 1]) > MIN_VERT_DIST_SQ)
                simplified.Add(path[i]);
        }

        // 检查首尾闭合
        if (simplified.Count > 2 && SqrDist(simplified[0], simplified[simplified.Count - 1]) < MIN_VERT_DIST_SQ)
            simplified.RemoveAt(simplified.Count - 1);

        return simplified;
    }

    // --- Helper Math ---
    private static Bounds CalculateBounds(List<Vector2> loop)
    {
        if (loop.Count == 0) return default;
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        for (int i = 0; i < loop.Count; i++)
        {
            Vector2 p = loop[i];
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        return new Bounds(new Vector3((minX + maxX) / 2, (minY + maxY) / 2, 0), new Vector3(maxX - minX, maxY - minY, 1));
    }

    private static float SignedArea(List<Vector2> points)
    {
        float area = 0;
        int count = points.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % count];
            area += (p1.x * p2.y) - (p2.x * p1.y);
        }
        return area / 2.0f;
    }

    private struct NativePolyTree
    {
        public struct FlatNode
        {
            public Bounds Box;
            public int PolygonIndex;
            public int Left;
            public int Right;
        }

        private FlatNode[] nodes;
        private int[] indices;
        private int nodesUsed;
        private List<PolygonData> srcData; // 引用 SlicerCore.PolygonData

        // 简单的数组缓存池，避免每次 Build 都 new int[]
        private static FlatNode[] nodeCache = new FlatNode[64];
        private static int[] indexCache = new int[32];

        public void Build(List<PolygonData> solids)
        {
            if (solids == null || solids.Count == 0) return;
            srcData = solids;
            int count = solids.Count;

            // 动态扩容缓存
            if (nodeCache.Length < count * 2) System.Array.Resize(ref nodeCache, count * 4);
            if (indexCache.Length < count) System.Array.Resize(ref indexCache, count * 2);

            nodes = nodeCache;
            indices = indexCache;

            for (int i = 0; i < count; i++) indices[i] = i;
            nodesUsed = 0;
            BuildRecursive(0, count);
        }

        private int BuildRecursive(int start, int count)
        {
            // 逻辑与原版完全一致，省略
            int nodeIndex = nodesUsed++;
            Bounds total = srcData[indices[start]].Bounds;
            for (int i = 1; i < count; i++) total.Encapsulate(srcData[indices[start + i]].Bounds);
            nodes[nodeIndex].Box = total;

            if (count == 1)
            {
                nodes[nodeIndex].PolygonIndex = indices[start];
                nodes[nodeIndex].Left = nodes[nodeIndex].Right = -1;
                return nodeIndex;
            }
            nodes[nodeIndex].PolygonIndex = -1;

            // ... Partition logic same as original ...
            // 简化的 Partition
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

            nodes[nodeIndex].Left = BuildRecursive(start, leftCount);
            nodes[nodeIndex].Right = BuildRecursive(start + leftCount, count - leftCount);
            return nodeIndex;
        }

        public PolygonData QueryBestParent(Vector2 point, float holeArea)
        {
            if (srcData == null || srcData.Count == 0) return null;
            return QueryRecursive(0, point, holeArea);
        }

        private PolygonData QueryRecursive(int nodeIdx, Vector2 point, float holeArea)
        {
            ref FlatNode node = ref nodes[nodeIdx]; // 使用 ref 减少拷贝
            if (!node.Box.Contains(new Vector3(point.x, point.y, 0))) return null;

            if (node.PolygonIndex != -1)
            {
                PolygonData candidate = srcData[node.PolygonIndex];
                if (candidate.Area > holeArea && IsPointInPolygon(point, candidate.OuterLoop)) return candidate;
                return null;
            }
            PolygonData l = QueryRecursive(node.Left, point, holeArea);
            PolygonData r = QueryRecursive(node.Right, point, holeArea);
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
            srcData = null; // 断开引用
            nodes = null;
            indices = null;
        }
    }
}