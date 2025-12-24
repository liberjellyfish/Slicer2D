using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

public static class Slicer
{
    // =================================================================================
    //                                  配置常量
    // =================================================================================
    private const float MIN_VERT_DIST = 0.01f;
    private const float MIN_VERT_DIST_SQ = MIN_VERT_DIST * MIN_VERT_DIST;
    private const float AREA_THRESHOLD = 0.01f;

    // =================================================================================
    //                                  数据结构
    // =================================================================================

    // 优化的边哈希键
    private readonly struct EdgeKey : System.IEquatable<EdgeKey>
    {
        private readonly int x1, y1, x2, y2;
        public EdgeKey(Vector2 u, Vector2 v)
        {
            // 精度匹配 MIN_VERT_DIST (0.01 -> *100)
            x1 = (int)(u.x * 100); y1 = (int)(u.y * 100);
            x2 = (int)(v.x * 100); y2 = (int)(v.y * 100);
        }
        public bool Equals(EdgeKey other) => x1 == other.x1 && y1 == other.y1 && x2 == other.x2 && y2 == other.y2;
        public override int GetHashCode()
        {
            // 使用更分散的哈希算法
            int hash = 17;
            hash = hash * 31 + x1; hash = hash * 31 + y1;
            hash = hash * 31 + x2; hash = hash * 31 + y2;
            return hash;
        }
    }

    private class PolygonData
    {
        public List<Vector2> OuterLoop;
        public List<List<Vector2>> Holes;
        public float Area;
        public Bounds Bounds;
        public PolygonData() { Holes = new List<List<Vector2>>(); }

        public void RecalculateBounds()
        {
            if (OuterLoop == null || OuterLoop.Count == 0) return;
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;
            for (int i = 0; i < OuterLoop.Count; i++)
            {
                Vector2 p = OuterLoop[i];
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }
            Bounds = new Bounds(new Vector3((minX + maxX) / 2, (minY + maxY) / 2, 0), new Vector3(maxX - minX, maxY - minY, 1));
        }
    }

    private struct IntersectionInfo
    {
        public Vector2 Point;
        public float T;
        public int SegmentIndex;
    }

    // [优化]：扁平化 AABB 树，替代原本的递归类 PolyNode
    // 专门用于存储 PolygonData，零 GC，缓存友好
    private struct NativePolyTree
    {
        public struct FlatNode
        {
            public Bounds Box;
            public int PolygonIndex; // 指向 solids 列表的索引，-1 表示非叶子
            public int Left;
            public int Right;
        }

        private FlatNode[] nodes;
        private int[] indices; // 间接索引数组，用于排序而不移动实际 PolygonData
        private int nodesUsed;
        private List<PolygonData> srcData;

        public void Build(List<PolygonData> solids)
        {
            if (solids == null || solids.Count == 0) return;
            srcData = solids;
            int count = solids.Count;

            // 分配内存 (节点数最多 2*N)
            if (nodes == null || nodes.Length < count * 2)
                nodes = new FlatNode[count * 2];

            if (indices == null || indices.Length < count)
                indices = new int[count];

            // 初始化索引
            for (int i = 0; i < count; i++) indices[i] = i;

            nodesUsed = 0;
            BuildRecursive(0, count);
        }

        private int BuildRecursive(int start, int count)
        {
            int nodeIndex = nodesUsed++;
            // 计算总包围盒
            Bounds total = srcData[indices[start]].Bounds;
            for (int i = 1; i < count; i++)
            {
                total.Encapsulate(srcData[indices[start + i]].Bounds);
            }
            nodes[nodeIndex].Box = total;

            // 叶子节点
            if (count == 1)
            {
                nodes[nodeIndex].PolygonIndex = indices[start];
                nodes[nodeIndex].Left = -1;
                nodes[nodeIndex].Right = -1;
                return nodeIndex;
            }

            nodes[nodeIndex].PolygonIndex = -1;

            // 划分 (Partition)
            bool splitX = total.size.x > total.size.y;
            float mid = splitX ? total.center.x : total.center.y;

            // In-Place Partitioning (类似快排)
            int left = start;
            int right = start + count - 1;
            while (left <= right)
            {
                PolygonData p = srcData[indices[left]];
                float center = splitX ? p.Bounds.center.x : p.Bounds.center.y;

                if (center < mid)
                {
                    left++;
                }
                else
                {
                    // Swap indices
                    int temp = indices[left];
                    indices[left] = indices[right];
                    indices[right] = temp;
                    right--;
                }
            }

            int leftCount = left - start;
            if (leftCount == 0 || leftCount == count) leftCount = count / 2; // 防止死循环

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
            // AABB 剔除
            if (!nodes[nodeIdx].Box.Contains(new Vector3(point.x, point.y, 0))) return null;

            // 叶子节点处理
            if (nodes[nodeIdx].PolygonIndex != -1)
            {
                PolygonData candidate = srcData[nodes[nodeIdx].PolygonIndex];
                // 面积检查 & 精确点包含检查
                if (candidate.Area > holeArea && IsPointInPolygon(point, candidate.OuterLoop))
                {
                    return candidate;
                }
                return null;
            }

            // 递归查询左右子树
            // 优先找面积更小的父节点？这里逻辑是找任何合法的，然后取最小的。
            PolygonData l = QueryRecursive(nodes[nodeIdx].Left, point, holeArea);
            PolygonData r = QueryRecursive(nodes[nodeIdx].Right, point, holeArea);

            if (l != null && r != null)
                return l.Area < r.Area ? l : r;
            return l != null ? l : r;
        }
    }

    // =================================================================================
    //                                  对外接口
    // =================================================================================

    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();

        if (polyCollider == null || meshFilter == null) return;

        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();

        if (generator != null && generator.hasUVReference)
        {
            referenceRect = generator.uvReferenceRect;
        }
        else
        {
            referenceRect = CalculateLocalBounds(polyCollider);
        }

        Vector2 localSliceStart = target.transform.InverseTransformPoint(worldStart);
        Vector2 localSliceEnd = target.transform.InverseTransformPoint(worldEnd);
        Vector2 cutDirection = (localSliceEnd - localSliceStart).normalized;

        if (cutDirection == Vector2.zero) return;

        float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;
        Vector2 extendedStart = localSliceStart - cutDirection * extensionLength;
        Vector2 extendedEnd = localSliceEnd + cutDirection * extensionLength;

        localSliceStart = extendedStart;
        localSliceEnd = extendedEnd;

        List<List<Vector2>> originalPaths = new List<List<Vector2>>();
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            originalPaths.Add(new List<Vector2>(polyCollider.GetPath(i)));
        }

        List<PolygonData> slicedPolygons = null;
        try
        {
            slicedPolygons = CalculateSlicedPolygons(originalPaths, localSliceStart, localSliceEnd);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Slicer] 算法错误: {e.Message}\n{e.StackTrace}");
            return;
        }

        if (slicedPolygons == null || slicedPolygons.Count == 0) return;

        bool success = true;
        try
        {
            foreach (var polyData in slicedPolygons)
            {
                CreateSlicedObject(polyData, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
            }
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] Mesh生成错误: {e.Message}");
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }

    private static Rect CalculateLocalBounds(PolygonCollider2D col)
    {
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;

        for (int i = 0; i < col.pathCount; i++)
        {
            Vector2[] path = col.GetPath(i);
            foreach (var p in path)
            {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }
        }
        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }

    // =================================================================================
    //                                  核心算法逻辑
    // =================================================================================

    private static List<PolygonData> CalculateSlicedPolygons(List<List<Vector2>> paths, Vector2 start, Vector2 end)
    {
        Dictionary<Vector2, List<Vector2>> graph = new Dictionary<Vector2, List<Vector2>>();
        List<Vector2> cutIntersections = new List<Vector2>();

        // --- Phase 1: 构建拓扑图 ---
        foreach (var path in paths)
        {
            List<IntersectionInfo> hits = new List<IntersectionInfo>();
            for (int i = 0; i < path.Count; i++)
            {
                Vector2 p1 = path[i];
                Vector2 p2 = path[(i + 1) % path.Count];

                if (GetLineIntersection(p1, p2, start, end, out Vector2 intersection, out float t))
                {
                    hits.Add(new IntersectionInfo { Point = intersection, T = t, SegmentIndex = i });
                }
            }

            hits.Sort((a, b) => {
                if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
                return Vector2.SqrMagnitude(a.Point - path[a.SegmentIndex]).CompareTo(Vector2.SqrMagnitude(b.Point - path[b.SegmentIndex]));
            });

            List<Vector2> newPathVertices = new List<Vector2>();
            int hitIndex = 0;

            for (int i = 0; i < path.Count; i++)
            {
                Vector2 currentVert = path[i];
                if (newPathVertices.Count == 0 || Vector2.SqrMagnitude(newPathVertices[newPathVertices.Count - 1] - currentVert) > MIN_VERT_DIST_SQ)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < hits.Count && hits[hitIndex].SegmentIndex == i)
                {
                    Vector2 p = hits[hitIndex].Point;
                    if (Vector2.SqrMagnitude(newPathVertices[newPathVertices.Count - 1] - p) > MIN_VERT_DIST_SQ)
                    {
                        newPathVertices.Add(p);
                        cutIntersections.Add(p);
                    }
                    hitIndex++;
                }
            }
            if (newPathVertices.Count > 1 && Vector2.SqrMagnitude(newPathVertices[0] - newPathVertices[newPathVertices.Count - 1]) < MIN_VERT_DIST_SQ)
                newPathVertices.RemoveAt(newPathVertices.Count - 1);

            for (int i = 0; i < newPathVertices.Count; i++)
            {
                AddEdge(graph, newPathVertices[i], newPathVertices[(i + 1) % newPathVertices.Count]);
            }
        }

        // --- Phase 2: 处理切割缝 ---
        for (int i = cutIntersections.Count - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (Vector2.SqrMagnitude(cutIntersections[i] - cutIntersections[j]) < MIN_VERT_DIST_SQ)
                {
                    cutIntersections.RemoveAt(i);
                    break;
                }
            }
        }

        if (cutIntersections.Count < 2) return null;

        cutIntersections.Sort((a, b) => {
            float distA = Vector2.Dot(a - start, end - start);
            float distB = Vector2.Dot(b - start, end - start);
            return distA.CompareTo(distB);
        });

        int validCount = (cutIntersections.Count % 2 == 0) ? cutIntersections.Count : cutIntersections.Count - 1;
        for (int i = 0; i < validCount; i += 2)
        {
            Vector2 pA = cutIntersections[i];
            Vector2 pB = cutIntersections[i + 1];
            if (Vector2.SqrMagnitude(pA - pB) > MIN_VERT_DIST_SQ)
            {
                AddEdge(graph, pA, pB);
                AddEdge(graph, pB, pA);
            }
        }

        // --- Phase 3: 提取回路与层级分析 ---
        List<List<Vector2>> allLoops = ExtractLoops(graph);
        List<PolygonData> solids = new List<PolygonData>();
        List<List<Vector2>> holes = new List<List<Vector2>>();

        foreach (var rawLoop in allLoops)
        {
            List<Vector2> loop = SimplifyPath(rawLoop);
            float area = SignedArea(loop);
            if (Mathf.Abs(area) < AREA_THRESHOLD) continue;

            if (area > 0)
            {
                PolygonData poly = new PolygonData();
                poly.OuterLoop = loop;
                poly.Area = area;
                poly.RecalculateBounds();
                solids.Add(poly);
            }
            else
            {
                holes.Add(loop);
            }
        }

        // --- Phase 4: 归属权分配 (AABB 树加速) ---
        // 使用新实现的 NativePolyTree 替代原本的递归类
        NativePolyTree tree = new NativePolyTree();
        tree.Build(solids);

        foreach (var hole in holes)
        {
            Vector2 centroid = GetCentroid(hole);
            float holeAreaAbs = Mathf.Abs(SignedArea(hole));

            // O(log S) 查询最佳父节点
            PolygonData bestParent = tree.QueryBestParent(centroid, holeAreaAbs);

            if (bestParent != null)
            {
                bestParent.Holes.Add(hole);
            }
        }

        return solids;
    }

    // =================================================================================
    //                                  图论算法
    // =================================================================================

    private static void AddEdge(Dictionary<Vector2, List<Vector2>> graph, Vector2 u, Vector2 v)
    {
        if (Vector2.SqrMagnitude(u - v) < 1e-6f) return;
        if (!graph.ContainsKey(u)) graph[u] = new List<Vector2>();

        bool exists = false;
        foreach (var neighbor in graph[u])
        {
            if (Vector2.SqrMagnitude(neighbor - v) < 1e-6f) { exists = true; break; }
        }
        if (!exists) graph[u].Add(v);
    }

    private static List<List<Vector2>> ExtractLoops(Dictionary<Vector2, List<Vector2>> graph)
    {
        List<List<Vector2>> loops = new List<List<Vector2>>();
        HashSet<EdgeKey> visitedEdges = new HashSet<EdgeKey>();

        foreach (var startNode in graph.Keys)
        {
            // 优化：不直接 foreach graph[startNode]，而是先复制一份，防止遍历时修改（虽然这里不修改）
            var neighbors = graph[startNode];
            foreach (var nextNode in neighbors)
            {
                EdgeKey edgeKey = new EdgeKey(startNode, nextNode);
                if (visitedEdges.Contains(edgeKey)) continue;

                List<Vector2> currentLoop = new List<Vector2>();
                Vector2 curr = startNode;
                Vector2 next = nextNode;
                currentLoop.Add(curr);

                int watchdog = 0;
                // 动态看门狗阈值，避免复杂物体被错误截断
                int maxIterations = graph.Count * 2 + 100;
                bool loopClosed = false;

                while (watchdog++ < maxIterations)
                {
                    visitedEdges.Add(new EdgeKey(curr, next));
                    currentLoop.Add(next);

                    if (Vector2.SqrMagnitude(next - startNode) < MIN_VERT_DIST_SQ)
                    {
                        loopClosed = true;
                        break;
                    }

                    Vector2 prev = curr;
                    curr = next;

                    if (!graph.ContainsKey(curr) || graph[curr].Count == 0) break;
                    next = GetLeftMostNeighbor(prev, curr, graph[curr]);

                    // 死胡同
                    if (next == Vector2.zero && graph[curr].Count > 0 && next != graph[curr][0]) break;
                    if (next == Vector2.zero && graph[curr].Count > 0) next = graph[curr][0]; // Fallback
                    if (next == Vector2.zero) break;
                }

                if (loopClosed && currentLoop.Count > 2)
                {
                    currentLoop.RemoveAt(currentLoop.Count - 1);
                    loops.Add(currentLoop);
                }
            }
        }
        return loops;
    }

    // =================================================================================
    //                                  几何计算 (内联优化)
    // =================================================================================

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 GetLeftMostNeighbor(Vector2 prev, Vector2 curr, List<Vector2> neighbors)
    {
        Vector2 incomingDir = (curr - prev).normalized;
        if (incomingDir == Vector2.zero) incomingDir = Vector2.right; // 修复零向量风险

        float bestAngle = -9999f;
        Vector2 bestNext = Vector2.zero;
        Vector2 backDir = -incomingDir;

        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 neighbor = neighbors[i];
            // 除非是死胡同，否则不走回头路
            if (Vector2.SqrMagnitude(neighbor - prev) < MIN_VERT_DIST_SQ && count > 1) continue;

            Vector2 outgoingDir = (neighbor - curr).normalized;
            if (outgoingDir == Vector2.zero) continue;

            // 使用 SignedAngle 寻找最左侧分支 (最大逆时针角度)
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

        // 稍微放宽一点容差，捕捉顶点相交
        if (u >= -1e-4f && u <= 1.0001f && v >= -1e-4f && v <= 1.0001f)
        {
            t = Mathf.Clamp01(v);
            intersection = p1 + Mathf.Clamp01(u) * (p2 - p1);
            return true;
        }
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 GetCentroid(List<Vector2> points)
    {
        if (points == null || points.Count == 0) return Vector2.zero;
        Vector2 sum = Vector2.zero;
        foreach (var p in points) sum += p;
        return sum / points.Count;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    private static List<Vector2> SimplifyPath(List<Vector2> path)
    {
        if (path.Count < 3) return path;
        List<Vector2> simplified = new List<Vector2>();
        simplified.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            if (Vector2.SqrMagnitude(path[i] - simplified[simplified.Count - 1]) > MIN_VERT_DIST_SQ)
                simplified.Add(path[i]);
        }
        if (simplified.Count > 2 && Vector2.SqrMagnitude(simplified[0] - simplified[simplified.Count - 1]) < MIN_VERT_DIST_SQ)
            simplified.RemoveAt(simplified.Count - 1);
        return simplified;
    }

    // =================================================================================
    //                                  物体生成实现
    // =================================================================================

    private static void CreateSlicedObject(PolygonData data, GameObject originalTemplate, Material mat, Rigidbody2D originalRb, Rect uvRefRect)
    {
        string baseName = originalTemplate.name.Replace("_Piece", "");
        GameObject newObj = new GameObject(baseName + "_Piece");
        newObj.transform.position = originalTemplate.transform.position;
        newObj.transform.rotation = originalTemplate.transform.rotation;
        newObj.transform.localScale = originalTemplate.transform.localScale;
        newObj.layer = originalTemplate.layer;
        newObj.tag = originalTemplate.tag;

        List<Vector2> mergedVertices = PolygonHoleMerger.Merge(data.OuterLoop, data.Holes);

        Vector3[] vertices3D = new Vector3[mergedVertices.Count];
        Vector2[] uvs = new Vector2[mergedVertices.Count];
        Vector2[] vertices2D = mergedVertices.ToArray();

        float width = uvRefRect.width;
        float height = uvRefRect.height;
        float minX = uvRefRect.x;
        float minY = uvRefRect.y;

        if (width < 0.0001f) width = 1;
        if (height < 0.0001f) height = 1;

        for (int i = 0; i < mergedVertices.Count; i++)
        {
            vertices3D[i] = mergedVertices[i];
            float u = (mergedVertices[i].x - minX) / width;
            float v = (mergedVertices[i].y - minY) / height;
            uvs[i] = new Vector2(u, v);
        }

        int[] indices = Triangulator.Triangulate(vertices2D);
        Mesh mesh = new Mesh();
        mesh.vertices = vertices3D;
        mesh.uv = uvs;
        mesh.triangles = indices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        MeshFilter mf = newObj.AddComponent<MeshFilter>();
        mf.mesh = mesh;
        MeshRenderer mr = newObj.AddComponent<MeshRenderer>();
        mr.material = mat;

        PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
        pc.pathCount = 1 + data.Holes.Count;
        pc.SetPath(0, data.OuterLoop.ToArray());
        for (int i = 0; i < data.Holes.Count; i++)
        {
            pc.SetPath(i + 1, data.Holes[i].ToArray());
        }

        SliceableGenerator newGen = newObj.AddComponent<SliceableGenerator>();
        newGen.hasUVReference = true;
        newGen.uvReferenceRect = uvRefRect;
        newGen.autoGenerateOnStart = false;

        if (originalRb != null)
        {
            Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
            newRb.mass = originalRb.mass * (data.Area / 10f);
            newRb.useAutoMass = true;
            newRb.linearDamping = originalRb.linearDamping;
            newRb.angularDamping = originalRb.angularDamping;
            newRb.gravityScale = originalRb.gravityScale;
            newRb.collisionDetectionMode = originalRb.collisionDetectionMode;
            newRb.interpolation = originalRb.interpolation;
            newRb.sharedMaterial = originalRb.sharedMaterial;
            newRb.linearVelocity = originalRb.linearVelocity;
            newRb.angularVelocity = originalRb.angularVelocity;
        }
    }
}