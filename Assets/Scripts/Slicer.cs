using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices; // 用于内联优化

public static class Slicer
{
    // =================================================================================
    //                                  配置常量
    // =================================================================================
    private const float MIN_VERT_DIST = 0.01f; // 最小顶点距离，用于去重
    private const float MIN_VERT_DIST_SQ = MIN_VERT_DIST * MIN_VERT_DIST;
    private const float AREA_THRESHOLD = 0.01f; // 忽略过小的碎片

    // =================================================================================
    //                                  数据结构
    // =================================================================================

    // 用于图遍历的边哈希键，替代字符串，避免GC
    private struct EdgeKey : System.IEquatable<EdgeKey>
    {
        private readonly int x1, y1, x2, y2;

        public EdgeKey(Vector2 u, Vector2 v)
        {
            // 量化坐标以匹配 MIN_VERT_DIST (0.01) 的精度
            // 乘以100并取整，相当于保留两位小数
            x1 = (int)(u.x * 100);
            y1 = (int)(u.y * 100);
            x2 = (int)(v.x * 100);
            y2 = (int)(v.y * 100);
        }

        public bool Equals(EdgeKey other) => x1 == other.x1 && y1 == other.y1 && x2 == other.x2 && y2 == other.y2;

        public override int GetHashCode()
        {
            // 简单的位移异或哈希
            int hash = 17;
            hash = hash * 31 + x1;
            hash = hash * 31 + y1;
            hash = hash * 31 + x2;
            hash = hash * 31 + y2;
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
            foreach (var p in OuterLoop)
            {
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

    // =================================================================================
    //                                  对外接口
    // =================================================================================

    /// <summary>
    /// 切割核心入口
    /// </summary>
    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();

        if (polyCollider == null || meshFilter == null) return;

        // 1. 优先尝试从 SliceableGenerator 组件中读取“老祖宗”的包围盒
        // 2. 如果没有（说明可能是普通物体），则计算当前的局部包围盒
        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();

        if (generator != null && generator.hasUVReference)
        {
            // 找到了黑匣子，直接用老祖宗的数据
            referenceRect = generator.uvReferenceRect;
        }
        else
        {
            // 没找到（或者是第一次切且没用Generator），计算当前的作为参照
            referenceRect = CalculateLocalBounds(polyCollider);
        }

        // 局部坐标转换
        Vector2 localSliceStart = target.transform.InverseTransformPoint(worldStart);
        Vector2 localSliceEnd = target.transform.InverseTransformPoint(worldEnd);


        Vector2 cutDirection = (localSliceEnd - localSliceStart).normalized;

        // 如果点重合导致方向为0，直接退出
        if (cutDirection == Vector2.zero) return;

        // 计算延长长度：取包围盒宽高的最大值，或者对角线长度，再乘个系数确保安全
        // referenceRect 是局部坐标下的包围盒（老祖宗的，或者当前的）
        // 如果物体被切得很小，referenceRect 依然是老祖宗的，这没问题，只会延长得更多一点，更安全
        float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;

        // 向两端延伸
        Vector2 extendedStart = localSliceStart - cutDirection * extensionLength;
        Vector2 extendedEnd = localSliceEnd + cutDirection * extensionLength;

        localSliceStart = extendedStart;
        localSliceEnd = extendedEnd;

        // 提取轮廓
        List<List<Vector2>> originalPaths = new List<List<Vector2>>();
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            originalPaths.Add(new List<Vector2>(polyCollider.GetPath(i)));
        }

        // 核心计算
        List<PolygonData> slicedPolygons = null;
        try
        {
            slicedPolygons = CalculateSlicedPolygons(originalPaths, localSliceStart, localSliceEnd);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Slicer] 算法错误: {e.Message}");
            return;
        }

        // 验证结果
        if (slicedPolygons == null || slicedPolygons.Count == 0) return;

        // 生成物体
        bool success = true;
        try
        {
            foreach (var polyData in slicedPolygons)
            {
                CreateSlicedObject(polyData, target, meshRenderer.sharedMaterial, originalRb,referenceRect);
            }
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] Mesh生成错误: {e.Message}");
        }

        // 仅在成功时销毁原物体
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
    }//计算包围盒

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
            // 1.1 计算交点
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

            // 1.2 排序交点
            hits.Sort((a, b) => {
                if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
                return Vector2.SqrMagnitude(a.Point - path[a.SegmentIndex]).CompareTo(Vector2.SqrMagnitude(b.Point - path[b.SegmentIndex]));
            });

            // 1.3 插入交点并重建轮廓
            List<Vector2> newPathVertices = new List<Vector2>();
            int hitIndex = 0;

            for (int i = 0; i < path.Count; i++)
            {
                Vector2 currentVert = path[i];
                // 顶点去重
                if (newPathVertices.Count == 0 || Vector2.SqrMagnitude(newPathVertices[newPathVertices.Count - 1] - currentVert) > MIN_VERT_DIST_SQ)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < hits.Count && hits[hitIndex].SegmentIndex == i)
                {
                    Vector2 p = hits[hitIndex].Point;
                    // 交点去重
                    if (Vector2.SqrMagnitude(newPathVertices[newPathVertices.Count - 1] - p) > MIN_VERT_DIST_SQ)
                    {
                        newPathVertices.Add(p);
                        cutIntersections.Add(p);
                    }
                    hitIndex++;
                }
            }
            // 环路首尾去重
            if (newPathVertices.Count > 1 && Vector2.SqrMagnitude(newPathVertices[0] - newPathVertices[newPathVertices.Count - 1]) < MIN_VERT_DIST_SQ)
                newPathVertices.RemoveAt(newPathVertices.Count - 1);

            // 1.4 将轮廓边加入图
            for (int i = 0; i < newPathVertices.Count; i++)
            {
                AddEdge(graph, newPathVertices[i], newPathVertices[(i + 1) % newPathVertices.Count]);
            }
        }

        // --- Phase 2: 处理切割缝 ---
        // 全局交点去重
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

        // 按在切割线上的位置排序
        cutIntersections.Sort((a, b) => {
            float distA = Vector2.Dot(a - start, end - start);
            float distB = Vector2.Dot(b - start, end - start);
            return distA.CompareTo(distB);
        });

        // 奇偶连接 (0-1, 2-3)
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

            // CCW (Area > 0) -> Solid, CW (Area < 0) -> Hole
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

        // --- Phase 4: 归属权分配 ---
        foreach (var hole in holes)
        {
            PolygonData bestParent = null;
            float minParentArea = float.MaxValue;
            Vector2 centroid = GetCentroid(hole);

            foreach (var solid in solids)
            {
                // Bounds Check
                if (!solid.Bounds.Contains(new Vector3(centroid.x, centroid.y, 0))) continue;
                // Area Check
                if (solid.Area < Mathf.Abs(SignedArea(hole))) continue;
                // Point-in-Polygon Check
                if (solid.Area < minParentArea && IsPointInPolygon(centroid, solid.OuterLoop))
                {
                    bestParent = solid;
                    minParentArea = solid.Area;
                }
            }

            if (bestParent != null)
            {
                bestParent.Holes.Add(hole);
            }
            // 孤儿孔洞直接丢弃
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

        // 避免重复边
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
        HashSet<EdgeKey> visitedEdges = new HashSet<EdgeKey>(); // 使用 Struct 替代 String，0 GC

        foreach (var startNode in graph.Keys)
        {
            var neighbors = new List<Vector2>(graph[startNode]);
            foreach (var nextNode in neighbors)
            {
                EdgeKey edgeKey = new EdgeKey(startNode, nextNode);
                if (visitedEdges.Contains(edgeKey)) continue;

                List<Vector2> currentLoop = new List<Vector2>();
                Vector2 curr = startNode;
                Vector2 next = nextNode;
                currentLoop.Add(curr);

                int watchdog = 0;
                bool loopClosed = false;

                while (watchdog++ < 3000)
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
        if (float.IsNaN(incomingDir.x)) return Vector2.zero;

        float bestAngle = -1.0f;
        Vector2 bestNext = Vector2.zero;
        Vector2 backDir = -incomingDir;

        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 neighbor = neighbors[i];
            // 除非是死胡同，否则不走回头路
            if (Vector2.SqrMagnitude(neighbor - prev) < MIN_VERT_DIST_SQ && count > 1) continue;

            Vector2 outgoingDir = (neighbor - curr).normalized;
            if (float.IsNaN(outgoingDir.x)) continue;

            float angle = Vector2.SignedAngle(backDir, outgoingDir);
            if (angle < 0) angle += 360f;

            // 找最大角 (左转)
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

    // 简化路径，移除共线点和过近点
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

    private static void CreateSlicedObject(PolygonData data, GameObject originalTemplate, Material mat, Rigidbody2D originalRb,Rect uvRefRect)
    {
        string baseName = originalTemplate.name.Replace("_Piece", "");
        GameObject newObj = new GameObject(baseName + "_Piece");
        newObj.transform.position = originalTemplate.transform.position;
        newObj.transform.rotation = originalTemplate.transform.rotation;
        newObj.transform.localScale = originalTemplate.transform.localScale;
        newObj.layer = originalTemplate.layer;
        newObj.tag = originalTemplate.tag;

        // 1. 合并
        List<Vector2> mergedVertices = PolygonHoleMerger.Merge(data.OuterLoop, data.Holes);

        // 2. Mesh
        Vector3[] vertices3D = new Vector3[mergedVertices.Count];
        Vector2[] uvs = new Vector2[mergedVertices.Count];
        Vector2[] vertices2D = mergedVertices.ToArray();

        // 基于原物体包围盒的 UV 插值
        // 原理：(当前点 - 最小点) / 总宽高 = 0~1 的比例
        float width = uvRefRect.width;
        float height = uvRefRect.height;
        float minX = uvRefRect.x;
        float minY = uvRefRect.y;

        // 防止除以0
        if (width < 0.0001f) width = 1;
        if (height < 0.0001f) height = 1;

        for (int i = 0; i < mergedVertices.Count; i++)
        {
            vertices3D[i] = mergedVertices[i];
            // 计算相对位置
            float u = (mergedVertices[i].x - minX) / width;
            float v = (mergedVertices[i].y - minY) / height;
            uvs[i] = new Vector2(u,v);
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

        // 3. Collider
        PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
        pc.pathCount = 1 + data.Holes.Count;
        pc.SetPath(0, data.OuterLoop.ToArray());
        for (int i = 0; i < data.Holes.Count; i++)
        {
            pc.SetPath(i + 1, data.Holes[i].ToArray());
        }
        // 4. 传递“黑匣子”给新碎片
        // 这样新碎片被切时，Slicer 就能读到老祖宗的数据，而不是用新碎片的数据
        SliceableGenerator newGen = newObj.AddComponent<SliceableGenerator>();
        newGen.hasUVReference = true;
        newGen.uvReferenceRect = uvRefRect;
        newGen.autoGenerateOnStart = false; // 已经是Mesh了，不需要再生成

        // 5.. 物理
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