using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices; // 用于性能优化注解

/// <summary>
/// 静态切割工具类：负责处理几何切割、拓扑重构与网格生成
/// </summary>
public static class Slicer
{
    #region 数据结构定义

    // 使用结构体减少GC，包含 ID 用于排序唯一性校验
    public struct VertexData
    {
        public Vector3 Position;    // 局部坐标
        public Vector2 UV;          // 纹理坐标
        public bool IsIntersection; // 是否为切割产生的交点
        public int ID;              // 全局唯一ID (用于字典索引)
    }

    #endregion

    #region Public API (对外接口)

    /// <summary>
    /// 执行切割操作的主入口
    /// </summary>
    /// <param name="target">被切割的物体</param>
    /// <param name="worldStart">切割线起点(世界坐标)</param>
    /// <param name="worldEnd">切割线终点(世界坐标)</param>
    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        // 1. === 坐标系转换 ===
        // 缓存 Transform 减少调用开销
        Transform t = target.transform;
        Vector3 localP1 = t.InverseTransformPoint(worldStart);
        Vector3 localP2 = t.InverseTransformPoint(worldEnd);

        // 转换为 2D 切割平面 (XY)
        Vector2 sliceStart = new Vector2(localP1.x, localP1.y);
        Vector2 sliceEnd = new Vector2(localP2.x, localP2.y);

        // 2. === 获取原始网格数据 ===
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        if (meshFilter == null) return;

        Mesh originalMesh = meshFilter.mesh;
        // 缓存顶点和UV数组
        Vector3[] oldVertices = originalMesh.vertices;
        Vector2[] oldUVs = originalMesh.uv;

        // 优化：预分配 List 容量，避免循环中频繁扩容
        // 预估容量：原顶点数 + 预留几个交点空间
        int capacity = oldVertices.Length + 4;
        List<VertexData> shapePoints = new List<VertexData>(capacity);

        int globalIDCounter = 0;

        // 初始化顶点数据
        for (int i = 0; i < oldVertices.Length; i++)
        {
            shapePoints.Add(new VertexData
            {
                Position = oldVertices[i],
                UV = oldUVs[i],
                IsIntersection = false,
                ID = globalIDCounter++
            });
        }

        // 3. === 核心几何切割 (Sutherland-Hodgman) ===
        // 预分配结果列表
        List<VertexData> posSide = new List<VertexData>(capacity);
        List<VertexData> negSide = new List<VertexData>(capacity);

        PerformSutherlandHodgman(shapePoints, sliceStart, sliceEnd, ref globalIDCounter, posSide, negSide);

        // 校验有效性
        if (posSide.Count < 3 || negSide.Count < 3) return;

        // 4. === 拓扑重构与物体生成 ===
        Material originalMat = target.GetComponent<MeshRenderer>().sharedMaterial;

        // 传入切线信息用于拓扑排序
        CreateObjectsFromTopology(target, posSide, originalMat, "PositiveMesh", sliceStart, sliceEnd);
        CreateObjectsFromTopology(target, negSide, originalMat, "NegativeMesh", sliceStart, sliceEnd);

        // 销毁原物体
        GameObject.Destroy(target);
    }

    #endregion

    #region Core Logic (核心切割逻辑)

    /// <summary>
    /// 执行 Sutherland-Hodgman 多边形裁剪算法
    /// </summary>
    private static void PerformSutherlandHodgman(
        List<VertexData> inputPoints,
        Vector2 lineStart,
        Vector2 lineEnd,
        ref int idCounter,
        List<VertexData> posSide,
        List<VertexData> negSide)
    {
        int count = inputPoints.Count;
        for (int i = 0; i < count; i++)
        {
            VertexData v1 = inputPoints[i];
            VertexData v2 = inputPoints[(i + 1) % count]; // 闭环处理

            bool v1Side = IsPointOnPositiveSide(lineStart, lineEnd, v1.Position);
            bool v2Side = IsPointOnPositiveSide(lineStart, lineEnd, v2.Position);

            if (v1Side == v2Side)
            {
                // 情况A：两点同侧 -> 保留 v1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);
            }
            else
            {
                // 情况B：两点异侧 -> 连线被切断
                // 1. 先保留 v1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);

                // 2. 计算并生成交点
                float t = GetIntersectionT(lineStart, lineEnd, v1.Position, v2.Position);

                VertexData intersectionPoint = new VertexData();
                intersectionPoint.Position = Vector3.Lerp(v1.Position, v2.Position, t);
                intersectionPoint.UV = Vector2.Lerp(v1.UV, v2.UV, t);
                intersectionPoint.IsIntersection = true;
                intersectionPoint.ID = idCounter++; // 分配唯一ID

                // 3. 交点同时加入两侧
                posSide.Add(intersectionPoint);
                negSide.Add(intersectionPoint);
            }
        }
    }

    /// <summary>
    /// 处理切割后的数据，生成游戏物体
    /// </summary>
    private static void CreateObjectsFromTopology(
        GameObject original,
        List<VertexData> rawPoints,
        Material mat,
        string baseName,
        Vector2 lineStart,
        Vector2 lineEnd)
    {
        // 1. 拓扑分离：根据交点排序剪断“非法跨越”的边
        List<List<VertexData>> polygons = SplitPolygonBySortedIntersections(rawPoints, lineStart, lineEnd);

        int counter = 0;
        foreach (var polyPoints in polygons)
        {
            // 过滤无效碎片
            if (polyPoints.Count < 3) continue;

            // 2. 生成临时大网格
            Mesh bigMesh = GenerateMesh(polyPoints);

            // 3. 二次检查：物理离岛分离 (处理那种没有交点但物理断开的情况)
            List<Mesh> separatedMeshes = MeshSeparator.Separate(bigMesh);

            // 4. 实例化最终物体
            foreach (var subMesh in separatedMeshes)
            {
                InstantiateSplitObject(original, subMesh, mat, $"{baseName}_{counter++}");
            }
        }
    }

    #endregion

    #region Topology (拓扑排序与重构)

    /// <summary>
    /// 【核心算法】基于交点 Rank 排序的拓扑分离
    /// </summary>
    private static List<List<VertexData>> SplitPolygonBySortedIntersections(
        List<VertexData> points,
        Vector2 lineStart,
        Vector2 lineEnd)
    {
        // Step 1: 提取交点
        List<VertexData> intersections = new List<VertexData>();
        for (int i = 0; i < points.Count; i++)
        {
            if (points[i].IsIntersection) intersections.Add(points[i]);
        }

        // 容错：必须成对出现
        if (intersections.Count % 2 != 0)
            return new List<List<VertexData>>() { points };

        // Step 2: 对交点排序 (投影到切割线上)
        Vector2 lineDir = (lineEnd - lineStart).normalized;
        intersections.Sort((a, b) => {
            float distA = Vector2.Dot(new Vector2(a.Position.x, a.Position.y), lineDir);
            float distB = Vector2.Dot(new Vector2(b.Position.x, b.Position.y), lineDir);
            return distA.CompareTo(distB);
        });

        // Step 3: 建立 Rank 索引表 (ID -> Rank)
        Dictionary<int, int> intersectionRank = new Dictionary<int, int>(intersections.Count);
        for (int i = 0; i < intersections.Count; i++)
        {
            intersectionRank[intersections[i].ID] = i;
        }

        // Step 4: 巡逻一圈，根据 Rank 判定是否剪断
        List<List<VertexData>> result = new List<List<VertexData>>();
        List<VertexData> currentChain = new List<VertexData>(points.Count);

        int n = points.Count;
        bool lastEdgeWasCut = false;

        for (int i = 0; i < n; i++)
        {
            VertexData p1 = points[i];
            VertexData p2 = points[(i + 1) % n];

            currentChain.Add(p1);

            // 如果这条边的两端都是交点，它是潜在的“桥”
            if (p1.IsIntersection && p2.IsIntersection)
            {
                if (intersectionRank.TryGetValue(p1.ID, out int rank1) &&
                    intersectionRank.TryGetValue(p2.ID, out int rank2))
                {
                    int rankDiff = Mathf.Abs(rank1 - rank2);
                    int minRank = Mathf.Min(rank1, rank2);

                    // === 剪断判据 ===
                    // 1. RankDiff != 1: 跨级连接（如 0 连到 3），必断
                    // 2. MinRank 是奇数: 位于 (Out->In) 区间，是空气，必断
                    if (rankDiff != 1 || minRank % 2 != 0)
                    {
                        // 剪断！打包当前链条
                        if (currentChain.Count >= 3)
                        {
                            result.Add(new List<VertexData>(currentChain));
                        }
                        // 开启新链条
                        currentChain = new List<VertexData>(points.Count);
                        if (i == n - 1) lastEdgeWasCut = true;
                    }
                }
            }
        }

        // Step 5: 首尾闭合逻辑 (修复环状物体被误切)
        if (currentChain.Count > 0)
        {
            // 如果最后一条边是实心的（没断），说明它是第一块碎片的前半部分
            if (!lastEdgeWasCut && result.Count > 0)
            {
                result[0].InsertRange(0, currentChain);
            }
            else if (currentChain.Count >= 3)
            {
                result.Add(currentChain);
            }
        }

        return result;
    }

    #endregion

    #region Mesh Generation (网格与对象生成)

    private static void InstantiateSplitObject(GameObject original, Mesh mesh, Material mat, string name)
    {
        GameObject newObj = new GameObject(name);
        Transform t = newObj.transform;
        Transform originalT = original.transform;

        // 继承变换
        t.SetPositionAndRotation(originalT.position, originalT.rotation);
        t.localScale = originalT.localScale;
        newObj.layer = original.layer;

        // 添加渲染组件
        newObj.AddComponent<MeshFilter>().mesh = mesh;
        newObj.AddComponent<MeshRenderer>().material = mat;

        // 生成碰撞体 (关键：边缘提取)
        PolygonCollider2D collider = newObj.AddComponent<PolygonCollider2D>();
        Vector2[] boundary = GetBoundaryPath(mesh);

        if (boundary != null && boundary.Length >= 3)
        {
            collider.SetPath(0, boundary);
        }
        else
        {
            // 兜底方案：使用凸包或直接点集
            Vector2[] fallback = new Vector2[mesh.vertexCount];
            for (int k = 0; k < mesh.vertexCount; k++)
                fallback[k] = mesh.vertices[k]; // Vector3隐式转Vector2
            collider.SetPath(0, fallback);
        }

        // 继承物理速度
        Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
        Rigidbody2D oldRb = original.GetComponent<Rigidbody2D>();
        if (oldRb != null)
        {
            newRb.linearVelocity = oldRb.linearVelocity;
            newRb.angularVelocity = oldRb.angularVelocity;
        }
    }

    private static Mesh GenerateMesh(List<VertexData> points)
    {
        int count = points.Count;
        Vector3[] vertices = new Vector3[count];
        Vector2[] uvs = new Vector2[count];
        Vector2[] points2D = new Vector2[count];

        for (int i = 0; i < count; i++)
        {
            vertices[i] = points[i].Position;
            uvs[i] = points[i].UV;
            points2D[i] = new Vector2(points[i].Position.x, points[i].Position.y);
        }

        // 调用耳切法
        int[] triangles = Triangulator.Triangulate(points2D);

        Mesh mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    /// <summary>
    /// 提取 Mesh 的唯一外轮廓路径
    /// </summary>
    private static Vector2[] GetBoundaryPath(Mesh mesh)
    {
        int[] tris = mesh.triangles;
        Vector3[] verts = mesh.vertices;

        // 优化：指定 Dictionary 容量
        int estimatedEdges = tris.Length;
        Dictionary<long, int> edgeCounts = new Dictionary<long, int>(estimatedEdges);
        Dictionary<int, int> edgeGraph = new Dictionary<int, int>(estimatedEdges / 3);

        // 1. 统计边频次
        for (int i = 0; i < tris.Length; i += 3)
        {
            AddEdge(edgeCounts, tris[i], tris[i + 1]);
            AddEdge(edgeCounts, tris[i + 1], tris[i + 2]);
            AddEdge(edgeCounts, tris[i + 2], tris[i]);
        }

        // 2. 筛选边界边 (频次=1)
        for (int i = 0; i < tris.Length; i += 3)
        {
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i], tris[i + 1]);
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i + 1], tris[i + 2]);
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i + 2], tris[i]);
        }

        if (edgeGraph.Count == 0) return null;

        // 3. 构建闭合回路
        List<Vector2> path = new List<Vector2>(verts.Length);
        int startNode = 0;
        foreach (var key in edgeGraph.Keys) { startNode = key; break; }

        int current = startNode;
        int safety = 0;
        int maxLoop = verts.Length * 2; // 安全限制

        while (safety++ < maxLoop)
        {
            path.Add(verts[current]); // Vector3 -> Vector2

            if (edgeGraph.TryGetValue(current, out int nextNode))
            {
                current = nextNode;
            }
            else break;

            if (current == startNode) break;
        }

        return path.ToArray();
    }

    // 内联优化：高频调用的 Helper
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddEdge(Dictionary<long, int> counts, int a, int b)
    {
        long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        if (counts.ContainsKey(key)) counts[key]++; else counts[key] = 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ProcessBoundaryEdge(Dictionary<long, int> counts, Dictionary<int, int> graph, int a, int b)
    {
        long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        if (counts[key] == 1)
        {
            if (!graph.ContainsKey(a)) graph[a] = b;
        }
    }

    #endregion

    #region Math & Geometry (高性能数学计算)

    // 内联优化
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointOnPositiveSide(Vector2 lineStart, Vector2 lineEnd, Vector3 p)
    {
        // 二维叉积：(LineVec) x (PointVec)
        // 展开: (x2-x1)*(py-y1) - (y2-y1)*(px-x1)
        return ((lineEnd.x - lineStart.x) * (p.y - lineStart.y) - (lineEnd.y - lineStart.y) * (p.x - lineStart.x)) > 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float GetIntersectionT(Vector2 lineStart, Vector2 lineEnd, Vector2 segStart, Vector2 segEnd)
    {
        float d1 = SignedDistance(lineStart, lineEnd, segStart);
        float d2 = SignedDistance(lineStart, lineEnd, segEnd);
        // 使用 Abs 避免符号判断，提高指令流水线效率
        return Mathf.Abs(d1) / (Mathf.Abs(d1) + Mathf.Abs(d2));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SignedDistance(Vector2 lineStart, Vector2 lineEnd, Vector2 p)
    {
        return (lineEnd.x - lineStart.x) * (p.y - lineStart.y) - (lineEnd.y - lineStart.y) * (p.x - lineStart.x);
    }

    #endregion
}