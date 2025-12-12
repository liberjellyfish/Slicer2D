using UnityEngine;
using System.Collections.Generic;

public static class Slicer
{
    //存储顶点信息
    public struct VertexData
    {
        public Vector3 Position;
        public Vector2 UV;
    }

    //切割核心入口
    public static void Slice(GameObject target, Vector3 worldStart,Vector3 worldEnd)
    {
        //世界坐标->局部坐标
        Vector3 localP1 = target.transform.InverseTransformPoint(worldStart);
        Vector3 localP2 = target.transform.InverseTransformPoint(worldEnd);
        //转换为2D计算
        Vector2 p1 = new Vector2(localP1.x,localP1.y);
        Vector2 p2 = new Vector2(localP2.x,localP2.y);

        //获取网格数据
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        if (meshFilter == null) return;

        Mesh originalMesh = meshFilter.mesh;
        Vector3[] oldVertices = originalMesh.vertices;
        Vector2[] oldUVs = originalMesh.uv;

        //准备数据表
        List<Vector2> originalPoly2D = new List<Vector2>();
        List<VertexData> shapePoints = new List<VertexData>();
        for(int i=0;i<oldVertices.Length;i++)
        {
            shapePoints.Add(new VertexData { Position = oldVertices[i], UV = oldUVs[i] });
            originalPoly2D.Add(new Vector2(oldVertices[i].x, oldVertices[i].y));
        }

        //准备两列表
        List<VertexData> posSide = new List<VertexData>();
        List<VertexData> negSide = new List<VertexData>();

        //核心几何算法
        for (int i = 0; i < shapePoints.Count; i++)
        {
            // 获取当前边的两个端点
            VertexData v1 = shapePoints[i];
            VertexData v2 = shapePoints[(i + 1) % shapePoints.Count]; // 取模以形成闭环

            // 判断这两个点在直线的哪一侧
            // 注意：这里使用的是基于直线的判定，哪怕线段没有物理接触，
            // 只要这两个点分布在无限直线的两侧，依然会被判为 true/false 不同
            bool v1Side = IsPointOnPositiveSide(p1, p2, v1.Position);
            bool v2Side = IsPointOnPositiveSide(p1, p2, v2.Position);

            // 逻辑分支
            if (v1Side == v2Side)
            {
                // 情况 A: 两个点在同一侧 -> 只要保留 V1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);
            }
            else
            {
                // 情况 B: 两个点在不同侧 -> 说明被线切断了
                // 1. 先加入 V1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);

                // 2. 计算交点 (Intersection)
                float t = GetIntersectionT(p1, p2, v1.Position, v2.Position);

                // 3. 插值生成新顶点
                VertexData intersectionPoint = new VertexData();
                intersectionPoint.Position = Vector3.Lerp(v1.Position, v2.Position, t);
                intersectionPoint.UV = Vector2.Lerp(v1.UV, v2.UV, t);

                //P = A + (B - A) * t

                // 4. 交点是两个新形状共有的，都要加入
                posSide.Add(intersectionPoint);
                negSide.Add(intersectionPoint);
            }
        }

        if (posSide.Count < 3 || negSide.Count < 3 )
        {
            //Debug.Log("[Slicer] 未产生有效切割 (所有点都在同一侧)");
            return;
        }

        //Debug.Log($"[Slicer] 切割成功! 上半部分点数: {posSide.Count}, 下半部分点数: {negSide.Count}");

        // 利用 posSide 和 negSide 生成两个新的 Mesh

        Material originalMat = target.GetComponent<MeshRenderer>().sharedMaterial;

        //生成两个新部分，销毁老部分
        CreateObjectsFromTopology(target, posSide, originalMat, "PositiveMesh", p1, p2, originalPoly2D);
        CreateObjectsFromTopology(target, negSide, originalMat, "NegativeMesh", p1, p2, originalPoly2D);


        GameObject.Destroy(target);

    }
    // 拓扑分离与生成逻辑
    private static void CreateObjectsFromTopology(GameObject original, List<VertexData> rawPoints, Material mat, string baseName, Vector2 lineStart, Vector2 lineEnd, List<Vector2> originalPoly)
    {
        // A. 预处理：分离多边形
        // 如果顶点列表穿过了"空气"（原始多边形外部），将其剪断
        List<List<VertexData>> polygons = SplitPolygonByValidity(rawPoints, lineStart, lineEnd, originalPoly);

        int counter = 0;
        foreach (var polyPoints in polygons)
        {
            if (polyPoints.Count < 3) continue;

            // B. 生成 Mesh (MeshSeparator 依然保留作为双重保险)
            Mesh bigMesh = GenerateMesh(polyPoints);
            List<Mesh> separatedMeshes = MeshSeparator.Separate(bigMesh);

            foreach (var subMesh in separatedMeshes)
            {
                CreateSingleGameObject(original, subMesh, mat, $"{baseName}_{counter++}");
            }
        }
    }

    // 【新增】根据边的有效性剪断多边形
    private static List<List<VertexData>> SplitPolygonByValidity(List<VertexData> points, Vector2 lineStart, Vector2 lineEnd, List<Vector2> originalPoly)
    {
        List<List<VertexData>> result = new List<List<VertexData>>();
        List<VertexData> currentChain = new List<VertexData>();

        int n = points.Count;
        for (int i = 0; i < n; i++)
        {
            VertexData p1 = points[i];
            VertexData p2 = points[(i + 1) % n];

            currentChain.Add(p1);

            // 检查这条边是否位于切割线上
            if (IsPointOnLine(lineStart, lineEnd, p1.Position) && IsPointOnLine(lineStart, lineEnd, p2.Position))
            {
                // 如果两点都在切割线上，这就是一个潜在的"切面"或"跨越空隙的线"
                // 检查中点是否在原始多边形内
                Vector2 midPoint = (new Vector2(p1.Position.x, p1.Position.y) + new Vector2(p2.Position.x, p2.Position.y)) * 0.5f;

                if (!IsPointInPolygon(originalPoly, midPoint))
                {
                    // === 发现非法边（跨越空隙） ===
                    // 在这里剪断！currentChain 结束，打包存入结果
                    if (currentChain.Count >= 3)
                    {
                        result.Add(new List<VertexData>(currentChain));
                    }
                    currentChain.Clear();
                    // 注意：因为是循环列表，断开处的 p2 应该是下一条链的起点，循环继续时自然会作为 p1 被加入
                }
            }
        }

        // 处理循环列表的闭合问题
        // 如果 currentChain 不为空，且没有被剪断（说明最后一环是合法的），
        // 或者因为剪断导致变成非闭合链。
        // 这里简化逻辑：如果是因剪断产生的链，通常需要闭合。
        // 但由于 Sutherland-Hodgman 的特性，剪断后的链首尾通常就是切割点，直接闭合即可形成合法多边形。

        if (currentChain.Count > 0)
        {
            // 如果只有一条链（从未剪断），直接返回
            if (result.Count == 0)
            {
                result.Add(currentChain);
            }
            else
            {
                // 如果有多条链，最后一条链其实是第一条链的"前半部分"（因为循环被切断了）
                // 我们需要把最后一条链拼接到第一条链的开头
                // 举例：原本是 O-O-X-X-O-O (X是断点)，切断后变成 [O-O], [O-O]。
                // 实际逻辑中，我们简单地把这一段也作为一个新多边形尝试闭合
                // 或者更严谨地：检查首尾是否相连。
                // 鉴于切割产生的拓扑通常比较整齐，直接添加大多能工作
                if (currentChain.Count >= 3)
                {
                    result.Add(currentChain);
                }
            }
        }

        return result;
    }

    private static void CreateSingleGameObject(GameObject original, Mesh mesh, Material mat, string name)
    {
        GameObject newObj = new GameObject(name);
        newObj.transform.position = original.transform.position;
        newObj.transform.rotation = original.transform.rotation;
        newObj.transform.localScale = original.transform.localScale;
        newObj.layer = original.layer;

        newObj.AddComponent<MeshFilter>().mesh = mesh;
        newObj.AddComponent<MeshRenderer>().material = mat;

        PolygonCollider2D collider = newObj.AddComponent<PolygonCollider2D>();
        Vector2[] boundary = GetBoundaryPath(mesh);

        if (boundary != null && boundary.Length >= 3)
        {
            collider.SetPath(0, boundary);
        }
        else
        {
            // 降级方案：凸包或原点集
            Vector2[] fallback = new Vector2[mesh.vertexCount];
            for (int k = 0; k < mesh.vertexCount; k++)
                fallback[k] = new Vector2(mesh.vertices[k].x, mesh.vertices[k].y);
            collider.SetPath(0, fallback);
        }

        Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
        Rigidbody2D oldRb = original.GetComponent<Rigidbody2D>();
        if (oldRb != null)
        {
            newRb.linearVelocity = oldRb.linearVelocity;
            newRb.angularVelocity = oldRb.angularVelocity;
        }
    }

    // 判断点是否在任意多边形内 (射线法)
    private static bool IsPointInPolygon(List<Vector2> polygon, Vector2 p)
    {
        bool inside = false;
        int j = polygon.Count - 1;
        for (int i = 0; i < polygon.Count; j = i++)
        {
            if (((polygon[i].y > p.y) != (polygon[j].y > p.y)) &&
                (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    // 判断点是否在直线上（带误差容忍）
    private static bool IsPointOnLine(Vector2 lineStart, Vector2 lineEnd, Vector3 point)
    {
        // 计算点到直线的距离
        float d = Mathf.Abs((lineEnd.x - lineStart.x) * (point.y - lineStart.y) - (lineEnd.y - lineStart.y) * (point.x - lineStart.x));
        // 分母（线段长）
        float len = Vector2.Distance(lineStart, lineEnd);
        // 距离公式 d / len
        return (d / len) < 0.001f; // 1mm 的误差容忍
    }

    //物体生成与物理继承逻辑
    private static void CreateGameObject(GameObject original,List<VertexData> points, Material mat, string name)
    {
        //生成包含所有碎片的总Mesh
        Mesh bigMesh = GenerateMesh(points);
        //分离Mesh
        List<Mesh> seperatedMeshes = MeshSeparator.Separate(bigMesh);

        for(int i = 0; i < seperatedMeshes.Count; i++)
        {
            Mesh subMesh = seperatedMeshes[i];
            //实例化新物体
            GameObject newObj = new GameObject($"{name}_{i}");
            //继承位置，旋转，缩放,层级
            newObj.transform.position = original.transform.position;
            newObj.transform.rotation = original.transform.rotation;
            newObj.transform.localScale = original.transform.localScale;
            newObj.layer = original.layer;
            //设置渲染组件
            newObj.AddComponent<MeshFilter>().mesh = subMesh;
            newObj.AddComponent<MeshRenderer>().material = mat;
            //设置碰撞体
            PolygonCollider2D collider = newObj.AddComponent<PolygonCollider2D>();


            Vector2[] boundaryPath = GetBoundaryPath(subMesh);
            if (boundaryPath != null && boundaryPath.Length >= 3)
            {
                collider.SetPath(0, boundaryPath);
            }
            else
            {
                // 如果提取失败，回退到凸包或原点集
                Vector2[] fallback = new Vector2[subMesh.vertexCount];
                for (int k = 0; k < subMesh.vertexCount; k++)
                {
                    fallback[k] = new Vector2(subMesh.vertices[k].x, subMesh.vertices[k].y);
                }
                collider.SetPath(0, fallback);
            }
            //设置物理效果并继承动量
            Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
            Rigidbody2D oldRb = original.GetComponent<Rigidbody2D>();
            if (oldRb != null)
            {
                newRb.linearVelocity = oldRb.linearVelocity;
                newRb.angularVelocity = oldRb.angularVelocity;
            }
        }
    }

    // 提取 Mesh 边缘算法
    // 原理：遍历所有边，只出现一次的边就是边缘边
    private static Vector2[] GetBoundaryPath(Mesh mesh)
    {
        int[] tris = mesh.triangles;
        Vector3[] verts = mesh.vertices;

        // 记录边的出现次数。Key: "小索引_大索引"
        Dictionary<long, int> edgeCounts = new Dictionary<long, int>();
        // 记录边的方向，以便重建回路。Key: 起点, Value: 终点
        Dictionary<int, int> edgeGraph = new Dictionary<int, int>();

        // 1. 统计边
        for (int i = 0; i < tris.Length; i += 3)
        {
            AddEdge(edgeCounts, tris[i], tris[i + 1]);
            AddEdge(edgeCounts, tris[i + 1], tris[i + 2]);
            AddEdge(edgeCounts, tris[i + 2], tris[i]);
        }

        // 2. 筛选出边缘边 (只出现1次的边)
        for (int i = 0; i < tris.Length; i += 3)
        {
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i], tris[i + 1]);
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i + 1], tris[i + 2]);
            ProcessBoundaryEdge(edgeCounts, edgeGraph, tris[i + 2], tris[i]);
        }

        if (edgeGraph.Count == 0) return null;

        // 3. 构建回路 (顺藤摸瓜)
        List<Vector2> path = new List<Vector2>();

        // 找到任意一个起始点
        int startNode = 0;
        foreach (var key in edgeGraph.Keys) { startNode = key; break; }

        int current = startNode;
        int safety = 0;

        while (safety++ < verts.Length + 10)
        {
            path.Add(new Vector2(verts[current].x, verts[current].y));

            if (edgeGraph.ContainsKey(current))
            {
                current = edgeGraph[current];
            }
            else
            {
                break; // 断路了
            }

            if (current == startNode) break; // 闭环完成
        }

        return path.ToArray();
    }

    private static void AddEdge(Dictionary<long, int> counts, int a, int b)
    {
        // 保证 Key 唯一，无视方向
        long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        if (counts.ContainsKey(key)) counts[key]++;
        else counts[key] = 1;
    }

    private static void ProcessBoundaryEdge(Dictionary<long, int> counts, Dictionary<int, int> graph, int a, int b)
    {
        long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        if (counts[key] == 1)
        {
            // 这是边缘边，记录方向 a -> b
            // 注意：这里假设三角形是顺/逆时针统一的
            // 如果图是不连通的，可能需要更复杂的逻辑，但在 MeshSeparator 之后应该是连通的
            if (!graph.ContainsKey(a)) graph[a] = b;
        }
    }


    //三角剖分(耳切法)
    private static Mesh GenerateMesh(List<VertexData> points)
    {
        Vector3[] vertices = new Vector3[points.Count];
        Vector2[] uvs = new Vector2[points.Count];

        Vector2[] points2D = new Vector2[points.Count];

        for(int i=0;i<points.Count;i++)
        {
            vertices[i] = points[i].Position;
            uvs[i] = points[i].UV;
            points2D[i] = new Vector2(points[i].Position.x, points[i].Position.y);
        }

        int[] triangles = Triangulator.Triangulate(points2D);
        

        Mesh mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        //计算法线修正渲染
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    //预备几何算法
    //点是否在直线左侧
    private static bool IsPointOnPositiveSide(Vector2 lineStart, Vector2 lineEnd, Vector2 p)
    {
        Vector2 lineVec = lineEnd - lineStart;
        Vector2 pointVec = p - lineStart;

        // 二维叉积: x1*y2 - x2*y1
        // 结果 > 0 代表在左侧， < 0 代表在右侧， = 0 在线上
        float crossProduct = lineVec.x * pointVec.y - lineVec.y * pointVec.x;

        return crossProduct > 0;
    }
    //计算直线与线段的交点比例（0-1）
    private static float GetIntersectionT(Vector2 lineStart, Vector2 lineEnd, Vector2 segStart, Vector2 segEnd)
    {
        float d1 = SignedDistance(lineStart, lineEnd, segStart);
        float d2 = SignedDistance(lineStart, lineEnd, segEnd);

        return Mathf.Abs(d1) / (Mathf.Abs(d1) + Mathf.Abs(d2));
    }
    //点到直线有向距离
    private static float SignedDistance(Vector2 lineStart, Vector2 lineEnd, Vector2 p)
    {
        return (lineEnd.x - lineStart.x) * (p.y - lineStart.y) - (lineEnd.y - lineStart.y) * (p.x - lineStart.x);
    }
}
