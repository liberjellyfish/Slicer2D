using UnityEngine;
using System.Collections.Generic;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(PolygonCollider2D))]
public class CustomPolygon : MonoBehaviour
{
    // 定义外圈 (逆时针)
    private List<Vector2> outerLoop = new List<Vector2>()
    {
        new Vector2(-2, -2),
        new Vector2(2, -2),
        new Vector2(2, 2),
        new Vector2(-2, 2)
    };

    // 定义内圈 (顺时针 - 这是一个洞)
    private List<Vector2> innerHole = new List<Vector2>()
    {
        new Vector2(-1, 1),
        new Vector2(1, 1),
        new Vector2(1, -1),
        new Vector2(-1, -1)
    };

    void OnEnable()
    {
        GenerateMesh();
    }

    [ContextMenu("Refresh Donut")]
    void GenerateMesh()
    {
        // 1. 准备数据
        List<List<Vector2>> holes = new List<List<Vector2>> { innerHole };

        // 2. 调用造桥算法，将洞融合进外圈
        List<Vector2> mergedPoints = PolygonHoleMerger.Merge(outerLoop, holes);

        // 3. 准备三角剖分数据
        Vector3[] vertices = new Vector3[mergedPoints.Count];
        Vector2[] uvs = new Vector2[mergedPoints.Count];
        Vector2[] points2D = new Vector2[mergedPoints.Count];

        for (int i = 0; i < mergedPoints.Count; i++)
        {
            vertices[i] = new Vector3(mergedPoints[i].x, mergedPoints[i].y, 0);
            uvs[i] = new Vector2((mergedPoints[i].x + 2) * 0.25f, (mergedPoints[i].y + 2) * 0.25f);
            points2D[i] = mergedPoints[i];
        }

        // 4. 耳切法生成三角形
        int[] triangles = Triangulator.Triangulate(points2D);

        // 5. 构建 Mesh
        Mesh mesh = new Mesh();
        mesh.name = "DonutMesh";
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        GetComponent<MeshFilter>().mesh = mesh;

        // 6. 设置 Collider (关键：多路径)
        PolygonCollider2D polyCol = GetComponent<PolygonCollider2D>();
        polyCol.pathCount = 2; // 设置有两个路径
        polyCol.SetPath(0, outerLoop.ToArray());
        polyCol.SetPath(1, innerHole.ToArray());
    }
}