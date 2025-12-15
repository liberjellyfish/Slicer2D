using UnityEngine;
using System.Collections.Generic;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(PolygonCollider2D))]
public class CustomPolygon : MonoBehaviour
{
    // 定义外圈 (逆时针) - 大正方形
    private List<Vector2> outerLoop = new List<Vector2>()
    {
        new Vector2(-3, -3),
        new Vector2(3, -3),
        new Vector2(3, 3),
        new Vector2(-3, 3)
    };

    // 定义四个内圈 (顺时针 - 洞)
    // 左上
    private List<Vector2> holeTL = new List<Vector2>()
    {
        new Vector2(-2, 2),
        new Vector2(-1, 2),
        new Vector2(-1, 1),
        new Vector2(-2, 1)
    };

    // 右上
    private List<Vector2> holeTR = new List<Vector2>()
    {
        new Vector2(1, 2),
        new Vector2(2, 2),
        new Vector2(2, 1),
        new Vector2(1, 1)
    };

    // 左下
    private List<Vector2> holeBL = new List<Vector2>()
    {
        new Vector2(-2, -1),
        new Vector2(-1, -1),
        new Vector2(-1, -2),
        new Vector2(-2, -2)
    };

    // 右下
    private List<Vector2> holeBR = new List<Vector2>()
    {
        new Vector2(1, -1),
        new Vector2(2, -1),
        new Vector2(2, -2),
        new Vector2(1, -2)
    };

    void OnEnable()
    {
        GenerateMesh();
    }

    [ContextMenu("Refresh Grid")]
    void GenerateMesh()
    {
        // 1. 准备数据
        List<List<Vector2>> holes = new List<List<Vector2>> { holeTL, holeTR, holeBL, holeBR };

        // 2. 调用造桥算法，将所有洞融合进外圈
        List<Vector2> mergedPoints = PolygonHoleMerger.Merge(outerLoop, holes);

        // 3. 准备三角剖分数据
        Vector3[] vertices = new Vector3[mergedPoints.Count];
        Vector2[] uvs = new Vector2[mergedPoints.Count];
        Vector2[] points2D = new Vector2[mergedPoints.Count];

        for (int i = 0; i < mergedPoints.Count; i++)
        {
            vertices[i] = new Vector3(mergedPoints[i].x, mergedPoints[i].y, 0);
            // 简单的 UV 映射，基于坐标归一化
            uvs[i] = new Vector2((mergedPoints[i].x + 3) / 6f, (mergedPoints[i].y + 3) / 6f);
            points2D[i] = mergedPoints[i];
        }

        // 4. 耳切法生成三角形
        int[] triangles = Triangulator.Triangulate(points2D);

        // 5. 构建 Mesh
        Mesh mesh = new Mesh();
        mesh.name = "GridMesh";
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        GetComponent<MeshFilter>().mesh = mesh;

        // 6. 设置 Collider (关键：多路径)
        PolygonCollider2D polyCol = GetComponent<PolygonCollider2D>();
        polyCol.pathCount = 1 + holes.Count; // 1个外圈 + 4个洞
        polyCol.SetPath(0, outerLoop.ToArray());

        for (int i = 0; i < holes.Count; i++)
        {
            polyCol.SetPath(i + 1, holes[i].ToArray());
        }
    }
}