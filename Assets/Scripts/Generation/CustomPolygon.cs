using UnityEngine;
using System.Collections.Generic;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(PolygonCollider2D))]
public class CustomPolygon : MonoBehaviour
{
    // 定义外圈 (逆时针) - 大正方形
    private List<Vector2> outerLoop = new List<Vector2>()
    {
        new Vector2(-3+0.5f, -3),
        new Vector2(3+0.5f, -3),
        new Vector2(1+0.5f, 0),
        new Vector2(3+0.5f, 3),
        new Vector2(-3+0.5f, 3)
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
    private List<Vector2> holeMID = new List<Vector2>()
    {
        new Vector2(-1.5f, 0),
        new Vector2(0.5f, 1.5f),
        new Vector2(0, 0),
        new Vector2(0.5f, -1.5f)
    };

    [Header("Job-System 压力测试")]
    [Tooltip("勾选后将自动覆盖生成超过 128 条边的超级圆环，用于触发 Job System。")]
    public bool generateGiantPolygon = false;

    void OnEnable()
    {
        GenerateMesh();
    }

    public void RegenerateMesh()
    {
        GenerateMesh();
    }

    [ContextMenu("Refresh Grid")]
    void GenerateMesh()
    {
        // 1. 准备数据
        List<Vector2> activeOuterLoop = new List<Vector2>(outerLoop);
        List<List<Vector2>> activeHoles = new List<List<Vector2>> { holeTL, holeTR, holeBL, holeBR, holeMID };

        if (generateGiantPolygon)
        {
            activeOuterLoop.Clear();
            activeHoles.Clear();
            int outLoopCount = 200;
            int innerLoopCount = 70;
            // 生成一个半径为 2.8 的大圆外圈 (100 条边)
            for (int i = 0; i < outLoopCount; i++)
            {
                float angle = i * Mathf.PI * 2f / (outLoopCount * 1.0f);
                // 逆时针
                activeOuterLoop.Add(new Vector2(Mathf.Cos(angle) * 2.8f, Mathf.Sin(angle) * 2.8f));
            }

            // 生成一个内圈 (35 条边) => 总边数 135 > 128
            List<Vector2> giantHole = new List<Vector2>();
            for (int i = 0; i < innerLoopCount; i++)
            {
                float angle = i * Mathf.PI * 2f / (innerLoopCount * 1.0f);
                // 顺时针
                giantHole.Add(new Vector2(Mathf.Cos(-angle) * 1.5f, Mathf.Sin(-angle) * 1.5f));
            }
            activeHoles.Add(giantHole);
        }

        // 2. 调用造桥算法，将所有洞融合进外圈
        List<Vector2> mergedPoints = PolygonHoleMerger.Merge(activeOuterLoop, activeHoles);

        // 3. 准备三角剖分数据
        Vector3[] vertices = new Vector3[mergedPoints.Count];
        Vector2[] uvs = new Vector2[mergedPoints.Count];
        Vector2[] points2D = new Vector2[mergedPoints.Count];

        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        foreach (var p in mergedPoints)
        {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        float width = maxX - minX; if (width < 0.0001f) width = 1;
        float height = maxY - minY; if (height < 0.0001f) height = 1;

        for (int i = 0; i < mergedPoints.Count; i++)
        {
            vertices[i] = new Vector3(mergedPoints[i].x, mergedPoints[i].y, 0);
            // 首次生成即使用与 Slicer 相同的严格外框裁切法，彻底消灭偏移差！
            uvs[i] = new Vector2((mergedPoints[i].x - minX) / width, (mergedPoints[i].y - minY) / height);
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
        polyCol.pathCount = 1 + activeHoles.Count; // 1个外圈 + 多个洞
        polyCol.SetPath(0, activeOuterLoop.ToArray());

        for (int i = 0; i < activeHoles.Count; i++)
        {
            polyCol.SetPath(i + 1, activeHoles[i].ToArray());
        }
    }
}
