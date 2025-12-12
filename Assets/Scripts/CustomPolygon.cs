using UnityEngine;
using UnityEngine.Rendering.Universal;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class CustomPolygon : MonoBehaviour
{
    public Vector2[] points;
    void OnEnable()
    {
        if(points == null || points.Length < 3)
        {
            points = new Vector2[]//默认凹多边形
            {
                new Vector2(0,0),
                new Vector2(1, 1),    
                new Vector2(-1, 1),   
                new Vector2(-1, -1),  
                new Vector2(1, -1)
            }; 
        }
        GenerateMesh();
    }

    [ContextMenu("Refresh Mesh")]
    void GenerateMesh()
    {
        //定义顶点，顺时针
        Vector3[] vertices = new Vector3[points.Length];

        //定义uv（对应顶点的纹理坐标，范围0-1，归一化）
        Vector2[] uvs = new Vector2[points.Length];

        for(int i = 0; i < points.Length; i++)
        {
            vertices[i] = new Vector3(points[i].x, points[i].y, 0);
            uvs[i] = new Vector2((points[i].x + 1) * 0.5f, (points[i].y + 1) * 0.5f);
        }

        //三角剖分
        int[] triangles = Triangulator.Triangulate(points);

        Mesh mesh = new Mesh();
        mesh.name = "CustomPoly";
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();
        //赋值给组件
        GetComponent<MeshFilter>().mesh = mesh;

        //同步Collider
        GetComponent<PolygonCollider2D>().SetPath(0,points);
    }
}
