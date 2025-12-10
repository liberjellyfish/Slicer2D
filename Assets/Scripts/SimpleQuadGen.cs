using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter),typeof(MeshRenderer))]
public class SimpleQuadGen : MonoBehaviour
{
    void OnEnable()
    {
        GenerateMesh();
    }
    void GenerateMesh()
    {
        //定义顶点，顺时针
        Vector3[] vertices = new Vector3[]
        {
            new Vector3(-1,-1, 0),
            new Vector3(-1, 1, 0),
            new Vector3( 1, 1, 0),
            new Vector3( 1,-1, 0)
        };

        //定义uv（对应顶点的纹理坐标，范围0-1，归一化）
        Vector2[] uv = new Vector2[]
        {
            new Vector2(0,0),
            new Vector2(0,1),
            new Vector2(1,1),
            new Vector2(1,0)
        };

        //定义三角形索引
        int[]triangles = new int[]
        {
            0,1,2,//第一个三角形
            0,2,3//第二个三角形
        };
        
        Mesh mesh = new Mesh();
        mesh.name = "ProceduralQuad";
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();
        //赋值给组件
        GetComponent<MeshFilter>().mesh = mesh;

        //使得其贴合正方形
        PolygonCollider2D polyCol = GetComponent<PolygonCollider2D>();
        if(polyCol != null)
        {
            Vector2[] path = new Vector2[vertices.Length];
            for(int i=0;i<vertices.Length; i++)
            {
                path[i] =new Vector2(vertices[i].x, vertices[i].y);
            }

            polyCol.SetPath(0,path);
        }
    }

    
}
