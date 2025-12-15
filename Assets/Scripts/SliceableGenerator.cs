using UnityEngine;
using System.Collections.Generic;

public class SliceableGenerator : MonoBehaviour
{
    [Tooltip("是否在开始运行时自动生成Mesh")]
    public bool autoGenerateOnStart = true;

    [Tooltip("物理材质")]
    public PhysicsMaterial2D physicsMaterial;

    // UV 参照数据 (黑匣子)
    // 这些数据用于告诉 Slicer，无论物体被切得多小，
    // 都要按照这个原始矩形来计算 UV，防止贴图错位。
    
    [HideInInspector] public bool hasUVReference = false;
    [HideInInspector] public Rect uvReferenceRect;

    void Start()
    {
        // 如果已经有引用数据（说明是切出来的碎片），就不要再自动生成了
        if (hasUVReference) return;

        if (autoGenerateOnStart)
        {
            GenerateSliceable();
        }
    }

    [ContextMenu("Generate Sliceable Mesh")]
    public void GenerateSliceable()
    {
        SpriteRenderer sr = GetComponent<SpriteRenderer>();
        // 如果已经是 Mesh 对象且有参照数据，跳过
        if (sr == null && GetComponent<MeshRenderer>() != null)
        {
            return;
        }
        if (sr == null || sr.sprite == null) return;

        // 1. 缓存关键数据
        Texture2D spriteTexture = sr.sprite.texture;
        Color spriteColor = sr.color;
        Bounds spriteBounds = sr.sprite.bounds;
        string spriteName = sr.sprite.name;

        // [核心]：记录原始 UV 参照系 (Local Space Bounds)
        // x, y 是最小点，width, height 是尺寸
        uvReferenceRect = new Rect(spriteBounds.min.x, spriteBounds.min.y, spriteBounds.size.x, spriteBounds.size.y);
        hasUVReference = true;

        // 提取 Collider 路径
        PolygonCollider2D polyCol = GetComponent<PolygonCollider2D>();
        if (polyCol == null)
        {
            polyCol = gameObject.AddComponent<PolygonCollider2D>();
        }

        List<Vector2> outerLoop = new List<Vector2>(polyCol.GetPath(0));
        List<List<Vector2>> holes = new List<List<Vector2>>();
        if (polyCol.pathCount > 1)
        {
            for (int i = 1; i < polyCol.pathCount; i++)
            {
                holes.Add(new List<Vector2>(polyCol.GetPath(i)));
            }
        }

        // 2. 立即销毁 SpriteRenderer
        DestroyImmediate(sr);

        // 3. 生成 Mesh
        List<Vector2> mergedVertices = PolygonHoleMerger.Merge(outerLoop, holes);
        int[] triangles = Triangulator.Triangulate(mergedVertices.ToArray());

        Mesh mesh = new Mesh();
        Vector3[] vertices = new Vector3[mergedVertices.Count];
        Vector2[] uvs = new Vector2[mergedVertices.Count];

        // 使用刚刚记录的 uvReferenceRect 来计算 UV
        float width = uvReferenceRect.width;
        float height = uvReferenceRect.height;
        float minX = uvReferenceRect.x;
        float minY = uvReferenceRect.y;

        for (int i = 0; i < mergedVertices.Count; i++)
        {
            vertices[i] = mergedVertices[i];

            // 计算 UV (始终相对于原始包围盒)
            float u = (mergedVertices[i].x - minX) / width;
            float v = (mergedVertices[i].y - minY) / height;
            uvs[i] = new Vector2(u, v);
        }

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.name = spriteName + "_GeneratedMesh";

        // 4. 添加新组件
        MeshFilter mf = GetComponent<MeshFilter>();
        if (mf == null) mf = gameObject.AddComponent<MeshFilter>();
        mf.mesh = mesh;

        MeshRenderer mr = GetComponent<MeshRenderer>();
        if (mr == null) mr = gameObject.AddComponent<MeshRenderer>();

        if (mr.sharedMaterial == null)
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Standard");
            Material mat = new Material(shader);
            mat.mainTexture = spriteTexture;
            mat.color = spriteColor;
            mr.material = mat;
        }

        Rigidbody2D rb = GetComponent<Rigidbody2D>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody2D>();
            rb.useAutoMass = true;
            if (physicsMaterial != null) rb.sharedMaterial = physicsMaterial;
        }

        Debug.Log($"[SliceableGenerator] 成功转换并记录 UV 参照：{name}");
    }
}