using UnityEngine;
using System.Collections.Generic;

public static class Slicer
{
    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        // 1. 获取 Unity 组件数据
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();
        if (polyCollider == null || meshRenderer == null) return;

        // 2. 坐标转换与准备
        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();
        if (generator != null && generator.hasUVReference) referenceRect = generator.uvReferenceRect;
        else referenceRect = CalculateLocalBounds(polyCollider);

        Vector2 localSliceStart = target.transform.InverseTransformPoint(worldStart);
        Vector2 localSliceEnd = target.transform.InverseTransformPoint(worldEnd);
        Vector2 cutDirection = (localSliceEnd - localSliceStart).normalized;
        if (cutDirection == Vector2.zero) return;

        // 延长切割线
        float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;
        localSliceStart = localSliceStart - cutDirection * extensionLength;
        localSliceEnd = localSliceEnd + cutDirection * extensionLength;

        // 3. 提取路径 
        List<List<Vector2>> originalPaths = new List<List<Vector2>>(polyCollider.pathCount);
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            Vector2[] pathArr = polyCollider.GetPath(i);
            var list = new List<Vector2>(pathArr);
            originalPaths.Add(list);
        }

        // 4. 调用核心算法 (Zero GC 热路径)
        List<SlicerCore.PolygonData> slicedPolygons = null;
        try
        {
            slicedPolygons = SlicerCore.Calculate(originalPaths, localSliceStart, localSliceEnd);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Slicer] Error: {e.Message}");
            return;
        }

        if (slicedPolygons == null || slicedPolygons.Count == 0) return;

        // 5. 生成物体
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
            Debug.LogError($"[Slicer] Mesh Generation Error: {e.Message}");
        }
        finally
        {
            // 归还 PolygonData 到池中
            SlicerCore.ReturnResultToPool(slicedPolygons);
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }

    private static Rect CalculateLocalBounds(PolygonCollider2D col)
    {
        // 逻辑不变...
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
    private static void CreateSlicedObject(SlicerCore.PolygonData data, GameObject originalTemplate, Material mat, Rigidbody2D originalRb, Rect uvRefRect)
    {
        string baseName = originalTemplate.name.Replace("_Piece", "");
        GameObject newObj = new GameObject(baseName + "_Piece");
        newObj.transform.SetPositionAndRotation(originalTemplate.transform.position, originalTemplate.transform.rotation);
        newObj.transform.localScale = originalTemplate.transform.localScale;
        newObj.layer = originalTemplate.layer;
        newObj.tag = originalTemplate.tag;

        List<Vector2> mergedVertices = PolygonHoleMerger.Merge(data.OuterLoop, data.Holes);

        Vector3[] vertices3D = new Vector3[mergedVertices.Count];
        Vector2[] uvs = new Vector2[mergedVertices.Count];
        Vector2[] vertices2D = mergedVertices.ToArray(); // Triangulator 需要数组

        float width = uvRefRect.width < 0.0001f ? 1 : uvRefRect.width;
        float height = uvRefRect.height < 0.0001f ? 1 : uvRefRect.height;
        float minX = uvRefRect.x;
        float minY = uvRefRect.y;

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