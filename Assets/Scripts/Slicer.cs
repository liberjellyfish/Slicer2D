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

        // --- Phase 1 引流：立即排期切割 Job，将依赖句柄送入队列做跨帧轮询 ---
        
        // 获取或注入 Native 数据中间件
        SliceableNativeData nativeData = target.GetComponent<SliceableNativeData>();
        if (nativeData == null)
        {
            // 添加组件时会自动触发其 Awake() 进行唯一一次数据缓存提取
            nativeData = target.AddComponent<SliceableNativeData>(); 
        }

        // 从池中提取 Context 令牌
        SliceContext context = SliceContextPool.Get();

        // 立刻发车到底层 Burst 线程池
        Unity.Jobs.JobHandle handle = SlicerCore.ScheduleSliceJob(
            nativeData.CachedVertices,
            nativeData.CachedPathRanges,
            localSliceStart,
            localSliceEnd,
            context
        );

        PendingSliceTask task = new PendingSliceTask
        {
            Context = context,
            Target = target,
            NativeData = nativeData,
            UVReferenceRect = referenceRect,
            MainJobHandle = handle,
            IsCurve = false
        };

        // 提交跨帧任务队列，彻底解耦调用。在队列后续执行完前，不再阻断主帧。
        SlicerTaskManager.Instance.Enqueue(task);
    }



    internal static Rect CalculateLocalBounds(PolygonCollider2D col)
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
    internal static void CreateSlicedObject(SlicerCore.PolygonData data, GameObject originalTemplate, Material mat, Rigidbody2D originalRb, Rect uvRefRect, SliceContext nativeCtx = null)
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
        // 2D 场景法线恒定为 (0,0,-1)，硬编码跳过 RecalculateNormals 的全 mesh 叉积遍历
        Vector3[] normals = new Vector3[vertices3D.Length];
        for (int i = 0; i < normals.Length; i++) normals[i] = new Vector3(0, 0, -1);
        mesh.normals = normals;
        mesh.RecalculateBounds();

        MeshFilter mf = newObj.AddComponent<MeshFilter>();
        mf.mesh = mesh;
        MeshRenderer mr = newObj.AddComponent<MeshRenderer>();
        mr.material = mat;

        // --- 碰撞体设置：使用 List<Vector2> 重载避免 ToArray() 的 GC ---
        PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
        pc.enabled = false; // 延迟物理重建：批量设完后再启用，避免每次 SetPath 触发凸分解
        pc.pathCount = 1 + data.Holes.Count;
        pc.SetPath(0, data.OuterLoop);  // List<Vector2> 重载，零拷贝
        for (int i = 0; i < data.Holes.Count; i++)
        {
            pc.SetPath(i + 1, data.Holes[i]);
        }
        pc.enabled = true; // 统一触发一次物理重建

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