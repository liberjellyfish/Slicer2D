using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

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

        // Phase C-1: 判断走 Native 零拷贝路径还是旧托管路径
        bool useNativePath = nativeCtx != null && data.NativeOuterRange.y > 0;

        int vertCount;
        Vector3[] vertices3D;
        Vector2[] uvs;
        NativeList<float2> mergedNative = default;
        NativeList<int> nativeIndices = default;

        float width = uvRefRect.width < 0.0001f ? 1 : uvRefRect.width;
        float height = uvRefRect.height < 0.0001f ? 1 : uvRefRect.height;
        float uMin = uvRefRect.x;
        float vMin = uvRefRect.y;

        NativeArray<float2> flatLoops = default;

        try
        {
            List<Vector2> mergedVertices = null; // 仅旧路径使用

            if (useNativePath)
            {
                // ★ Native 零拷贝路径：从 FlattenedLoops 直接读取，不经过 List<Vector2>
                flatLoops = nativeCtx.FlattenedLoops.AsArray();
                mergedNative = PolygonHoleMerger.MergeNative(flatLoops, data.NativeOuterRange, data.NativeHoleRanges);

                vertCount = mergedNative.Length;
                vertices3D = new Vector3[vertCount];
                uvs = new Vector2[vertCount];
                // Phase C-2: 不再分配 new Vector2[vertCount] — 直接走 TriangulateNative

                for (int i = 0; i < vertCount; i++)
                {
                    float2 v = mergedNative[i];
                    vertices3D[i] = new Vector3(v.x, v.y, 0);
                    uvs[i] = new Vector2((v.x - uMin) / width, (v.y - vMin) / height);
                }
            }
            else
            {
                // ★ 旧托管路径回退（CurveSlicer.PerformHolePunch 等无 nativeCtx 的调用）
                mergedVertices = PolygonHoleMerger.Merge(data.OuterLoop, data.Holes);

                vertCount = mergedVertices.Count;
                vertices3D = new Vector3[vertCount];
                uvs = new Vector2[vertCount];

                for (int i = 0; i < vertCount; i++)
                {
                    vertices3D[i] = mergedVertices[i];
                    uvs[i] = new Vector2((mergedVertices[i].x - uMin) / width, (mergedVertices[i].y - vMin) / height);
                }
            }

            // Phase C-2: 三角剖分分流
            int[] indices;
            if (useNativePath)
            {
                // ★ Native 路径：MergeNative 输出直通 TriangulateNative，零中间分配
                nativeIndices = Triangulator.TriangulateNative(mergedNative);
                indices = nativeIndices.AsArray().ToArray();
            }
            else
            {
                // 旧路径：仍需 Vector2[] 给 Triangulate
                Vector2[] vertices2D = mergedVertices.ToArray();
                indices = Triangulator.Triangulate(vertices2D);
            }

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

            // --- 碰撞体设置 ---
            PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
            pc.enabled = false; // 延迟物理重建：批量设完后再启用

            if (useNativePath)
            {
                // Native 路径：使用单个池化列表复用给所有 SetPath，零稳态 GC
                int holeCount = data.NativeHoleRanges != null ? data.NativeHoleRanges.Count : 0;
                pc.pathCount = 1 + holeCount;

                List<Vector2> tempPathList = nativeCtx.GetList();

                FillListFromNativeRange(tempPathList, flatLoops, data.NativeOuterRange);
                pc.SetPath(0, tempPathList);

                for (int i = 0; i < holeCount; i++)
                {
                    FillListFromNativeRange(tempPathList, flatLoops, data.NativeHoleRanges[i]);
                    pc.SetPath(i + 1, tempPathList);
                }

                nativeCtx.ReturnList(tempPathList);
            }
            else
            {
                // 旧路径
                pc.pathCount = 1 + data.Holes.Count;
                pc.SetPath(0, data.OuterLoop);
                for (int i = 0; i < data.Holes.Count; i++)
                {
                    pc.SetPath(i + 1, data.Holes[i]);
                }
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
        finally
        {
            if (mergedNative.IsCreated) mergedNative.Dispose();
            if (nativeIndices.IsCreated) nativeIndices.Dispose();
        }
    }

    /// <summary>
    /// Phase 6: 从 NativeStream 输出直接重构对象（代替原有基于 PolygonData 的方案）
    /// </summary>
    public static void CreateSlicedObjectFromStream(
        GameObject originalObj, 
        Material mat, 
        Rigidbody2D originalRb, 
        Rect uvRefRect,
        SliceContext nativeCtx,
        Vector3[] vertices3D, 
        Vector2[] uvs, 
        int[] indices,
        int2 outerRange, 
        int2 holeData,
        float area)
    {
        GameObject newObj = new GameObject(originalObj.name + "_Slice");
        newObj.transform.position = originalObj.transform.position;
        newObj.transform.rotation = originalObj.transform.rotation;
        newObj.transform.localScale = originalObj.transform.localScale;
        newObj.layer = originalObj.layer;
        newObj.tag = originalObj.tag;

        Mesh mesh = new Mesh();
        mesh.vertices = vertices3D;
        mesh.uv = uvs;
        mesh.triangles = indices;
        
        Vector3[] normals = new Vector3[vertices3D.Length];
        for (int i = 0; i < normals.Length; i++) normals[i] = new Vector3(0, 0, -1);
        mesh.normals = normals;
        mesh.RecalculateBounds();

        MeshFilter mf = newObj.AddComponent<MeshFilter>();
        mf.mesh = mesh;
        MeshRenderer mr = newObj.AddComponent<MeshRenderer>();
        mr.material = mat;

        PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
        pc.enabled = false;

        NativeArray<float2> flatLoops = nativeCtx.FlattenedLoops.AsArray();
        pc.pathCount = 1 + holeData.y;

        List<Vector2> tempPathList = nativeCtx.GetList();

        FillListFromNativeRange(tempPathList, flatLoops, outerRange);
        pc.SetPath(0, tempPathList);

        for (int i = 0; i < holeData.y; i++)
        {
            int2 holeRange = nativeCtx.HoleRangeBuffer[holeData.x + i];
            FillListFromNativeRange(tempPathList, flatLoops, holeRange);
            pc.SetPath(i + 1, tempPathList);
        }

        nativeCtx.ReturnList(tempPathList);
        pc.enabled = true;

        SliceableGenerator newGen = newObj.AddComponent<SliceableGenerator>();
        newGen.hasUVReference = true;
        newGen.uvReferenceRect = uvRefRect;
        newGen.autoGenerateOnStart = false;

        if (originalRb != null)
        {
            Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
            newRb.mass = originalRb.mass * (area / 10f); // 保持原有的简单比例计算
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

    /// <summary>
    /// 从 FlattenedLoops 的范围切片填充池化 List（仅用于 PolygonCollider2D.SetPath）
    /// </summary>
    private static void FillListFromNativeRange(List<Vector2> list, NativeArray<float2> flatLoops, int2 range)
    {
        list.Clear();
        for (int i = 0; i < range.y; i++)
        {
            float2 v = flatLoops[range.x + i];
            list.Add(new Vector2(v.x, v.y));
        }
    }
}