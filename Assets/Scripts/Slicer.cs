using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public static class Slicer
{
    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        if (polyCollider == null || meshRenderer == null)
        {
            return;
        }

        Rect referenceRect;
        SliceableGenerator generator = target.GetComponent<SliceableGenerator>();
        if (generator != null && generator.hasUVReference)
        {
            referenceRect = generator.uvReferenceRect;
        }
        else
        {
            referenceRect = CalculateLocalBounds(polyCollider);
        }

        Vector2 localSliceStart = target.transform.InverseTransformPoint(worldStart);
        Vector2 localSliceEnd = target.transform.InverseTransformPoint(worldEnd);
        Vector2 cutDirection = (localSliceEnd - localSliceStart).normalized;
        if (cutDirection == Vector2.zero)
        {
            return;
        }

        float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;
        localSliceStart -= cutDirection * extensionLength;
        localSliceEnd += cutDirection * extensionLength;

        SliceableNativeData nativeData = target.GetComponent<SliceableNativeData>();
        if (nativeData == null)
        {
            nativeData = target.AddComponent<SliceableNativeData>();
        }

        SliceContext context = SliceContextPool.Get();
        Unity.Jobs.JobHandle handle = SlicerCore.ScheduleSliceJob(
            nativeData.CachedVertices,
            nativeData.CachedPathRanges,
            localSliceStart,
            localSliceEnd,
            context);

        PooledSlicePiece targetPiece = CaptureTaskLease(target, out int targetVersion);

        PendingSliceTask task = new PendingSliceTask
        {
            Context = context,
            Target = target,
            NativeData = nativeData,
            UVReferenceRect = referenceRect,
            MainJobHandle = handle,
            IsCurve = false,
            TargetPiece = targetPiece,
            TargetVersion = targetVersion
        };

        SlicerTaskManager.Instance.Enqueue(task);
    }

    internal static Rect CalculateLocalBounds(PolygonCollider2D col)
    {
        float minX = float.MaxValue;
        float minY = float.MaxValue;
        float maxX = float.MinValue;
        float maxY = float.MinValue;

        for (int i = 0; i < col.pathCount; i++)
        {
            Vector2[] path = col.GetPath(i);
            foreach (Vector2 p in path)
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
            List<Vector2> mergedVertices = null;

            if (useNativePath)
            {
                flatLoops = nativeCtx.FlattenedLoops.AsArray();
                mergedNative = PolygonHoleMerger.MergeNative(flatLoops, data.NativeOuterRange, data.NativeHoleRanges);

                vertCount = mergedNative.Length;
                vertices3D = new Vector3[vertCount];
                uvs = new Vector2[vertCount];

                for (int i = 0; i < vertCount; i++)
                {
                    float2 v = mergedNative[i];
                    vertices3D[i] = new Vector3(v.x, v.y, 0);
                    uvs[i] = new Vector2((v.x - uMin) / width, (v.y - vMin) / height);
                }
            }
            else
            {
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

            int[] indices;
            if (useNativePath)
            {
                nativeIndices = Triangulator.TriangulateNative(mergedNative);
                indices = nativeIndices.AsArray().ToArray();
            }
            else
            {
                Vector2[] vertices2D = mergedVertices.ToArray();
                indices = Triangulator.Triangulate(vertices2D);
            }

            Mesh mesh = new Mesh();
            mesh.vertices = vertices3D;
            mesh.uv = uvs;
            mesh.triangles = indices;

            Vector3[] normals = new Vector3[vertices3D.Length];
            for (int i = 0; i < normals.Length; i++)
            {
                normals[i] = new Vector3(0, 0, -1);
            }

            mesh.normals = normals;
            mesh.RecalculateBounds();

            MeshFilter mf = newObj.AddComponent<MeshFilter>();
            mf.sharedMesh = mesh;
            MeshRenderer mr = newObj.AddComponent<MeshRenderer>();
            mr.sharedMaterial = mat;

            PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
            pc.enabled = false;

            if (useNativePath)
            {
                pc.pathCount = 0;
                int holeCount = data.NativeHoleRanges != null ? data.NativeHoleRanges.Count : 0;

                int totalVerts = data.NativeOuterRange.y;
                for (int i = 0; i < holeCount; i++)
                {
                    totalVerts += data.NativeHoleRanges[i].y;
                }

                NativeArray<float2> newVertices = default;
                NativeArray<int2> newPathRanges = default;
                NativeArray<float2> colliderVertices = default;
                NativeArray<int2> colliderRanges = default;

                try
                {
                    newVertices = new NativeArray<float2>(totalVerts, Allocator.Persistent);
                    newPathRanges = new NativeArray<int2>(1 + holeCount, Allocator.Persistent);

                    int currentOffset = 0;
                    NativeArray<float2>.Copy(flatLoops, data.NativeOuterRange.x, newVertices, currentOffset, data.NativeOuterRange.y);
                    newPathRanges[0] = new int2(currentOffset, data.NativeOuterRange.y);
                    currentOffset += data.NativeOuterRange.y;

                    for (int i = 0; i < holeCount; i++)
                    {
                        int2 holeRange = data.NativeHoleRanges[i];
                        NativeArray<float2>.Copy(flatLoops, holeRange.x, newVertices, currentOffset, holeRange.y);
                        newPathRanges[1 + i] = new int2(currentOffset, holeRange.y);
                        currentOffset += holeRange.y;
                    }

                    pc.pathCount = 0;
                    SliceableNativeData nativeData = newObj.AddComponent<SliceableNativeData>();
                    nativeData.InitFromNative(newVertices, newPathRanges);
                    colliderVertices = newVertices;
                    colliderRanges = newPathRanges;
                    newVertices = default;
                    newPathRanges = default;

                    pc.pathCount = 1 + holeCount;
                    List<Vector2> tempPathList = nativeCtx.GetList();
                    try
                    {
                        FillListFromPersistentRange(tempPathList, colliderVertices, colliderRanges[0].x, colliderRanges[0].y);
                        pc.SetPath(0, tempPathList);

                        for (int i = 0; i < holeCount; i++)
                        {
                            int2 currentRange = colliderRanges[1 + i];
                            FillListFromPersistentRange(tempPathList, colliderVertices, currentRange.x, currentRange.y);
                            pc.SetPath(i + 1, tempPathList);
                        }
                    }
                    finally
                    {
                        nativeCtx.ReturnList(tempPathList);
                    }
                }
                catch (System.Exception e)
                {
                    if (newVertices.IsCreated) newVertices.Dispose();
                    if (newPathRanges.IsCreated) newPathRanges.Dispose();
                    Debug.LogError($"[Slicer] NativeData Injection Error: {e.Message}");
                }
            }
            else
            {
                pc.pathCount = 1 + data.Holes.Count;
                pc.SetPath(0, data.OuterLoop);
                for (int i = 0; i < data.Holes.Count; i++)
                {
                    pc.SetPath(i + 1, data.Holes[i]);
                }
            }

            pc.enabled = true;

            SliceableGenerator newGen = newObj.AddComponent<SliceableGenerator>();
            newGen.hasUVReference = true;
            newGen.uvReferenceRect = uvRefRect;
            newGen.autoGenerateOnStart = false;

            if (originalRb != null)
            {
                Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
                newRb.linearDamping = originalRb.linearDamping;
                newRb.angularDamping = originalRb.angularDamping;
                newRb.gravityScale = originalRb.gravityScale;
                newRb.collisionDetectionMode = originalRb.collisionDetectionMode;
                newRb.interpolation = originalRb.interpolation;
                newRb.sharedMaterial = originalRb.sharedMaterial;
                newRb.linearVelocity = originalRb.linearVelocity;
                newRb.angularVelocity = originalRb.angularVelocity;
                ApplyMassSettings(newRb, originalRb, data.Area);
            }
        }
        finally
        {
            if (mergedNative.IsCreated) mergedNative.Dispose();
            if (nativeIndices.IsCreated) nativeIndices.Dispose();
        }
    }

    public static bool CreateSlicedObjectFromMesh(
        PooledSlicePiece piece,
        GameObject originalObj,
        Material mat,
        Rigidbody2D originalRb,
        Rect uvRefRect,
        SliceContext nativeCtx,
        Mesh mesh,
        int2 outerRange,
        int2 holeData,
        float area)
    {
        if (piece == null || originalObj == null || nativeCtx == null || mesh == null)
        {
            return false;
        }

        piece.PrepareForSpawn(originalObj, mat, originalRb, uvRefRect);

        MeshFilter mf = piece.MeshFilter;
        MeshRenderer mr = piece.MeshRenderer;
        PolygonCollider2D pc = piece.PolygonCollider;
        SliceableNativeData nativeData = piece.SliceableNativeData;
        Rigidbody2D rb = piece.Rigidbody2D;

        mf.sharedMesh = mesh;
        mr.sharedMaterial = mat;

        NativeArray<float2> flatLoops = nativeCtx.FlattenedLoops.AsArray();
        int totalVerts = outerRange.y;
        for (int i = 0; i < holeData.y; i++)
        {
            totalVerts += nativeCtx.HoleRangeBuffer[holeData.x + i].y;
        }

        NativeArray<float2> newVertices = default;
        NativeArray<int2> newPathRanges = default;
        NativeArray<float2> colliderVertices = default;
        NativeArray<int2> colliderRanges = default;

        try
        {
            newVertices = new NativeArray<float2>(totalVerts, Allocator.Persistent);
            newPathRanges = new NativeArray<int2>(1 + holeData.y, Allocator.Persistent);

            int currentOffset = 0;
            NativeArray<float2>.Copy(flatLoops, outerRange.x, newVertices, currentOffset, outerRange.y);
            newPathRanges[0] = new int2(currentOffset, outerRange.y);
            currentOffset += outerRange.y;

            for (int i = 0; i < holeData.y; i++)
            {
                int2 holeRange = nativeCtx.HoleRangeBuffer[holeData.x + i];
                NativeArray<float2>.Copy(flatLoops, holeRange.x, newVertices, currentOffset, holeRange.y);
                newPathRanges[1 + i] = new int2(currentOffset, holeRange.y);
                currentOffset += holeRange.y;
            }

            pc.pathCount = 0;
            nativeData.InitFromNative(newVertices, newPathRanges);
            colliderVertices = newVertices;
            colliderRanges = newPathRanges;
            newVertices = default;
            newPathRanges = default;

            pc.pathCount = 1 + holeData.y;
            List<Vector2> tempPathList = nativeCtx.GetList();
            try
            {
                FillListFromPersistentRange(tempPathList, colliderVertices, colliderRanges[0].x, colliderRanges[0].y);
                pc.SetPath(0, tempPathList);

                for (int i = 0; i < holeData.y; i++)
                {
                    int2 currentRange = colliderRanges[1 + i];
                    FillListFromPersistentRange(tempPathList, colliderVertices, currentRange.x, currentRange.y);
                    pc.SetPath(i + 1, tempPathList);
                }
            }
            finally
            {
                nativeCtx.ReturnList(tempPathList);
            }
        }
        catch (System.Exception e)
        {
            if (newVertices.IsCreated) newVertices.Dispose();
            if (newPathRanges.IsCreated) newPathRanges.Dispose();
            Debug.LogError($"[Slicer] NativeData Injection Error: {e.Message}");
            return false;
        }

        if (originalRb != null)
        {
            rb.linearDamping = originalRb.linearDamping;
            rb.angularDamping = originalRb.angularDamping;
            rb.gravityScale = originalRb.gravityScale;
            rb.collisionDetectionMode = originalRb.collisionDetectionMode;
            rb.interpolation = originalRb.interpolation;
            rb.sharedMaterial = originalRb.sharedMaterial;
            rb.linearVelocity = originalRb.linearVelocity;
            rb.angularVelocity = originalRb.angularVelocity;
            ApplyMassSettings(rb, originalRb, area);
        }

        piece.CompleteSpawn(originalRb != null);
        return true;
    }

    internal static PooledSlicePiece CaptureTaskLease(GameObject target, out int targetVersion)
    {
        PooledSlicePiece targetPiece = target != null ? target.GetComponent<PooledSlicePiece>() : null;
        if (targetPiece != null)
        {
            targetPiece.RetainForTask();
            targetVersion = targetPiece.SpawnVersion;
            return targetPiece;
        }

        targetVersion = 0;
        return null;
    }

    private static void FillListFromPersistentRange(List<Vector2> list, NativeArray<float2> vertices, int start, int count)
    {
        list.Clear();
        if (list.Capacity < count)
        {
            list.Capacity = count;
        }

        for (int i = 0; i < count; i++)
        {
            float2 v = vertices[start + i];
            list.Add(new Vector2(v.x, v.y));
        }
    }

    private static void ApplyMassSettings(Rigidbody2D targetRb, Rigidbody2D sourceRb, float area)
    {
        if (targetRb == null || sourceRb == null)
        {
            return;
        }

        if (sourceRb.useAutoMass)
        {
            targetRb.useAutoMass = true;
            return;
        }

        targetRb.useAutoMass = false;
        targetRb.mass = sourceRb.mass * (area / 10f);
    }
}
