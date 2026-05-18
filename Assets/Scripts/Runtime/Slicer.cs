using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public static class Slicer
{
    private const float MIN_SCALED_AREA = 1e-6f;
    private const float MIN_FRAGMENT_MASS = 0.01f;
    private const float MIN_FRAGMENT_INERTIA = 1e-4f;

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
        float2 scaleAbs = GetAbsoluteLossyScale(target.transform);
        float originalScaledArea = CalculateScaledNetArea(polyCollider, scaleAbs);

        PendingSliceTask task = new PendingSliceTask
        {
            Context = context,
            Target = target,
            NativeData = nativeData,
            UVReferenceRect = referenceRect,
            MainJobHandle = handle,
            IsCurve = false,
            TargetPiece = targetPiece,
            TargetVersion = targetVersion,
            OriginalScaledArea = originalScaledArea,
            ScaleAbs = scaleAbs
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
        SlicerCore.FragmentPhysicsData physicsData,
        float baseDensity)
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
            ApplyFragmentPhysics(rb, physicsData, baseDensity);
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

    internal static float CalculateFragmentDensity(Rigidbody2D sourceRb, PolygonCollider2D sourceCollider, float originalScaledArea)
    {
        if (sourceRb == null)
        {
            return 1f;
        }

        if (sourceRb.useAutoMass && sourceCollider != null)
        {
            return Mathf.Max(sourceCollider.density, 1e-4f);
        }

        return Mathf.Max(sourceRb.mass / Mathf.Max(originalScaledArea, MIN_SCALED_AREA), 1e-4f);
    }

    internal static float2 GetAbsoluteLossyScale(Transform targetTransform)
    {
        Vector3 lossyScale = targetTransform.lossyScale;
        return new float2(Mathf.Abs(lossyScale.x), Mathf.Abs(lossyScale.y));
    }

    internal static float CalculateScaledNetArea(PolygonCollider2D col, float2 scaleAbs)
    {
        if (col == null || col.pathCount == 0)
        {
            return MIN_SCALED_AREA;
        }

        float localArea = 0f;
        for (int i = 0; i < col.pathCount; i++)
        {
            Vector2[] path = col.GetPath(i);
            float signedArea = SignedArea(path);
            if (i == 0)
            {
                localArea += Mathf.Abs(signedArea);
            }
            else
            {
                localArea -= Mathf.Abs(signedArea);
            }
        }

        float scaledArea = Mathf.Abs(localArea) * Mathf.Max(scaleAbs.x * scaleAbs.y, MIN_SCALED_AREA);
        return Mathf.Max(scaledArea, MIN_SCALED_AREA);
    }

    private static void ApplyFragmentPhysics(Rigidbody2D targetRb, SlicerCore.FragmentPhysicsData physicsData, float baseDensity)
    {
        if (targetRb == null)
        {
            return;
        }

        float safeDensity = Mathf.Max(baseDensity, 1e-4f);
        float finalMass = Mathf.Max(physicsData.ScaledArea * safeDensity, MIN_FRAGMENT_MASS);
        float finalInertia = Mathf.Max(physicsData.GeometricInertia * safeDensity, MIN_FRAGMENT_INERTIA);

        targetRb.useAutoMass = false;
        targetRb.mass = finalMass;
        targetRb.centerOfMass = new Vector2(physicsData.LocalCenter.x, physicsData.LocalCenter.y);
        targetRb.inertia = finalInertia;
    }

    private static float SignedArea(IReadOnlyList<Vector2> points)
    {
        if (points == null || points.Count < 3)
        {
            return 0f;
        }

        float area = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % points.Count];
            area += (p1.x * p2.y) - (p2.x * p1.y);
        }

        return area * 0.5f;
    }
}
