using System;
using System.Buffers;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public struct PendingSliceTask
{
    public SliceContext Context;
    public GameObject Target;
    public SliceableNativeData NativeData;
    public Rect UVReferenceRect;
    public JobHandle MainJobHandle;
    public Unity.Collections.NativeArray<float2> CurveCutPathArray;
    public bool IsCurve;
    public int State; // 0: topology stages, 1: mesh build stages
    public bool IsPureHolePunch;
    public PooledSlicePiece TargetPiece;
    public int TargetVersion;
    public float OriginalScaledArea;
    public float2 ScaleAbs;
}

public class SlicerTaskManager : MonoBehaviour
{
    private static SlicerTaskManager _instance;

    public static SlicerTaskManager Instance
    {
        get
        {
            if (_instance == null)
            {
                GameObject go = new GameObject("SlicerTaskManager");
                _instance = go.AddComponent<SlicerTaskManager>();
                DontDestroyOnLoad(go);
            }

            return _instance;
        }
    }

    private readonly List<PendingSliceTask> activeTasks = new List<PendingSliceTask>(256);

    public void Enqueue(PendingSliceTask task)
    {
        activeTasks.Add(task);
    }

    private void Update()
    {
        if (activeTasks.Count == 0)
        {
            return;
        }

        System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();

        for (int i = activeTasks.Count - 1; i >= 0; i--)
        {
            if (sw.ElapsedMilliseconds > 4)
            {
                break;
            }

            PendingSliceTask task = activeTasks[i];

            if (task.State == 0)
            {
                if (!task.MainJobHandle.IsCompleted)
                {
                    continue;
                }

                activeTasks.RemoveAt(i);

                try
                {
                    task.MainJobHandle.Complete();
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SlicerTaskManager] Phase 0 Job Error: {e.Message}\n{e.StackTrace}");
                    CleanupTask(task, disposeMeshDataArray: false);
                    continue;
                }

                if (!IsTaskTargetStillValid(task))
                {
                    CleanupTask(task, disposeMeshDataArray: false);
                    continue;
                }

                task.Context.UVRect = new float4(task.UVReferenceRect.xMin, task.UVReferenceRect.yMin, task.UVReferenceRect.width, task.UVReferenceRect.height);

                int loopCount = task.Context.LoopRanges.Length;
                task.Context.MeshDataStream = new Unity.Collections.NativeStream(loopCount, Unity.Collections.Allocator.TempJob);
                task.Context.MeshDataArray = Mesh.AllocateWritableMeshData(loopCount);
                task.Context.LoopPhysicsData = new Unity.Collections.NativeArray<SlicerCore.FragmentPhysicsData>(
                    loopCount,
                    Unity.Collections.Allocator.Persistent,
                    Unity.Collections.NativeArrayOptions.ClearMemory);

                JobHandle prepHandle = new SlicerCore.BuildSolidHoleMapJob
                {
                    LoopRanges = task.Context.LoopRanges,
                    LoopTypes = task.Context.LoopTypes,
                    HoleParents = task.Context.HoleParents,
                    SolidHoleMap = task.Context.SolidHoleMap,
                    HoleRangeBuffer = task.Context.HoleRangeBuffer
                }.Schedule();

                JobHandle mergeHandle = new SlicerCore.MergeTriangulateJob
                {
                    FlattenedLoops = task.Context.FlattenedLoops.AsArray(),
                    LoopRanges = task.Context.LoopRanges.AsArray(),
                    LoopTypes = task.Context.LoopTypes.AsArray(),
                    SolidHoleMap = task.Context.SolidHoleMap.AsDeferredJobArray(),
                    HoleRangeBuffer = task.Context.HoleRangeBuffer.AsDeferredJobArray(),
                    UVRect = task.Context.UVRect,
                    ScaleAbs = task.ScaleAbs,
                    MeshDataWriter = task.Context.MeshDataStream.AsWriter(),
                    LoopPhysicsData = task.Context.LoopPhysicsData
                }.Schedule(loopCount, 1, prepHandle);

                JobHandle buildMeshHandle = new SlicerCore.BuildMeshDataJob
                {
                    StreamReader = task.Context.MeshDataStream.AsReader(),
                    MeshDataArray = task.Context.MeshDataArray
                }.Schedule(loopCount, 1, mergeHandle);

                task.MainJobHandle = buildMeshHandle;
                task.State = 1;
                activeTasks.Add(task);
            }
            else if (task.State == 1)
            {
                if (!task.MainJobHandle.IsCompleted)
                {
                    continue;
                }

                activeTasks.RemoveAt(i);

                try
                {
                    task.MainJobHandle.Complete();
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SlicerTaskManager] Phase 1 Job Error: {e.Message}\n{e.StackTrace}");
                    CleanupTask(task, disposeMeshDataArray: true);
                    continue;
                }

                if (!IsTaskTargetStillValid(task))
                {
                    CleanupTask(task, disposeMeshDataArray: true);
                    continue;
                }

                ProcessTaskResolve(task);
            }
        }

        sw.Stop();
    }

    private void ProcessTaskResolve(PendingSliceTask task)
    {
        Mesh[] resultMeshes = null;
        PooledSlicePiece[] rentedPieces = null;
        bool createdAny = false;

        try
        {
            if (!IsTaskTargetStillValid(task))
            {
                DisposeMeshDataArray(ref task);
                return;
            }

            Rigidbody2D originalRb = task.Target.GetComponent<Rigidbody2D>();
            PolygonCollider2D originalCollider = task.Target.GetComponent<PolygonCollider2D>();
            MeshRenderer meshRenderer = task.Target.GetComponent<MeshRenderer>();
            Material mat = meshRenderer != null ? meshRenderer.sharedMaterial : null;
            float baseDensity = Slicer.CalculateFragmentDensity(originalRb, originalCollider, task.OriginalScaledArea);

            int loopCount = task.Context.LoopRanges.Length;
            resultMeshes = SliceMeshArrayPool.Rent(loopCount);
            rentedPieces = ArrayPool<PooledSlicePiece>.Shared.Rent(loopCount);

            for (int i = 0; i < loopCount; i++)
            {
                PooledSlicePiece piece = SlicePiecePool.Instance.Rent();
                rentedPieces[i] = piece;
                resultMeshes[i] = piece.ReusableMesh;
            }

            Mesh.ApplyAndDisposeWritableMeshData(
                task.Context.MeshDataArray,
                resultMeshes,
                UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds | UnityEngine.Rendering.MeshUpdateFlags.DontValidateIndices);

            task.Context.MeshDataArray = default;

            for (int i = 0; i < loopCount; i++)
            {
                PooledSlicePiece piece = rentedPieces[i];
                Mesh mesh = resultMeshes[i];

                if (piece == null || mesh == null || mesh.vertexCount == 0)
                {
                    if (piece != null)
                    {
                        piece.RequestDespawn();
                        rentedPieces[i] = null;
                    }

                    continue;
                }

                float4 boundsInfo = task.Context.LoopBounds[i];
                mesh.bounds = new Bounds(
                    new Vector3((boundsInfo.x + boundsInfo.z) * 0.5f, (boundsInfo.y + boundsInfo.w) * 0.5f, 0f),
                    new Vector3(boundsInfo.z - boundsInfo.x, boundsInfo.w - boundsInfo.y, 0.1f));

                int2 outerRange = task.Context.LoopRanges[i];
                int2 holeData = task.Context.SolidHoleMap[i];
                SlicerCore.FragmentPhysicsData physicsData = task.Context.LoopPhysicsData[i];

                bool success = Slicer.CreateSlicedObjectFromMesh(
                    piece,
                    task.Target,
                    mat,
                    originalRb,
                    task.UVReferenceRect,
                    task.Context,
                    mesh,
                    outerRange,
                    holeData,
                    physicsData,
                    baseDensity);

                if (!success)
                {
                    piece.RequestDespawn();
                    rentedPieces[i] = null;
                    continue;
                }

                rentedPieces[i] = null;
                createdAny = true;
            }

            if (createdAny)
            {
                RecycleOrDestroyTarget(task);
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[SlicerTaskManager] Execution Error: {e.Message}\n{e.StackTrace}");
            DisposeMeshDataArray(ref task);
        }
        finally
        {
            if (rentedPieces != null)
            {
                for (int i = 0; i < rentedPieces.Length; i++)
                {
                    if (rentedPieces[i] != null)
                    {
                        rentedPieces[i].RequestDespawn();
                    }
                }

                ArrayPool<PooledSlicePiece>.Shared.Return(rentedPieces, clearArray: true);
            }

            if (resultMeshes != null)
            {
                SliceMeshArrayPool.Return(resultMeshes);
            }

            CleanupTask(task, disposeMeshDataArray: false);
        }
    }

    private void CleanupTask(PendingSliceTask task, bool disposeMeshDataArray)
    {
        DisposeCurveCutPath(ref task);

        if (disposeMeshDataArray)
        {
            DisposeMeshDataArray(ref task);
        }

        ReleaseTaskReservation(task);
        SliceContextPool.Return(task.Context);
    }

    private static bool IsTaskTargetStillValid(PendingSliceTask task)
    {
        if (task.Target == null || !task.Target.activeInHierarchy)
        {
            return false;
        }

        if (task.TargetPiece == null)
        {
            return true;
        }

        return task.TargetPiece.SpawnVersion == task.TargetVersion;
    }

    private static void DisposeCurveCutPath(ref PendingSliceTask task)
    {
        if (task.IsCurve && task.CurveCutPathArray.IsCreated)
        {
            task.CurveCutPathArray.Dispose();
            task.CurveCutPathArray = default;
        }
    }

    private static void DisposeMeshDataArray(ref PendingSliceTask task)
    {
        if (task.Context.MeshDataArray.Length > 0)
        {
            task.Context.MeshDataArray.Dispose();
            task.Context.MeshDataArray = default;
        }
    }

    private static void ReleaseTaskReservation(PendingSliceTask task)
    {
        if (task.TargetPiece != null)
        {
            task.TargetPiece.ReleaseTaskReservation();
        }
    }

    private static void RecycleOrDestroyTarget(PendingSliceTask task)
    {
        if (task.TargetPiece != null)
        {
            task.TargetPiece.RequestDespawn();
        }
        else if (task.Target != null)
        {
            Destroy(task.Target);
        }
    }
}
