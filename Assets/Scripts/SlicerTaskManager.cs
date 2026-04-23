using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using Unity.Jobs;

public struct PendingSliceTask
{
    public SliceContext Context;
    public GameObject Target;
    public SliceableNativeData NativeData;
    public Rect UVReferenceRect;
    public JobHandle MainJobHandle;
    // 用于处理遗留内存：存储对曲面绘制临时分配的 native arrays
    public Unity.Collections.NativeArray<float2> CurveCutPathArray;
    public bool IsCurve;
    public int State; // 0: 拓扑构建中(Phase 1-5), 1: 网格与搭桥处理中(Phase 6)
}

/// <summary>
/// 全局切割任务调度器，提供异步跨帧轮询机制，加入单帧实例化预算保护。
/// </summary>
public class SlicerTaskManager : MonoBehaviour
{
    private static SlicerTaskManager _instance;
    public static SlicerTaskManager Instance
    {
        get
        {
            if (_instance == null)
            {
                var go = new GameObject("SlicerTaskManager");
                _instance = go.AddComponent<SlicerTaskManager>();
                DontDestroyOnLoad(go);
            }
            return _instance;
        }
    }

    // 使用 List 作为动态队列，支持乱序执行完毕时的中间删除
    private List<PendingSliceTask> activeTasks = new List<PendingSliceTask>(256);

    public void Enqueue(PendingSliceTask task)
    {
        activeTasks.Add(task);
    }

    private void Update()
    {
        if (activeTasks.Count == 0) return;

        var sw = System.Diagnostics.Stopwatch.StartNew();
        
        // 倒序遍历，方便在乱序完成后安全移除元素
        for (int i = activeTasks.Count - 1; i >= 0; i--)
        {
            // 单帧预算控制：如果解析耗时超过 4ms（约占 16ms 预算的 25%），立刻截断，留到下一帧，保证高帧率平滑
            if (sw.ElapsedMilliseconds > 4) break;

            PendingSliceTask task = activeTasks[i];

            if (task.State == 0)
            {
                if (task.MainJobHandle.IsCompleted)
                {
                    task.MainJobHandle.Complete();
                    activeTasks.RemoveAt(i);
                    
                    if (task.Target == null || !task.Target.activeInHierarchy)
                    {
                        if (task.IsCurve && task.CurveCutPathArray.IsCreated) task.CurveCutPathArray.Dispose();
                        SliceContextPool.Return(task.Context);
                        continue;
                    }

                    // Setup UV Rect before phase 6
                    task.Context.UVRect = new float4(task.UVReferenceRect.xMin, task.UVReferenceRect.yMin, task.UVReferenceRect.width, task.UVReferenceRect.height);
                    
                    // NativeStream 创建 (上界为 LoopRanges.Length)
                    int loopCount = task.Context.LoopRanges.Length;
                    task.Context.MeshDataStream = new Unity.Collections.NativeStream(loopCount, Unity.Collections.Allocator.TempJob);

                    // 调度 Phase 6
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
                        MeshDataWriter = task.Context.MeshDataStream.AsWriter()
                    }.Schedule(loopCount, 1, prepHandle);

                    task.MainJobHandle = mergeHandle;
                    task.State = 1;
                    activeTasks.Add(task); // Re-enqueue for Phase 6
                }
            }
            else if (task.State == 1)
            {
                if (task.MainJobHandle.IsCompleted)
                {
                    task.MainJobHandle.Complete();
                    activeTasks.RemoveAt(i);

                    if (task.IsCurve && task.CurveCutPathArray.IsCreated)
                    {
                        task.CurveCutPathArray.Dispose();
                    }

                    if (task.Target == null)
                    {
                        SliceContextPool.Return(task.Context);
                        continue;
                    }

                    ProcessTaskResolve(task);
                }
            }
        }
        sw.Stop();
    }

    private void ProcessTaskResolve(PendingSliceTask task)
    {
        try 
        {
            var originalRb = task.Target.GetComponent<Rigidbody2D>();
            var meshRenderer = task.Target.GetComponent<MeshRenderer>();
            var mat = meshRenderer != null ? meshRenderer.sharedMaterial : null;

            Unity.Collections.NativeStream.Reader reader = task.Context.MeshDataStream.AsReader();
            int loopCount = task.Context.LoopRanges.Length;
            bool createdAny = false;

            for (int i = 0; i < loopCount; i++)
            {
                int count = reader.BeginForEachIndex(i);
                if (count == 0)
                {
                    reader.EndForEachIndex();
                    continue;
                }

                int vertCount = reader.Read<int>();
                Vector3[] vertices = new Vector3[vertCount];
                Vector2[] uvs = new Vector2[vertCount];

                for (int v = 0; v < vertCount; v++)
                {
                    var pos = reader.Read<float3>();
                    var uv = reader.Read<float2>();
                    vertices[v] = new Vector3(pos.x, pos.y, 0f);
                    uvs[v] = new Vector2(uv.x, uv.y);
                }

                int triCount = reader.Read<int>();
                int[] triangles = new int[triCount];
                for (int t = 0; t < triCount; t++)
                {
                    triangles[t] = reader.Read<int>();
                }
                
                reader.EndForEachIndex();

                // Build Collider Paths from original loops
                int2 outerRange = task.Context.LoopRanges[i];
                int2 holeData = task.Context.SolidHoleMap[i];
                float area = task.Context.LoopAreas[i];

                Slicer.CreateSlicedObjectFromStream(task.Target, mat, originalRb, task.UVReferenceRect, task.Context, vertices, uvs, triangles, outerRange, holeData, area);
                createdAny = true;
            }

            if (createdAny)
            {
                Destroy(task.Target); 
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[SlicerTaskManager] Execution Error: {e.Message}\n{e.StackTrace}");
        }
        finally
        {
            SliceContextPool.Return(task.Context);
        }
    }
}
