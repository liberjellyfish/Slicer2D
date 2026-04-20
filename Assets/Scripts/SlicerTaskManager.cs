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

            if (task.MainJobHandle.IsCompleted)
            {
                // 彻底确保依赖冲刷与系统底层资源内存释放回调
                task.MainJobHandle.Complete();
                
                // 从活动列表中移除
                activeTasks.RemoveAt(i);
                
                // 清理曲线切割时的残留传递数据
                if (task.IsCurve && task.CurveCutPathArray.IsCreated)
                {
                    task.CurveCutPathArray.Dispose();
                }

                // 游戏业务解耦：经过 1-3 帧异步，实体可能已经主动销毁、回收
                if (task.Target == null)
                {
                    SliceContextPool.Return(task.Context);
                    continue;
                }

                ProcessTaskResolve(task);
            }
        }
        sw.Stop();
    }

    private void ProcessTaskResolve(PendingSliceTask task)
    {
        try 
        {
            // 将原有的在底层被强制封锁的多边形解析（托管装箱过程）脱钩到这里
            var slicedPolygons = SlicerCore.ResolveCutResult(task.Context);

            if (slicedPolygons != null && slicedPolygons.Count > 0)
            {
                var originalRb = task.Target.GetComponent<Rigidbody2D>();
                var meshRenderer = task.Target.GetComponent<MeshRenderer>();
                var mat = meshRenderer != null ? meshRenderer.sharedMaterial : null;

                foreach (var polyData in slicedPolygons)
                {
                    // 此处包含了 PolygonHoleMerger, Triangulate(已 Burst 化) 和 GameObject 实例化
                    Slicer.CreateSlicedObject(polyData, task.Target, mat, originalRb, task.UVReferenceRect, task.Context);
                }

                Destroy(task.Target); 
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[SlicerTaskManager] Execution Error: {e.Message}\n{e.StackTrace}");
        }
        finally
        {
            // 在这一阶段必然安全收回最初发放的 context
            SliceContextPool.Return(task.Context);
        }
    }
}
