using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

public struct PendingSliceTask
{
    public SliceContext Context;
    public GameObject Target;
    public SliceableNativeData NativeData;
    public float2 LocalStart;
    public float2 LocalEnd;
    public Rect UVReferenceRect;
    // 未来可扩展存入 JobHandle 等异步追踪变量
}

/// <summary>
/// 全局切割任务调度器，提供异步跨帧轮询机制，并采用固定容量环形队列彻底阻断排队产生的 GC 分配。
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

    // Zero-GC 环形队列
    private PendingSliceTask[] ringBuffer = new PendingSliceTask[1024];
    private int head = 0;
    private int tail = 0;

    public void Enqueue(PendingSliceTask task)
    {
        int nextTail = (tail + 1) % ringBuffer.Length;
        if (nextTail == head)
        {
            Debug.LogError("[SlicerTaskManager] Ring Buffer Full! Dropping task.");
            // 需要扩容或截断，由于预设了 1024 极大容量，同帧切断 1024 次超出常规设计，直接跳过保护
            return;
        }

        ringBuffer[tail] = task;
        tail = nextTail;
    }

    private void Update()
    {
        // 由于处于 Phase 1 ，我们先在 Update 里取出 Task 执行测试，验证 Context 和数据缓存能够跑通
        // 等待 Phase 3 正式接入 JobHandle 的 IsCompleted 校验逻辑
        
        while (head != tail)
        {
            PendingSliceTask task = ringBuffer[head];
            head = (head + 1) % ringBuffer.Length;
            
            ProcessTaskSynchronously(task); // 本阶段暂作全同步测试
        }
    }

    private void ProcessTaskSynchronously(PendingSliceTask task)
    {
        // 验证目标是否依然存活，如果已被销毁（例如同一帧被另一刀清除了），直接归还 Context
        if (task.Target == null || task.NativeData == null)
        {
            SliceContextPool.Return(task.Context);
            return;
        }

        // 调用后续的核心测试运算，验证 Native 数据缓存能正确出片 
        try 
        {
            var slicedPolygons = SlicerCore.Calculate(
                task.NativeData.CachedVertices,
                task.NativeData.CachedPathRanges,
                new Vector2(task.LocalStart.x, task.LocalStart.y),
                new Vector2(task.LocalEnd.x, task.LocalEnd.y),
                task.Context);

            if (slicedPolygons != null && slicedPolygons.Count > 0)
            {
                var originalRb = task.Target.GetComponent<Rigidbody2D>();
                var meshRenderer = task.Target.GetComponent<MeshRenderer>();
                var mat = meshRenderer != null ? meshRenderer.sharedMaterial : null;

                foreach (var polyData in slicedPolygons)
                {
                    Slicer.CreateSlicedObject(polyData, task.Target, mat, originalRb, task.UVReferenceRect);
                }

                Destroy(task.Target); // Phase1 先继续沿用 Destroy 来消灭原物体
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
