using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;

public static class CurveSlicer
{
    /// <summary>
    /// 曲线贯穿切割：使用折线路径切割目标物体。
    /// 可以同时处理开放折线切透（直线、抛物线）、跨越边界的闭合环提取（如咬掉一个角）、以及纯净内部挖孔。
    /// </summary>
    public static void CurveSlice(GameObject target, List<Vector3> worldCutPath, bool isClosed)
    {
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();
        if (polyCollider == null || meshRenderer == null) return;

        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();
        if (generator != null && generator.hasUVReference) referenceRect = generator.uvReferenceRect;
        else referenceRect = Slicer.CalculateLocalBounds(polyCollider);

        // 将世界空间路径转换为物体局部空间
        List<Vector2> localCutPath = new List<Vector2>(worldCutPath.Count);
        for (int i = 0; i < worldCutPath.Count; i++)
        {
            localCutPath.Add(target.transform.InverseTransformPoint(worldCutPath[i]));
        }

        if (localCutPath.Count < 2) return;

        // 提取原有路径数据
        List<List<Vector2>> originalPaths = new List<List<Vector2>>(polyCollider.pathCount);
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            originalPaths.Add(new List<Vector2>(polyCollider.GetPath(i)));
        }

        bool isPureHolePunch = false;

        // === Step A: 对非闭合路径——仅执行稳定的标准延长 ===
        if (!isClosed)
        {
            float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;

            int last = localCutPath.Count - 1;
            Vector2 headDir = (localCutPath[1] - localCutPath[0]).normalized;
            Vector2 tailDir = (localCutPath[last] - localCutPath[last - 1]).normalized;

            // 废弃旧的“射线收敛检测”，由于其面对高曲率螺旋线/钩型线时极容易暴走生成遮天蔽日的幽灵巨型闭环，
            // 直接采用极其稳定可靠的首尾双向爆展拉飞策略，确保切割必出边界、不留暗病。
            if (headDir != Vector2.zero)
            {
                localCutPath[0] = localCutPath[0] - headDir * extensionLength;
            }

            int lastIdx = localCutPath.Count - 1;
            if (tailDir != Vector2.zero)
            {
                localCutPath[lastIdx] = localCutPath[lastIdx] + tailDir * extensionLength;
            }

            // 延长后自交检测（兜底：标准延长截断可能激发出新的闭合）
            if (SlicerMath.DetectAndResolveSelfIntersection(localCutPath, out List<Vector2> postExtLoop))
            {
                isClosed = true;
                Debug.Log($"[Slicer] 延长后检测到自交闭环，已提取环 ({localCutPath.Count} 点)，切换为闭环模式");
            }
        }

        // === Step B: 对闭合路径（包括延长后新发现的闭合）执行虚空自旋 ===
        if (isClosed)
        {
            int emptySpaceIndex = -1;
            List<Vector2> outerPath = originalPaths[0];
            
            bool IsPointInEmptySpace(Vector2 p)
            {
                if (!SlicerMath.PointInPolygon(p, outerPath)) return true;
                for (int h = 1; h < originalPaths.Count; h++)
                {
                    if (SlicerMath.PointInPolygon(p, originalPaths[h])) return true;
                }
                return false;
            }

            for (int i = 0; i < localCutPath.Count; i++)
            {
                if (IsPointInEmptySpace(localCutPath[i]))
                {
                    emptySpaceIndex = i;
                    break;
                }
                
                if (i < localCutPath.Count - 1)
                {
                    Vector2 p0 = localCutPath[i];
                    Vector2 p1 = localCutPath[i + 1];
                    float segLen = Vector2.Distance(p0, p1);
                    int samples = Mathf.Max(4, Mathf.CeilToInt(segLen / 0.05f));
                    for (int s = 1; s < samples; s++)
                    {
                        Vector2 mid = Vector2.Lerp(p0, p1, (float)s / samples);
                        if (IsPointInEmptySpace(mid))
                        {
                            localCutPath.Insert(i + 1, mid);
                            emptySpaceIndex = i + 1;
                            break;
                        }
                    }
                    if (emptySpaceIndex != -1) break;
                }
            }

            if (emptySpaceIndex == -1)
            {
                // [完全包围] 环线全在肉内，即使包围了内部孔也是纯净的实体吞噬，走合并算法
                isPureHolePunch = true;
            }
            else
            {
                // [跨越边界/孔洞] 环线部分在肉外，存在拓扑切割，必须自旋对齐虚空起点
                // 即使 emptySpaceIndex == 0 也走一遍，因为必须确保首尾在内存结构上完美物理缝合（rotated[0] = rotated[last]）
                List<Vector2> rotated = new List<Vector2>(localCutPath.Count);
                for (int i = emptySpaceIndex; i < localCutPath.Count - 1; i++) // -1 抛弃任何残缺的末尾
                    rotated.Add(localCutPath[i]);
                for (int i = 0; i < emptySpaceIndex; i++)
                    rotated.Add(localCutPath[i]);
                
                rotated.Add(rotated[0]); // 重新完美闭合
                localCutPath = rotated;
            }
        }

        // 分发逻辑
        if (isPureHolePunch)
        {
            PerformHolePunch(target, meshRenderer, originalRb, referenceRect, originalPaths, localCutPath);
            return;
        }

        // ==============================================================
        // Phase 1 异步调度：激活或提取原生缓冲组件，下发到底层线程挂机。
        // ==============================================================
        SliceableNativeData nativeData = target.GetComponent<SliceableNativeData>();
        if (nativeData == null) nativeData = target.AddComponent<SliceableNativeData>();

        // 跨帧数据容器，由于要在完成后的 Update 周期回收，必须使用 Persistent
        NativeArray<Unity.Mathematics.float2> nativeCutPath = new NativeArray<Unity.Mathematics.float2>(localCutPath.Count, Allocator.Persistent);
        for (int i = 0; i < localCutPath.Count; i++) nativeCutPath[i] = new Unity.Mathematics.float2(localCutPath[i].x, localCutPath[i].y);

        SliceContext context = SliceContextPool.Get();

        // 立即发车，将重负荷工作扔给 Worker Thread
        Unity.Jobs.JobHandle handle = CurveSlicerCore.ScheduleCurveSliceJob(
            nativeData.CachedVertices, 
            nativeData.CachedPathRanges, 
            nativeCutPath, 
            context
        );

        PendingSliceTask task = new PendingSliceTask
        {
            Context = context,
            Target = target,
            NativeData = nativeData,
            UVReferenceRect = referenceRect,
            MainJobHandle = handle,
            CurveCutPathArray = nativeCutPath,
            IsCurve = true
        };

        // 解放主线程：把返回凭证塞进任务中心轮询
        SlicerTaskManager.Instance.Enqueue(task);
    }

    /// <summary>
    /// 纯净内部挖孔切割：当闭合环完全处于多边形内部，且未切割外圈与内穴边时执行。
    /// </summary>
    private static void PerformHolePunch(GameObject target, MeshRenderer meshRenderer, Rigidbody2D originalRb, 
                                         Rect referenceRect, List<List<Vector2>> originalPaths, List<Vector2> localLoop)
    {
        List<Vector2> outerPath = originalPaths[0];
        Vector2 testPoint = localLoop[0];
        if (!SlicerMath.PointInPolygon(testPoint, outerPath))
        {
            Debug.LogWarning("[Slicer] HolePunch: 闭环不位于实体内部，这应在自旋阶段被拦截");
            return;
        }

        float loopArea = 0;
        for (int i = 0; i < localLoop.Count; i++)
        {
            Vector2 p1 = localLoop[i];
            Vector2 p2 = localLoop[(i + 1) % localLoop.Count];
            loopArea += (p1.x * p2.y) - (p2.x * p1.y);
        }

        if (Mathf.Abs(loopArea / 2f) < 0.001f)
        {
            return;
        }

        if (loopArea > 0) localLoop.Reverse(); 

        SlicerCore.PolygonData motherPoly = new SlicerCore.PolygonData();
        motherPoly.OuterLoop = originalPaths[0];
        motherPoly.Holes = new List<List<Vector2>>();
        for (int i = 1; i < originalPaths.Count; i++)
        {
            motherPoly.Holes.Add(originalPaths[i]);
        }
        motherPoly.Holes.Add(localLoop);

        float motherArea = 0;
        for (int i = 0; i < motherPoly.OuterLoop.Count; i++)
        {
            Vector2 p1 = motherPoly.OuterLoop[i];
            Vector2 p2 = motherPoly.OuterLoop[(i + 1) % motherPoly.OuterLoop.Count];
            motherArea += (p1.x * p2.y) - (p2.x * p1.y);
        }
        motherPoly.Area = Mathf.Abs(motherArea / 2f);

        List<Vector2> pieceBoundary = new List<Vector2>(localLoop);
        pieceBoundary.Reverse(); 

        SlicerCore.PolygonData piecePoly = new SlicerCore.PolygonData();
        piecePoly.OuterLoop = pieceBoundary;
        piecePoly.Holes = new List<List<Vector2>>();

        for (int i = motherPoly.Holes.Count - 2; i >= 0; i--) 
        {
            List<Vector2> existingHole = motherPoly.Holes[i];
            if (existingHole.Count > 0 && SlicerMath.PointInPolygon(existingHole[0], pieceBoundary))
            {
                piecePoly.Holes.Add(existingHole);
                motherPoly.Holes.RemoveAt(i);
            }
        }

        float pieceArea = 0;
        for (int i = 0; i < pieceBoundary.Count; i++)
        {
            Vector2 p1 = pieceBoundary[i];
            Vector2 p2 = pieceBoundary[(i + 1) % pieceBoundary.Count];
            pieceArea += (p1.x * p2.y) - (p2.x * p1.y);
        }
        piecePoly.Area = Mathf.Abs(pieceArea / 2f);

        bool success = true;
        try
        {
            Slicer.CreateSlicedObject(motherPoly, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
            Slicer.CreateSlicedObject(piecePoly, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] HolePunch MeshGen Error: {e.Message}");
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }
}
