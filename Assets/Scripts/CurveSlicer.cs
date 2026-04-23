using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;

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

        // === Step A: 全面升级的智能闭合与延长策略 ===
        if (!isClosed)
        {
            // 1. 物理自交检测 (原味笔迹自交，如画了个 "8" 字或相交圈)
            if (SlicerMath.DetectAndResolveSelfIntersection(localCutPath, out List<Vector2> postExtLoop))
            {
                isClosed = true;
                Debug.Log($"[Slicer] 原始路径自交闭环，切换为闭环模式");
            }
            // 2. 强效近似闭合检测 (处理没完全接拢的圈)
            // 阻断没画拢的圈被当成开路线并产生远端交叉暴射！
            else if (localCutPath.Count >= 3)
            {
                float gap = Vector2.Distance(localCutPath[0], localCutPath[localCutPath.Count - 1]);
                float pathLength = SlicerMath.PolylineLength(localCutPath);

                // 如果首尾距离小于总长度的 25%，或者绝对距离很近，直接视为玩家意图挖孔
                if (gap < pathLength * 0.25f || gap < referenceRect.width * 0.1f)
                {
                    isClosed = true;
                    Debug.Log($"[Slicer] 路径近似闭合 (Gap: {gap:F2})，自动转为挖孔模式");
                }
            }

            // 3. 确认为纯开路切割，执行安全的“防自交”延长
            if (!isClosed && localCutPath.Count >= 2)
            {
                float maxExt = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;
                Vector2 headDir = (localCutPath[1] - localCutPath[0]).normalized;
                Vector2 tailDir = (localCutPath[localCutPath.Count - 1] - localCutPath[localCutPath.Count - 2]).normalized;

                Vector2 extHead = localCutPath[0] - headDir * maxExt;
                Vector2 extTail = localCutPath[localCutPath.Count - 1] + tailDir * maxExt;

                // 防止两根人工延长线在远方相交，形成自交幽灵线！
                if (SlicerMath.SegmentSegmentIntersect(extHead, localCutPath[0], localCutPath[localCutPath.Count - 1], extTail, out Vector2 intersection))
                {
                    // 刹车截断：在相交点前稍微缩回，保持完美开路且绝对不自交
                    extHead = intersection + headDir * 0.05f;
                    extTail = intersection - tailDir * 0.05f;
                    Debug.Log("[Slicer] 延长线发生远端交叉，已安全截断");
                }

                if (headDir != Vector2.zero)
                    localCutPath.Insert(0, extHead);

                if (tailDir != Vector2.zero)
                    localCutPath.Add(extTail);

                // 兜底：如果延长线向内切到了玩家自己的笔迹，提取成纯净闭环
                if (SlicerMath.DetectAndResolveSelfIntersection(localCutPath, out List<Vector2> finalLoop))
                {
                    isClosed = true;
                    Debug.Log("[Slicer] 延长后触发自交，转为闭环");
                }
            }
        }

        // === Step B: 对闭合路径执行虚空自旋 ===
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
                // 【核心修复】：由于我们在 SlicerMath 删除了多余的交点，这里直接遍历全部点即可。
                // 彻底去掉 validCount 和 hasDuplicateEnd 逻辑，防止误删最后一个拐点！
                List<Vector2> rotated = new List<Vector2>(localCutPath.Count);

                for (int i = emptySpaceIndex; i < localCutPath.Count; i++)
                    rotated.Add(localCutPath[i]);
                for (int i = 0; i < emptySpaceIndex; i++)
                    rotated.Add(localCutPath[i]);

                localCutPath = rotated;
            }
        }

        // 分发逻辑
        if (isPureHolePunch)
        {
            PerformHolePunch(target, meshRenderer, originalRb, referenceRect, originalPaths, localCutPath);
            return;
        }

        // 【核心隔离修复】：CurveSlicerCore 原生层依靠相邻点连线 (0->1, 1->2...)。
        // 如果判定为闭环（且不是完全在内部的纯挖孔，也就是跨越边界的“咬边”切割），
        // 必须在末尾补一个首点，让最后一条边物理闭合！
        if (isClosed && localCutPath.Count > 0)
        {
            // 防御性验证，确保没有重复加点
            if ((localCutPath[0] - localCutPath[localCutPath.Count - 1]).sqrMagnitude > 1e-6f)
            {
                localCutPath.Add(localCutPath[0]);
            }
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

        SliceableNativeData nativeData = target.GetComponent<SliceableNativeData>();
        if (nativeData == null) nativeData = target.AddComponent<SliceableNativeData>();

        SliceContext sys = SliceContextPool.Get();

        // 1. 提前计算总顶点数和总环数
        int totalPoints = motherPoly.OuterLoop.Count + piecePoly.OuterLoop.Count;
        int totalLoops = 2 + motherPoly.Holes.Count + piecePoly.Holes.Count;
        for (int i = 0; i < motherPoly.Holes.Count; i++) totalPoints += motherPoly.Holes[i].Count;
        for (int i = 0; i < piecePoly.Holes.Count; i++) totalPoints += piecePoly.Holes[i].Count;

        // 2. 一次性精准预分配
        sys.FlattenedLoops.SetCapacity(totalPoints);
        sys.LoopRanges.SetCapacity(totalLoops);
        sys.LoopTypes.SetCapacity(totalLoops);
        sys.LoopAreas.SetCapacity(totalLoops);
        sys.LoopBounds.SetCapacity(totalLoops);
        sys.HoleParents.SetCapacity(totalLoops);

        // 3. 极简的 AddLoop 闭包
        void AddLoop(List<Vector2> points, int parentIndex)
        {
            int offset = sys.FlattenedLoops.Length;
            int count = points.Count;
            for (int i = 0; i < count; i++)
            {
                sys.FlattenedLoops.Add(new Unity.Mathematics.float2(points[i].x, points[i].y));
            }
            sys.LoopRanges.Add(new Unity.Mathematics.int2(offset, count));
            sys.LoopTypes.Add(0);
            sys.LoopAreas.Add(0f);
            sys.LoopBounds.Add(Unity.Mathematics.float4.zero);
            sys.HoleParents.Add(parentIndex);
        }

        // 4. 显式挂载序注入
        int motherSolidIndex = sys.LoopRanges.Length;
        AddLoop(motherPoly.OuterLoop, -1);

        int pieceSolidIndex = sys.LoopRanges.Length;
        AddLoop(piecePoly.OuterLoop, -1);

        for (int i = 0; i < motherPoly.Holes.Count; i++)
        {
            AddLoop(motherPoly.Holes[i], motherSolidIndex);
        }

        for (int i = 0; i < piecePoly.Holes.Count; i++)
        {
            AddLoop(piecePoly.Holes[i], pieceSolidIndex);
        }

        // 5. 调度 Phase 5 基础清理管线
        Unity.Jobs.JobHandle simplifyHandle = new SlicerCore.SimplifyLoopsJob
        {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            MinVertDistSq = 0.0001f
        }.Schedule();

        Unity.Jobs.JobHandle classifyHandle = new SlicerCore.ClassifyLoopsJob
        {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            LoopTypes = sys.LoopTypes,
            LoopAreas = sys.LoopAreas,
            LoopBounds = sys.LoopBounds,
            AreaThreshold = 0.01f
        }.Schedule(simplifyHandle);

        // 6. 组装异步任务交接
        PendingSliceTask task = new PendingSliceTask
        {
            Context = sys,
            Target = target,
            NativeData = nativeData,
            UVReferenceRect = referenceRect,
            MainJobHandle = classifyHandle,
            IsCurve = true,
            IsPureHolePunch = true // 状态机硬隔离
        };

        SlicerTaskManager.Instance.Enqueue(task);
        // 注意：原先的 Object.Destroy(target) 已删除，生命周期管理移交至 SlicerTaskManager 末端
    }
}
