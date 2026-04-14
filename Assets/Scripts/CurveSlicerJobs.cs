using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

public static partial class CurveSlicerCore
{
    // =================================================================================
    //                          曲线切割 Job 数据结构
    // =================================================================================

    /// <summary>
    /// 曲线碰撞检测的输入对：一条多边形边 × 一段切割曲线
    /// </summary>
    public struct CurvePair
    {
        public Vector2 EdgeA, EdgeB;
        public Vector2 CutA, CutB;
        public int PathId;
        public int EdgeIdx;
        public int CutIdx;
    }

    /// <summary>
    /// 曲线碰撞检测的输出结果
    /// </summary>
    public struct CurveHitResult
    {
        public bool Hit;
        public Vector2 Point;
        public float GlobalT;         // cutIdx + tOnCut
        public float LocalTOnEdge;    // t on polygon edge
        public int PathId;
        public int EdgeIdx;
        public bool IsEntry;          // cross(edgeDir, cutDir) > 0
    }

    /// <summary>
    /// 曲线切割并行交点检测 Job。
    /// 每个线程独立检测一组 (多边形边, 曲线段) 的交叉，并计算 Entry/Exit 方向。
    /// </summary>
    [BurstCompile]
    public struct CurveIntersectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CurvePair> Pairs;
        [WriteOnly] public NativeArray<CurveHitResult> Results;

        public void Execute(int index)
        {
            CurvePair pair = Pairs[index];
            CurveHitResult result = new CurveHitResult();
            result.Hit = false;

            // 线段-线段相交检测
            float d1x = pair.EdgeB.x - pair.EdgeA.x;
            float d1y = pair.EdgeB.y - pair.EdgeA.y;
            float d2x = pair.CutB.x - pair.CutA.x;
            float d2y = pair.CutB.y - pair.CutA.y;

            float cross = d1x * d2y - d1y * d2x;

            if (cross > -1e-8f && cross < 1e-8f)
            {
                Results[index] = result;
                return;
            }

            float diffX = pair.CutA.x - pair.EdgeA.x;
            float diffY = pair.CutA.y - pair.EdgeA.y;

            float t = (diffX * d2y - diffY * d2x) / cross; // t on edge
            float u = (diffX * d1y - diffY * d1x) / cross; // u on cut

            if (t > 1e-6f && t < 1f - 1e-6f && u > 1e-6f && u < 1f - 1e-6f)
            {
                result.Hit = true;
                result.Point = new Vector2(
                    pair.EdgeA.x + t * d1x,
                    pair.EdgeA.y + t * d1y
                );
                result.GlobalT = pair.CutIdx + u;
                result.LocalTOnEdge = t;
                result.PathId = pair.PathId;
                result.EdgeIdx = pair.EdgeIdx;
                // 叉积判定穿越方向：cross(edgeDir, cutDir) > 0 → 进入实体
                result.IsEntry = cross > 0;
            }

            Results[index] = result;
        }
    }

    // =================================================================================
    //                          IComparer 结构体（零 GC 排序）
    // =================================================================================

    /// <summary>
    /// CurveIntersectionInfo 按 (SegmentIndex, LocalTOnEdge) 排序——替代 Lambda 委托，零 GC。
    /// </summary>
    public struct CurvePathHitsComparer : System.Collections.Generic.IComparer<CurveIntersectionInfo>
    {
        public int Compare(CurveIntersectionInfo a, CurveIntersectionInfo b)
        {
            if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
            return a.LocalTOnEdge.CompareTo(b.LocalTOnEdge);
        }
    }

    /// <summary>
    /// CurveIntersectionInfo 按 GlobalT 排序——替代 Lambda 委托，零 GC。
    /// </summary>
    public struct CurveGlobalTComparer : System.Collections.Generic.IComparer<CurveIntersectionInfo>
    {
        public int Compare(CurveIntersectionInfo a, CurveIntersectionInfo b)
        {
            return a.GlobalT.CompareTo(b.GlobalT);
        }
    }
}
