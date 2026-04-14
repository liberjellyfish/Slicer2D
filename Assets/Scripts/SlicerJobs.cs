using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

/// <summary>
/// SlicerCore 的 Job/Burst 子模块——所有并行化的碰撞检测 Job 集中在此文件管理。
/// 使用 partial class 保持与 SlicerCore 的同一命名空间。
/// </summary>
public static partial class SlicerCore
{
    // =================================================================================
    //                          直线切割 Job 数据结构
    // =================================================================================

    public struct CutSegment
    {
        public Vector2 P1;
        public Vector2 P2;
    }

    public struct CutHitResult
    {
        public bool Hit;
        public Vector2 Point;
        public float T;
    }

    /// <summary>
    /// 直线切割并行交点检测 Job。
    /// 每个线程独立检测一条多边形边与切割直线的交点。
    /// </summary>
    [BurstCompile]
    public struct LineIntersectionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<CutSegment> Segments;
        public Vector2 SliceStart;
        public Vector2 SliceEnd;

        [WriteOnly] public NativeArray<CutHitResult> Results;

        public void Execute(int index)
        {
            Vector2 p1 = Segments[index].P1;
            Vector2 p2 = Segments[index].P2;

            CutHitResult res = new CutHitResult();
            res.Hit = false;

            // 切线作为基准分隔平面
            Vector2 dir = SliceEnd - SliceStart;
            float lenSq = dir.x * dir.x + dir.y * dir.y;
            if (lenSq < 1e-8f)
            {
                Results[index] = res;
                return;
            }

            Vector2 normal = new Vector2(-dir.y, dir.x);

            // 求符号距离，大于 0 为超平面左方阵营，小于 0 为右方阵营
            float dist1 = math.dot(normal, p1 - SliceStart);
            float dist2 = math.dot(normal, p2 - SliceStart);

            // 将绝对距离挤压为绝对符号。> 0 和 <= 0 是极其核心的排爆墙！
            // 它强行规定哪怕正好摩擦在刀口（dist == 0），也会被强制分配到右侧阵营(-1)
            // 如此这般，相邻的两条边产生同擦点时，只有一条边能触发"跨域"，保证只输出 1 个且唯一 1 个合法交点。
            int sign1 = dist1 > 0f ? 1 : -1;
            int sign2 = dist2 > 0f ? 1 : -1;

            if (sign1 != sign2)
            {
                // 等比计算交点
                float u = dist1 / (dist1 - dist2);
                Vector2 intersection = p1 + u * (p2 - p1);

                // 将该交点投影在切割线上的绝对长度进度 T (用于极速排序)
                float t = math.dot(intersection - SliceStart, dir) / lenSq;

                if (t >= -1e-5f && t <= 1f + 1e-5f)
                {
                    res.Hit = true;
                    res.T = t;
                    res.Point = intersection;
                }
            }
            Results[index] = res;
        }
    }


}
