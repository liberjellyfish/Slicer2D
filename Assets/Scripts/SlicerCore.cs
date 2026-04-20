using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using System.Diagnostics;

public static partial class SlicerCore
{
    public class PolygonData
    {
        public List<Vector2> OuterLoop;
        public List<List<Vector2>> Holes;
        public float Area;
        public Bounds Bounds;

        public PolygonData() { } 

        public void Init()
        {
            if (Holes == null) Holes = new List<List<Vector2>>();
            else Holes.Clear();
            OuterLoop = null; 
            Area = 0;
            Bounds = default;
        }
    }

    private const float MIN_VERT_DIST_SQ = 0.0001f; 
    private const float AREA_THRESHOLD = 0.01f;

    public struct IntersectionInfo
    {
        public Vector2 Point;
        public float T;
        public int SegmentIndex;
    }

    public static JobHandle ScheduleSliceJob(NativeArray<float2> pathVerts, NativeArray<int2> pathRanges, Vector2 start, Vector2 end, SliceContext sys)
    {
        sys.ClearForReuse(); // 保险安全清理

        int pathsCount = pathRanges.Length;
        if (pathsCount == 0) return default;

        // Step 1: 建立无锁交叉读写的双平行域 Stream
        NativeStream edgeStream = new NativeStream(pathsCount, Allocator.TempJob);
        NativeStream cutHitStream = new NativeStream(pathsCount, Allocator.TempJob);

        // Step 2: 派发 RebuildPathJob
        var rebuildJob = new RebuildPathJob
        {
            PathVerts = pathVerts,
            PathRanges = pathRanges,
            SliceStart = new float2(start.x, start.y),
            SliceEnd = new float2(end.x, end.y),
            EdgeStreamWriter = edgeStream.AsWriter(),
            CutHitStreamWriter = cutHitStream.AsWriter()
        };
        // 每个切割外环自成一体高度独立，进行 ParallelFor 并发运算
        var rebuildHandle = rebuildJob.Schedule(pathsCount, 1);

        // Step 3: Stream 归约打平与轴断面缝合 (FlattenAndSewJob)
        var flattenJob = new FlattenAndSewJob
        {
            EdgeStreamReader = edgeStream.AsReader(),
            CutHitStreamReader = cutHitStream.AsReader(),
            PathCount = pathsCount,
            SliceStart = new float2(start.x, start.y),
            SliceEnd = new float2(end.x, end.y),
            RawEdges = sys.RawEdges // 直接写入池化持有的容器
        };
        // 流水线句柄直接向后传递
        var flattenHandle = flattenJob.Schedule(rebuildHandle);

        var graphHandle = ScheduleNativeGraphPipeline(sys, flattenHandle);

        // 绑定异步依赖回收，无需主线程插手
        edgeStream.Dispose(graphHandle);
        cutHitStream.Dispose(graphHandle);

        return graphHandle;
    }

    internal static JobHandle ScheduleNativeGraphPipeline(SliceContext sys, JobHandle dependency)
    {
        JobHandle weldHandle = new WeldingJob {
            RawEdges = sys.RawEdges,
            UniqueVertices = sys.UniqueVertices,
            AliasMap = sys.AliasMap,
            ToleranceSq = 1e-8f, 
            ToleranceX = 1e-4f   
        }.Schedule(dependency);

        JobHandle graphHandle = new BuildGraphJob {
            AliasMap = sys.AliasMap,
            Graph = sys.NativeGraph
        }.Schedule(weldHandle);

        JobHandle extractHandle = new ExtractLoopsJob {
            Graph = sys.NativeGraph,
            UniqueVertices = sys.UniqueVertices,
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges
        }.Schedule(graphHandle);

        // Phase 5: 原生路径简化 → 环分类（将原主线程 SimplifyPath + SignedArea 下沉到 Burst）
        JobHandle simplifyHandle = new SimplifyLoopsJob {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            MinVertDistSq = MIN_VERT_DIST_SQ
        }.Schedule(extractHandle);

        JobHandle classifyHandle = new ClassifyLoopsJob {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            LoopTypes = sys.LoopTypes,
            LoopAreas = sys.LoopAreas,
            LoopBounds = sys.LoopBounds,
            AreaThreshold = AREA_THRESHOLD
        }.Schedule(simplifyHandle);

        return classifyHandle;
    }

    public static List<PolygonData> ResolveCutResult(SliceContext sys)
    {
        List<PolygonData> solids = new List<PolygonData>();
        List<List<Vector2>> holes = new List<List<Vector2>>();
        List<float> holeAreas = new List<float>();

        var flatLoops = sys.FlattenedLoops.AsArray();
        for (int i = 0; i < sys.LoopRanges.Length; i++)
        {
            // 使用 Burst Job 预计算的分类结果，跳过主线程的 SimplifyPath / SignedArea
            int loopType = sys.LoopTypes[i];
            if (loopType == 0) continue; // 已被 SimplifyLoopsJob + ClassifyLoopsJob 标记为废弃

            int2 range = sys.LoopRanges[i];

            // 从已简化的 Native 数据直接构建 List<Vector2>（不再需要 SimplifyPath）
            List<Vector2> loop = sys.GetList();
            for (int k = 0; k < range.y; k++)
            {
                float2 v = flatLoops[range.x + k];
                loop.Add(new Vector2(v.x, v.y));
            }

            if (loopType == 1) // solid (CCW, area > 0)
            {
                PolygonData poly = sys.GetPoly();
                poly.OuterLoop = loop;
                poly.Area = sys.LoopAreas[i];
                float4 b = sys.LoopBounds[i];
                poly.Bounds = new Bounds(
                    new Vector3((b.x + b.z) * 0.5f, (b.y + b.w) * 0.5f, 0),
                    new Vector3(b.z - b.x, b.w - b.y, 1)
                );
                solids.Add(poly);
            }
            else // hole (CW, area < 0)
            {
                holes.Add(loop);
                holeAreas.Add(sys.LoopAreas[i]);
            }
        }

        NativePolyTree tree = new NativePolyTree();
        tree.Build(solids); 

        for (int i = 0; i < holes.Count; i++)
        {
            List<Vector2> hole = holes[i];
            if (hole.Count < 3)
            {
                sys.ReturnList(hole);
                continue;
            }
            Vector2 testPoint = (hole[0] + hole[1]) * 0.5f;
            float holeAreaAbs = holeAreas[i]; // 直接使用 ClassifyLoopsJob 预计算的面积

            PolygonData bestParent = tree.QueryBestParent(testPoint, holeAreaAbs);

            if (bestParent != null)
            {
                bestParent.Holes.Add(hole);
            }
            else
            {
                sys.ReturnList(hole);
            }
        }

        tree.Dispose(); 

        return solids;
    }

    public static void ReturnResultToPool(List<PolygonData> results, SliceContext sys)
    {
        if (results == null) return;
        foreach (var poly in results)
        {
            sys.ReturnPoly(poly);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static float SqrDist(Vector2 a, Vector2 b)
    {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool GetLineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 intersection, out float t)
    {
        intersection = Vector2.zero;
        t = 0f;

        Vector2 dir = p4 - p3;
        float lenSq = dir.x * dir.x + dir.y * dir.y;
        if (lenSq < 1e-8f) return false;

        Vector2 normal = new Vector2(-dir.y, dir.x);

        float dist1 = Vector2.Dot(normal, p1 - p3);
        float dist2 = Vector2.Dot(normal, p2 - p3);

        int sign1 = dist1 > 0f ? 1 : -1;
        int sign2 = dist2 > 0f ? 1 : -1;

        if (sign1 != sign2)
        {
            float u = dist1 / (dist1 - dist2);
            intersection = p1 + u * (p2 - p1);
            t = Vector2.Dot(intersection - p3, dir) / lenSq;

            if (t >= -1e-5f && t <= 1f + 1e-5f) return true;
        }
        return false;
    }

    internal static List<Vector2> SimplifyPath(List<Vector2> path, SliceContext sys)
    {
        if (path.Count < 3)
        {
            var copy = sys.GetList();
            copy.AddRange(path);
            return copy;
        }

        List<Vector2> simplified = sys.GetList();
        simplified.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            if (SqrDist(path[i], simplified[simplified.Count - 1]) > MIN_VERT_DIST_SQ)
                simplified.Add(path[i]);
        }

        if (simplified.Count > 2 && SqrDist(simplified[0], simplified[simplified.Count - 1]) < MIN_VERT_DIST_SQ)
            simplified.RemoveAt(simplified.Count - 1);

        return simplified;
    }

    internal static Bounds CalculateBounds(List<Vector2> loop)
    {
        if (loop.Count == 0) return default;
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        for (int i = 0; i < loop.Count; i++)
        {
            Vector2 p = loop[i];
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        return new Bounds(new Vector3((minX + maxX) / 2, (minY + maxY) / 2, 0), new Vector3(maxX - minX, maxY - minY, 1));
    }

    internal static float SignedArea(List<Vector2> points)
    {
        float area = 0;
        int count = points.Count;
        for (int i = 0; i < count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % count];
            area += (p1.x * p2.y) - (p2.x * p1.y);
        }
        return area / 2.0f;
    }
}