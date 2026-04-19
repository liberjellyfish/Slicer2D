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

    public static List<PolygonData> Calculate(NativeArray<float2> pathVerts, NativeArray<int2> pathRanges, Vector2 start, Vector2 end, SliceContext sys)
    {
        sys.ClearForReuse(); // 保险安全清理

        int pathsCount = pathRanges.Length;
        if (pathsCount == 0) return null;

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
        // 流水线句柄直接向后传递，取消此处的阻塞 Complete
        var flattenHandle = flattenJob.Schedule(rebuildHandle);

        // 原 RawEdges 检测已废弃，图层后续会过滤无效几何。直接进行最终向后流转。
        RunNativeGraphPipeline(sys, flattenHandle, out List<PolygonData> solids, out List<List<Vector2>> holes);

        // RunNativeGraphPipeline 已经内部 Complete 到大决堤，现在可以安全同步释放
        edgeStream.Dispose();
        cutHitStream.Dispose();

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
            float holeAreaAbs = Mathf.Abs(SignedArea(hole));

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

    internal static void RunNativeGraphPipeline(SliceContext sys, JobHandle dependency, out List<PolygonData> solids, out List<List<Vector2>> holes)
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

        extractHandle.Complete();

        solids = new List<PolygonData>();
        holes = new List<List<Vector2>>();

        var flatLoops = sys.FlattenedLoops.AsArray();
        for (int i = 0; i < sys.LoopRanges.Length; i++)
        {
            int2 range = sys.LoopRanges[i];
            List<Vector2> rawLoop = sys.GetList();
            for(int k = 0; k < range.y; k++) {
                float2 v = flatLoops[range.x + k];
                rawLoop.Add(new Vector2(v.x, v.y));
            }
            
            List<Vector2> loop = SimplifyPath(rawLoop, sys);
            sys.ReturnList(rawLoop);

            float area = SignedArea(loop);
            if (Mathf.Abs(area) < AREA_THRESHOLD)
            {
                sys.ReturnList(loop);
                continue;
            }

            if (area > 0)
            {
                PolygonData poly = sys.GetPoly();
                poly.OuterLoop = loop;
                poly.Area = area;
                poly.Bounds = CalculateBounds(loop);
                solids.Add(poly);
            }
            else
            {
                holes.Add(loop);
            }
        }
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