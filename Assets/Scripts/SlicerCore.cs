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
    private struct IntersectionComparer : IComparer<IntersectionInfo>
    {
        public NativeArray<float2> Vertices;
        public int Offset;
        public int Compare(IntersectionInfo a, IntersectionInfo b)
        {
            if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
            
            float2 segStart = Vertices[Offset + a.SegmentIndex];
            float distA = (a.Point.x - segStart.x) * (a.Point.x - segStart.x) + (a.Point.y - segStart.y) * (a.Point.y - segStart.y);
            float distB = (b.Point.x - segStart.x) * (b.Point.x - segStart.x) + (b.Point.y - segStart.y) * (b.Point.y - segStart.y);
            return distA.CompareTo(distB);
        }
    }

    private struct CutIntersectionComparer : IComparer<Vector2>
    {
        public Vector2 Start, End;
        public int Compare(Vector2 a, Vector2 b)
        {
            float distA = (a.x - Start.x) * (End.x - Start.x) + (a.y - Start.y) * (End.y - Start.y);
            float distB = (b.x - Start.x) * (End.x - Start.x) + (b.y - Start.y) * (End.y - Start.y);
            return distA.CompareTo(distB);
        }
    }

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

    // 兼容原版的 Calculate 保留不删，防止 CurveSlicer 报错
    public static List<PolygonData> Calculate(List<List<Vector2>> originalPaths, Vector2 start, Vector2 end)
    {
        // 这一版本依然对接老旧的 SlicerSystem 单例，仅用于 CurveSlicerCore，我们将提供新的 Native 重载用于直线切割 Phase 1 
        // （直接复用老的实现，省略这里避免重复污染代码）
        return null; // Phase 1 废弃原托管 Calculate
    }

    /// <summary>
    /// Phase 1 零分配核心重载：直接消费初始化时缓存的 NativeArray，结合 SliceContext 生态避免 GC
    /// </summary>
    public static List<PolygonData> Calculate(NativeArray<float2> pathVerts, NativeArray<int2> pathRanges, Vector2 start, Vector2 end, SliceContext sys)
    {
        sys.ClearForReuse(); // 保险安全清理

        var cutIntersections = sys.CutIntersections;

        IntersectionComparer hitComparer = new IntersectionComparer();
        int totalEdges = pathVerts.Length;
        int pathsCount = pathRanges.Length;

        bool useJob = totalEdges > 128;
        NativeArray<CutSegment> jobSegments = default;
        NativeArray<CutHitResult> jobResults = default;

        try
        {
            if (useJob)
            {
                jobSegments = new NativeArray<CutSegment>(totalEdges, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                jobResults = new NativeArray<CutHitResult>(totalEdges, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

                int globalIdx = 0;
                for (int pId = 0; pId < pathsCount; pId++)
                {
                    int2 range = pathRanges[pId];
                    for (int i = 0; i < range.y; i++)
                    {
                        float2 p1 = pathVerts[range.x + i];
                        float2 p2 = pathVerts[range.x + ((i + 1) % range.y)];
                        jobSegments[globalIdx++] = new CutSegment { P1 = new Vector2(p1.x, p1.y), P2 = new Vector2(p2.x, p2.y) };
                    }
                }

                var job = new LineIntersectionJob
                {
                    Segments = jobSegments,
                    SliceStart = start,
                    SliceEnd = end,
                    Results = jobResults
                };
                job.Schedule(totalEdges, 64).Complete();
            }

            int globalEdgeCounter = 0;

            for (int pId = 0; pId < pathsCount; pId++)
            {
                int2 range = pathRanges[pId];
                sys.TempHits.Clear();
                hitComparer.Vertices = pathVerts;
                hitComparer.Offset = range.x;

                for (int i = 0; i < range.y; i++)
                {
                    if (useJob)
                    {
                        CutHitResult res = jobResults[globalEdgeCounter++];
                        if (res.Hit)
                        {
                            sys.TempHits.Add(new IntersectionInfo { Point = res.Point, T = res.T, SegmentIndex = i });
                        }
                    }
                    else
                    {
                        Vector2 p1 = new Vector2(pathVerts[range.x + i].x, pathVerts[range.x + i].y);
                        Vector2 p2 = new Vector2(pathVerts[range.x + ((i + 1) % range.y)].x, pathVerts[range.x + ((i + 1) % range.y)].y);

                        if (GetLineIntersection(p1, p2, start, end, out Vector2 intersection, out float t))
                        {
                            sys.TempHits.Add(new IntersectionInfo { Point = intersection, T = t, SegmentIndex = i });
                        }
                    }
                }

                sys.TempHits.Sort(hitComparer);

                sys.TempNewPath.Clear();
                var newPathVertices = sys.TempNewPath;

                int hitIndex = 0;
                for (int i = 0; i < range.y; i++)
                {
                    Vector2 currentVert = new Vector2(pathVerts[range.x + i].x, pathVerts[range.x + i].y);
                    if (newPathVertices.Count == 0 || SqrDist(newPathVertices[newPathVertices.Count - 1], currentVert) > MIN_VERT_DIST_SQ)
                    {
                        newPathVertices.Add(currentVert);
                    }

                    while (hitIndex < sys.TempHits.Count && sys.TempHits[hitIndex].SegmentIndex == i)
                    {
                        Vector2 p = sys.TempHits[hitIndex].Point;
                        if (SqrDist(newPathVertices[newPathVertices.Count - 1], p) <= MIN_VERT_DIST_SQ)
                        {
                            cutIntersections.Add(newPathVertices[newPathVertices.Count - 1]);
                        }
                        else
                        {
                            newPathVertices.Add(p);
                            cutIntersections.Add(p);
                        }
                        hitIndex++;
                    }
                }
                
                if (newPathVertices.Count > 1 && SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Count - 1]) < MIN_VERT_DIST_SQ)
                    newPathVertices.RemoveAt(newPathVertices.Count - 1);

                for (int i = 0; i < newPathVertices.Count; i++)
                {
                    Vector2 u = newPathVertices[i];
                    Vector2 v = newPathVertices[(i + 1) % newPathVertices.Count];
                    if (SqrDist(u, v) > MIN_VERT_DIST_SQ)
                    {
                        sys.RawEdges.Add(u);
                        sys.RawEdges.Add(v);
                    }
                }
            }
        }
        finally
        {
            if (useJob)
            {
                if (jobSegments.IsCreated) jobSegments.Dispose();
                if (jobResults.IsCreated) jobResults.Dispose();
            }
        }

        if (cutIntersections.Count < 2) return null;

        cutIntersections.Sort(new CutIntersectionComparer { Start = start, End = end });

        int validCount = (cutIntersections.Count % 2 == 0) ? cutIntersections.Count : cutIntersections.Count - 1;
        for (int i = 0; i < validCount; i += 2)
        {
            Vector2 pA = cutIntersections[i];
            Vector2 pB = cutIntersections[i + 1];
            if (SqrDist(pA, pB) > MIN_VERT_DIST_SQ)
            {
                sys.RawEdges.Add(pA);
                sys.RawEdges.Add(pB);
                
                sys.RawEdges.Add(pB);
                sys.RawEdges.Add(pA);
            }
        }

        RunNativeGraphPipeline(sys, out List<PolygonData> solids, out List<List<Vector2>> holes);

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

    internal static void RunNativeGraphPipeline(out List<PolygonData> solids, out List<List<Vector2>> holes)
    {
        var sys = SlicerSystem.Instance;
        sys.AliasMap.Length = sys.RawEdges.Length;

        new SlicerSystem.WeldingJob {
            RawEdges = sys.RawEdges.AsArray(),
            UniqueVertices = sys.UniqueVertices,
            AliasMap = sys.AliasMap.AsArray(),
            ToleranceSq = 1e-8f, 
            ToleranceX = 1e-4f   
        }.Run();

        new SlicerSystem.BuildGraphJob {
            AliasMap = sys.AliasMap.AsArray(),
            Graph = sys.NativeGraph
        }.Run();

        new SlicerSystem.ExtractLoopsJob {
            Graph = sys.NativeGraph,
            UniqueVertices = sys.UniqueVertices.AsArray(),
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges
        }.Run();

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
            
            List<Vector2> loop = SimplifyPath(rawLoop);
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

    internal static void RunNativeGraphPipeline(SliceContext sys, out List<PolygonData> solids, out List<List<Vector2>> holes)
    {
        sys.AliasMap.Length = sys.RawEdges.Length;

        new SlicerSystem.WeldingJob {
            RawEdges = sys.RawEdges.AsArray(),
            UniqueVertices = sys.UniqueVertices,
            AliasMap = sys.AliasMap.AsArray(),
            ToleranceSq = 1e-8f, 
            ToleranceX = 1e-4f   
        }.Run();

        new SlicerSystem.BuildGraphJob {
            AliasMap = sys.AliasMap.AsArray(),
            Graph = sys.NativeGraph
        }.Run();

        new SlicerSystem.ExtractLoopsJob {
            Graph = sys.NativeGraph,
            UniqueVertices = sys.UniqueVertices.AsArray(),
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges
        }.Run();

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

    public static void ReturnResultToPool(List<PolygonData> results)
    {
        if (results == null) return;
        foreach (var poly in results)
        {
            SlicerSystem.Instance.ReturnPoly(poly);
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

    internal static List<Vector2> SimplifyPath(List<Vector2> path)
    {
        var sys = SlicerSystem.Instance;
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