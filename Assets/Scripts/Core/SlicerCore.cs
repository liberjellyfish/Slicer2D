using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public static partial class SlicerCore
{
    private const float MIN_VERT_DIST_SQ = 0.0001f;
    private const float AREA_THRESHOLD = 0.01f;

    public static JobHandle ScheduleSliceJob(
        NativeArray<float2> pathVerts,
        NativeArray<int2> pathRanges,
        Vector2 start,
        Vector2 end,
        SliceContext sys)
    {
        sys.ClearForReuse();

        int pathsCount = pathRanges.Length;
        if (pathsCount == 0)
        {
            return default;
        }

        NativeStream edgeStream = new NativeStream(pathsCount, Allocator.TempJob);
        NativeStream cutHitStream = new NativeStream(pathsCount, Allocator.TempJob);

        JobHandle rebuildHandle = new RebuildPathJob
        {
            PathVerts = pathVerts,
            PathRanges = pathRanges,
            SliceStart = new float2(start.x, start.y),
            SliceEnd = new float2(end.x, end.y),
            EdgeStreamWriter = edgeStream.AsWriter(),
            CutHitStreamWriter = cutHitStream.AsWriter()
        }.Schedule(pathsCount, 1);

        JobHandle flattenHandle = new FlattenAndSewJob
        {
            EdgeStreamReader = edgeStream.AsReader(),
            CutHitStreamReader = cutHitStream.AsReader(),
            PathCount = pathsCount,
            SliceStart = new float2(start.x, start.y),
            SliceEnd = new float2(end.x, end.y),
            RawEdges = sys.RawEdges
        }.Schedule(rebuildHandle);

        JobHandle graphHandle = ScheduleNativeGraphPipeline(sys, flattenHandle);

        edgeStream.Dispose(graphHandle);
        cutHitStream.Dispose(graphHandle);

        return graphHandle;
    }

    internal static JobHandle ScheduleNativeGraphPipeline(SliceContext sys, JobHandle dependency)
    {
        JobHandle weldHandle = new WeldingJob
        {
            RawEdges = sys.RawEdges,
            UniqueVertices = sys.UniqueVertices,
            AliasMap = sys.AliasMap,
            ToleranceSq = 1e-8f,
            ToleranceX = 1e-4f
        }.Schedule(dependency);

        JobHandle graphHandle = new BuildGraphJob
        {
            AliasMap = sys.AliasMap,
            Graph = sys.NativeGraph
        }.Schedule(weldHandle);

        JobHandle extractHandle = new ExtractLoopsJob
        {
            Graph = sys.NativeGraph,
            UniqueVertices = sys.UniqueVertices,
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges
        }.Schedule(graphHandle);

        JobHandle simplifyHandle = new SimplifyLoopsJob
        {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            MinVertDistSq = MIN_VERT_DIST_SQ
        }.Schedule(extractHandle);

        JobHandle classifyHandle = new ClassifyLoopsJob
        {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            LoopTypes = sys.LoopTypes,
            LoopAreas = sys.LoopAreas,
            LoopBounds = sys.LoopBounds,
            AreaThreshold = AREA_THRESHOLD
        }.Schedule(simplifyHandle);

        JobHandle assignHandle = new AssignHolesJob
        {
            FlattenedLoops = sys.FlattenedLoops,
            LoopRanges = sys.LoopRanges,
            LoopTypes = sys.LoopTypes,
            LoopAreas = sys.LoopAreas,
            LoopBounds = sys.LoopBounds,
            HoleParents = sys.HoleParents
        }.Schedule(classifyHandle);

        return assignHandle;
    }
}
