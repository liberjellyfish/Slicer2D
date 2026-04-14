using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;
using System;

/// <summary>
/// 切割解算数据平台与管线中枢
/// 生命周期受场景脚本或者管理器制约，通过调用 Dispose() 进行终极销毁。
/// 彻底规避静态 Persistent 导致 Editor Reload 时 Native Memory Leak 问题。
/// </summary>
public class SlicerSystem : IDisposable
{
    private static SlicerSystem _instance;
    public static SlicerSystem Instance
    {
        get
        {
            if (_instance == null) _instance = new SlicerSystem();
            return _instance;
        }
    }

    // === 托管复用池 ===
    public Stack<List<Vector2>> ListPool;
    public Stack<SlicerCore.PolygonData> PolyPool;
    public List<Vector2> CutIntersections;
    public List<Vector2> TempNewPath;
    public List<SlicerCore.IntersectionInfo> TempHits;

    // === Native 缓冲 (Allocator.Persistent) 避免频繁申请 ===
    public NativeList<float2> RawEdges; // u, v, u, v...
    public NativeList<float2> UniqueVertices;
    public NativeList<int> AliasMap;
    public NativeParallelMultiHashMap<int, int> NativeGraph;
    
    // DFS 图提取使用的共享数组
    public NativeList<float2> FlattenedLoops;
    public NativeList<int2> LoopRanges;

    private SlicerSystem()
    {
        ListPool = new Stack<List<Vector2>>();
        PolyPool = new Stack<SlicerCore.PolygonData>();
        CutIntersections = new List<Vector2>(64);
        TempNewPath = new List<Vector2>(128);
        TempHits = new List<SlicerCore.IntersectionInfo>(64);

        RawEdges = new NativeList<float2>(4096, Allocator.Persistent);
        UniqueVertices = new NativeList<float2>(2048, Allocator.Persistent);
        AliasMap = new NativeList<int>(4096, Allocator.Persistent);
        NativeGraph = new NativeParallelMultiHashMap<int, int>(2048, Allocator.Persistent);
        FlattenedLoops = new NativeList<float2>(4096, Allocator.Persistent);
        LoopRanges = new NativeList<int2>(128, Allocator.Persistent);
    }

    public List<Vector2> GetList()
    {
        return ListPool.Count > 0 ? ListPool.Pop() : new List<Vector2>();
    }

    public void ReturnList(List<Vector2> list)
    {
        if (list == null) return;
        list.Clear();
        ListPool.Push(list);
    }

    public SlicerCore.PolygonData GetPoly()
    {
        SlicerCore.PolygonData p = PolyPool.Count > 0 ? PolyPool.Pop() : new SlicerCore.PolygonData();
        p.Init();
        return p;
    }

    public void ReturnPoly(SlicerCore.PolygonData p)
    {
        if (p == null) return;
        if (p.OuterLoop != null) ReturnList(p.OuterLoop);
        if (p.Holes != null)
        {
            foreach (var hole in p.Holes) ReturnList(hole);
            p.Holes.Clear();
        }
        PolyPool.Push(p);
    }

    public void ClearAll()
    {
        CutIntersections.Clear();
        TempNewPath.Clear();
        TempHits.Clear();

        RawEdges.Clear();
        UniqueVertices.Clear();
        AliasMap.Clear();
        NativeGraph.Clear();
        FlattenedLoops.Clear();
        LoopRanges.Clear();
    }

    public void Dispose()
    {
        if (RawEdges.IsCreated) RawEdges.Dispose();
        if (UniqueVertices.IsCreated) UniqueVertices.Dispose();
        if (AliasMap.IsCreated) AliasMap.Dispose();
        if (NativeGraph.IsCreated) NativeGraph.Dispose();
        if (FlattenedLoops.IsCreated) FlattenedLoops.Dispose();
        if (LoopRanges.IsCreated) LoopRanges.Dispose();
    }

    // 提供给场景退出钩子
    public static void DisposeGlobal()
    {
        if (_instance != null)
        {
            _instance.Dispose();
            _instance = null;
            Debug.Log("[SlicerSystem] Native collections disposed safely.");
        }
    }

    // =====================================================================
    //                         AUTO-DISPOSE HOOKS
    // =====================================================================
#if UNITY_EDITOR
    [UnityEditor.InitializeOnLoadMethod]
    private static void EditorInit()
    {
        UnityEditor.EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        UnityEditor.EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void OnPlayModeStateChanged(UnityEditor.PlayModeStateChange state)
    {
        if (state == UnityEditor.PlayModeStateChange.ExitingPlayMode)
        {
            DisposeGlobal();
        }
    }
#endif

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void RuntimeInit()
    {
        DisposeGlobal(); // Editor Domain reload disabled coverage
        Application.quitting -= DisposeGlobal;
        Application.quitting += DisposeGlobal;
    }

    // =====================================================================
    //                             BURST JOBS
    // =====================================================================
    
    public struct PointWithIndex : System.IComparable<PointWithIndex>
    {
        public float2 P;
        public int OrigIndex;

        public int CompareTo(PointWithIndex other)
        {
            return P.x.CompareTo(other.P.x); // 按 X 单调排序
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct WeldingJob : IJob
    {
        [ReadOnly] public NativeArray<float2> RawEdges;
        public NativeList<float2> UniqueVertices;
        public NativeArray<int> AliasMap;
        public float ToleranceSq;
        public float ToleranceX;

        public void Execute()
        {
            int N = RawEdges.Length;
            if (N == 0) return;

            NativeArray<PointWithIndex> sorted = new NativeArray<PointWithIndex>(N, Allocator.Temp);
            for (int i = 0; i < N; i++) {
                sorted[i] = new PointWithIndex { P = RawEdges[i], OrigIndex = i };
            }
            
            sorted.Sort();

            int currentUniqueId = 0;
            
            for (int i = 0; i < N; i++)
            {
                if (i == 0) {
                    UniqueVertices.Add(sorted[i].P);
                    AliasMap[sorted[i].OrigIndex] = currentUniqueId;
                } else {
                    int matchedId = -1;
                    // 一维 Sweep-line 向后扫掠，剔除超限者
                    for (int j = i - 1; j >= 0; j--) {
                        if (sorted[i].P.x - sorted[j].P.x > ToleranceX) break; // 超过 X 容差中断，缓存高命中

                        if (math.distancesq(sorted[i].P, sorted[j].P) < ToleranceSq) {
                            matchedId = AliasMap[sorted[j].OrigIndex];
                            break;
                        }
                    }

                    if (matchedId != -1) {
                        AliasMap[sorted[i].OrigIndex] = matchedId;
                    } else {
                        currentUniqueId++;
                        UniqueVertices.Add(sorted[i].P);
                        AliasMap[sorted[i].OrigIndex] = currentUniqueId;
                    }
                }
            }
            sorted.Dispose();
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct BuildGraphJob : IJob
    {
        [ReadOnly] public NativeArray<int> AliasMap;
        public NativeParallelMultiHashMap<int, int> Graph;

        public void Execute()
        {
            for (int i = 0; i < AliasMap.Length; i += 2)
            {
                int u = AliasMap[i];
                int v = AliasMap[i + 1];

                if (u == v) continue; 

                // 查重过滤
                bool exists = false;
                if (Graph.TryGetFirstValue(u, out int neighbor, out var iterator))
                {
                    do {
                        if (neighbor == v) { exists = true; break; }
                    } while (Graph.TryGetNextValue(out neighbor, ref iterator));
                }

                if (!exists) Graph.Add(u, v);
            }
        }
    }

    [BurstCompile(CompileSynchronously = true, FloatMode = FloatMode.Fast)]
    public struct ExtractLoopsJob : IJob
    {
        [ReadOnly] public NativeParallelMultiHashMap<int, int> Graph;
        [ReadOnly] public NativeArray<float2> UniqueVertices;
        
        public NativeList<float2> FlattenedLoops;
        public NativeList<int2> LoopRanges;

        public void Execute()
        {
            NativeHashSet<long> visitedEdges = new NativeHashSet<long>(Graph.Capacity, Allocator.Temp);
            var (keys, keysCount) = Graph.GetUniqueKeyArray(Allocator.Temp);

            for (int k = 0; k < keysCount; k++)
            {
                int startNode = keys[k];
                
                if (Graph.TryGetFirstValue(startNode, out int nextNodeInitial, out var iterInitial))
                {
                    do
                    {
                        long edgeKey = ((long)startNode << 32) | (uint)nextNodeInitial;
                        if (visitedEdges.Contains(edgeKey)) continue;

                        int startIndex = FlattenedLoops.Length;
                        int curr = startNode;
                        int next = nextNodeInitial;

                        FlattenedLoops.Add(UniqueVertices[curr]);

                        int watchdog = 0;
                        int maxIter = keys.Length * 2 + 100;
                        bool loopClosed = false;

                        while (watchdog++ < maxIter)
                        {
                            visitedEdges.Add(((long)curr << 32) | (uint)next);
                            FlattenedLoops.Add(UniqueVertices[next]);

                            if (next == startNode)
                            {
                                loopClosed = true;
                                break;
                            }

                            int prev = curr;
                            curr = next;

                            next = GetLeftMostNeighbor(prev, curr);
                            if (next == -1) break; 
                        }

                        int count = FlattenedLoops.Length - startIndex;
                        if (loopClosed && count > 2)
                        {
                            FlattenedLoops.Length -= 1; // 裁掉强行封口的重合尾点
                            LoopRanges.Add(new int2(startIndex, count - 1));
                        }
                        else
                        {
                            // Rollback 失效回路
                            FlattenedLoops.Length = startIndex;
                        }

                    } while (Graph.TryGetNextValue(out nextNodeInitial, ref iterInitial));
                }
            }
            
            visitedEdges.Dispose();
            keys.Dispose();
        }

        private int GetLeftMostNeighbor(int prev, int curr)
        {
            float2 prevP = UniqueVertices[prev];
            float2 currP = UniqueVertices[curr];
            
            float2 inDir = currP - prevP;
            float lenSq = inDir.x * inDir.x + inDir.y * inDir.y;
            if (lenSq < 1e-10f) inDir = new float2(1, 0); 
            
            FixedList64Bytes<int> neighbors = new FixedList64Bytes<int>();
            if (Graph.TryGetFirstValue(curr, out int n, out var it))
            {
                do {
                    if (n == prev && Graph.CountValuesForKey(curr) > 1) continue; 
                    if (neighbors.Length < 15) neighbors.Add(n);
                } while (Graph.TryGetNextValue(out n, ref it));
            }

            if (neighbors.Length == 0) return -1;
            if (neighbors.Length == 1) return neighbors[0];

            int bestNeighbor = neighbors[0];
            for (int i = 1; i < neighbors.Length; i++)
            {
                int cand = neighbors[i];
                float2 OutCand = UniqueVertices[cand] - currP;
                float2 OutBest = UniqueVertices[bestNeighbor] - currP;
                
                int cmp = CompareLeftMost(OutBest, OutCand, inDir);
                if (cmp < 0) { // Cand is more left than Best
                    bestNeighbor = cand;
                }
            }
            return bestNeighbor;
        }

        private int CompareLeftMost(float2 OutA, float2 OutB, float2 inDir)
        {
            int catA = GetCategory(OutA, inDir);
            int catB = GetCategory(OutB, inDir);

            if (catA != catB) return catA > catB ? 1 : -1;

            if (catA == 0 || catA == 2) return 0;

            float cross = OutA.x * OutB.y - OutA.y * OutB.x;
            
            if (cross > 1e-5f) return -1; // B is CCW over A (OutA < OutB)
            if (cross < -1e-5f) return 1; // A is CCW over B (OutA > OutB)
            return 0;
        }

        private int GetCategory(float2 V, float2 inDir)
        {
            float c = inDir.x * V.y - inDir.y * V.x; 
            float d = inDir.x * V.x + inDir.y * V.y; 

            if (c > 1e-5f) return 3; // Left Plane
            if (c < -1e-5f) return 1; // Right Plane
            if (d > 0) return 2; // Forward Line
            return 0; // Backward Line
        }
    }
}
