using System;
using System.Collections.Generic;
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
    //                    Phase 2 直线切割并行重建与缝合 Job
    // =================================================================================

    public struct NativeIntersectionInfo
    {
        public float2 Point;
        public float T;
        public int SegmentIndex;
    }

    public struct NativeIntersectionComparer : IComparer<NativeIntersectionInfo>
    {
        public NativeArray<float2> Vertices;
        public int Offset;
        public int Compare(NativeIntersectionInfo a, NativeIntersectionInfo b)
        {
            if (a.SegmentIndex != b.SegmentIndex) return a.SegmentIndex.CompareTo(b.SegmentIndex);
            float2 segStart = Vertices[Offset + a.SegmentIndex];
            float distA = (a.Point.x - segStart.x) * (a.Point.x - segStart.x) + (a.Point.y - segStart.y) * (a.Point.y - segStart.y);
            float distB = (b.Point.x - segStart.x) * (b.Point.x - segStart.x) + (b.Point.y - segStart.y) * (b.Point.y - segStart.y);
            return distA.CompareTo(distB);
        }
    }

    public struct NativeCutIntersectionComparer : IComparer<float2>
    {
        public float2 Start, End;
        public int Compare(float2 a, float2 b)
        {
            float distA = (a.x - Start.x) * (End.x - Start.x) + (a.y - Start.y) * (End.y - Start.y);
            float distB = (b.x - Start.x) * (End.x - Start.x) + (b.y - Start.y) * (End.y - Start.y);
            return distA.CompareTo(distB);
        }
    }

    [BurstCompile]
    public struct RebuildPathJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> PathVerts;
        [ReadOnly] public NativeArray<int2> PathRanges;
        public float2 SliceStart;
        public float2 SliceEnd;

        public NativeStream.Writer EdgeStreamWriter;
        public NativeStream.Writer CutHitStreamWriter;

        private float SqrDist(float2 a, float2 b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        private bool GetLineIntersection(float2 p1, float2 p2, out float2 intersection, out float t)
        {
            intersection = float2.zero;
            t = 0f;

            float2 dir = SliceEnd - SliceStart;
            float lenSq = dir.x * dir.x + dir.y * dir.y;
            if (lenSq < 1e-8f) return false;

            float2 normal = new float2(-dir.y, dir.x);

            float dist1 = math.dot(normal, p1 - SliceStart);
            float dist2 = math.dot(normal, p2 - SliceStart);

            int sign1 = dist1 > 0f ? 1 : -1;
            int sign2 = dist2 > 0f ? 1 : -1;

            if (sign1 != sign2)
            {
                float u = dist1 / (dist1 - dist2);
                intersection = p1 + u * (p2 - p1);
                t = math.dot(intersection - SliceStart, dir) / lenSq;

                if (t >= -1e-5f && t <= 1f + 1e-5f) return true;
            }
            return false;
        }

        public void Execute(int index)
        {
            EdgeStreamWriter.BeginForEachIndex(index);
            CutHitStreamWriter.BeginForEachIndex(index);

            int2 range = PathRanges[index];
            int pCount = range.y;
            int offset = range.x;

            NativeList<NativeIntersectionInfo> tempHits = new NativeList<NativeIntersectionInfo>(16, Allocator.Temp);
            NativeList<float2> newPathVertices = new NativeList<float2>(pCount + 16, Allocator.Temp);

            for (int i = 0; i < pCount; i++)
            {
                float2 p1 = PathVerts[offset + i];
                float2 p2 = PathVerts[offset + ((i + 1) % pCount)];

                if (GetLineIntersection(p1, p2, out float2 intersection, out float t))
                {
                    tempHits.Add(new NativeIntersectionInfo { Point = intersection, T = t, SegmentIndex = i });
                }
            }

            if (tempHits.Length > 1) {
                tempHits.Sort(new NativeIntersectionComparer { Vertices = PathVerts, Offset = offset });
            }

            int hitIndex = 0;
            for (int i = 0; i < pCount; i++)
            {
                float2 currentVert = PathVerts[offset + i];
                if (newPathVertices.Length == 0 || SqrDist(newPathVertices[newPathVertices.Length - 1], currentVert) > 0.0001f)
                {
                    newPathVertices.Add(currentVert);
                }

                while (hitIndex < tempHits.Length && tempHits[hitIndex].SegmentIndex == i)
                {
                    float2 p = tempHits[hitIndex].Point;
                    if (SqrDist(newPathVertices[newPathVertices.Length - 1], p) <= 0.0001f)
                    {
                        CutHitStreamWriter.Write(newPathVertices[newPathVertices.Length - 1]);
                    }
                    else
                    {
                        newPathVertices.Add(p);
                        CutHitStreamWriter.Write(p);
                    }
                    hitIndex++;
                }
            }

            if (newPathVertices.Length > 1 && SqrDist(newPathVertices[0], newPathVertices[newPathVertices.Length - 1]) < 0.0001f)
            {
                newPathVertices.Length = newPathVertices.Length - 1; 
            }

            for (int i = 0; i < newPathVertices.Length; i++)
            {
                float2 u = newPathVertices[i];
                float2 v = newPathVertices[(i + 1) % newPathVertices.Length];
                if (SqrDist(u, v) > 0.0001f)
                {
                    EdgeStreamWriter.Write(u);
                    EdgeStreamWriter.Write(v);
                }
            }

            tempHits.Dispose();
            newPathVertices.Dispose();

            EdgeStreamWriter.EndForEachIndex();
            CutHitStreamWriter.EndForEachIndex();
        }
    }

    [BurstCompile]
    public struct FlattenAndSewJob : IJob
    {
        public NativeStream.Reader EdgeStreamReader;
        public NativeStream.Reader CutHitStreamReader;
        public int PathCount;
        public float2 SliceStart;
        public float2 SliceEnd;

        public NativeList<float2> RawEdges;

        public void Execute()
        {
            NativeList<float2> cutIntersections = new NativeList<float2>(64, Allocator.Temp);

            for (int i = 0; i < PathCount; i++)
            {
                int edgeCount = EdgeStreamReader.BeginForEachIndex(i);
                for (int e = 0; e < edgeCount; e++) {
                    RawEdges.Add(EdgeStreamReader.Read<float2>());
                }
                EdgeStreamReader.EndForEachIndex();

                int cutCount = CutHitStreamReader.BeginForEachIndex(i);
                for (int c = 0; c < cutCount; c++) {
                    cutIntersections.Add(CutHitStreamReader.Read<float2>());
                }
                CutHitStreamReader.EndForEachIndex();
            }

            if (cutIntersections.Length < 2) 
            {
                cutIntersections.Dispose();
                return;
            }

            cutIntersections.Sort(new NativeCutIntersectionComparer { Start = SliceStart, End = SliceEnd });

            int validCount = (cutIntersections.Length % 2 == 0) ? cutIntersections.Length : cutIntersections.Length - 1;
            for (int i = 0; i < validCount; i += 2)
            {
                float2 pA = cutIntersections[i];
                float2 pB = cutIntersections[i + 1];
                float dx = pA.x - pB.x; float dy = pA.y - pB.y;
                if (dx * dx + dy * dy > 0.0001f)
                {
                    RawEdges.Add(pA);
                    RawEdges.Add(pB);
                    RawEdges.Add(pB);
                    RawEdges.Add(pA);
                }
            }

            cutIntersections.Dispose();
        }
    }


}
