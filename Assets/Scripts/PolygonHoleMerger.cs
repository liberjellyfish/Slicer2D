using UnityEngine;
using System.Collections.Generic;
using System;
public class PolygonHoleMerger
{
    public static List<Vector2> Merge(List<Vector2> outRing, List<List<Vector2>> holes)
    {
        List <Vector2>merged = new List<Vector2>(outRing);
        if(holes == null||holes.Count==0)return merged;

        //从最右边洞开始合并
        holes.Sort((a, b) => GetMaxX(b).CompareTo(GetMaxX(a)));

        for(int h = 0; h < holes.Count; h++)
        {
            List<Vector2> currentHole = holes[h];

            int holeMaxIndex = 0;
            float maxX = -float.MaxValue;
            for (int i = 0; i < currentHole.Count; i++)
            {
                if (currentHole[i].x > maxX)
                {
                    maxX = currentHole[i].x;
                    holeMaxIndex = i;
                }
            }
            Vector2 M = currentHole[holeMaxIndex];

            // 寻找最佳切点 (P)
            int bestConnectIndex = -1;
            float minDistSq = float.MaxValue;

            for (int i = 0; i < merged.Count; i++)
            {
                Vector2 P = merged[i];
                // P 必须在 M 的右侧 (简化逻辑，确保桥的方向统一)
                if (P.x > M.x)
                {
                    float distSq = (P - M).sqrMagnitude;
                    if (distSq < minDistSq)
                    {
                        // 【关键修复】这里不仅检查 merged，还要检查其他所有未合并的洞
                        if (IsBridgeValid(M, P, merged, holes, h))
                        {
                            minDistSq = distSq;
                            bestConnectIndex = i;
                        }
                    }
                }
            }
            // 如果找不到右侧点，尝试全图搜索（兜底逻辑）
            if (bestConnectIndex == -1)
            {
                for (int i = 0; i < merged.Count; i++)
                {
                    float distSq = (merged[i] - M).sqrMagnitude;
                    if (distSq < minDistSq)
                    {
                        if (IsBridgeValid(M, merged[i], merged, holes, h))
                        {
                            minDistSq = distSq;
                            bestConnectIndex = i;
                        }
                    }
                }
            }
            if(bestConnectIndex != -1)
            {
                List<Vector2> insertion = new List<Vector2>();

                for(int i=0;i<currentHole.Count;i++)
                {
                    insertion.Add(currentHole[(holeMaxIndex + i)%currentHole.Count]);
                }
                insertion.Add(M);//闭合
                insertion.Add(merged[bestConnectIndex]);//返回此点

                merged.InsertRange(bestConnectIndex + 1, insertion);
            }
            else
            {
                Debug.LogWarning("无法为洞找到合法的桥接点，该洞将被忽略。可能洞在多边形外或几何过于复杂。");
            }
        }
        return merged;
        

    }

    private static float GetMaxX(List<Vector2> poly)
    {
        float max = -float.MaxValue;
        foreach(var p in poly)
        {
            if(p.x > max) max = p.x;
        }
        return max;
    }

    private static bool IsBridgeValid(Vector2 start, Vector2 end, List<Vector2> mergedPoly, List<List<Vector2>> allHoles, int currentHoleIndex)
    {
        // 1. 检查是否与已合并的主体 (Outer + Merged Holes) 相交
        if (LineIntersectsPolygon(start, end, mergedPoly)) return false;

        // 2. 检查是否与尚未合并的其他洞相交
        // 只检查 index > currentHoleIndex 的洞，因为之前的已经合并进 mergedPoly 了
        for (int h = currentHoleIndex + 1; h < allHoles.Count; h++)
        {
            if (LineIntersectsPolygon(start, end, allHoles[h])) return false;
        }

        // 3. 检查是否与当前洞本身相交 
        if (LineIntersectsPolygon(start, end, allHoles[currentHoleIndex])) return false;

        

        return true;
    }

    private static bool LineIntersectsPolygon(Vector2 start, Vector2 end, List<Vector2> poly)
    {
        for (int i = 0; i < poly.Count; i++)
        {
            Vector2 p1 = poly[i];
            Vector2 p2 = poly[(i + 1) % poly.Count];

            // 忽略共享顶点的边（桥的端点肯定在多边形上，相交不算阻挡）
            if (p1 == start || p1 == end || p2 == start || p2 == end) continue;

            if (SegmentsIntersect(start, end, p1, p2)) return true;
        }
        return false;
    }

    private static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float denominator = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (denominator == 0) return false;

        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / denominator;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / denominator;

        return (u > 0 && u < 1 && v > 0 && v < 1);
    }

}