using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

public static class Slicer
{
    //存储顶点信息
    public struct VertexData
    {
        public Vector3 Position;
        public Vector2 UV;
    }

    //切割核心入口
    public static void Slice(GameObject target, Vector3 worldStart,Vector3 worldEnd)
    {
        //世界坐标->局部坐标
        Vector3 localP1 = target.transform.InverseTransformPoint(worldStart);
        Vector3 localP2 = target.transform.InverseTransformPoint(worldEnd);


        //转换为2D计算
        Vector2 p1 = new Vector2(localP1.x,localP1.y);
        Vector2 p2 = new Vector2(localP2.x,localP2.y);

        //获取网格数据
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        if (meshFilter == null) return;

        Mesh originalMesh = meshFilter.mesh;
        Vector3[] oldVertices = originalMesh.vertices;
        Vector2[] oldUVs = originalMesh.uv;

        //准备数据表
        List<VertexData> shapePoints = new List<VertexData>();
        for(int i=0;i<oldVertices.Length;i++)
        {
            shapePoints.Add(new VertexData { Position = oldVertices[i], UV = oldUVs[i] });
        }

        //准备两列表
        List<VertexData> posSide = new List<VertexData>();
        List<VertexData> negSide = new List<VertexData>();

        //核心几何算法
        for (int i = 0; i < shapePoints.Count; i++)
        {
            // 获取当前边的两个端点
            VertexData v1 = shapePoints[i];
            VertexData v2 = shapePoints[(i + 1) % shapePoints.Count]; // 取模以形成闭环

            // 判断这两个点在直线的哪一侧
            // 注意：这里使用的是基于直线的判定，哪怕线段没有物理接触，
            // 只要这两个点分布在无限直线的两侧，依然会被判为 true/false 不同
            bool v1Side = IsPointOnPositiveSide(p1, p2, v1.Position);
            bool v2Side = IsPointOnPositiveSide(p1, p2, v2.Position);

            // 逻辑分支
            if (v1Side == v2Side)
            {
                // 情况 A: 两个点在同一侧 -> 只要保留 V1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);
            }
            else
            {
                // 情况 B: 两个点在不同侧 -> 说明被线切断了
                // 1. 先加入 V1
                if (v1Side) posSide.Add(v1);
                else negSide.Add(v1);

                // 2. 计算交点 (Intersection)
                float t = GetIntersectionT(p1, p2, v1.Position, v2.Position);

                // 3. 插值生成新顶点
                VertexData intersectionPoint = new VertexData();
                intersectionPoint.Position = Vector3.Lerp(v1.Position, v2.Position, t);
                intersectionPoint.UV = Vector2.Lerp(v1.UV, v2.UV, t);

                //P = A + (B - A) * t

                // 4. 交点是两个新形状共有的，都要加入
                posSide.Add(intersectionPoint);
                negSide.Add(intersectionPoint);
            }
        }

        if (posSide.Count == 0 || negSide.Count == 0)
        {
            Debug.Log("[Slicer] 未产生有效切割 (所有点都在同一侧)");
            return;
        }

        Debug.Log($"[Slicer] 切割成功! 上半部分点数: {posSide.Count}, 下半部分点数: {negSide.Count}");

        // TODO: 下一步就是利用 posSide 和 negSide 生成两个新的 Mesh


        
 
    }
    //预备几何算法
    //点是否在直线左侧
    private static bool IsPointOnPositiveSide(Vector2 lineStart, Vector2 lineEnd, Vector2 p)
    {
        Vector2 lineVec = lineEnd - lineStart;
        Vector2 pointVec = p - lineStart;

        // 二维叉积: x1*y2 - x2*y1
        // 结果 > 0 代表在左侧， < 0 代表在右侧， = 0 在线上
        float crossProduct = lineVec.x * pointVec.y - lineVec.y * pointVec.x;

        return crossProduct > 0;
    }
    //计算直线与线段的交点比例（0-1）
    private static float GetIntersectionT(Vector2 lineStart, Vector2 lineEnd, Vector2 segStart, Vector2 segEnd)
    {
        float d1 = SignedDistance(lineStart, lineEnd, segStart);
        float d2 = SignedDistance(lineStart, lineEnd, segEnd);

        return Mathf.Abs(d1) / (Mathf.Abs(d1) + Mathf.Abs(d2));
    }
    //点到直线有向距离
    private static float SignedDistance(Vector2 lineStart, Vector2 lineEnd, Vector2 p)
    {
        return (lineEnd.x - lineStart.x) * (p.y - lineStart.y) - (lineEnd.y - lineStart.y) * (p.x - lineStart.x);
    }
}
