using UnityEngine;
using System.Collections.Generic;
public static class Triangulator
{
    //双向循环链表节点
    private class VertexNode
    {
        public Vector2 Position;
        public int Index;//索引
        public bool IsReflex;//是否凹点
        public VertexNode Prev;
        public VertexNode Next;

        public VertexNode(Vector2 pos, int index)
        {
            Position = pos;
            Index = index;
            IsReflex = false;
        }
    }

    //进行三角剖分
    public static int[] Trianglulate(Vector2[] vertices)
    {
        int n = vertices.Length;
        if (n < 3) return new int[0];

        List<VertexNode> nodeList = new List<VertexNode>(n);
        for(int i = 0; i < n; i++)
        {
            nodeList.Add(new VertexNode(vertices[i], i));
        }
        for(int i = 0; i < n; i++)
        {
            nodeList[i].Prev = nodeList[(i-1+n)%n];
            nodeList[i].Next = nodeList[(i + 1) % n];
        }
        float area = 0;//确保逆时针
        for(int i = 0; i < n; i++)
        {
            Vector2 p1 = nodeList[i].Position;
            Vector2 p2 = nodeList[i].Next.Position;
            area += (p2.x - p1.x) * (p2.y + p1.y);//梯形面积计算公式，与叉积法等效
        }
        if(area > 0)//顺时针情况翻转过来
        {
            for(int i = 0; i < n; i++)
            {
                VertexNode node = nodeList[i];
                VertexNode temp = node.Prev;
                node.Prev = node.Next;
                node.Next = temp;   
            }
        }
        //识别凹点
        List<VertexNode> reflexVertices = new List<VertexNode>();
        VertexNode current = nodeList[0];
        for(int i = 0; i < n; i++)
        {
            if (IsReflex(current))
            {
                current.IsReflex = true;
                reflexVertices.Add(current);
            }
            current = current.Next;
        }

        List<int> triangles = new List<int>();
        int pointCount = n;
        current = nodeList[0];

        int iteration = 0;
        int maxIteration = n * n * 2;//防止死循环

        while(pointCount > 3 && iteration < maxIteration)
        {
            iteration++;
            bool earFound = false;
            for(int i = 0; i < pointCount; i++)
            {
                if (IsEar(current, reflexVertices))//所有凹点都在当前三角形外部且本身为凸点
                {
                    earFound = true;
                    triangles.Add(current.Prev.Index);
                    triangles.Add(current.Index);
                    triangles.Add(current.Next.Index);

                    VertexNode prevNode = current.Prev;//移除节点，双向链表基础操作
                    VertexNode nextNode = current.Next;
                    prevNode.Next = nextNode;
                    nextNode.Prev = prevNode;

                    if (current.IsReflex) reflexVertices.Remove(current);
                    pointCount--;
                    //更新相邻节点的凹凸性
                    if(prevNode.IsReflex && !IsReflex(prevNode))
                    {
                        prevNode.IsReflex = false;
                        reflexVertices.Remove(prevNode);
                    }
                    if (nextNode.IsReflex && !IsReflex(nextNode))
                    {
                        nextNode.IsReflex = false;
                        reflexVertices.Remove(nextNode);
                    }

                    current = nextNode;
                    break;
                }
                current = current.Next;//移动当前指针
            }
            if (!earFound) break;
        }
        if(pointCount == 3)//剩下三个节点
        {
            triangles.Add(current.Prev.Index);
            triangles.Add(current.Index);
            triangles.Add(current.Next.Index);
        }
        return triangles.ToArray();
    }
    private static bool IsReflex(VertexNode v)
    {
        Vector2 a = v.Prev.Position;
        Vector2 b = v.Position;
        Vector2 c = v.Next.Position;
        return ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)) <= 0;
    }
    private static bool  IsEar(VertexNode v,List<VertexNode> reflecVertices)
    {
        if (v.IsReflex) return false;

        Vector2 a = v.Prev.Position;
        Vector2 b = v.Position;
        Vector2 c = v.Next.Position;

        foreach(var r in reflecVertices)
        {
            if (r == v.Prev || r == v || r == v.Next) continue;
            if (IsPointInTriangle(a, b, c, r.Position)) return false;
        }
        return true;
    }
    private static bool IsPointInTriangle(Vector2 a,Vector2 b, Vector2 c, Vector2 p)
    {
        bool check1 = ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) >= 0;
        bool check2 = ((c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x)) >= 0;
        bool check3 = ((a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x)) >= 0;
        return check1 && check2 && check3;
    }
}