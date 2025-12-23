using UnityEngine;
using System.Collections.Generic;
using Unity.VisualScripting;

public class SegmentAABBTree
{
    public struct Segment
    {
        public Vector2 P1;
        public Vector2 P2;
        public Bounds Box;

        public Segment(Vector2 p1, Vector2 p2)
        {
            P1 = p1;
            P2 = p2;
            Box = new Bounds((p1 + p2) * 0.5f, Vector3.zero);
            Box.Encapsulate(p1);
            Box.Encapsulate(p2);
            Box.Expand(0.001f);
        }
    }
    private class Node
    {
        public Bounds Box;
        public Node Left;
        public Node Right;
        public List<Segment> Segments;
        public bool IsLeaf => Left == null && Right == null;
    }

    private Node root;
    private const int MAX_SEG_EMTS_PER_LEAF = 4;//每个叶子最多四条边

    public SegmentAABBTree(List<Vector2> outerLoop, List<List<Vector2>> holes)
    {
        List<Segment> allSegments = new List<Segment>();

        AddLoop(outerLoop, allSegments);
        if(holes!=null)
        {
            foreach (var hole in holes) AddLoop(hole, allSegments);
        }

        root = BuildRecursive(allSegments);
    }

    private void AddLoop(List<Vector2> Loop, List<Segment> list)
    {
        int count = Loop.Count;
        for(int i=0;i<count;i++)
        {
            list.Add(new Segment(Loop[i], Loop[(i+1)%count]));
        }
    }

    private Node BuildRecursive(List<Segment> segments) 
    {

        Node node = new Node();

        Bounds bounds = segments[0].Box;
        for(int i=1;i<segments.Count;i++)
            bounds.Encapsulate(segments[i].Box);//扩展边界
        node.Box = bounds;
        if (segments.Count <= MAX_SEG_EMTS_PER_LEAF)
        {
            node.Segments = segments;
            return node;
        }
        bool splitX = bounds.size.x > bounds.size.y;//选择最长轴分裂
        float midPoint = splitX ? bounds.center.x : bounds.center.y;

        List<Segment> leftSegs = new List<Segment>();
        List<Segment> rightSegs = new List<Segment>();

        foreach(var seg in segments)
        {
            float segCenter = splitX?seg.Box.center.x:seg.Box.center.y;
            if(segCenter<midPoint)leftSegs.Add(seg);
            else rightSegs.Add(seg);
        }

        if(leftSegs.Count ==0 || rightSegs.Count == 0)
        {
            int half = segments.Count / 2;
            leftSegs = segments.GetRange(0,half);
            rightSegs = segments.GetRange(half, segments.Count - half);
        }

        node.Left = BuildRecursive(leftSegs);
        node.Right = BuildRecursive(rightSegs); 

        return node;
    }

    public bool Intersects(Vector2 start,Vector2 end)
    {
        Bounds queryBox = new Bounds((start+end)*0.5f,Vector3.zero);
        queryBox.Encapsulate(start);
        queryBox.Encapsulate(end);

        return IntersectsRecursive(root,queryBox,start,end);
    }

    private bool IntersectsRecursive(Node node,Bounds queryBox,Vector2 start,Vector2 end)
    {
        if(node == null)return false;
        if(!node.Box.Intersects(queryBox)) return false;

        if (node.IsLeaf)
        {
            foreach(var seg in node.Segments)
            {
                if (IsSamePoint(seg.P1, start) || IsSamePoint(seg.P1, end) ||
                    IsSamePoint(seg.P2, start) || IsSamePoint(seg.P2, end))
                    continue;

                if (SegmentsIntersect(start, end, seg.P1, seg.P2)) return true;
            }
            return false;
        }
        if (IntersectsRecursive(node.Left, queryBox, start, end)) return true;
        return IntersectsRecursive(node.Right, queryBox, start, end);
    }

    private bool IsSamePoint(Vector2 a, Vector2 b)
    {
        return (a - b).sqrMagnitude < 1e-9f;
    }

    private bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (den == 0) return false;

        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        float v = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;

        // 严格内部相交，不包括端点
        return (u > 1e-5f && u < 1f - 1e-5f && v > 1e-5f && v < 1f - 1e-5f);
    }
}