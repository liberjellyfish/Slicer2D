using UnityEngine;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

public static class MeshSeparator
{
    //并查集数据结构
    private struct DSU
    {
        private int[] parent;
        public DSU(int n)
        {
            parent = new int[n];
            for(int i = 0; i < n; i++) parent[i] = i;
        }
        public int Find(int x)
        {
            if (parent[x] == x) return x;
            return parent[x] = Find(parent[x]);
        }
        public void Union(int x, int y)
        {
            int rootX = Find(x);
            int rootY = Find(y);
            if(rootX !=rootY)
            {
                parent[rootY] = rootX;
            }
        }
    }

    public static List<Mesh> Separate(Mesh mesh)
    {
        List<Mesh> islands = new List<Mesh>();
        int[] triangles = mesh.triangles;
        Vector3[] vertices = mesh.vertices;
        Vector2[] uv = mesh.uv;

        int vertexCount = vertices.Length;
        int triangleCount = triangles.Length / 3;

        if (vertexCount == 0) return islands;

        DSU dsu = new DSU(vertexCount);//初始化

        //合并联通的顶点
        for(int i=0;i<triangleCount;i++)
        {
            int v1 = triangles[i*3];
            int v2 = triangles[i*3+1];
            int v3 = triangles[i*3+2];

            dsu.Union(v1, v2);
            dsu.Union(v2, v3);
        }

        // 将三角形按照连通分量（Root ID）分组
        Dictionary<int,List<int>> components = new Dictionary<int,List<int>>();

        for(int i = 0; i < triangleCount; i++)
        {
            int v1 = triangles[i * 3];
            int root = dsu.Find(v1);

            if (!components.ContainsKey(root))
            {
                components[root] = new List<int>();
            }
            //记录的是三角形的ID (第几个三角形)，而不是顶点的索引
            components[root].Add(i);
        }
        //重建mesh
        foreach (var kvp in components)
        {
            islands.Add(CreateMeshFromTriangles(kvp.Value,vertices,uv,triangles));
        }

        return islands;

    }
    private static Mesh CreateMeshFromTriangles(List<int> triangleIndices, Vector3[] sourceVerts, Vector2[] sourceUV, int[] sourceTriangles)
    {
        //旧索引映射到新索引，接生内存
        Dictionary<int,int> indexMap = new Dictionary<int,int>();
        List<Vector3> newVerts = new List<Vector3>();
        List<Vector2> newUVs = new List<Vector2>();
        List<int> newTriangles = new List<int>();

        foreach(int triIdx in triangleIndices)
        {
            for(int k = 0; k < 3; k++)
            {
                int oldVertIdx = sourceTriangles[triIdx * 3 + k];
                if (!indexMap.ContainsKey(oldVertIdx))
                {
                    //旧索引变成新索引
                    indexMap[oldVertIdx] = newVerts.Count;
                    newVerts.Add(sourceVerts[oldVertIdx]);
                    newUVs.Add(sourceUV[oldVertIdx]);
                }
                //加入三角形顶点（索引是紧凑的）
                newTriangles.Add(indexMap[oldVertIdx]);
            }
        }

        Mesh newMesh = new Mesh();
        newMesh.vertices = newVerts.ToArray();
        newMesh.uv = newUVs.ToArray();
        newMesh.triangles = newTriangles.ToArray();

        newMesh.RecalculateNormals();
        newMesh.RecalculateBounds();

        return newMesh;
    }
}
