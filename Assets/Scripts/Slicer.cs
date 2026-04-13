using UnityEngine;
using System.Collections.Generic;

public static class Slicer
{
    public static void Slice(GameObject target, Vector3 worldStart, Vector3 worldEnd)
    {
        // 1. 获取 Unity 组件数据
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();
        if (polyCollider == null || meshRenderer == null) return;

        // 2. 坐标转换与准备
        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();
        if (generator != null && generator.hasUVReference) referenceRect = generator.uvReferenceRect;
        else referenceRect = CalculateLocalBounds(polyCollider);

        Vector2 localSliceStart = target.transform.InverseTransformPoint(worldStart);
        Vector2 localSliceEnd = target.transform.InverseTransformPoint(worldEnd);
        Vector2 cutDirection = (localSliceEnd - localSliceStart).normalized;
        if (cutDirection == Vector2.zero) return;

        // 延长切割线
        float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;
        localSliceStart = localSliceStart - cutDirection * extensionLength;
        localSliceEnd = localSliceEnd + cutDirection * extensionLength;

        // 3. 提取路径 
        List<List<Vector2>> originalPaths = new List<List<Vector2>>(polyCollider.pathCount);
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            Vector2[] pathArr = polyCollider.GetPath(i);
            var list = new List<Vector2>(pathArr);
            originalPaths.Add(list);
        }

        // 4. 调用核心算法 (Zero GC 热路径)
        List<SlicerCore.PolygonData> slicedPolygons = null;
        try
        {
            slicedPolygons = SlicerCore.Calculate(originalPaths, localSliceStart, localSliceEnd);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Slicer] Error: {e.Message}");
            return;
        }

        if (slicedPolygons == null || slicedPolygons.Count == 0) return;

        // 5. 生成物体
        bool success = true;
        try
        {
            foreach (var polyData in slicedPolygons)
            {
                CreateSlicedObject(polyData, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
            }
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] Mesh Generation Error: {e.Message}");
        }
        finally
        {
            // 归还 PolygonData 到池中
            SlicerCore.ReturnResultToPool(slicedPolygons);
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }

    /// <summary>
    /// 曲线贯穿切割：使用折线路径切割目标物体。
    /// 可以同时处理开放折线切透（直线、抛物线）、跨越边界的闭合环提取（如咬掉一个角）、以及纯净内部挖孔。
    /// </summary>
    public static void CurveSlice(GameObject target, List<Vector3> worldCutPath, bool isClosed)
    {
        PolygonCollider2D polyCollider = target.GetComponent<PolygonCollider2D>();
        MeshRenderer meshRenderer = target.GetComponent<MeshRenderer>();
        Rigidbody2D originalRb = target.GetComponent<Rigidbody2D>();
        if (polyCollider == null || meshRenderer == null) return;

        Rect referenceRect;
        var generator = target.GetComponent<SliceableGenerator>();
        if (generator != null && generator.hasUVReference) referenceRect = generator.uvReferenceRect;
        else referenceRect = CalculateLocalBounds(polyCollider);

        // 将世界空间路径转换为物体局部空间
        List<Vector2> localCutPath = new List<Vector2>(worldCutPath.Count);
        for (int i = 0; i < worldCutPath.Count; i++)
        {
            localCutPath.Add(target.transform.InverseTransformPoint(worldCutPath[i]));
        }

        if (localCutPath.Count < 2) return;

        // 提取原有路径数据
        List<List<Vector2>> originalPaths = new List<List<Vector2>>(polyCollider.pathCount);
        for (int i = 0; i < polyCollider.pathCount; i++)
        {
            originalPaths.Add(new List<Vector2>(polyCollider.GetPath(i)));
        }

        bool isPureHolePunch = false;

        // === Step A: 对非闭合路径——射线收敛检测 或 标准延长 ===
        if (!isClosed)
        {
            float extensionLength = Mathf.Max(referenceRect.width, referenceRect.height) * 1.5f + 1.0f;

            int last = localCutPath.Count - 1;
            Vector2 headDir = (localCutPath[1] - localCutPath[0]).normalized;
            Vector2 tailDir = (localCutPath[last] - localCutPath[last - 1]).normalized;

            // --- 射线收敛检测 (Ray Convergence) ---
            // 检测头部反向射线与尾部正向射线是否会在前方交汇。
            // 场景：玩家在多边形内部画了一条 V 形的"几乎闭合"路径，
            // 两端延长线必然在多边形外部交汇。此时应视为闭环，而非开放切割。
            // 标准有限延长可能不够长无法让线段实际相交，但射线检测不受长度限制。
            if (headDir != Vector2.zero && tailDir != Vector2.zero && localCutPath.Count >= 3)
            {
                Vector2 headRayDir = -headDir; // P0 的反向延长方向
                Vector2 tailRayDir = tailDir;   // PN 的正向延长方向

                // 射线相交公式：P0 + t * headRayDir = PN + s * tailRayDir
                // 利用叉积求解参数 t, s
                float crossRS = headRayDir.x * tailRayDir.y - headRayDir.y * tailRayDir.x;

                if (Mathf.Abs(crossRS) > 1e-6f) // 非平行
                {
                    Vector2 diff = localCutPath[last] - localCutPath[0];
                    float t = (diff.x * tailRayDir.y - diff.y * tailRayDir.x) / crossRS;
                    float s = (diff.x * headRayDir.y - diff.y * headRayDir.x) / crossRS;

                    // t > 0 且 s > 0：两条射线收敛（非发散），确实会在前方交汇
                    // 距离上限：防止近平行线产生极远交汇点导致退化（cap = 标准延长的 5 倍）
                    float maxRayDist = extensionLength * 5f;
                    if (t > 1e-4f && s > 1e-4f && t < maxRayDist && s < maxRayDist)
                    {
                        Vector2 meetPoint = localCutPath[0] + t * headRayDir;

                        // 将切割路径闭合为 [meetPoint, P0, P1, ..., PN, meetPoint]
                        localCutPath.Insert(0, meetPoint);
                        localCutPath.Add(meetPoint);
                        isClosed = true;

                        Debug.Log($"[Slicer] 射线收敛闭合: 交汇点 {meetPoint}, 头距 t={t:F3}, 尾距 s={s:F3}");
                    }
                }
            }

            // --- 标准延长（仅当射线收敛未触发时执行） ---
            if (!isClosed)
            {
                if (headDir != Vector2.zero)
                {
                    localCutPath[0] = localCutPath[0] - headDir * extensionLength;
                }

                int lastIdx = localCutPath.Count - 1;
                if (tailDir != Vector2.zero)
                {
                    localCutPath[lastIdx] = localCutPath[lastIdx] + tailDir * extensionLength;
                }

                // 延长后自交检测（兜底：标准延长足够长时也能捕获自交闭环）
                if (SlicerMath.DetectAndResolveSelfIntersection(localCutPath, out List<Vector2> postExtLoop))
                {
                    isClosed = true;
                    Debug.Log($"[Slicer] 延长后检测到自交闭环，已提取环 ({localCutPath.Count} 点)，切换为闭环模式");
                }
            }
        }

        // === Step B: 对闭合路径（包括延长后新发现的闭合）执行虚空自旋 ===
        if (isClosed)
        {
            // --- 虚空自旋算法 (Shift & Boolean) ---
            // 闭合环由于其闭包特性，必须保证起点不在实体肉内，以防止图论引擎的正交判定（Entry/Exit）反转。
            
            // 1. 寻找"虚空"中的点：在外边界之外（不在肉里），或者在任何一个孔洞之中（内部的虚空间）
            int emptySpaceIndex = -1;
            List<Vector2> outerPath = originalPaths[0];
            
            // 局部函数：验证点是否在虚空
            bool IsPointInEmptySpace(Vector2 p)
            {
                if (!SlicerMath.PointInPolygon(p, outerPath)) return true;
                for (int h = 1; h < originalPaths.Count; h++)
                {
                    if (SlicerMath.PointInPolygon(p, originalPaths[h])) return true;
                }
                return false;
            }

            for (int i = 0; i < localCutPath.Count; i++)
            {
                if (IsPointInEmptySpace(localCutPath[i]))
                {
                    emptySpaceIndex = i;
                    break;
                }
                
                // 线段过度采样 (Oversampling)：对抗 RDP 导致的弦高直线切边穿透内部曲线孔洞。
                // 防止线段两头端点在肉里，但线段中段已经把孔洞切开的严重漏检漏判。
                if (i < localCutPath.Count - 1)
                {
                    Vector2 p0 = localCutPath[i];
                    Vector2 p1 = localCutPath[i + 1];
                    // 自适应采样：按固定间距（0.05 单位）采样，防止长线段漏检虚空
                    float segLen = Vector2.Distance(p0, p1);
                    int samples = Mathf.Max(4, Mathf.CeilToInt(segLen / 0.05f));
                    for (int s = 1; s < samples; s++)
                    {
                        Vector2 mid = Vector2.Lerp(p0, p1, (float)s / samples);
                        if (IsPointInEmptySpace(mid))
                        {
                            localCutPath.Insert(i + 1, mid);
                            emptySpaceIndex = i + 1;
                            break;
                        }
                    }
                    if (emptySpaceIndex != -1) break;
                }
            }

            if (emptySpaceIndex == -1)
            {
                // [完全包围] 环线全在肉内，即使包围了内部孔也是纯净的实体吞噬，走合并算法
                isPureHolePunch = true;
            }
            else if (emptySpaceIndex > 0)
            {
                // [跨越边界/孔洞] 环线部分在肉外，存在拓扑切割，必须自旋对齐虚空起点
                List<Vector2> rotated = new List<Vector2>(localCutPath.Count);
                for (int i = emptySpaceIndex; i < localCutPath.Count - 1; i++) // -1 抛弃末尾，由最后补回
                    rotated.Add(localCutPath[i]);
                for (int i = 0; i < emptySpaceIndex; i++)
                    rotated.Add(localCutPath[i]);
                
                rotated.Add(rotated[0]); // 重新完美闭合
                localCutPath = rotated;
            }
        }

        // 分发逻辑
        if (isPureHolePunch)
        {
            PerformHolePunch(target, meshRenderer, originalRb, referenceRect, originalPaths, localCutPath);
            return;
        }

        // 调用核心曲线布尔运算算法
        List<SlicerCore.PolygonData> slicedPolygons = null;
        try
        {
            slicedPolygons = SlicerCore.CalculateCurve(originalPaths, localCutPath);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Slicer] CurveSlice Error: {e.Message}\n{e.StackTrace}");
            return;
        }

        if (slicedPolygons == null || slicedPolygons.Count == 0) return;

        bool success = true;
        try
        {
            foreach (var polyData in slicedPolygons)
            {
                CreateSlicedObject(polyData, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
            }
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] CurveSlice MeshGen Error: {e.Message}");
        }
        finally
        {
            SlicerCore.ReturnResultToPool(slicedPolygons);
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }

    /// <summary>
    /// 纯净内部挖孔切割：当闭合环完全处于多边形内部，且未切割外圈与内穴边时执行。
    /// （现已作为 CurveSlice 混合分流的一个私有保护层，不再向外界直接暴露）
    /// </summary>
    private static void PerformHolePunch(GameObject target, MeshRenderer meshRenderer, Rigidbody2D originalRb, 
                                         Rect referenceRect, List<List<Vector2>> originalPaths, List<Vector2> localLoop)
    {
        // 验证环中心点在多边形内
        List<Vector2> outerPath = originalPaths[0];
        Vector2 testPoint = localLoop[0];
        if (!SlicerMath.PointInPolygon(testPoint, outerPath))
        {
            Debug.LogWarning("[Slicer] HolePunch: 闭环不位于实体内部，这应在自旋阶段被拦截");
            return;
        }

        // 确保环的绕序为顺时针（作为孔洞）
        float loopArea = 0;
        for (int i = 0; i < localLoop.Count; i++)
        {
            Vector2 p1 = localLoop[i];
            Vector2 p2 = localLoop[(i + 1) % localLoop.Count];
            loopArea += (p1.x * p2.y) - (p2.x * p1.y);
        }

        // 面积太小的碎屑洞直接熔毁，不进行物理与合并计算，防止三角剖分崩溃
        if (Mathf.Abs(loopArea / 2f) < 0.001f)
        {
            return;
        }

        if (loopArea > 0) localLoop.Reverse(); // 如果是逆时针，翻转为顺时针

        // 构造母体碎片（带上新孔洞）
        SlicerCore.PolygonData motherPoly = new SlicerCore.PolygonData();
        motherPoly.OuterLoop = originalPaths[0];
        motherPoly.Holes = new List<List<Vector2>>();
        // 继承原有孔洞
        for (int i = 1; i < originalPaths.Count; i++)
        {
            motherPoly.Holes.Add(originalPaths[i]);
        }
        // 添加新孔洞
        motherPoly.Holes.Add(localLoop);

        float motherArea = 0;
        for (int i = 0; i < motherPoly.OuterLoop.Count; i++)
        {
            Vector2 p1 = motherPoly.OuterLoop[i];
            Vector2 p2 = motherPoly.OuterLoop[(i + 1) % motherPoly.OuterLoop.Count];
            motherArea += (p1.x * p2.y) - (p2.x * p1.y);
        }
        motherPoly.Area = Mathf.Abs(motherArea / 2f);

        // 构造掉落碎片（孔洞的反转形即为碎片的外边界）
        List<Vector2> pieceBoundary = new List<Vector2>(localLoop);
        pieceBoundary.Reverse(); // 顺时针 → 逆时针 = 实体外边界

        SlicerCore.PolygonData piecePoly = new SlicerCore.PolygonData();
        piecePoly.OuterLoop = pieceBoundary;
        piecePoly.Holes = new List<List<Vector2>>();

        // 检测原有孔洞中是否有被新环完全吞噬的孔洞
        for (int i = motherPoly.Holes.Count - 2; i >= 0; i--) // -2 因为最后一个是新孔洞
        {
            List<Vector2> existingHole = motherPoly.Holes[i];
            if (existingHole.Count > 0 && SlicerMath.PointInPolygon(existingHole[0], pieceBoundary))
            {
                // 旧孔被新环吞噬：从母体移除，转移给碎片
                piecePoly.Holes.Add(existingHole);
                motherPoly.Holes.RemoveAt(i);
            }
        }

        float pieceArea = 0;
        for (int i = 0; i < pieceBoundary.Count; i++)
        {
            Vector2 p1 = pieceBoundary[i];
            Vector2 p2 = pieceBoundary[(i + 1) % pieceBoundary.Count];
            pieceArea += (p1.x * p2.y) - (p2.x * p1.y);
        }
        piecePoly.Area = Mathf.Abs(pieceArea / 2f);

        // 生成两个物体
        bool success = true;
        try
        {
            CreateSlicedObject(motherPoly, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
            CreateSlicedObject(piecePoly, target, meshRenderer.sharedMaterial, originalRb, referenceRect);
        }
        catch (System.Exception e)
        {
            success = false;
            Debug.LogError($"[Slicer] HolePunch MeshGen Error: {e.Message}");
        }

        if (success)
        {
            Object.Destroy(target);
        }
    }

    private static Rect CalculateLocalBounds(PolygonCollider2D col)
    {
        // 逻辑不变...
        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;
        for (int i = 0; i < col.pathCount; i++)
        {
            Vector2[] path = col.GetPath(i);
            foreach (var p in path)
            {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }
        }
        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }
    private static void CreateSlicedObject(SlicerCore.PolygonData data, GameObject originalTemplate, Material mat, Rigidbody2D originalRb, Rect uvRefRect)
    {
        string baseName = originalTemplate.name.Replace("_Piece", "");
        GameObject newObj = new GameObject(baseName + "_Piece");
        newObj.transform.SetPositionAndRotation(originalTemplate.transform.position, originalTemplate.transform.rotation);
        newObj.transform.localScale = originalTemplate.transform.localScale;
        newObj.layer = originalTemplate.layer;
        newObj.tag = originalTemplate.tag;

        List<Vector2> mergedVertices = PolygonHoleMerger.Merge(data.OuterLoop, data.Holes);

        Vector3[] vertices3D = new Vector3[mergedVertices.Count];
        Vector2[] uvs = new Vector2[mergedVertices.Count];
        Vector2[] vertices2D = mergedVertices.ToArray(); // Triangulator 需要数组

        float width = uvRefRect.width < 0.0001f ? 1 : uvRefRect.width;
        float height = uvRefRect.height < 0.0001f ? 1 : uvRefRect.height;
        float minX = uvRefRect.x;
        float minY = uvRefRect.y;

        for (int i = 0; i < mergedVertices.Count; i++)
        {
            vertices3D[i] = mergedVertices[i];
            float u = (mergedVertices[i].x - minX) / width;
            float v = (mergedVertices[i].y - minY) / height;
            uvs[i] = new Vector2(u, v);
        }

        int[] indices = Triangulator.Triangulate(vertices2D);

        Mesh mesh = new Mesh();
        mesh.vertices = vertices3D;
        mesh.uv = uvs;
        mesh.triangles = indices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        MeshFilter mf = newObj.AddComponent<MeshFilter>();
        mf.mesh = mesh;
        MeshRenderer mr = newObj.AddComponent<MeshRenderer>();
        mr.material = mat;

        PolygonCollider2D pc = newObj.AddComponent<PolygonCollider2D>();
        pc.pathCount = 1 + data.Holes.Count;
        pc.SetPath(0, data.OuterLoop.ToArray());
        for (int i = 0; i < data.Holes.Count; i++)
        {
            pc.SetPath(i + 1, data.Holes[i].ToArray());
        }

        SliceableGenerator newGen = newObj.AddComponent<SliceableGenerator>();
        newGen.hasUVReference = true;
        newGen.uvReferenceRect = uvRefRect;
        newGen.autoGenerateOnStart = false;

        if (originalRb != null)
        {
            Rigidbody2D newRb = newObj.AddComponent<Rigidbody2D>();
            newRb.mass = originalRb.mass * (data.Area / 10f);
            newRb.useAutoMass = true;
            newRb.linearDamping = originalRb.linearDamping;
            newRb.angularDamping = originalRb.angularDamping;
            newRb.gravityScale = originalRb.gravityScale;
            newRb.collisionDetectionMode = originalRb.collisionDetectionMode;
            newRb.interpolation = originalRb.interpolation;
            newRb.sharedMaterial = originalRb.sharedMaterial;
            newRb.linearVelocity = originalRb.linearVelocity;
            newRb.angularVelocity = originalRb.angularVelocity;
        }
    }
}