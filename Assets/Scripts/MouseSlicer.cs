using System;
using System.Collections.Generic;
using UnityEngine;

public class MouseSlicer : MonoBehaviour
{
    // =========================================================================
    //                          Inspector 配置
    // =========================================================================

    public enum SliceMode
    {
        [Tooltip("经典直线切割：按下 → 拖动 → 松开，两点一线")]
        Straight,

        [Tooltip("曲线切割：按下后沿鼠标轨迹采样折线，松开后提交")]
        Curved
    }

    [Header("切割模式")]
    public SliceMode sliceMode = SliceMode.Straight;

    [Header("曲线模式参数")]
    [Tooltip("相邻采样点的最小世界距离（越小越密，越大越省性能）")]
    [Range(0.05f, 0.5f)]
    public float curveMinSampleDist = 0.1f;

    [Tooltip("RDP 抽稀容差（越大越激进地删除点）")]
    [Range(0.01f, 0.2f)]
    public float rdpTolerance = 0.03f;

    [Header("物理检测")]
    public LayerMask sliceableLayer;

    // =========================================================================
    //                          私有状态
    // =========================================================================

    // 直线模式
    private Vector3 startPoint;
    private Vector3 endPoint;

    // 曲线模式
    private List<Vector3> curvePath = new List<Vector3>(256);

    // 通用
    private bool isDragging = false;
    private LineRenderer lineVisualizer;

    // 预分配射线检测结果数组（避免 GC）
    private RaycastHit2D[] hitResults = new RaycastHit2D[32];
    private ContactFilter2D contactFilter;

    // =========================================================================
    //                          生命周期
    // =========================================================================

    void Start()
    {
        lineVisualizer = GetComponent<LineRenderer>();
        lineVisualizer.positionCount = 2;
        lineVisualizer.enabled = false;
        lineVisualizer.startWidth = 0.05f;
        lineVisualizer.endWidth = 0.05f;
    }

    void Update()
    {
        // 鼠标按下
        if (Input.GetMouseButtonDown(0))
        {
            Vector3 worldPos = GetWorldMousePosition();
            isDragging = true;
            lineVisualizer.enabled = true;

            if (sliceMode == SliceMode.Straight)
            {
                startPoint = worldPos;
                lineVisualizer.positionCount = 2;
                lineVisualizer.SetPosition(0, startPoint);
                lineVisualizer.SetPosition(1, startPoint);
            }
            else // Curved
            {
                curvePath.Clear();
                curvePath.Add(worldPos);
                lineVisualizer.positionCount = 1;
                lineVisualizer.SetPosition(0, worldPos);
            }
        }

        // 鼠标拖拽中
        if (isDragging && Input.GetMouseButton(0))
        {
            Vector3 worldPos = GetWorldMousePosition();

            if (sliceMode == SliceMode.Straight)
            {
                endPoint = worldPos;
                lineVisualizer.SetPosition(1, endPoint);
            }
            else // Curved
            {
                // 只在移动超过最小距离时才采样新点
                if (curvePath.Count == 0 ||
                    Vector3.Distance(curvePath[curvePath.Count - 1], worldPos) >= curveMinSampleDist)
                {
                    curvePath.Add(worldPos);
                    lineVisualizer.positionCount = curvePath.Count;
                    lineVisualizer.SetPosition(curvePath.Count - 1, worldPos);
                }
            }
        }

        // 鼠标松开
        if (isDragging && Input.GetMouseButtonUp(0))
        {
            isDragging = false;
            lineVisualizer.enabled = false;

            if (sliceMode == SliceMode.Straight)
            {
                endPoint = GetWorldMousePosition();
                PerformStraightSlice(startPoint, endPoint);
            }
            else // Curved
            {
                // 添加最终松手点
                Vector3 finalPos = GetWorldMousePosition();
                if (curvePath.Count == 0 ||
                    Vector3.Distance(curvePath[curvePath.Count - 1], finalPos) > 0.01f)
                {
                    curvePath.Add(finalPos);
                }

                PerformCurvedSlice(curvePath);
            }
        }
    }

    // =========================================================================
    //                          坐标转换
    // =========================================================================

    private Vector3 GetWorldMousePosition()
    {
        Vector3 screenPosition = Input.mousePosition;
        float distanceToCamera = -Camera.main.transform.position.z;
        screenPosition.z = distanceToCamera;
        Vector3 worldPos = Camera.main.ScreenToWorldPoint(screenPosition);
        worldPos.z = -1f;
        return worldPos;
    }

    // =========================================================================
    //                       直线切割（保留原有逻辑）
    // =========================================================================

    private void PerformStraightSlice(Vector3 slicerStart, Vector3 slicerEnd)
    {
        if (Vector3.Distance(slicerStart, slicerEnd) < 0.1f) return;

        Debug.Log($"[直线切割] Start: {slicerStart} -> End: {slicerEnd}");

        contactFilter.SetLayerMask(sliceableLayer);
        int hitCount = Physics2D.Linecast(slicerStart, slicerEnd, contactFilter, hitResults);
        Debug.Log($"[MouseSlicer] 直线模式切到了 {hitCount} 个物体");

        HashSet<GameObject> processedObjects = new HashSet<GameObject>();
        for (int i = 0; i < hitCount; i++)
        {
            GameObject target = hitResults[i].collider.gameObject;
            if (target == null || processedObjects.Contains(target)) continue;
            processedObjects.Add(target);

            Slicer.Slice(target, slicerStart, slicerEnd);
        }
    }

    // =========================================================================
    //                       曲线切割（全新流水线）
    // =========================================================================

    private void PerformCurvedSlice(List<Vector3> rawPath)
    {
        if (rawPath.Count < 2) return;

        // Step 1: 转换为 Vector2 路径
        List<Vector2> path2D = new List<Vector2>(rawPath.Count);
        for (int i = 0; i < rawPath.Count; i++)
        {
            path2D.Add(new Vector2(rawPath[i].x, rawPath[i].y));
        }

        // Step 2: RDP 抽稀——在精度和性能间取平衡
        List<Vector2> simplified = SlicerMath.SimplifyRDP(path2D, rdpTolerance);

        // 抽稀后点数太少则放弃
        if (simplified.Count < 2) return;

        Debug.Log($"[曲线切割] 原始 {path2D.Count} 点 -> RDP 后 {simplified.Count} 点");

        // Step 3: 自交检测与回路提取
        bool isSelfIntersecting = SlicerMath.DetectAndResolveSelfIntersection(simplified, out List<Vector2> extractedLoop);

        if (isSelfIntersecting && extractedLoop != null)
        {
            // 发现自交回路——走内部挖孔逻辑
            Debug.Log($"[曲线切割] 检测到自交回路，提取为闭合环 ({extractedLoop.Count} 点)");
            PerformHolePunch(extractedLoop);
            return;
        }

        // Step 4: 检查首尾是否自然闭合（玩家慢慢画了一个圈）
        if (SlicerMath.IsClosedLoop(simplified))
        {
            // 闭合路径——走内部挖孔逻辑
            Debug.Log($"[曲线切割] 检测到闭合轨迹，走挖孔路径 ({simplified.Count} 点)");
            PerformHolePunch(simplified);
            return;
        }

        // Step 5: 非闭合曲线——走"曲线贯穿切割"逻辑
        Debug.Log($"[曲线切割] 提交曲线贯穿切割 ({simplified.Count} 点)");

        // 转回 Vector3 用于物理检测和 Slicer 接口
        List<Vector3> worldPath = new List<Vector3>(simplified.Count);
        for (int i = 0; i < simplified.Count; i++)
        {
            worldPath.Add(new Vector3(simplified[i].x, simplified[i].y, -1f));
        }

        // 使用 OverlapArea 或沿折线进行多段碰撞来检测哪些物体被曲线穿越
        PerformCurveSliceOnTargets(worldPath);
    }

    /// <summary>
    /// 挖孔逻辑：将闭合环作为新孔洞直接注入被切割物体。
    /// </summary>
    private void PerformHolePunch(List<Vector2> closedLoop)
    {
        // 通过环上的中心点寻找目标物体
        Vector2 centroid = Vector2.zero;
        for (int i = 0; i < closedLoop.Count; i++)
            centroid += closedLoop[i];
        centroid /= closedLoop.Count;

        contactFilter.SetLayerMask(sliceableLayer);
        Collider2D hit = Physics2D.OverlapPoint(centroid, contactFilter.layerMask);

        if (hit == null)
        {
            Debug.Log("[曲线切割] 挖孔环的中心点未命中任何可切割物体");
            return;
        }

        Slicer.HolePunch(hit.gameObject, closedLoop);
    }

    /// <summary>
    /// 曲线贯穿切割：对沿曲线扫掠到的物体执行多段线切割。
    /// </summary>
    private void PerformCurveSliceOnTargets(List<Vector3> worldPath)
    {
        contactFilter.SetLayerMask(sliceableLayer);
        HashSet<GameObject> processedObjects = new HashSet<GameObject>();

        // 沿折线的每一小段做 Linecast，收集所有命中物体
        for (int i = 0; i < worldPath.Count - 1; i++)
        {
            int hitCount = Physics2D.Linecast(worldPath[i], worldPath[i + 1], contactFilter, hitResults);
            for (int j = 0; j < hitCount; j++)
            {
                GameObject target = hitResults[j].collider.gameObject;
                if (target == null || processedObjects.Contains(target)) continue;
                processedObjects.Add(target);
            }
        }

        // 对收集到的每一个物体，执行曲线切割
        foreach (GameObject target in processedObjects)
        {
            if (target == null) continue;
            Slicer.CurveSlice(target, worldPath);
        }
    }
}
