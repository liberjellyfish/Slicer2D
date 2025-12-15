using System;
using UnityEngine;

public class MouseSlicer : MonoBehaviour
{ 

    //起点终点坐标
    private Vector3 startPoint;
    private Vector3 endPoint;

    //当前是否在拖拽鼠标
    private bool isDragging = false;

    //引用组件绘制可见红线
    private LineRenderer lineVisualizer;

    public LayerMask sliceableLayer;//层级遮罩，只检测此层级

    //预分配射线检测结果数组。避免GC
    private RaycastHit2D[] hitResults = new RaycastHit2D[16];

    private ContactFilter2D contactFilter;

    void Start()
    {
        lineVisualizer = GetComponent<LineRenderer>();
        lineVisualizer.positionCount = 2;//起点终点
        lineVisualizer.enabled = false;//初始禁用渲染

        lineVisualizer.startWidth = 0.05f;//限制线条宽度
        lineVisualizer.endWidth = 0.05f;
    }

    void Update()
    {
        //鼠标按下
        if(Input.GetMouseButtonDown(0))
        {
            startPoint = GetWorldMousePosition();
            isDragging = true;
            lineVisualizer.enabled = true;
            //刚按下时起点终点重合
            lineVisualizer.SetPosition(0, startPoint);
            lineVisualizer.SetPosition(1, startPoint);
        }

        //鼠标拖拽中
        if (isDragging && Input.GetMouseButton(0))
        {
            endPoint = GetWorldMousePosition();

            lineVisualizer.SetPosition(1, endPoint);
        }

        if(isDragging && Input.GetMouseButtonUp(0))
        {
            isDragging = false;
            lineVisualizer.enabled = false; 
            endPoint = GetWorldMousePosition();

            PerformSlice(startPoint, endPoint);
        }
    }
    //将鼠标的屏幕坐标(像素)转换为世界坐标
    private Vector3 GetWorldMousePosition()
    {
        Vector3 screenPosition = Input.mousePosition;


        float distanceToCamera = -Camera.main.transform.position.z;
        screenPosition.z = distanceToCamera;


        Vector3 worldPos =  Camera.main.ScreenToWorldPoint(screenPosition);

        worldPos.z = -1f;

        return worldPos;
    }

    private void PerformSlice(Vector3 slicerStart,Vector3 slicerEnd)
    {
        //如果太短不切割
        if (Vector3.Distance(slicerStart, slicerEnd) < 0.1f) 
        {
            return;
        }
        Debug.Log($"[切割指令] Start: {slicerStart} -> End: {slicerEnd}");

        contactFilter.SetLayerMask(sliceableLayer);

        int hitCount = Physics2D.Linecast(slicerStart, slicerEnd, contactFilter,hitResults);

        Debug.Log($"[MouseSlicer] 这一刀切到了 {hitCount} 个物体");
        //实施切割算法
        for(int i=0;i<hitCount;i++)
        {
            GameObject target = hitResults[i].collider.gameObject;

            Slicer.Slice(target,slicerStart, slicerEnd);
        }

        

    }
}
