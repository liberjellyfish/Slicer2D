using UnityEngine;

[DisallowMultipleComponent]
public class CustomPolygonBatchSpawner : MonoBehaviour
{
    [Header("Template")]
    [Tooltip("拖入一个已经配置好的 CustomPolygon 测试物体，批量生成时会以它为模板克隆。")]
    public GameObject template;

    [Tooltip("生成时是否自动清空上一次生成的测试批次。")]
    public bool clearBeforeGenerate = true;

    [Tooltip("生成完成后是否临时隐藏模板本体，方便只观察批量测试对象。")]
    public bool hideTemplateAfterGenerate = false;

    [Header("Layout")]
    [Min(1)]
    public int columns = 4;

    [Min(1)]
    public int rows = 3;

    [Tooltip("自动按模板包围盒尺寸决定间距。建议保持开启。")]
    public bool autoSpacing = true;

    [Tooltip("自动间距下额外加出的横纵留白。")]
    public Vector2 extraSpacing = new Vector2(1.5f, 1.5f);

    [Tooltip("关闭自动间距后，使用固定的世界坐标间距。")]
    public Vector2 manualSpacing = new Vector2(8f, 8f);

    [Tooltip("是否以当前物体位置为中心生成网格。关闭后会从当前物体位置向右上展开。")]
    public bool centerGrid = true;

    [Header("Variation")]
    [Tooltip("给每个生成物体额外施加的随机位置扰动。")]
    public Vector2 positionJitter = Vector2.zero;

    [Tooltip("是否给每个生成物体一个随机 Z 轴旋转。")]
    public bool randomZRotation = false;

    public Vector2 randomZRotationRange = new Vector2(-8f, 8f);

    [Tooltip("是否对每个生成物体施加统一随机缩放。")]
    public bool randomUniformScale = false;

    public Vector2 uniformScaleRange = Vector2.one;

    [Header("Random")]
    [Tooltip("开启后，每次生成都会使用同一个随机种子，方便复现实验。")]
    public bool deterministicSeed = true;

    public int seed = 12345;

    [Header("Bookkeeping")]
    public string generatedRootName = "GeneratedCustomPolygonBatch";

    [SerializeField]
    private Transform generatedRoot;

    [ContextMenu("Generate Batch")]
    public void GenerateBatch()
    {
        if (template == null)
        {
            Debug.LogWarning("[CustomPolygonBatchSpawner] 请先指定模板物体。");
            return;
        }

        if (template == gameObject)
        {
            Debug.LogWarning("[CustomPolygonBatchSpawner] 模板物体不能是挂载生成器的同一个对象。");
            return;
        }

        if (clearBeforeGenerate)
        {
            ClearGenerated();
        }

        Transform root = EnsureGeneratedRoot();
        Vector2 cellSize = GetCellSize();
        Vector2 gridOffset = GetGridOffset(cellSize);

        Random.State savedState = Random.state;
        if (deterministicSeed)
        {
            Random.InitState(seed);
        }

        int totalCount = rows * columns;
        Quaternion baseRotation = template.transform.rotation;
        Vector3 baseScale = template.transform.localScale;

        for (int index = 0; index < totalCount; index++)
        {
            int col = index % columns;
            int row = index / columns;

            Vector2 cellOffset = new Vector2(col * cellSize.x, -row * cellSize.y) + gridOffset;
            Vector2 jitter = new Vector2(
                Random.Range(-positionJitter.x, positionJitter.x),
                Random.Range(-positionJitter.y, positionJitter.y));

            Vector3 worldPosition = transform.position +
                                    (transform.right * (cellOffset.x + jitter.x)) +
                                    (transform.up * (cellOffset.y + jitter.y));

            GameObject instance = Instantiate(template, root);
            instance.name = $"{template.name}_{index:000}";
            instance.SetActive(true);

            float zRotation = randomZRotation
                ? Random.Range(randomZRotationRange.x, randomZRotationRange.y)
                : 0f;
            instance.transform.SetPositionAndRotation(worldPosition, baseRotation * Quaternion.Euler(0f, 0f, zRotation));

            float uniformScale = randomUniformScale
                ? Random.Range(uniformScaleRange.x, uniformScaleRange.y)
                : 1f;
            instance.transform.localScale = baseScale * uniformScale;

            CustomPolygon polygon = instance.GetComponent<CustomPolygon>();
            if (polygon != null)
            {
                polygon.RegenerateMesh();
            }
        }

        if (deterministicSeed)
        {
            Random.state = savedState;
        }

        if (hideTemplateAfterGenerate)
        {
            template.SetActive(false);
        }
    }

    [ContextMenu("Regenerate Batch")]
    public void RegenerateBatch()
    {
        ClearGenerated();
        GenerateBatch();
    }

    [ContextMenu("Clear Generated Batch")]
    public void ClearGenerated()
    {
        Transform root = FindGeneratedRoot();
        if (root == null)
        {
            return;
        }

        for (int i = root.childCount - 1; i >= 0; i--)
        {
            GameObject child = root.GetChild(i).gameObject;
            if (Application.isPlaying)
            {
                Destroy(child);
            }
            else
            {
                DestroyImmediate(child);
            }
        }

        if (!Application.isPlaying)
        {
            template?.SetActive(true);
        }
    }

    private Transform EnsureGeneratedRoot()
    {
        Transform root = FindGeneratedRoot();
        if (root != null)
        {
            generatedRoot = root;
            return root;
        }

        GameObject rootObject = new GameObject(generatedRootName);
        rootObject.transform.SetParent(transform, false);
        generatedRoot = rootObject.transform;
        return generatedRoot;
    }

    private Transform FindGeneratedRoot()
    {
        if (generatedRoot != null)
        {
            return generatedRoot;
        }

        Transform child = transform.Find(generatedRootName);
        if (child != null)
        {
            generatedRoot = child;
        }

        return generatedRoot;
    }

    private Vector2 GetCellSize()
    {
        if (!autoSpacing)
        {
            return new Vector2(
                Mathf.Max(manualSpacing.x, 0.01f),
                Mathf.Max(manualSpacing.y, 0.01f));
        }

        Vector2 boundsSize = Vector2.one;

        Renderer renderer = template.GetComponent<Renderer>();
        if (renderer != null)
        {
            boundsSize = Vector2.Max(boundsSize, renderer.bounds.size);
        }

        PolygonCollider2D polygonCollider = template.GetComponent<PolygonCollider2D>();
        if (polygonCollider != null)
        {
            boundsSize = Vector2.Max(boundsSize, polygonCollider.bounds.size);
        }

        boundsSize.x = Mathf.Max(boundsSize.x + extraSpacing.x, 0.01f);
        boundsSize.y = Mathf.Max(boundsSize.y + extraSpacing.y, 0.01f);
        return boundsSize;
    }

    private Vector2 GetGridOffset(Vector2 cellSize)
    {
        if (!centerGrid)
        {
            return Vector2.zero;
        }

        float width = (columns - 1) * cellSize.x;
        float height = (rows - 1) * cellSize.y;
        return new Vector2(-width * 0.5f, height * 0.5f);
    }
}
