using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(PolygonCollider2D))]
[RequireComponent(typeof(Rigidbody2D), typeof(SliceableGenerator), typeof(SliceableNativeData))]
public class PooledSlicePiece : MonoBehaviour
{
    public MeshFilter MeshFilter { get; private set; }
    public MeshRenderer MeshRenderer { get; private set; }
    public PolygonCollider2D PolygonCollider { get; private set; }
    public Rigidbody2D Rigidbody2D { get; private set; }
    public SliceableGenerator SliceableGenerator { get; private set; }
    public SliceableNativeData SliceableNativeData { get; private set; }
    public Mesh ReusableMesh { get; private set; }

    public int SpawnVersion { get; private set; }
    public int PendingTaskCount { get; private set; }
    public bool ReturnRequested { get; private set; }
    internal bool IsInAvailablePool { get; set; }

    private void Awake()
    {
        MeshFilter = GetComponent<MeshFilter>();
        MeshRenderer = GetComponent<MeshRenderer>();
        PolygonCollider = GetComponent<PolygonCollider2D>();
        Rigidbody2D = GetComponent<Rigidbody2D>();
        SliceableGenerator = GetComponent<SliceableGenerator>();
        SliceableNativeData = GetComponent<SliceableNativeData>();

        if (ReusableMesh == null)
        {
            ReusableMesh = new Mesh { name = name + "_ReusableMesh" };
            ReusableMesh.MarkDynamic();
        }

        MeshFilter.sharedMesh = null;
        MeshRenderer.sharedMaterial = null;
        PolygonCollider.enabled = false;
        PolygonCollider.pathCount = 0;
        Rigidbody2D.simulated = false;
        SliceableGenerator.autoGenerateOnStart = false;
    }

    public void RetainForTask()
    {
        PendingTaskCount++;
    }

    public void ReleaseTaskReservation()
    {
        if (PendingTaskCount > 0)
        {
            PendingTaskCount--;
        }

        if (PendingTaskCount == 0 && ReturnRequested)
        {
            SlicePiecePool.Instance.FinalizeReturn(this);
        }
    }

    public void PrepareForSpawn(GameObject source, Material material, Rigidbody2D sourceBody, Rect uvReferenceRect)
    {
        SpawnVersion++;
        ReturnRequested = false;
        IsInAvailablePool = false;

        gameObject.name = source.name + "_Slice";
        transform.SetPositionAndRotation(source.transform.position, source.transform.rotation);
        transform.localScale = source.transform.localScale;
        transform.SetParent(null, true);
        gameObject.layer = source.layer;
        gameObject.tag = source.tag;
        gameObject.SetActive(true);

        MeshFilter.sharedMesh = ReusableMesh;
        MeshRenderer.sharedMaterial = material;

        PolygonCollider.enabled = false;
        PolygonCollider.pathCount = 0;

        Rigidbody2D.linearVelocity = Vector2.zero;
        Rigidbody2D.angularVelocity = 0f;
        Rigidbody2D.Sleep();
        Rigidbody2D.simulated = false;

        SliceableGenerator.hasUVReference = true;
        SliceableGenerator.uvReferenceRect = uvReferenceRect;
        SliceableGenerator.autoGenerateOnStart = false;

        if (sourceBody != null)
        {
            Rigidbody2D.useAutoMass = sourceBody.useAutoMass;
            Rigidbody2D.linearDamping = sourceBody.linearDamping;
            Rigidbody2D.angularDamping = sourceBody.angularDamping;
            Rigidbody2D.gravityScale = sourceBody.gravityScale;
            Rigidbody2D.collisionDetectionMode = sourceBody.collisionDetectionMode;
            Rigidbody2D.interpolation = sourceBody.interpolation;
            Rigidbody2D.sharedMaterial = sourceBody.sharedMaterial;
            Rigidbody2D.linearVelocity = sourceBody.linearVelocity;
            Rigidbody2D.angularVelocity = sourceBody.angularVelocity;
        }
        else
        {
            Rigidbody2D.sharedMaterial = null;
        }
    }

    public void CompleteSpawn(bool simulatePhysics)
    {
        PolygonCollider.enabled = true;
        Rigidbody2D.simulated = simulatePhysics;
    }

    public void RequestDespawn()
    {
        ReturnRequested = true;
        MeshFilter.sharedMesh = null;
        MeshRenderer.sharedMaterial = null;

        if (ReusableMesh != null)
        {
            ReusableMesh.Clear(false);
        }

        PolygonCollider.enabled = false;
        PolygonCollider.pathCount = 0;

        Rigidbody2D.linearVelocity = Vector2.zero;
        Rigidbody2D.angularVelocity = 0f;
        Rigidbody2D.Sleep();
        Rigidbody2D.simulated = false;

        SliceableGenerator.hasUVReference = false;
        SliceableGenerator.uvReferenceRect = default;
        SliceableGenerator.autoGenerateOnStart = false;

        gameObject.SetActive(false);

        if (PendingTaskCount == 0)
        {
            SlicePiecePool.Instance.FinalizeReturn(this);
        }
    }

    private void OnDestroy()
    {
        if (ReusableMesh != null)
        {
            Destroy(ReusableMesh);
            ReusableMesh = null;
        }
    }
}
