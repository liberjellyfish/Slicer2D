using UnityEngine;

public static class SlicePieceFactory
{
    public static PooledSlicePiece Create(Transform parent)
    {
        GameObject go = new GameObject("PooledSlicePiece");
        go.transform.SetParent(parent, false);

        go.AddComponent<MeshFilter>();
        go.AddComponent<MeshRenderer>();
        go.AddComponent<PolygonCollider2D>();
        go.AddComponent<Rigidbody2D>();
        go.AddComponent<SliceableGenerator>();
        go.AddComponent<SliceableNativeData>();

        PooledSlicePiece piece = go.AddComponent<PooledSlicePiece>();
        piece.gameObject.SetActive(false);
        return piece;
    }
}
