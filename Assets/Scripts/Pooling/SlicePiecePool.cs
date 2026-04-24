using System.Collections.Generic;
using UnityEngine;

public class SlicePiecePool : MonoBehaviour
{
    private static SlicePiecePool _instance;

    public static SlicePiecePool Instance
    {
        get
        {
            if (_instance == null)
            {
                GameObject go = new GameObject("SlicePiecePool");
                _instance = go.AddComponent<SlicePiecePool>();
                DontDestroyOnLoad(go);
            }

            return _instance;
        }
    }

    private readonly Stack<PooledSlicePiece> availablePieces = new Stack<PooledSlicePiece>(128);
    private Transform poolRoot;

    private void Awake()
    {
        if (_instance != null && _instance != this)
        {
            Destroy(gameObject);
            return;
        }

        _instance = this;
        DontDestroyOnLoad(gameObject);
        EnsurePoolRoot();
    }

    public PooledSlicePiece Rent()
    {
        EnsurePoolRoot();

        PooledSlicePiece piece = availablePieces.Count > 0 ? availablePieces.Pop() : SlicePieceFactory.Create(poolRoot);
        piece.IsInAvailablePool = false;
        return piece;
    }

    public void Prewarm(int count)
    {
        EnsurePoolRoot();
        while (availablePieces.Count < count)
        {
            PooledSlicePiece piece = SlicePieceFactory.Create(poolRoot);
            piece.IsInAvailablePool = true;
            availablePieces.Push(piece);
        }
    }

    internal void FinalizeReturn(PooledSlicePiece piece)
    {
        if (piece == null || piece.IsInAvailablePool)
        {
            return;
        }

        EnsurePoolRoot();
        piece.transform.SetParent(poolRoot, false);
        piece.IsInAvailablePool = true;
        availablePieces.Push(piece);
    }

    private void EnsurePoolRoot()
    {
        if (poolRoot != null)
        {
            return;
        }

        GameObject root = new GameObject("SlicePiecePoolRoot");
        root.transform.SetParent(transform, false);
        poolRoot = root.transform;
    }
}
