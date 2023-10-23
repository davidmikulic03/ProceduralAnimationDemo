using System.Runtime.CompilerServices;
using UnityEngine;

[ExecuteAlways]
public class ThreeBoneIK : MonoBehaviour
{
    [SerializeField] private Transform root, mid, tip, tipEnd;
    [Range(0, 50)]
    [SerializeField] private int maxIterations;


    private Matrix4x4 GetJacobian(Vector3 position)
    {
        Matrix4x4 output = new Matrix4x4();

        
    }

    private float JacobianElement(Vector2 position, int i, int k)
    {
        if (i > 1 || k > 1)
            Debug.LogError("Column or row outside of range");

        float h = 0.001f;
        Vector2 unitVectorByK = k == 0 ? Vector2.right : Vector2.up;
        Vector2 newAdded = (position[k] * unitVectorByK - position) / h;
    }
}
