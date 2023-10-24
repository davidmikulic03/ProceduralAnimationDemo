using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using UnityEngine;

public class ThreeBoneIK : MonoBehaviour
{
    [SerializeField] private Transform[] links;
    [SerializeField] private Transform target;
    [Range(1, 1000)]
    [SerializeField] private int maxIterations;
    [Range(0.0001f, 0.5f)]
    [SerializeField] float stepSize = 0.01f;
    [SerializeField] float epsilon = 0.01f;

    private void Start()
    {
        float[] angles;
        Vector3[] axes;

        GetLinkData(out angles, out axes);

        foreach (var angle in angles)
        {
            Debug.Log(angle * Mathf.Rad2Deg);
        }
    }

    private void FixedUpdate()
    {
        
    }

    void TargetTriangle(int linkIndex, out float angle, out Vector3 axis)
    {
        Vector3 toEnd = links[^1].position - links[linkIndex].position;
        Vector3 toTarget = target.position - links[linkIndex].position;

        axis = Vector3.Cross(toEnd, toTarget).normalized;
        angle = Mathf.Acos(Vector3.Dot(toEnd.normalized, toTarget.normalized));
    }

    void GetLinkData(out float[] angles, out Vector3[] axes)
    {
        int numBones = links.Length - 1;
        angles = new float[numBones];
        axes = new Vector3[numBones];

        for (int i = 0; i < numBones; i++)
            TargetTriangle(i, out angles[i], out axes[i]);
    }
}
