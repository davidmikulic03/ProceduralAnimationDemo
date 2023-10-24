using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using UnityEngine;

[ExecuteAlways]
public class ThreeBoneIKOLD : MonoBehaviour
{
    [SerializeField] private Transform root, mid, tip, end;
    [SerializeField] private Transform target;
    [Range(1, 1000)]
    [SerializeField] private int maxIterations;
    [Range(0.0001f, 0.5f)]
    [SerializeField] float stepSize = 0.01f;
    [SerializeField] float epsilon = 0.01f;

    private Vector3 jointVector;

    private void FixedUpdate()
    {
        JacobianIK(epsilon);
    }

    void JacobianIK(float epsilon)
    {
        GetJointVector();
        int i = 0;

        for (i = 0; i < maxIterations; i++)
        {
            
            if((end.position - target.position).sqrMagnitude < epsilon * epsilon)
            {
                ApplyJointVector();
                break;
            }
                
            Vector3 deltaJoint = GetDeltaJointVector();
            jointVector += stepSize * deltaJoint;
            ApplyJointVector();
        }
        
        Debug.Log(i);
    }

    void GetJointVector()
    {
        jointVector = new Vector3(root.localEulerAngles.z, mid.localEulerAngles.z, tip.localEulerAngles.z);
    }
    void ApplyJointVector()
    {
        root.localEulerAngles = new Vector3(0, 0, jointVector.x);
        mid.localEulerAngles = new Vector3(0, 0, jointVector.y);
        tip.localEulerAngles = new Vector3(0, 0, jointVector.z);
    }

    Vector3 GetDeltaJointVector()
    {
        Matrix4x4 jacobianTranspose = GetJacobianTranspose();
        return jacobianTranspose * jointVector;
    }

    Matrix4x4 GetJacobianTranspose()
    {
        Vector3 fromRoot, fromMid, fromEnd;
        Vector3 columnA, columnB, columnC;

        fromRoot = target.position - root.position;
        fromMid = target.position - mid.position;
        fromEnd = target.position - end.position;

        columnA = Vector3.Cross(root.forward, fromRoot);
        columnB = Vector3.Cross(mid.forward, fromMid);
        columnC = Vector3.Cross(tip.forward, fromEnd);

        Matrix4x4 jacobian = new Matrix4x4();
        jacobian = new Matrix4x4(columnA, columnB, columnC, Vector3.one);
        jacobian.m30 = 1f;
        jacobian.m31 = 1f;

        return jacobian.transpose;
    }

    public static Vector3 CrossForward(Vector3 vector) 
    { 
        return new Vector3(vector.y, vector.x, 0f); 
    }
}
