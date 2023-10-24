using System;
using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using UnityEngine;

public class InverseKinematics : MonoBehaviour
{
    [Tooltip("Every bone object in the order root, ..., end.")]
    [SerializeField] private JointIK[] joints;
    [SerializeField] private Transform target;
    [Range(1, 100)]
    [SerializeField] private int maxIterations;
    [Range(0.1f, 20f)]
    [SerializeField] float stepSize = 0.01f;
    [SerializeField] float epsilon = 0.01f;

    private void Start()
    {
        
    }

    private void FixedUpdate()
    {
        IK();
    }

    void TargetTriangle(int jointIndex, out float angle, out Vector3 axis)
    {
        Vector3 toEnd = joints[^1].transform.position - joints[jointIndex].transform.position;
        Vector3 toTarget = target.position - joints[jointIndex].transform.position;

        axis = Vector3.Cross(toEnd, toTarget);
        axis.y = 0;
        axis.x = 0;
        axis = axis.normalized;
        Debug.Log(axis);
        angle = Mathf.Acos(Vector3.Dot(toEnd.normalized, toTarget.normalized));
    }

    void GetLinkData(out float[] angles, out Vector3[] axes)
    {
        int numBones = joints.Length - 1;
        angles = new float[numBones];
        axes = new Vector3[numBones];

        for (int i = 0; i < numBones; i++)
            TargetTriangle(i, out angles[i], out axes[i]);
    }

    float GetJointLength(int jointIndex)
    {
        return (joints[jointIndex + 1].transform.position - joints[jointIndex].transform.position).magnitude;
    }

    float GetDistanceToTarget(int jointIndex)
    {
        return (target.position - joints[jointIndex].transform.position).magnitude;
    }

    float GetLengthOfRemainingChain(int jointIndex)
    {
        float output = 0;

        for(int i = jointIndex; i < joints.Length - 1; i++)
        {
            output += GetJointLength(i);
        }

        return output;
    }

    void IK()
    {
        int i = 0;
        for(i = 0; i < maxIterations - 1; i++) 
        {
            if ((target.position - joints[^1].transform.position).sqrMagnitude < epsilon * epsilon)
                break;

            IKStep();
        }
        Debug.Log(i);
    }

    void IKStep()
    {
        for(int i = 0; i < joints.Length - 1; i++)
        {
            SetRotationFor(i);
        }
    }

    void SetRotationFor(int jointIndex)
    {
        float a = GetJointLength(jointIndex);
        float b = GetLengthOfRemainingChain(jointIndex);
        float c = GetDistanceToTarget(jointIndex);

        float angle;
        Vector3 axis;
        TargetTriangle(jointIndex, out angle, out axis);

        if(c > a + b)
        {
            joints[jointIndex].transform.eulerAngles += angle * axis;
            return;
        }

        if(c < Mathf.Abs(a - b)) 
        {
            joints[jointIndex].transform.eulerAngles -= angle * axis;
            return;
        }

        float aSqr = a * a;
        float bSqr = b * b;
        float cSqr = c * c;

        if(aSqr + bSqr - cSqr > 0)
        {
            float cosPsi = (aSqr - bSqr + cSqr) / (2 * a * c);
            float psi = Mathf.Acos(cosPsi);

            float betaT = angle - psi;

            if (betaT > 0.5 * Mathf.PI) betaT -= Mathf.PI;
            float increment = stepSize * Mathf.Deg2Rad;
            betaT += increment;

            joints[jointIndex].transform.eulerAngles += betaT * axis;
        }
    }
}

[Serializable]
public struct JointIK
{
    public Transform transform;
    public IKConstraint constraint;
}

[Serializable]
public struct IKConstraint
{
    public float minAngle, maxAngle;
    public bool useMin, useMax;
}