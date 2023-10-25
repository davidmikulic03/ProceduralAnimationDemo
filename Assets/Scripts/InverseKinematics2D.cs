using System;
using UnityEngine;

public class InverseKinematics2D : MonoBehaviour
{
    [Tooltip("Every bone object in the order root, ..., end.")]
    [SerializeField] private JointIK[] joints;
    [SerializeField] private Transform target;
    [Range(1, 100)]
    [SerializeField] private int maxIterations;
    [Range(0.001f, 0.5f)]
    [SerializeField] float maxStepSize = 0.01f;
    [SerializeField] float epsilon = 0.01f;

    private void Start()
    {
        Initialize();
    }

    private void FixedUpdate()
    {
        IK();
    }

    void Initialize()
    {
        int numBones = joints.Length - 1;

        for (int i = 0; i < numBones; i++)
        {
            joints[i].length = GetJointLength(i);
            joints[i].restOfChainLength = GetLengthOfRemainingChain(i);
        }
    }

    float GetAngleToTarget(int jointIndex)
    {
        Vector3 toEnd = joints[^1].transform.position - joints[jointIndex].transform.position;
        Vector3 toTarget = target.position - joints[jointIndex].transform.position;

        Vector3 axis = Vector3.Cross(toEnd, toTarget);
        float angle = Mathf.Acos(Vector3.Dot(toEnd.normalized, toTarget.normalized)) * Mathf.Sign(axis.z);

        if (angle == float.NaN) 
            return 0;

        return angle;
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

        for(int i = jointIndex + 1; i < joints.Length - 1; i++)
        {
            output += GetJointLength(i);
        }

        return output;
    }

    void IK()
    {
        int i = 0;
        for(i = 0; i < maxIterations; i++) 
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
        float a = joints[jointIndex].length;
        float b = joints[jointIndex].restOfChainLength;
        float c = GetDistanceToTarget(jointIndex);

        float angle = GetAngleToTarget(jointIndex);

        Debug.Log(angle * Mathf.Rad2Deg);

        if (c > a + b)
        {
            joints[jointIndex].transform.localEulerAngles += angle * Vector3.forward;
            return;
        }

        if(c < Mathf.Abs(a - b)) 
        {
            joints[jointIndex].transform.localEulerAngles -= angle * Vector3.forward;
            return;
        }

        float aSqr = a * a;
        float bSqr = b * b;
        float cSqr = c * c;

        if(aSqr + bSqr > cSqr)
        {
            float cosPsi = (aSqr - bSqr + cSqr) / (2 * a * c);
            float psi = Mathf.Acos(cosPsi);
            float sign = MathF.Sign(angle);
            float betaT = Mathf.Abs(angle) - psi;
            betaT *= sign;

            if (betaT > 0.5 * Mathf.PI) betaT -= Mathf.PI;
            float increment = maxStepSize * Mathf.Deg2Rad;
            betaT += angle < 0 ? increment : -increment;

            joints[jointIndex].transform.eulerAngles += betaT * Vector3.forward;
        }
    }
}

[Serializable]
public struct JointIK
{
    public Transform transform;
    [HideInInspector] public float length;
    [HideInInspector] public float restOfChainLength;
    public IKConstraint constraint;
}

[Serializable]
public struct IKConstraint
{
    public float minAngle, maxAngle;
    public bool useMin, useMax;
}