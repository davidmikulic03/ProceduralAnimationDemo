using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bone : MonoBehaviour
{
    [SerializeField] Mesh boneModel;
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        //Gizmos.DrawWireMesh
    }
}
