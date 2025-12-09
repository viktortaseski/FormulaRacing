using UnityEngine;

public class WheelVisual : MonoBehaviour
{
    public WheelCollider targetCollider;  // the collider
    public Transform wheelMesh;           // the visual mesh (child)

    void LateUpdate()
    {
        if (targetCollider == null || wheelMesh == null)
            return;

        Vector3 pos;
        Quaternion rot;
        targetCollider.GetWorldPose(out pos, out rot);

        wheelMesh.position = pos;
        wheelMesh.rotation = rot;
    }
}
