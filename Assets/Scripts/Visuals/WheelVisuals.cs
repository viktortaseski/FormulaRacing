using UnityEngine;

public class WheelVisuals : MonoBehaviour
{
    [SerializeField] private WheelCollider wheelCollider;
    [SerializeField] private bool isSteeringWheel;
    [SerializeField] private Transform steeringFin;
    [SerializeField] private float finSteerSmooth = 10f;

    private Transform wheelMesh;
    private float currentFinAngle;
    private float wheelSpin;
    private Quaternion baseRotation;

    private void Awake()
    {
        wheelMesh = transform;
        baseRotation = wheelMesh.localRotation;
    }

    private void LateUpdate()
    {
        UpdateWheelVisual();
    }

    private void UpdateWheelVisual()
    {
        Vector3 position;
        Quaternion colliderRotation;

        wheelCollider.GetWorldPose(out position, out colliderRotation);

        // Position always follows collider
        wheelMesh.position = position;

        if (isSteeringWheel)
        {
            // Front wheels: full collider rotation (this is correct)
            wheelMesh.rotation = colliderRotation;

            // Steering fins: steer only
            if (steeringFin != null)
            {
                float targetAngle = wheelCollider.steerAngle;

                currentFinAngle = Mathf.Lerp(
                    currentFinAngle,
                    targetAngle,
                    Time.deltaTime * finSteerSmooth
                );

                steeringFin.localRotation = Quaternion.Euler(
                    0f,
                    0f,
                    -currentFinAngle
                );
            }
        }
        else
        {
            wheelSpin += wheelCollider.rpm * 6f * Time.deltaTime;

            wheelMesh.localRotation =
                baseRotation * Quaternion.Euler(wheelSpin, 0f, 0f);
        }
    }
}
