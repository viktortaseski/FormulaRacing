using UnityEngine;

public class SteeringWheelVisual : MonoBehaviour
{
    [SerializeField] private WheelCollider referenceSteeringWheel;
    [SerializeField] private float maxSteerAngle = 30f;
    private float maxVisualRotation = 290f;
    private float rotationSmooth = 1f;

    private float baseX;

    private float currentX;

    private void Awake()
    {
        Vector3 baseEuler = transform.localEulerAngles;
        baseX = baseEuler.x;

        currentX = baseX;
    }

    private void LateUpdate()
    {
        UpdateSteeringWheel();
    }

    private void UpdateSteeringWheel()
    {
        float steerNormalized = Mathf.Clamp(
            referenceSteeringWheel.steerAngle / maxSteerAngle,
            -1f,
            1f
        );

        float targetX = baseX - steerNormalized * maxVisualRotation;

        currentX = Mathf.Lerp(
            currentX,
            targetX,
            Time.deltaTime * rotationSmooth
        );

        transform.localRotation = Quaternion.Euler(
            currentX,
            -90,
            -90
        );
    }
}
