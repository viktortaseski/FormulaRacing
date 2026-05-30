using UnityEngine;

public class SpeedFovBoost : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private CarPhysicsController carController;
    [SerializeField] private Rigidbody targetRigidbody;
    [SerializeField] private Camera[] cameras;

    [Header("FOV")]
    [SerializeField] private bool usePerCameraBaseFov = true;
    [SerializeField] private float baseFov = 60f;
    [SerializeField] private float fovBoost = 5f;

    [Header("Acceleration Detection")]
    [SerializeField] private bool useForwardAcceleration = true;
    [SerializeField] private float accelThreshold = 1.6f;
    [SerializeField] private float accelFullBoost = 6f;
    [SerializeField] private float accelReleaseThreshold = 1.1f;
    [SerializeField] private float minSpeedForBoostKph = 25f;
    [SerializeField] private float accelSmoothing = 10f;
    [SerializeField] private float fovLerpSpeed = 6f;

    private float[] baseFovs;
    private Vector3 lastVelocity;
    private float boostT;
    private float smoothedAccelValue;

    private void Awake()
    {
        if (targetRigidbody == null && carController != null)
            targetRigidbody = carController.Rigidbody;

        if (targetRigidbody == null)
        {
            var foundCar = FindFirstObjectByType<CarPhysicsController>();
            if (foundCar != null)
                targetRigidbody = foundCar.Rigidbody;
        }

        if (cameras == null || cameras.Length == 0)
            cameras = GetComponentsInChildren<Camera>(true);

        if (cameras == null || cameras.Length == 0)
        {
            enabled = false;
            return;
        }

        baseFovs = new float[cameras.Length];
        for (int i = 0; i < cameras.Length; i++)
            baseFovs[i] = usePerCameraBaseFov ? cameras[i].fieldOfView : baseFov;

        if (targetRigidbody != null)
            lastVelocity = targetRigidbody.linearVelocity;
    }

    private void FixedUpdate()
    {
        if (targetRigidbody == null)
            return;

        Vector3 velocity = targetRigidbody.linearVelocity;
        Vector3 accel = (velocity - lastVelocity) / Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        lastVelocity = velocity;

        float rawAccelValue = useForwardAcceleration
            ? Vector3.Dot(accel, targetRigidbody.transform.forward)
            : accel.magnitude;

        float smoothingRate = Mathf.Max(0f, accelSmoothing);
        if (smoothingRate > 0f)
            smoothedAccelValue = Mathf.Lerp(smoothedAccelValue, rawAccelValue, 1f - Mathf.Exp(-smoothingRate * Time.fixedDeltaTime));
        else
            smoothedAccelValue = rawAccelValue;

        float speedKph = velocity.magnitude * 3.6f;
        float target = 0f;

        if (speedKph >= minSpeedForBoostKph && smoothedAccelValue > accelReleaseThreshold)
        {
            float fullBoostAccel = Mathf.Max(accelThreshold + 0.01f, accelFullBoost);
            target = Mathf.InverseLerp(accelThreshold, fullBoostAccel, smoothedAccelValue);
        }

        boostT = Mathf.MoveTowards(boostT, target, fovLerpSpeed * Time.fixedDeltaTime);
    }

    private void LateUpdate()
    {
        if (cameras == null || baseFovs == null)
            return;

        for (int i = 0; i < cameras.Length; i++)
        {
            if (cameras[i] == null)
                continue;

            float targetFov = Mathf.Lerp(baseFovs[i], baseFovs[i] + fovBoost, boostT);
            cameras[i].fieldOfView = targetFov;
        }
    }
}
