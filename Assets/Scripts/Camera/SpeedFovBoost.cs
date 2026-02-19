using UnityEngine;

public class SpeedFovBoost : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private SimpleCarController carController;
    [SerializeField] private Rigidbody targetRigidbody;
    [SerializeField] private Camera[] cameras;

    [Header("FOV")]
    [SerializeField] private bool usePerCameraBaseFov = true;
    [SerializeField] private float baseFov = 60f;
    [SerializeField] private float fovBoost = 5f;

    [Header("Acceleration Detection")]
    [SerializeField] private bool useForwardAcceleration = true;
    [SerializeField] private float accelThreshold = 0.4f;
    [SerializeField] private float fovLerpSpeed = 6f;

    private float[] baseFovs;
    private Vector3 lastVelocity;
    private float boostT;

    private void Awake()
    {
        if (targetRigidbody == null && carController != null)
            targetRigidbody = carController.Rigidbody;

        if (targetRigidbody == null)
        {
            var foundCar = FindObjectOfType<SimpleCarController>();
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

        float accelValue = useForwardAcceleration
            ? Vector3.Dot(accel, targetRigidbody.transform.forward)
            : accel.magnitude;

        float target = accelValue > accelThreshold ? 1f : 0f;
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
