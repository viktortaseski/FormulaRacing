using UnityEngine;

public class VehicleStability : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Rigidbody rb;
    [SerializeField] private WheelCollider frontLeftWheel;
    [SerializeField] private WheelCollider frontRightWheel;
    [SerializeField] private WheelCollider rearLeftWheel;
    [SerializeField] private WheelCollider rearRightWheel;

    [Header("Downforce")]
    [SerializeField] private bool enableDownforce = true;
    [SerializeField] private float downforceCoefficient = 0.02f;
    [SerializeField] private float maxDownforce = 2500f;
    [SerializeField] private float minDownforceSpeedKph = 30f;
    [SerializeField] private float downforceMultiplier = 1f;

    [Header("Airborne Stabilizer")]
    [SerializeField] private bool enableAirborneStabilizer = true;
    [SerializeField] private float airborneDownforce = 8f;
    [SerializeField] private float maxAirborneDownforce = 30f;
    [SerializeField] private float airborneMinSpeedKph = 70f;
    [SerializeField] private float airborneMinUpwardVelocity = 1.25f;

    [Header("Stability Assist")]
    [SerializeField] private bool enableSideSlipDamping = true;
    [SerializeField] private float sideSlipDamping = 2.2f;
    [SerializeField] private float maxSideSlipAcceleration = 8f;
    [SerializeField] private float sideSlipMinSpeedKph = 60f;
    [SerializeField] private bool enableYawStability = true;
    [SerializeField] private float yawStability = 2f;
    [SerializeField] private float yawMinSpeedKph = 60f;

    public float DownforceMultiplier
    {
        get => downforceMultiplier;
        set => downforceMultiplier = Mathf.Clamp(value, 0f, 2f);
    }

    private void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        ApplyStabilityForces();
    }

    private void ApplyStabilityForces()
    {
        ApplyDownforce();
        ApplyAirborneStabilizer();
        ApplySideSlipDamping();
        ApplyYawStability();
    }

    private void ApplyDownforce()
    {
        if (!enableDownforce || rb == null)
            return;

        float speedKph = GetSpeedKph();
        if (speedKph < minDownforceSpeedKph)
            return;

        float multiplier = Mathf.Max(0f, downforceMultiplier);
        float downforce = downforceCoefficient * speedKph * speedKph * multiplier;
        downforce = Mathf.Min(downforce, maxDownforce * multiplier);
        rb.AddForce(-transform.up * downforce, ForceMode.Force);
    }

    private void ApplyAirborneStabilizer()
    {
        if (!enableAirborneStabilizer || rb == null)
            return;

        if (IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < airborneMinSpeedKph)
            return;

        if (rb.linearVelocity.y < airborneMinUpwardVelocity)
            return;

        float force = Mathf.Min(airborneDownforce, maxAirborneDownforce);
        rb.AddForce(-transform.up * force, ForceMode.Acceleration);
    }

    private void ApplySideSlipDamping()
    {
        if (!enableSideSlipDamping || rb == null)
            return;

        if (!IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < sideSlipMinSpeedKph)
            return;

        float sideSpeed = Vector3.Dot(rb.linearVelocity, transform.right);
        float accel = Mathf.Clamp(-sideSpeed * sideSlipDamping, -maxSideSlipAcceleration, maxSideSlipAcceleration);
        rb.AddForce(transform.right * accel, ForceMode.Acceleration);
    }

    private void ApplyYawStability()
    {
        if (!enableYawStability || rb == null)
            return;

        if (!IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < yawMinSpeedKph)
            return;

        float yaw = rb.angularVelocity.y;
        rb.AddTorque(-transform.up * yaw * yawStability, ForceMode.Acceleration);
    }

    private bool IsAnyWheelGrounded()
    {
        if (!AnyWheelAssigned())
            return true;

        return (frontLeftWheel != null && frontLeftWheel.isGrounded)
            || (frontRightWheel != null && frontRightWheel.isGrounded)
            || (rearLeftWheel != null && rearLeftWheel.isGrounded)
            || (rearRightWheel != null && rearRightWheel.isGrounded);
    }

    private bool AnyWheelAssigned()
    {
        return frontLeftWheel != null
            || frontRightWheel != null
            || rearLeftWheel != null
            || rearRightWheel != null;
    }

    private float GetSpeedKph()
    {
        return rb != null ? rb.linearVelocity.magnitude * 3.6f : 0f;
    }
}
