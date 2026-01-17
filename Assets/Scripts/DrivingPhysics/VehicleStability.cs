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

    [Header("Airborne Stabilizer")]
    [SerializeField] private bool enableAirborneStabilizer = true;
    [SerializeField] private float airborneDownforce = 8f;
    [SerializeField] private float maxAirborneDownforce = 30f;
    [SerializeField] private float airborneMinSpeedKph = 70f;
    [SerializeField] private float airborneMinUpwardVelocity = 1.25f;

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
    }

    private void ApplyDownforce()
    {
        if (!enableDownforce || rb == null)
            return;

        float speedKph = GetSpeedKph();
        if (speedKph < minDownforceSpeedKph)
            return;

        float downforce = downforceCoefficient * speedKph * speedKph;
        downforce = Mathf.Min(downforce, maxDownforce);
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
