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
    [SerializeField] private float downforceCoefficient = 0.035f;
    [SerializeField] private float maxDownforce = 3500f;
    [SerializeField] private float minDownforceSpeedKph = 20f;
    [SerializeField] private float downforceMultiplier = 1f;

    [Header("Airborne Stabilizer")]
    [SerializeField] private bool enableAirborneStabilizer = true;
    [SerializeField] private float airborneDownforce = 8f;
    [SerializeField] private float maxAirborneDownforce = 30f;
    [SerializeField] private float airborneMinSpeedKph = 70f;
    [SerializeField] private float airborneMinUpwardVelocity = 1.25f;

    [Header("Stability Assist")]
    [SerializeField] private bool enableSideSlipDamping = true;
    [SerializeField] private float sideSlipDamping = 5.5f;
    [SerializeField] private float maxSideSlipAcceleration = 22f;
    [SerializeField] private float sideSlipMinSpeedKph = 15f;
    [SerializeField] private bool enableYawStability = true;
    [SerializeField] private float yawStability = 5.0f;
    [SerializeField] private float yawMinSpeedKph = 15f;

    [Header("Rear Traction Assist")]
    [Tooltip("Applies a lateral corrective force at the rear axle to prevent the back from sliding out.")]
    [SerializeField] private bool enableRearTractionAssist = true;
    [SerializeField] private float rearTractionDamping = 7.0f;
    [SerializeField] private float maxRearTractionAcceleration = 28f;
    [SerializeField] private float rearTractionMinSpeedKph = 10f;

    [Header("Brake Assist Turning")]
    [SerializeField] private bool useBrakeAssistTurningDropdown = true;
    [SerializeField][Range(0f, 1f)] private float turningAssistOffMultiplier = 0f;
    [SerializeField][Range(0f, 1f)] private float turningAssistLowMultiplier = 0.4f;
    [SerializeField][Range(0f, 1f)] private float turningAssistMediumMultiplier = 0.7f;
    [SerializeField][Range(0f, 1f)] private float turningAssistHighMultiplier = 1f;
    [SerializeField] private bool scaleWheelFrictionWithTurningAssist = true;
    [SerializeField][Range(0.2f, 1f)] private float offSidewaysFrictionStiffnessMultiplier = 0.9f;
    [Tooltip("Override sideways friction stiffness for front wheels (0 = read from WheelCollider).")]
    [SerializeField] private float frontSidewaysStiffness = 0f;
    [Tooltip("Override sideways friction stiffness for rear wheels (0 = read from WheelCollider).")]
    [SerializeField] private float rearSidewaysStiffness = 0f;

    private WheelCollider[] assistWheels = new WheelCollider[0];
    private float[] baseSidewaysStiffness = new float[0];

    public float DownforceMultiplier
    {
        get => downforceMultiplier;
        set => downforceMultiplier = Mathf.Clamp(value, 0f, 2f);
    }

    private void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        CacheWheelFrictionBaselines();
    }

    private void OnDisable()
    {
        RestoreBaseWheelFriction();
    }

    private void FixedUpdate()
    {
        ApplyStabilityForces();
    }

    private void ApplyStabilityForces()
    {
        float turningAssistMultiplier = GetTurningAssistMultiplier();
        ApplyWheelFrictionAssist(turningAssistMultiplier);
        ApplyDownforce();
        ApplyAirborneStabilizer();
        ApplySideSlipDamping(turningAssistMultiplier);
        ApplyYawStability(turningAssistMultiplier);
        ApplyRearTractionAssist(turningAssistMultiplier);
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
        rb.AddForce(-Vector3.up * force, ForceMode.Acceleration);
    }

    private void ApplySideSlipDamping(float turningAssistMultiplier)
    {
        if (!enableSideSlipDamping || rb == null)
            return;

        if (!IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < sideSlipMinSpeedKph)
            return;

        if (turningAssistMultiplier <= 0f)
            return;

        float effectiveDamping = sideSlipDamping * turningAssistMultiplier;
        float effectiveMaxSideSlipAcceleration = maxSideSlipAcceleration * turningAssistMultiplier;
        float sideSpeed = Vector3.Dot(rb.linearVelocity, transform.right);
        float accel = Mathf.Clamp(-sideSpeed * effectiveDamping, -effectiveMaxSideSlipAcceleration, effectiveMaxSideSlipAcceleration);
        rb.AddForce(transform.right * accel, ForceMode.Acceleration);
    }

    private void ApplyYawStability(float turningAssistMultiplier)
    {
        if (!enableYawStability || rb == null)
            return;

        if (!IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < yawMinSpeedKph)
            return;

        if (turningAssistMultiplier <= 0f)
            return;

        float yaw = rb.angularVelocity.y;
        float effectiveYawStability = yawStability * turningAssistMultiplier;
        rb.AddTorque(-transform.up * yaw * effectiveYawStability, ForceMode.Acceleration);
    }

    private void ApplyRearTractionAssist(float turningAssistMultiplier)
    {
        if (!enableRearTractionAssist || rb == null)
            return;

        if (rearLeftWheel == null || rearRightWheel == null)
            return;

        if (!IsAnyWheelGrounded())
            return;

        if (GetSpeedKph() < rearTractionMinSpeedKph)
            return;

        if (turningAssistMultiplier <= 0f)
            return;

        // Sample the velocity at the rear axle midpoint — this is more accurate than
        // the centre-of-mass velocity because it accounts for the car's yaw rotation.
        Vector3 rearAxleMidpoint = (rearLeftWheel.transform.position + rearRightWheel.transform.position) * 0.5f;
        float rearLateralSpeed = Vector3.Dot(rb.GetPointVelocity(rearAxleMidpoint), transform.right);

        float effectiveDamping = rearTractionDamping * turningAssistMultiplier;
        float effectiveMax = maxRearTractionAcceleration * turningAssistMultiplier;
        float accel = Mathf.Clamp(-rearLateralSpeed * effectiveDamping, -effectiveMax, effectiveMax);

        // Applying the force at the rear axle (not CoM) generates a corrective torque
        // that pulls the rear back into alignment as well as countering the lateral slip.
        rb.AddForceAtPosition(transform.right * accel, rearAxleMidpoint, ForceMode.Acceleration);
    }

    private float GetTurningAssistMultiplier()
    {
        if (!useBrakeAssistTurningDropdown)
            return 1f;

        switch (SettingsManager.GetTurningAssistLevelIndex(3))
        {
            case 0:
                return turningAssistOffMultiplier;
            case 1:
                return turningAssistLowMultiplier;
            case 2:
                return turningAssistMediumMultiplier;
            case 3:
            default:
                return turningAssistHighMultiplier;
        }
    }

    private void CacheWheelFrictionBaselines()
    {
        var wheels = new System.Collections.Generic.List<WheelCollider>(4);
        if (frontLeftWheel != null) wheels.Add(frontLeftWheel);
        if (frontRightWheel != null) wheels.Add(frontRightWheel);
        if (rearLeftWheel != null) wheels.Add(rearLeftWheel);
        if (rearRightWheel != null) wheels.Add(rearRightWheel);

        assistWheels = wheels.ToArray();
        baseSidewaysStiffness = new float[assistWheels.Length];
        for (int i = 0; i < assistWheels.Length; i++)
        {
            bool isFront = assistWheels[i] == frontLeftWheel || assistWheels[i] == frontRightWheel;
            float overrideValue = isFront ? frontSidewaysStiffness : rearSidewaysStiffness;
            baseSidewaysStiffness[i] = overrideValue > 0f ? overrideValue : assistWheels[i].sidewaysFriction.stiffness;
        }
    }

    private void ApplyWheelFrictionAssist(float turningAssistMultiplier)
    {
        if (!scaleWheelFrictionWithTurningAssist || assistWheels == null || baseSidewaysStiffness == null)
            return;

        float t = Mathf.Clamp01(turningAssistMultiplier);
        float sidewaysMultiplier = Mathf.Lerp(offSidewaysFrictionStiffnessMultiplier, 1f, t);

        for (int i = 0; i < assistWheels.Length; i++)
        {
            var wheel = assistWheels[i];
            if (wheel == null)
                continue;

            var sideways = wheel.sidewaysFriction;
            sideways.stiffness = baseSidewaysStiffness[i] * sidewaysMultiplier;
            wheel.sidewaysFriction = sideways;
        }
    }

    private void RestoreBaseWheelFriction()
    {
        if (assistWheels == null || baseSidewaysStiffness == null)
            return;

        for (int i = 0; i < assistWheels.Length; i++)
        {
            var wheel = assistWheels[i];
            if (wheel == null)
                continue;

            var sideways = wheel.sidewaysFriction;
            sideways.stiffness = baseSidewaysStiffness[i];
            wheel.sidewaysFriction = sideways;
        }
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
