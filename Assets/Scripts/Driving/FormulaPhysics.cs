using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class FormulaPhysics : MonoBehaviour
{
    [Header("Debug")]
    public bool debugNoDrive = false;

    [Header("References")]
    public Rigidbody rb;
    public Transform centerOfMass;

    [Header("Wheel Colliders")]
    public WheelCollider frontLeft;
    public WheelCollider frontRight;
    public WheelCollider rearLeft;
    public WheelCollider rearRight;

    [Header("Engine & Gears")]
    [Tooltip("Approx idle RPM.")]
    public float minEngineRPM = 4000f;

    [Tooltip("Max engine RPM.")]
    public float maxEngineRPM = 15000f;

    [Tooltip("Auto upshift RPM.")]
    public float upshiftRPM = 13500f;

    [Tooltip("Auto downshift RPM.")]
    public float downshiftRPM = 7000f;

    [Tooltip("Final drive ratio (diff).")]
    public float finalDriveRatio = 3.6f;

    [Tooltip("Forward gear ratios: index 0 = 1st gear.")]
    public float[] gearRatios = new float[]
    {
        3.10f,  // 1st
        2.30f,  // 2nd
        1.80f,  // 3rd
        1.50f,  // 4th
        1.30f,  // 5th
        1.15f,  // 6th
        1.05f,  // 7th
        0.95f   // 8th
    };

    [Tooltip("Torque curve: x = RPM, y = Nm.")]
    public AnimationCurve engineTorqueCurve = new AnimationCurve(
        new Keyframe(4000f, 450f),
        new Keyframe(8000f, 700f),
        new Keyframe(11000f, 820f),
        new Keyframe(13000f, 860f),
        new Keyframe(15000f, 780f)
    );

    [Tooltip("Max wheel torque per wheel (for stability).")]
    public float maxWheelTorque = 8000f;


    [Header("Steering")]
    [Tooltip("Max steering angle in degrees.")]
    public float maxSteerAngle = 30f;
    [Tooltip("How quickly wheels follow steering input.")]
    public float steerResponse = 8f;

    [Header("Brakes")]
    [Tooltip("Max brake torque (Nm) per wheel.")]
    public float maxBrakeTorque = 9000f;


    [Header("Tyre Friction Tuning")]
    public float forwardStiffness = 2.5f;
    public float sidewaysStiffness = 2.0f;

    [Header("Aero")]
    [Tooltip("Downforce coefficient. Force ~ k * v^2.")]
    public float downforceCoefficient = 45f;

    [Header("Inputs (from controls scripts)")]
    [Range(-1f, 1f)] public float steeringInput;  // -1..1
    [Range(0f, 1f)] public float throttleInput;  // 0..1
    [Range(0f, 1f)] public float brakeInput;     // 0..1

    [Header("Debug (read-only)")]
    public float engineRPM;
    public float speedKPH;
    public int currentGear = 0;       // 0 = 1st gear
    public float theoreticalTopSpeedKPH;

    private float _wheelRadius;
    private float _smoothSteerL;
    private float _smoothSteerR;

    // =========================
    // PUBLIC API (for controls)
    // =========================
    public void SetInputs(float steer, float throttle, float brake)
    {
        steeringInput = Mathf.Clamp(steer, -1f, 1f);
        throttleInput = Mathf.Clamp01(throttle);
        brakeInput = Mathf.Clamp01(brake);
    }

    // =========================
    // LIFECYCLE
    // =========================
    private void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        if (centerOfMass != null)
        {
            rb.centerOfMass = centerOfMass.localPosition;
        }

        // Try to auto-detect wheel radius
        if (rearLeft != null) _wheelRadius = rearLeft.radius;
        else if (rearRight != null) _wheelRadius = rearRight.radius;
        else if (frontLeft != null) _wheelRadius = frontLeft.radius;
        else if (frontRight != null) _wheelRadius = frontRight.radius;

        // Apply tyre friction settings
        ConfigureWheelFriction(frontLeft);
        ConfigureWheelFriction(frontRight);
        ConfigureWheelFriction(rearLeft);
        ConfigureWheelFriction(rearRight);

        ComputeTheoreticalTopSpeed();
    }

    private void FixedUpdate()
    {
        // Update speed for info
        speedKPH = rb.linearVelocity.magnitude * 3.6f;

        UpdateEngineRPM();
        AutoGearbox();

        if (!debugNoDrive)
        {
            ApplySteering();
            ApplyMotorTorque();
            ApplyBrakes();
        }

        ApplyDownforce();
    }

    // =========================
    // FRICTION
    // =========================
    private void ConfigureWheelFriction(WheelCollider wc)
    {
        if (wc == null) return;

        WheelFrictionCurve f = wc.forwardFriction;
        f.extremumSlip = 0.15f;
        f.extremumValue = 1.2f;
        f.asymptoteSlip = 0.8f;
        f.asymptoteValue = 0.8f;
        f.stiffness = forwardStiffness;
        wc.forwardFriction = f;

        WheelFrictionCurve s = wc.sidewaysFriction;
        s.extremumSlip = 0.2f;
        s.extremumValue = 1.0f;
        s.asymptoteSlip = 0.5f;
        s.asymptoteValue = 0.75f;
        s.stiffness = sidewaysStiffness;
        wc.sidewaysFriction = s;
    }


    // =========================
    // ENGINE & GEARS
    // =========================
    private void UpdateEngineRPM()
    {
        if (_wheelRadius <= 0.0001f || gearRatios.Length == 0)
        {
            // No wheels? just keep at idle
            engineRPM = Mathf.Lerp(engineRPM, minEngineRPM, Time.fixedDeltaTime * 2f);
            return;
        }

        float speedMS = rb.linearVelocity.magnitude;
        float wheelRPM = speedMS * 60f / (2f * Mathf.PI * _wheelRadius);

        float gearRatio = gearRatios[Mathf.Clamp(currentGear, 0, gearRatios.Length - 1)];

        float targetEngineRPM = wheelRPM * gearRatio * finalDriveRatio;
        targetEngineRPM = Mathf.Clamp(targetEngineRPM, minEngineRPM, maxEngineRPM);

        engineRPM = Mathf.Lerp(engineRPM, targetEngineRPM, Time.fixedDeltaTime * 5f);
    }

    private void AutoGearbox()
    {
        if (gearRatios.Length == 0)
            return;

        // Don't shift when basically parked
        if (speedKPH < 5f)
        {
            currentGear = 0;
            return;
        }

        // Upshift
        if (engineRPM > upshiftRPM && currentGear < gearRatios.Length - 1)
        {
            currentGear++;
        }

        // Downshift
        if (engineRPM < downshiftRPM && currentGear > 0)
        {
            currentGear--;
        }
    }

    // =========================
    // TORQUE / BRAKES
    // =========================
    private void ApplyMotorTorque()
    {
        if (rearLeft == null || rearRight == null || gearRatios.Length == 0)
            return;

        float gearRatio = gearRatios[Mathf.Clamp(currentGear, 0, gearRatios.Length - 1)];

        // Throttle → engine torque → wheel torque
        float baseTorque = engineTorqueCurve.Evaluate(engineRPM);
        float engineTorque = baseTorque * throttleInput;

        float driveTorque = engineTorque * gearRatio * finalDriveRatio;
        driveTorque = Mathf.Clamp(driveTorque, -maxWheelTorque * 2f, maxWheelTorque * 2f);
        float wheelTorque = driveTorque * 0.5f;


        rearLeft.motorTorque = wheelTorque;
        rearRight.motorTorque = wheelTorque;
    }

    private void ApplyBrakes()
    {
        float brakeTorque = brakeInput * maxBrakeTorque;

        if (frontLeft != null) frontLeft.brakeTorque = brakeTorque;
        if (frontRight != null) frontRight.brakeTorque = brakeTorque;
        if (rearLeft != null) rearLeft.brakeTorque = brakeTorque;
        if (rearRight != null) rearRight.brakeTorque = brakeTorque;
    }

    // =========================
    // STEERING & AERO
    // =========================
    private void ApplySteering()
    {
        if (frontLeft == null || frontRight == null)
            return;

        float targetSteer = steeringInput * maxSteerAngle;

        _smoothSteerL = Mathf.Lerp(_smoothSteerL, targetSteer, steerResponse * Time.fixedDeltaTime);
        _smoothSteerR = Mathf.Lerp(_smoothSteerR, targetSteer, steerResponse * Time.fixedDeltaTime);

        frontLeft.steerAngle = _smoothSteerL;
        frontRight.steerAngle = _smoothSteerR;
    }

    private void ApplyDownforce()
    {
        float speed = rb.linearVelocity.magnitude;
        float downforce = downforceCoefficient * speed * speed;

        rb.AddForce(-transform.up * downforce, ForceMode.Force);
    }

    // =========================
    // DEBUG: TOP SPEED
    // =========================
    private void ComputeTheoreticalTopSpeed()
    {
        if (_wheelRadius <= 0.0001f || gearRatios.Length == 0)
        {
            theoreticalTopSpeedKPH = 0f;
            return;
        }

        int topGearIndex = gearRatios.Length - 1;
        float topGearRatio = gearRatios[topGearIndex];

        if (topGearRatio <= 0f)
        {
            theoreticalTopSpeedKPH = 0f;
            return;
        }

        float speedMS = (maxEngineRPM / 60f) * (2f * Mathf.PI * _wheelRadius) / (topGearRatio * finalDriveRatio);
        theoreticalTopSpeedKPH = speedMS * 3.6f;
    }
}
