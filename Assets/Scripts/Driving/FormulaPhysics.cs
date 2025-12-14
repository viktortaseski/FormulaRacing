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

    [Tooltip("Final drive ratio (diff). Higher = shorter gearing (more revs per speed).")]
    public float finalDriveRatio = 9.0f;

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
    public float maxWheelTorque = 2500f;


    [Header("Steering")]
    [Tooltip("Max steering angle in degrees at low speed.")]
    public float maxSteerAngle = 30f;
    [Tooltip("Max steering angle allowed at high speed to avoid scrubbing all speed.")]
    public float highSpeedSteerAngle = 6f;
    [Tooltip("Speed (KPH) where we start fading steering lock.")]
    public float steerFadeStartKPH = 160f;
    [Tooltip("Speed (KPH) where steering lock reaches high-speed value.")]
    public float steerFadeEndKPH = 340f;
    [Tooltip("How quickly wheels follow steering input.")]
    public float steerResponse = 4f;

    [Header("Brakes")]
    [Tooltip("Max brake torque (Nm) per wheel.")]
    public float maxBrakeTorque = 9000f;


    [Header("Tyre Friction Tuning")]
    public float forwardStiffness = 1.3f;
    public float sidewaysStiffness = 2.2f;
    [Tooltip("Sideways stiffness multiplier when at very high speed (reduces grip so you slide instead of bogging down).")]
    [Range(0.1f, 1f)] public float highSpeedSidewaysMultiplier = 0.7f;
    [Tooltip("Sideways stiffness multiplier when at full steering input (adds progressive slip).")]
    [Range(0.1f, 1f)] public float fullSteerSidewaysMultiplier = 0.85f;
    [Tooltip("Speed (KPH) where lateral grip starts to bleed off.")]
    public float gripFadeStartKPH = 120f;
    [Tooltip("Speed (KPH) where lateral grip hits the high-speed multiplier.")]
    public float gripFadeEndKPH = 280f;

    [Header("Aero")]
    [Tooltip("Downforce coefficient. Force ~ k * v^2.")]
    public float downforceCoefficient = 45f;

    [Header("Inputs (from controls scripts)")]
    [Range(-1f, 1f)] public float steeringInput;  // -1..1
    [Range(-1f, 1f)] public float throttleInput;  // 0..1
    [Range(0f, 1f)] public float brakeInput;     // 0..1

    [Header("Multipliers (for DRS, upgrades, etc.)")]
    public float engineTorqueMultiplier = 1f;
    public float downforceMultiplier = 1f;


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
        steer = Mathf.Abs(steer) < 0.05f ? 0f : steer;
        brake = brake < 0.02f ? 0f : brake;

        steeringInput = Mathf.Clamp(steer, -1f, 1f);
        throttleInput = Mathf.Clamp(throttle, -1f, 1f);
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

        UpdateDynamicFriction();
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
    private void ConfigureWheelFriction(WheelCollider wc, float sidewaysOverride = -1f)
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
        float sideStiff = sidewaysOverride > 0f ? sidewaysOverride : sidewaysStiffness;
        s.stiffness = Mathf.Max(0.01f, sideStiff);
        wc.sidewaysFriction = s;
    }

    private void UpdateDynamicFriction()
    {
        // Less aggressive grip loss so car is stable
        float speedGripT = Mathf.InverseLerp(gripFadeStartKPH, gripFadeEndKPH, speedKPH);
        float mulSpeed = Mathf.Lerp(1f, highSpeedSidewaysMultiplier, speedGripT);
        float mulSteer = Mathf.Lerp(1f, fullSteerSidewaysMultiplier, Mathf.Abs(steeringInput));

        float sideways = sidewaysStiffness * mulSpeed * mulSteer;

        // same friction front & rear (we'll tweak later if needed)
        ConfigureWheelFriction(frontLeft, sideways);
        ConfigureWheelFriction(frontRight, sideways);
        ConfigureWheelFriction(rearLeft, sideways);
        ConfigureWheelFriction(rearRight, sideways);
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

        // no throttle = no drive
        if (Mathf.Approximately(throttleInput, 0f))
        {
            rearLeft.motorTorque = 0f;
            rearRight.motorTorque = 0f;
            return;
        }

        float gearRatio = gearRatios[Mathf.Clamp(currentGear, 0, gearRatios.Length - 1)];

        float baseTorque = engineTorqueCurve.Evaluate(engineRPM);
        float engineTorque = baseTorque * engineTorqueMultiplier * Mathf.Abs(throttleInput);

        float direction = Mathf.Sign(throttleInput);              // +1 forward, -1 reverse

        float driveTorque = engineTorque * gearRatio * finalDriveRatio * direction;
        driveTorque = Mathf.Clamp(driveTorque, -maxWheelTorque * 2f, maxWheelTorque * 2f);

        float wheelTorque = driveTorque * 0.5f;   // two driven wheels

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

        float steerFadeT = Mathf.InverseLerp(steerFadeStartKPH, steerFadeEndKPH, speedKPH);
        float maxSteer = Mathf.Lerp(maxSteerAngle, highSpeedSteerAngle, steerFadeT);
        maxSteer = Mathf.Max(0f, maxSteer);

        float targetSteer = steeringInput * maxSteer;

        _smoothSteerL = Mathf.Lerp(_smoothSteerL, targetSteer, steerResponse * Time.fixedDeltaTime);
        _smoothSteerR = Mathf.Lerp(_smoothSteerR, targetSteer, steerResponse * Time.fixedDeltaTime);

        frontLeft.steerAngle = _smoothSteerL;
        frontRight.steerAngle = _smoothSteerR;
    }

    private void ApplyDownforce()
    {
        float speed = rb.linearVelocity.magnitude;
        float downforce = downforceCoefficient * downforceMultiplier * speed * speed;
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
