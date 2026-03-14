using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleCarController : MonoBehaviour
{
    // ===== Input =====
    private Vector2 driveInput;
    private float steerInput;
    private float throttleInput;
    private float brakeInput;
    private bool useExternalInput;

    private Rigidbody rb;

    // ===== Wheel Colliders =====
    [SerializeField] private WheelCollider frontLeftWheel;
    [SerializeField] private WheelCollider frontRightWheel;
    [SerializeField] private WheelCollider rearLeftWheel;
    [SerializeField] private WheelCollider rearRightWheel;

    // ===== Car Settings =====
    [SerializeField] private float motorTorque = 1500f;
    [SerializeField] private float maxSteerAngle = 35f;
    [SerializeField] private float brakeForce = 3000f;
    [SerializeField] private bool autoBrakeWhenIdle = true;
    [SerializeField][Range(0f, 1f)] private float idleBrakeStrength = 0.15f;
    [SerializeField] private float idleBrakeStartKph = 8f;
    [SerializeField] private float idleBrakeFullKph = 45f;
    [SerializeField][Min(0.1f)] private float idleBrakeRampExponent = 1.35f;
    [Header("Brake Input")]
    [SerializeField][Range(0f, 1f)] private float brakeButtonStrength = 1f;
    [SerializeField] private bool cutThrottleWhenBraking = true;
    [Header("Top Speed")]
    [SerializeField] private bool limitTopSpeed = true;
    [SerializeField] private float maxSpeedKph = 240f;
    [SerializeField] private float speedLimiterRangeKph = 15f;
    [SerializeField] private float maxSpeedKphMultiplier = 1f;
    [SerializeField][Range(0f, 1f)] private float overspeedBrakeStrength = 0.2f;
    [SerializeField] private float overspeedBrakeRangeKph = 10f;
    private float inputSpeedMultiplier = 1f;
    [Header("High Speed Steering")]
    [Tooltip("Start reducing max steer angle at this speed (kph).")]
    [SerializeField] private float highSpeedSteerStartKph = 150f;
    [Tooltip("Max steer angle at or above highSpeedSteerEndKph (kph).")]
    [SerializeField] private float highSpeedMaxSteerAngle = 20f;
    [Tooltip("Fully apply high-speed steer limit at this speed (kph).")]
    [SerializeField] private float highSpeedSteerEndKph = 280f;
    [Header("Low Speed Steering")]
    [Tooltip("Extra steering authority near standstill.")]
    [SerializeField][Range(1f, 2f)] private float lowSpeedSteerBoost = 1.35f;
    [Tooltip("Extra steering angle near standstill.")]
    [SerializeField][Range(1f, 1.6f)] private float lowSpeedSteerAngleBoost = 1.15f;
    [Tooltip("Low-speed steering boost fades out by this speed (kph).")]
    [SerializeField] private float lowSpeedSteerBoostEndKph = 45f;
    [SerializeField] private bool smoothSteering = true;
    [Header("Steering Speed Bands")]
    [Tooltip("Band 1 / Band 2 threshold (kph). Below this uses band 1 values.")]
    [SerializeField] private float steerBandThreshold1 = 70f;
    [Tooltip("Band 2 / Band 3 threshold (kph). Above this uses band 3 values.")]
    [SerializeField] private float steerBandThreshold2 = 120f;
    [Tooltip("Input/return steer speed below " + nameof(steerBandThreshold1) + " kph.")]
    [SerializeField] private float steerInputSpeedBand1 = 8f;
    [SerializeField] private float steerReturnSpeedBand1 = 10f;
    [Tooltip("Input/return steer speed between band 1 and band 2.")]
    [SerializeField] private float steerInputSpeedBand2 = 5f;
    [SerializeField] private float steerReturnSpeedBand2 = 7f;
    [Tooltip("Input/return steer speed above " + nameof(steerBandThreshold2) + " kph.")]
    [SerializeField] private float steerInputSpeedBand3 = 2f;
    [SerializeField] private float steerReturnSpeedBand3 = 3f;
    [SerializeField]
    private AnimationCurve steerSensitivityBySpeed = new AnimationCurve(
        new Keyframe(0f, 1f),
        new Keyframe(60f, 0.85f),
        new Keyframe(120f, 0.65f),
        new Keyframe(200f, 0.45f),
        new Keyframe(280f, 0.35f)
    );
    [SerializeField] private float minSteerMultiplier = 0.5f;
    [SerializeField] private float maxSteerMultiplier = 1.2f;
    private float currentTorque;
    private float motorTorqueMultiplier = 1f;
    private float currentSteerInput;
    private bool brakeActionHeld;
    private bool brakeButtonHeld;

    public Rigidbody Rigidbody => rb;

    public float MotorTorqueMultiplier
    {
        get => motorTorqueMultiplier;
        set => motorTorqueMultiplier = Mathf.Max(0f, value);
    }

    public float MaxSpeedKphMultiplier
    {
        get => maxSpeedKphMultiplier;
        set => maxSpeedKphMultiplier = Mathf.Max(0.1f, value);
    }

    public void SetInputSpeedMultiplier(float multiplier)
    {
        inputSpeedMultiplier = Mathf.Clamp01(multiplier);
    }

    public void SetExternalInputEnabled(bool enabled)
    {
        useExternalInput = enabled;
        if (!enabled)
        {
            steerInput = 0f;
            throttleInput = 0f;
            brakeInput = 0f;
            currentSteerInput = 0f;
        }
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        GetInputs(out var steer, out var throttle, out var brake);
        steer = SmoothSteerInput(steer, true);
        ApplySteering(steer);
        ApplyMotor(throttle);
        ApplyBrakes(brake, throttle);
    }

    // ===== Input System callback =====
    private void OnDrive(InputValue value)
    {
        driveInput = value.Get<Vector2>();
        useExternalInput = false;
    }

    private void OnBrake(InputValue value)
    {
        brakeActionHeld = value.isPressed;
        useExternalInput = false;
    }

    private void OnBrakePerformed(InputValue value)
    {
        brakeActionHeld = value.isPressed;
        useExternalInput = false;
    }

    private void OnBrakeCanceled(InputValue value)
    {
        brakeActionHeld = false;
        useExternalInput = false;
    }

    public void BrakeButtonDown()
    {
        brakeButtonHeld = true;
    }

    public void BrakeButtonUp()
    {
        brakeButtonHeld = false;
    }

    public void SetInputs(float steer, float throttle, float brake)
    {
        steerInput = Mathf.Clamp(steer, -1f, 1f);
        throttleInput = Mathf.Clamp(throttle, -1f, 1f);
        brakeInput = Mathf.Clamp01(brake);
        useExternalInput = true;
    }

    private void GetInputs(out float steer, out float throttle, out float brake)
    {
        if (useExternalInput)
        {
            steer = steerInput;
            throttle = throttleInput;
            brake = brakeInput;
            return;
        }

        steer = Mathf.Clamp(driveInput.x, -1f, 1f);
        throttle = Mathf.Clamp(driveInput.y, -1f, 1f);
        brake = GetIdleBrake(throttle);

        float uiBrake = brakeButtonHeld ? brakeButtonStrength : 0f;
        float actionBrake = brakeActionHeld ? brakeButtonStrength : 0f;
        brake = Mathf.Max(brake, actionBrake, uiBrake);

        if (cutThrottleWhenBraking && brake > 0f)
            throttle = 0f;
    }

    private void GetSteeringRates(out float inputSpeed, out float returnSpeed)
    {
        float kph = GetSpeedKph();

        if (kph <= steerBandThreshold1)
        {
            inputSpeed = steerInputSpeedBand1;
            returnSpeed = steerReturnSpeedBand1;
        }
        else if (kph <= steerBandThreshold2)
        {
            float t = Mathf.InverseLerp(steerBandThreshold1, steerBandThreshold2, kph);
            inputSpeed = Mathf.Lerp(steerInputSpeedBand1, steerInputSpeedBand2, t);
            returnSpeed = Mathf.Lerp(steerReturnSpeedBand1, steerReturnSpeedBand2, t);
        }
        else
        {
            float t = Mathf.InverseLerp(steerBandThreshold2, steerBandThreshold2 + 80f, kph);
            inputSpeed = Mathf.Lerp(steerInputSpeedBand2, steerInputSpeedBand3, t);
            returnSpeed = Mathf.Lerp(steerReturnSpeedBand2, steerReturnSpeedBand3, t);
        }
    }

    private float SmoothSteerInput(float target, bool allowSmoothing)
    {
        GetSteeringRates(out float inputSpeed, out float returnSpeed);

        if (!allowSmoothing || !smoothSteering || inputSpeed <= 0f)
        {
            currentSteerInput = target;
            return target;
        }

        float rate = Mathf.Abs(target) > Mathf.Abs(currentSteerInput) ? inputSpeed : returnSpeed;
        currentSteerInput = Mathf.MoveTowards(currentSteerInput, target, rate * Time.fixedDeltaTime);
        return currentSteerInput;
    }

    // ===== Steering (Front Wheels) =====
    private void ApplySteering(float steerInputValue)
    {
        float sensitivity = SettingsManager.GetSteeringSensitivityMultiplier();
        float adjustedInput = Mathf.Clamp(steerInputValue * sensitivity, -1f, 1f);
        float steerMultiplier = GetSteerMultiplier();
        float speedKph = GetSpeedKph();
        float maxAngle = maxSteerAngle;
        // Reducing the angle at higher speeds
        if (speedKph > highSpeedSteerStartKph && highSpeedSteerEndKph > highSpeedSteerStartKph)
        {
            float t = Mathf.InverseLerp(highSpeedSteerStartKph, highSpeedSteerEndKph, speedKph);
            maxAngle = Mathf.Lerp(maxSteerAngle, highSpeedMaxSteerAngle, t);
        }

        if (lowSpeedSteerAngleBoost > 1f && lowSpeedSteerBoostEndKph > 0f)
        {
            float t = 1f - Mathf.Clamp01(speedKph / lowSpeedSteerBoostEndKph);
            maxAngle *= Mathf.Lerp(1f, lowSpeedSteerAngleBoost, t);
        }

        float steer = adjustedInput * maxAngle * steerMultiplier;

        frontLeftWheel.steerAngle = steer;
        frontRightWheel.steerAngle = steer;
    }

    // ===== Motor (Rear Wheels Only) =====
    private void ApplyMotor(float throttleInputValue)
    {
        float limiter = GetSpeedLimiterFactor(throttleInputValue);
        float torque = throttleInputValue * motorTorque * motorTorqueMultiplier * limiter;
        currentTorque = Mathf.Lerp(currentTorque, torque, Time.fixedDeltaTime * 5f);

        rearLeftWheel.motorTorque = currentTorque;
        rearRightWheel.motorTorque = currentTorque;
    }

    // ===== Simple Brake =====
    private void ApplyBrakes(float brakeInputValue, float throttleInputValue)
    {
        var brake = Mathf.Clamp01(brakeInputValue);
        brake = Mathf.Max(brake, GetIdleBrake(throttleInputValue));

        if (limitTopSpeed && rb != null && maxSpeedKph > 0f)
        {
            float speedKph = GetSpeedKph();
            float effectiveMaxSpeed = GetEffectiveMaxSpeedKph();
            if (speedKph > effectiveMaxSpeed)
            {
                float range = Mathf.Max(0.1f, overspeedBrakeRangeKph);
                float t = Mathf.InverseLerp(effectiveMaxSpeed, effectiveMaxSpeed + range, speedKph);
                float overspeedBrake = Mathf.Clamp01(t) * Mathf.Clamp01(overspeedBrakeStrength);
                brake = Mathf.Max(brake, overspeedBrake);
            }
        }

        rearLeftWheel.brakeTorque = brake * brakeForce;
        rearRightWheel.brakeTorque = brake * brakeForce;
        frontLeftWheel.brakeTorque = brake * brakeForce;
        frontRightWheel.brakeTorque = brake * brakeForce;
    }

    private float GetSteerMultiplier()
    {
        if (rb == null || steerSensitivityBySpeed == null)
            return 1f;

        float speedKph = GetSpeedKph();
        float minMultiplier = Mathf.Clamp(minSteerMultiplier, 0.1f, 2f);
        float maxMultiplier = Mathf.Max(minMultiplier, maxSteerMultiplier);
        float baseMultiplier = Mathf.Clamp(steerSensitivityBySpeed.Evaluate(speedKph), minMultiplier, maxMultiplier);

        float boost = 1f;
        if (lowSpeedSteerBoost > 1f && lowSpeedSteerBoostEndKph > 0f)
        {
            float t = 1f - Mathf.Clamp01(speedKph / lowSpeedSteerBoostEndKph);
            boost = Mathf.Lerp(1f, lowSpeedSteerBoost, t);
        }

        return Mathf.Clamp(baseMultiplier * boost, minMultiplier, maxMultiplier);
    }

    private float GetIdleBrake(float throttleInputValue)
    {
        if (!autoBrakeWhenIdle || !Mathf.Approximately(throttleInputValue, 0f))
            return 0f;

        float speedKph = GetSpeedKph();
        if (speedKph <= idleBrakeStartKph)
            return 0f;

        float fullKph = Mathf.Max(idleBrakeStartKph + 0.1f, idleBrakeFullKph);
        float t = Mathf.InverseLerp(idleBrakeStartKph, fullKph, speedKph);
        float ramp = Mathf.Pow(Mathf.Clamp01(t), Mathf.Max(0.1f, idleBrakeRampExponent));
        return Mathf.Clamp01(idleBrakeStrength) * ramp;
    }

    private float GetSpeedKph()
    {
        return rb != null ? rb.linearVelocity.magnitude * 3.6f : 0f;
    }

    private float GetSpeedLimiterFactor(float throttleInputValue)
    {
        if (!limitTopSpeed || rb == null || maxSpeedKph <= 0f)
            return 1f;

        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        if (Mathf.Approximately(throttleInputValue, 0f))
            return 1f;

        // Only limit when accelerating in the same direction as travel.
        if (Mathf.Sign(throttleInputValue) != Mathf.Sign(forwardSpeed))
            return 1f;

        float speedKph = Mathf.Abs(forwardSpeed) * 3.6f;
        float effectiveMaxSpeed = GetEffectiveMaxSpeedKph();
        if (speedLimiterRangeKph <= 0f)
            return speedKph >= effectiveMaxSpeed ? 0f : 1f;

        float startKph = Mathf.Max(0f, effectiveMaxSpeed - speedLimiterRangeKph);
        float t = Mathf.InverseLerp(startKph, effectiveMaxSpeed, speedKph);
        return 1f - Mathf.Clamp01(t);
    }

    private float GetEffectiveMaxSpeedKph()
    {
        return maxSpeedKph * maxSpeedKphMultiplier * Mathf.Clamp01(inputSpeedMultiplier);
    }
}
