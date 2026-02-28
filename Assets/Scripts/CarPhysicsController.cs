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
    [SerializeField] private float maxSteerAngle = 30f;
    [SerializeField] private float brakeForce = 3000f;
    [SerializeField] private bool autoBrakeWhenIdle = true;
    [SerializeField][Range(0f, 1f)] private float idleBrakeStrength = 0.15f;
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
    [SerializeField] private float highSpeedSteerStartKph = 120f;
    [Tooltip("Max steer angle at or above highSpeedSteerEndKph (kph).")]
    [SerializeField] private float highSpeedMaxSteerAngle = 12f;
    [Tooltip("Fully apply high-speed steer limit at this speed (kph).")]
    [SerializeField] private float highSpeedSteerEndKph = 200f;
    [SerializeField] private bool smoothSteering = true;
    [SerializeField] private float steerInputSpeed = 4f;
    [SerializeField] private float steerReturnSpeed = 6f;
    [SerializeField]
    private AnimationCurve steerSensitivityBySpeed = new AnimationCurve(
        new Keyframe(0f, 1f),
        new Keyframe(60f, 0.85f),
        new Keyframe(120f, 0.65f),
        new Keyframe(200f, 0.45f),
        new Keyframe(280f, 0.35f)
    );
    [SerializeField] private float minSteerMultiplier = 0.25f;
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
        brake = autoBrakeWhenIdle && Mathf.Approximately(throttle, 0f) ? idleBrakeStrength : 0f;

        float uiBrake = brakeButtonHeld ? brakeButtonStrength : 0f;
        float actionBrake = brakeActionHeld ? brakeButtonStrength : 0f;
        brake = Mathf.Max(brake, actionBrake, uiBrake);

        if (cutThrottleWhenBraking && brake > 0f)
            throttle = 0f;
    }

    private float SmoothSteerInput(float target, bool allowSmoothing)
    {
        if (!allowSmoothing || !smoothSteering || steerInputSpeed <= 0f)
        {
            currentSteerInput = target;
            return target;
        }

        float rate = Mathf.Abs(target) > Mathf.Abs(currentSteerInput) ? steerInputSpeed : steerReturnSpeed;
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
        if (speedKph > highSpeedSteerStartKph && highSpeedSteerEndKph > highSpeedSteerStartKph)
        {
            float t = Mathf.InverseLerp(highSpeedSteerStartKph, highSpeedSteerEndKph, speedKph);
            maxAngle = Mathf.Lerp(maxSteerAngle, highSpeedMaxSteerAngle, t);
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
        if (brake <= 0f && autoBrakeWhenIdle && Mathf.Approximately(throttleInputValue, 0f))
        {
            brake = idleBrakeStrength;
        }

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

        float value = steerSensitivityBySpeed.Evaluate(GetSpeedKph());
        return Mathf.Clamp(value, minSteerMultiplier, 1f);
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
