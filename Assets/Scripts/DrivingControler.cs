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
    private float currentTorque;
    private float motorTorqueMultiplier = 1f;

    public Rigidbody Rigidbody => rb;

    public float MotorTorqueMultiplier
    {
        get => motorTorqueMultiplier;
        set => motorTorqueMultiplier = Mathf.Max(0f, value);
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        GetInputs(out var steer, out var throttle, out var brake);
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
        brake = autoBrakeWhenIdle && Mathf.Approximately(throttle, 0f) ? 1f : 0f;
    }

    // ===== Steering (Front Wheels) =====
    private void ApplySteering(float steerInputValue)
    {
        float steer = steerInputValue * maxSteerAngle;

        frontLeftWheel.steerAngle = steer;
        frontRightWheel.steerAngle = steer;
    }

    // ===== Motor (Rear Wheels Only) =====
    private void ApplyMotor(float throttleInputValue)
    {
        float torque = throttleInputValue * motorTorque * motorTorqueMultiplier;
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
            brake = 1f;
        }

        rearLeftWheel.brakeTorque = brake * brakeForce;
        rearRightWheel.brakeTorque = brake * brakeForce;
    }
}
