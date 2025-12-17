using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleCarController : MonoBehaviour
{
    // ===== Input =====
    private Vector2 driveInput;

    // ===== Wheel Colliders =====
    [SerializeField] private WheelCollider frontLeftWheel;
    [SerializeField] private WheelCollider frontRightWheel;
    [SerializeField] private WheelCollider rearLeftWheel;
    [SerializeField] private WheelCollider rearRightWheel;

    // ===== Car Settings =====
    [SerializeField] private float motorTorque = 1500f;
    [SerializeField] private float maxSteerAngle = 30f;
    [SerializeField] private float brakeForce = 3000f;
    private float currentTorque;

    private void FixedUpdate()
    {
        ApplySteering();
        ApplyMotor();
        ApplyBrakes();
    }

    // ===== Input System callback =====
    private void OnDrive(InputValue value)
    {
        driveInput = value.Get<Vector2>();
    }

    // ===== Steering (Front Wheels) =====
    private void ApplySteering()
    {
        float steer = driveInput.x * maxSteerAngle;

        frontLeftWheel.steerAngle = steer;
        frontRightWheel.steerAngle = steer;
    }

    // ===== Motor (Rear Wheels Only) =====
    private void ApplyMotor()
    {
        float torque = driveInput.y * motorTorque;
        currentTorque = Mathf.Lerp(currentTorque, torque, Time.fixedDeltaTime * 5f);

        rearLeftWheel.motorTorque = currentTorque;
        rearRightWheel.motorTorque = currentTorque;
    }

    // ===== Simple Brake =====
    private void ApplyBrakes()
    {
        if (driveInput.y == 0f)
        {
            rearLeftWheel.brakeTorque = brakeForce;
            rearRightWheel.brakeTorque = brakeForce;
        }
        else
        {
            rearLeftWheel.brakeTorque = 0f;
            rearRightWheel.brakeTorque = 0f;
        }
    }
}
