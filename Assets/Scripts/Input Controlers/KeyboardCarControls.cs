using UnityEngine;

public class CarControls : MonoBehaviour
{
    [Header("Target Car Physics")]
    [SerializeField] private SimpleCarController car;

    [Header("Input Settings")]
    [SerializeField] private string steerAxis = "Horizontal";
    [SerializeField] private string moveAxis = "Vertical";

    [Header("Brake")]
    [SerializeField] private KeyCode brakeKey = KeyCode.Space;
    [SerializeField][Range(0f, 1f)] private float brakeAmount = 1f;

    [Header("Extra Brake (Additive)")]
    [SerializeField] private KeyCode extraBrakeKey = KeyCode.Space;
    [SerializeField][Range(0f, 1f)] private float extraBrakeAmount = 0.7f;

    private Rigidbody carRb;

    private void Awake()
    {
        // Initialization
        CacheCar();
        CacheRigidbody();
    }

    private void Update()
    {
        if (!EnsureCar())
        {
            return;
        }

        ReadInput(out var steer, out var move);
        ComputeThrottleBrake(move, out var throttle, out var brake);
        ApplyBrakeKey(ref throttle, ref brake);
        ApplyExtraBrake(ref brake);
        car.SetInputs(steer, throttle, brake);
    }

    private void ReadInput(out float steer, out float move)
    {
        steer = Input.GetAxis(steerAxis);
        move = Input.GetAxis(moveAxis);
    }

    private void ComputeThrottleBrake(float move, out float throttle, out float brake)
    {
        throttle = 0f;
        brake = 0f;

        float localZ = GetLocalForwardSpeed();

        if (move > 0f)
        {
            if (localZ < -1f)
            {
                throttle = 0f;
                brake = move;
            }
            else
            {
                throttle = move;
                brake = 0f;
            }
        }
        else if (move < 0f)
        {
            if (localZ > 1f)
            {
                throttle = 0f;
                brake = -move;
            }
            else
            {
                throttle = move;
                brake = 0f;
            }
        }
    }

    private void ApplyExtraBrake(ref float brake)
    {
        if (!Input.GetKey(extraBrakeKey))
            return;

        brake = Mathf.Clamp01(brake + extraBrakeAmount);
    }

    private void ApplyBrakeKey(ref float throttle, ref float brake)
    {
        if (!Input.GetKey(brakeKey))
            return;

        throttle = 0f;
        brake = Mathf.Max(brake, brakeAmount);
    }

    private bool EnsureCar()
    {
        if (car == null)
            CacheCar();

        if (car == null)
            return false;

        CacheRigidbody();
        return true;
    }

    private void CacheCar()
    {
        if (car == null)
            car = GetComponent<SimpleCarController>();
    }

    private void CacheRigidbody()
    {
        if (carRb != null || car == null)
            return;

        carRb = car.Rigidbody != null ? car.Rigidbody : car.GetComponent<Rigidbody>();
    }

    private float GetLocalForwardSpeed()
    {
        if (carRb == null || car == null)
            return 0f;

        return Vector3.Dot(carRb.linearVelocity, car.transform.forward);
    }
}
