using UnityEngine;

public class CarControls : MonoBehaviour
{
    [Header("Target Car Physics")]
    public SimpleCarController car;

    [Header("Input Settings")]
    public string steerAxis = "Horizontal";
    public string moveAxis = "Vertical";

    public KeyCode extraBrakeKey = KeyCode.Space;
    [Range(0f, 1f)] public float extraBrakeAmount = 0.7f;

    private Rigidbody carRb;

    private void Awake()
    {
        if (car == null)
            car = GetComponent<SimpleCarController>();

        if (car != null)
            carRb = car.Rigidbody != null ? car.Rigidbody : car.GetComponent<Rigidbody>();
    }

    private void Update()
    {
        if (car == null)
            return;

        float steer = Input.GetAxis(steerAxis);
        float move = Input.GetAxis(moveAxis); // W/S or stick Y

        float throttle = 0f;
        float brake = 0f;

        // forward speed in local Z direction (positive = going forward)
        float localZ = 0f;
        if (carRb != null)
            localZ = Vector3.Dot(carRb.linearVelocity, car.transform.forward);

        if (move > 0f)
        {
            // player wants to go forward
            if (localZ < -1f)
            {
                // we're actually rolling backwards → W should brake
                throttle = 0f;
                brake = move;
            }
            else
            {
                // stopped or already moving forward → normal throttle
                throttle = move;
                brake = 0f;
            }
        }
        else if (move < 0f)
        {
            // player wants to go backwards
            if (localZ > 1f)
            {
                // still moving forward fast → S should brake
                throttle = 0f;
                brake = -move;
            }
            else
            {
                // stopped or already going backwards → reverse throttle
                throttle = move;   // negative value
                brake = 0f;
            }
        }

        if (Input.GetKey(extraBrakeKey))
        {
            brake = Mathf.Clamp01(brake + extraBrakeAmount);
        }

        car.SetInputs(steer, throttle, brake);
    }
}
