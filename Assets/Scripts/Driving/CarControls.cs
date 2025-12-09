using UnityEngine;

public class CarControls : MonoBehaviour
{
    [Header("Target Car Physics")]
    public FormulaPhysics car;

    [Header("Input Settings")]
    [Tooltip("Name of horizontal axis (Project Settings -> Input Manager).")]
    public string steerAxis = "Horizontal";

    [Tooltip("Name of vertical axis for throttle/brake.")]
    public string moveAxis = "Vertical";

    [Tooltip("Extra brake when holding Space.")]
    public KeyCode extraBrakeKey = KeyCode.Space;

    [Range(0f, 1f)]
    public float extraBrakeAmount = 0.7f;

    private void Reset()
    {
        // Auto-assign physics on same GameObject
        if (car == null)
            car = GetComponent<FormulaPhysics>();
    }

    private void Update()
    {
        if (car == null)
            return;

        // Keyboard / gamepad axes
        float steer = Input.GetAxis(steerAxis);   // A/D or left stick X
        float move = Input.GetAxis(moveAxis);     // W/S or left stick Y

        float throttle = 0f;
        float brake = 0f;

        if (move >= 0f)
        {
            throttle = move;
            brake = 0f;
        }
        else
        {
            throttle = 0f;
            brake = -move; // pressing S = brake
        }

        if (Input.GetKey(extraBrakeKey))
        {
            brake = Mathf.Clamp01(brake + extraBrakeAmount);
        }

        car.SetInputs(steer, throttle, brake);
    }
}
