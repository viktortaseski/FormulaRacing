using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;

public class MobileCarControls : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private SimpleCarController car;
    [SerializeField] private Slider throttleSlider;
    [SerializeField] private Button brakeButton;

    [Header("Throttle")]
    [Tooltip("Extra multiplier for the slider value (0...1).")]
    [SerializeField][Range(0.1f, 2f)] private float throttleSensitivity = 1f;

    [Header("Brake")]
    [Tooltip("How strong the brake is when button is held (0..1).")]
    [SerializeField][Range(0f, 1f)] private float brakeStrength = 1f;

    [Header("Tilt Steering")]
    [Tooltip("Tilt (in g) that corresponds to full steering. Typical 0.3–0.5.")]
    [SerializeField] private float tiltMaxG = 0.35f;
    [Tooltip("Ignore small tilt under this value (in g).")]
    [SerializeField] private float tiltDeadzone = 0.05f;
    [Tooltip("Exponent > 1 softens small tilts; 1 = linear.")]
    [SerializeField][Range(1f, 3f)] private float tiltExponent = 1.6f;
    [Tooltip("Global multiplier on tilt output (lower = less sensitive).")]
    [SerializeField][Range(0.1f, 1f)] private float tiltOutputMultiplier = 0.6f;
    [Tooltip("Hard cap on steering output from tilt (0..1).")]
    [SerializeField][Range(0.1f, 1f)] private float maxSteerOutput = 0.85f;
    [Tooltip("How smoothly steering follows tilt.")]
    [SerializeField] private float steerSmoothing = 5f;

    private bool brakeHeld = false;
    private float currentSteer = 0f;
    private Rigidbody carRb;


    private void Reset()
    {
        CacheCar();
    }

    private void Awake()
    {
        // Initialization
        CacheCar();
        CacheRigidbody();
        HookBrakeButton();
    }

    private void Update()
    {
        if (!EnsureCar())
            return;

        float move = ReadMoveInput();
        ComputeThrottleBrake(move, out var throttle, out var brake);
        float steer = UpdateSteer();
        car.SetInputs(steer, throttle, brake);
    }

    // ----------------------------
    // Tilt steering using Accelerometer
    // ----------------------------
    private float ReadTiltSteer()
    {
        if (Accelerometer.current == null)
        {
            // No accelerometer available (e.g. in editor) -> no steering
            return 0f;
        }

        // Read acceleration in g's (-1..1). Depending on landscape orientation you
        // might need to swap X/Y or invert the sign.
        Vector3 acc = Accelerometer.current.acceleration.ReadValue();

        // Assume phone in LANDSCAPE with the top of the phone to the left:
        // tilting the top of the phone DOWN should steer right.
        float raw = acc.y; // try x first; if steering feels wrong, swap to acc.y or negate

        // Deadzone
        if (Mathf.Abs(raw) < tiltDeadzone)
            return 0f;

        // Normalize by maxG so that when |accel| = tiltMaxG, steer = ±1
        float normalized = Mathf.Clamp(raw / tiltMaxG, -1f, 1f);
        // Apply exponential to calm center feel, then cap so tiny tilts don't give full lock
        float curved = Mathf.Sign(normalized) * Mathf.Pow(Mathf.Abs(normalized), tiltExponent);
        float limited = Mathf.Clamp(curved, -maxSteerOutput, maxSteerOutput);

        return limited * tiltOutputMultiplier;
    }

    // ----------------------------
    // Brake button helpers
    // ----------------------------
    public void BrakeButtonDown()
    {
        brakeHeld = true;
    }

    public void BrakeButtonUp()
    {
        brakeHeld = false;
    }

    private void LateUpdate()
    {
        MaintainReferences();
    }

    // short tap on brake button = quick brake pulse (Implement later?)
    private void OnBrakeTap()
    {

    }

    private float ReadMoveInput()
    {
        float move = 0f;
        if (throttleSlider != null)
            move = Mathf.Clamp01(throttleSlider.value * throttleSensitivity);

        if (brakeHeld)
            move = -Mathf.Clamp01(brakeStrength);

        return move;
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

    private float UpdateSteer()
    {
        float targetSteer = ReadTiltSteer();
        currentSteer = Mathf.Lerp(currentSteer, targetSteer, steerSmoothing * Time.deltaTime);
        return currentSteer;
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

    private void HookBrakeButton()
    {
        if (brakeButton == null)
            return;

        brakeButton.onClick.AddListener(OnBrakeTap);
    }

    private void MaintainReferences()
    {
        if (carRb == null && car != null)
            carRb = car.Rigidbody != null ? car.Rigidbody : car.GetComponent<Rigidbody>();
    }

    private float GetLocalForwardSpeed()
    {
        if (carRb == null || car == null)
            return 0f;

        return Vector3.Dot(carRb.linearVelocity, car.transform.forward);
    }
}
