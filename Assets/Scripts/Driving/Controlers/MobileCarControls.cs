using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;

public class MobileCarControls : MonoBehaviour
{
    [Header("References")]
    public FormulaPhysics car;
    public Slider throttleSlider;
    public Button brakeButton;

    [Header("Throttle")]
    [Tooltip("Extra multiplier for the slider value (0...1).")]
    [Range(0.1f, 2f)] public float throttleSensitivity = 1f;

    [Header("Brake")]
    [Tooltip("How strong the brake is when button is held (0..1).")]
    [Range(0f, 1f)] public float brakeStrength = 1f;

    [Header("Tilt Steering")]
    [Tooltip("Tilt (in g) that corresponds to full steering. Typical 0.3–0.5.")]
    public float tiltMaxG = 0.35f;
    [Tooltip("Ignore small tilt under this value (in g).")]
    public float tiltDeadzone = 0.05f;
    [Tooltip("Exponent > 1 softens small tilts; 1 = linear.")]
    [Range(1f, 3f)] public float tiltExponent = 1.6f;
    [Tooltip("Global multiplier on tilt output (lower = less sensitive).")]
    [Range(0.1f, 1f)] public float tiltOutputMultiplier = 0.6f;
    [Tooltip("Hard cap on steering output from tilt (0..1).")]
    [Range(0.1f, 1f)] public float maxSteerOutput = 0.85f;
    [Tooltip("How smoothly steering follows tilt.")]
    public float steerSmoothing = 5f;

    private bool brakeHeld = false;
    private float currentSteer = 0f;
    private Rigidbody carRb;


    private void Reset()
    {
        if (car == null)
            car = GetComponent<FormulaPhysics>();
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created

    private void Awake()
    {
        if (car == null)
            car = GetComponent<FormulaPhysics>();

        if (car != null)
            carRb = car.rb;

        // Hook brake button events (we'll also show how to do via EventTrigger)
        if (brakeButton != null)
        {
            // This onClick is optional; real hold behaviour will use the
            // BrakeButtonDown/Up methods via EventTrigger.
            brakeButton.onClick.AddListener(OnBrakeTap);
        }
    }

    private void Update()
    {
        if (car == null)
            return;

        // ----- THROTTLE -----
        float throttle = 0f;
        float brake = 0f;

        float move = 0f; // like Vertical axis in CarControls
        if (throttleSlider != null)
            move = Mathf.Clamp01(throttleSlider.value * throttleSensitivity);

        // ----- BRAKE -----
        if (brakeHeld)
            move = -Mathf.Clamp01(brakeStrength);

        // Determine if brake button should brake or reverse (mirrors CarControls logic)
        float localZ = 0f;
        if (carRb != null)
            localZ = Vector3.Dot(carRb.linearVelocity, car.transform.forward);

        if (move > 0f)
        {
            // forward input
            if (localZ < -1f)
            {
                throttle = 0f;
                brake = move;   // pressing forward while rolling backward = brake
            }
            else
            {
                throttle = move;
                brake = 0f;
            }
        }
        else if (move < 0f)
        {
            // reverse input (brake button)
            if (localZ > 1f)
            {
                throttle = 0f;
                brake = -move; // braking if still going forward
            }
            else
            {
                throttle = move; // negative = reverse
                brake = 0f;
            }
        }

        // ----- STEERING FROM TILT -----
        float targetSteer = ReadTiltSteer();
        currentSteer = Mathf.Lerp(currentSteer, targetSteer, steerSmoothing * Time.deltaTime);

        // Send inputs to physics
        car.SetInputs(currentSteer, throttle, brake);
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
    // These are meant to be called from UI EventTriggers
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
        // Keep rb ref alive if set later
        if (carRb == null && car != null)
            carRb = car.rb;
    }

    // Optional: short tap on brake button = quick brake pulse
    private void OnBrakeTap()
    {

    }
}
