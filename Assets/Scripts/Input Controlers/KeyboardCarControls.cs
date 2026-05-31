using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Reads keyboard (and gamepad) input via the PlayerInput component and feeds
/// it to CarPhysicsController. Must live on the SAME GameObject as the car's
/// PlayerInput component, whose Behavior is "Send Messages", so the OnDrive /
/// OnBrake callbacks below are invoked.
///
/// Enabled/disabled at runtime by InputSourceController when the player
/// switches between Keyboard and Mobile in Settings.
/// </summary>
[RequireComponent(typeof(CarPhysicsController))]
public class KeyboardCarControls : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private CarPhysicsController car;

    [Header("Brake")]
    [Tooltip("How strong the brake action is when held (0..1).")]
    [SerializeField][Range(0f, 1f)] private float brakeStrength = 1f;

    private Vector2 driveInput;
    private bool brakeHeld;

    private void Reset() => CacheCar();
    private void Awake() => CacheCar();

    private void OnDisable()
    {
        // Release the car so it doesn't hold the last keyboard input
        // after switching to mobile controls.
        driveInput = Vector2.zero;
        brakeHeld = false;
        if (car != null)
            car.SetInputs(0f, 0f, 0f);
    }

    private void Update()
    {
        if (car == null)
        {
            CacheCar();
            if (car == null) return;
        }

        // Hold the car still during the start countdown.
        if (StartSemaphore.ControlsLocked)
        {
            car.SetInputs(0f, 0f, 0f);
            return;
        }

        float steer = Mathf.Clamp(driveInput.x, -1f, 1f);
        float throttle = Mathf.Clamp(driveInput.y, -1f, 1f);
        float brake = brakeHeld ? brakeStrength : 0f;

        car.SetInputs(steer, throttle, brake);
    }

    // ----------------------------
    // PlayerInput "Send Messages" callbacks
    // ----------------------------
    private void OnDrive(InputValue value) => driveInput = value.Get<Vector2>();
    private void OnBrake(InputValue value) => brakeHeld = value.isPressed;

    private void CacheCar()
    {
        if (car == null)
            car = GetComponent<CarPhysicsController>();
    }
}
