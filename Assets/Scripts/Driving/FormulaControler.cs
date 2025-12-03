using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.EventSystems;

public class FormulaControler : MonoBehaviour
{
    public enum Axle { Front, Rear }

    // ======================================
    // UI CONTROLS
    // ======================================
    [Header("UI Controls")]
    public UnityEngine.UI.Button brakeButton;
    public UnityEngine.UI.Slider throttleSlider;

    private bool brakePressed = false;
    private bool sliderHeld = false;

    private float targetSpeed = 0f;
    public float maxSpeedKPH = 360f;
    public float speedHoldAssist = 10f;
    public float reverseEntryKPH = 10f;
    public float brakeSliderDecayPerSecond = 2f;

    // ======================================
    // WHEELS
    // ======================================
    [System.Serializable]
    public class Wheel
    {
        public string wheelName;
        public Axle axle;
        public WheelCollider collider;
        public Transform visual;
        [HideInInspector] public Quaternion initialVisualRotation;
    }

    [Header("Wheels")]
    public Wheel[] wheels;

    // ======================================
    // CAR SETTINGS
    // ======================================
    [Header("Car settings")]
    public float maxMotorTorque = 2000f;
    public float brakeTorque = 4000f;

    [Header("Steering")]
    public float maxSteerAngle = 30f;
    public float highSpeedSteerAngle = 8f;
    public float steerAngleRate = 50f;   // degrees per second

    [Header("Stability")]
    public float downforceMultiplier = 50f;
    public float antiRollStrength = 6000f;
    public float suspensionDistanceFactor = 0.35f;
    public float suspensionSpringMultiplier = 1.1f;
    public float suspensionDamperMultiplier = 2.2f;

    // ======================================
    // TILT / ACCELEROMETER INPUT
    // ======================================
    [Header("Tilt Steering")]
    public float tiltSteerSensitivity = 1.5f;
    public float tiltDeadzone = 0.05f;

    private InputAction tiltAction;

    // ======================================
    // INPUT SYSTEM
    // ======================================
    [Header("Input")]
    public InputActionAsset inputActions;

    private Rigidbody rb;
    private InputActionMap playerMap;
    private InputAction moveAction;
    private Vector2 moveInput;
    private float maxSpeedMS;
    private bool brakeEventsBound = false;
    private float currentSteerAngle = 0f;

    public Vector3 _centerOfMass;

    // ======================================
    // AWAKE
    // ======================================
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = _centerOfMass;

        maxSpeedMS = maxSpeedKPH / 3.6f;
        ConfigureSuspension();
        CacheInitialWheelRotations();

        if (inputActions != null)
        {
            playerMap = inputActions.FindActionMap("Player", true);
            moveAction = playerMap.FindAction("Move", true);
            tiltAction = playerMap.FindAction("Tilt", false);   // optional; bind in asset
        }

        EnableTiltDevices();
    }

    // ======================================
    // ON ENABLE
    // ======================================
    private void OnEnable()
    {
        if (throttleSlider != null)
            throttleSlider.onValueChanged.AddListener(OnThrottleChanged);

        BindBrakeButtonHold();

        if (playerMap == null || moveAction == null) return;

        playerMap.Enable();
        moveAction.performed += OnMove;
        moveAction.canceled += OnMove;

        if (tiltAction != null)
            tiltAction.Enable();   // << ENABLE TILT INPUT
    }

    private void OnDisable()
    {
        if (throttleSlider != null)
            throttleSlider.onValueChanged.RemoveListener(OnThrottleChanged);

        BrakeUp();

        if (playerMap == null || moveAction == null) return;

        moveAction.performed -= OnMove;
        moveAction.canceled -= OnMove;
        playerMap.Disable();

        if (tiltAction != null)
            tiltAction.Disable();
    }

    private void OnMove(InputAction.CallbackContext ctx)
    {
        moveInput = ctx.ReadValue<Vector2>();
    }

    // BRAKE BUTTON EVENTS
    private void BrakeDown() => brakePressed = true;
    private void BrakeUp() => brakePressed = false;

    // THROTTLE SLIDER
    private void OnThrottleChanged(float value)
    {
        sliderHeld = true;
        targetSpeed = value * maxSpeedMS;
    }

    public void BeginSliderDrag() => sliderHeld = true;
    public void EndSliderDrag() => sliderHeld = false;


    // ======================================
    // UPDATE → SLIDER DECAY
    // ======================================
    private void Update()
    {
        HandleSliderDecay();
    }

    // ======================================
    // FIXED UPDATE → CAR PHYSICS
    // ======================================
    private void FixedUpdate()
    {
        // ----------------------------------
        // READ TILT INPUT
        // ----------------------------------
        float tiltSteer = ReadTilt();

        // Combine tilt with Move input
        float steerInput = moveInput.x + tiltSteer;
        steerInput = Mathf.Clamp(steerInput, -1f, 1f);

        // ----------------------------------
        // CAR FORCE LOGIC
        // ----------------------------------
        float currentSpeed = rb.linearVelocity.magnitude;
        float throttleTorque = 0f;
        float brakeInput = 0f;
        float reverseEntryMS = reverseEntryKPH / 3.6f;

        float allowedSteerAngle = GetSpeedAdjustedSteerAngle(currentSpeed);
        float targetSteerAngle = steerInput * allowedSteerAngle;
        float maxSteerStep = steerAngleRate * Time.fixedDeltaTime;
        currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, targetSteerAngle, maxSteerStep);

        if (brakePressed)
        {
            if (currentSpeed > reverseEntryMS)
            {
                throttleTorque = 0f;
                brakeInput = 1f;
            }
            else
            {
                throttleTorque = -1f;
                brakeInput = 0f;
            }
        }
        else
        {
            if (currentSpeed < targetSpeed - 0.5f)
            {
                throttleTorque = 1f;
            }
            else if (currentSpeed > targetSpeed + 0.5f)
            {
                brakeInput = Mathf.Clamp((currentSpeed - targetSpeed) / speedHoldAssist, 0f, 1f);
            }
        }

        // DOWNFORCE
        rb.AddForce(-transform.up * currentSpeed * downforceMultiplier);

        // ANTI-ROLL
        ApplyAntiRollBars();

        // APPLY TO WHEELS
        foreach (var w in wheels)
        {
            if (w.collider == null) continue;

            if (w.axle == Axle.Front)
                w.collider.steerAngle = currentSteerAngle;
            else
                w.collider.steerAngle = 0f;   // lock rear steering

            w.collider.motorTorque = throttleTorque * maxMotorTorque;
            w.collider.brakeTorque = brakeInput * brakeTorque;

            UpdateWheelVisual(w);
        }
    }

    // ======================================
    // UPDATE WHEEL VISUALS
    // ======================================
    private void UpdateWheelVisual(Wheel w)
    {
        if (w.visual == null || w.collider == null) return;
        w.collider.GetWorldPose(out Vector3 pos, out Quaternion rot);
        w.visual.position = pos;

        if (w.axle == Axle.Rear)
        {
            Vector3 euler = rot.eulerAngles;
            euler.y = w.initialVisualRotation.eulerAngles.y;
            w.visual.rotation = Quaternion.Euler(euler);
        }
        else
        {
            w.visual.rotation = rot;
        }
    }

    // ======================================
    // ANTI-ROLL SYSTEM
    // ======================================
    private void ApplyAntiRollBars()
    {
        if (wheels.Length < 4) return;

        ApplyAntiRoll(wheels[0].collider, wheels[1].collider);
        ApplyAntiRoll(wheels[2].collider, wheels[3].collider);
    }

    private void ApplyAntiRoll(WheelCollider left, WheelCollider right)
    {
        WheelHit hit;
        float travelL = 1f;
        float travelR = 1f;

        bool groundedL = left.GetGroundHit(out hit);
        if (groundedL)
            travelL = (-left.transform.InverseTransformPoint(hit.point).y - left.radius) / left.suspensionDistance;

        bool groundedR = right.GetGroundHit(out hit);
        if (groundedR)
            travelR = (-right.transform.InverseTransformPoint(hit.point).y - right.radius) / right.suspensionDistance;

        float antiRollForce = (travelL - travelR) * antiRollStrength;

        if (groundedL)
            rb.AddForceAtPosition(left.transform.up * -antiRollForce, left.transform.position);
        if (groundedR)
            rb.AddForceAtPosition(right.transform.up * antiRollForce, right.transform.position);
    }

    // ======================================
    // HELPERS
    // ======================================
    private void BindBrakeButtonHold()
    {
        if (brakeButton == null || brakeEventsBound) return;

        var trigger = brakeButton.GetComponent<EventTrigger>();
        if (trigger == null)
            trigger = brakeButton.gameObject.AddComponent<EventTrigger>();

        if (trigger.triggers == null)
            trigger.triggers = new System.Collections.Generic.List<EventTrigger.Entry>();

        AddBrakeTrigger(trigger, EventTriggerType.PointerDown, BrakeDown);
        AddBrakeTrigger(trigger, EventTriggerType.PointerUp, BrakeUp);
        AddBrakeTrigger(trigger, EventTriggerType.PointerExit, BrakeUp);

        brakeEventsBound = true;
    }

    private void AddBrakeTrigger(EventTrigger trigger, EventTriggerType type, System.Action action)
    {
        var entry = new EventTrigger.Entry { eventID = type };
        entry.callback.AddListener(_ => action());
        trigger.triggers.Add(entry);
    }

    private void HandleSliderDecay()
    {
        if (throttleSlider == null) return;

        if (brakePressed)
        {
            float newValue = Mathf.MoveTowards(throttleSlider.value, 0f, brakeSliderDecayPerSecond * Time.deltaTime);
            throttleSlider.SetValueWithoutNotify(newValue);
            targetSpeed = newValue * maxSpeedMS;
            return;
        }

        if (!sliderHeld)
        {
            float newValue = Mathf.Lerp(throttleSlider.value, 0f, Time.deltaTime * 4f);
            throttleSlider.SetValueWithoutNotify(newValue);
            targetSpeed = newValue * maxSpeedMS;
        }
    }

    private void ConfigureSuspension()
    {
        if (wheels == null) return;

        foreach (var w in wheels)
        {
            if (w.collider == null) continue;

            w.collider.suspensionDistance *= Mathf.Max(0.05f, suspensionDistanceFactor);

            var spring = w.collider.suspensionSpring;
            spring.spring *= suspensionSpringMultiplier;
            spring.damper *= suspensionDamperMultiplier;
            w.collider.suspensionSpring = spring;
        }
    }

    private void CacheInitialWheelRotations()
    {
        if (wheels == null) return;

        foreach (var w in wheels)
        {
            if (w.visual != null)
                w.initialVisualRotation = w.visual.rotation;
        }
    }

    private void EnableTiltDevices()
    {
        // Ensure mobile sensors are on so the Tilt action receives values
        if (Accelerometer.current != null && !Accelerometer.current.enabled)
            InputSystem.EnableDevice(Accelerometer.current);

        if (AttitudeSensor.current != null && !AttitudeSensor.current.enabled)
            InputSystem.EnableDevice(AttitudeSensor.current);
    }

    private float ReadTilt()
    {
        Vector3 tilt = Vector3.zero;
        bool hasTilt = false;

        // Preferred path: use the bound Tilt action
        if (tiltAction != null && tiltAction.enabled)
        {
            tilt = tiltAction.ReadValue<Vector3>();
            hasTilt = tilt != Vector3.zero || tiltAction.activeControl != null;
        }

        // Fallback: read raw accelerometer
        if (!hasTilt && Accelerometer.current != null)
        {
            tilt = Accelerometer.current.acceleration.ReadValue();
            hasTilt = true;
        }

        // Only use X (roll) for steering; ignore Y/Z completely
        float steer = tilt.x;
        if (Mathf.Abs(steer) < tiltDeadzone)
            steer = 0f;

        return steer * tiltSteerSensitivity;
    }

    private float GetSpeedAdjustedSteerAngle(float speed)
    {
        float speedFactor = Mathf.Clamp01(speed / Mathf.Max(0.01f, maxSpeedMS));
        return Mathf.Lerp(maxSteerAngle, highSpeedSteerAngle, speedFactor);
    }
}
