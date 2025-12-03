using UnityEngine;
using UnityEngine.InputSystem;   // new Input System
using UnityEngine.EventSystems;

public class FormulaControler : MonoBehaviour
{
    public enum Axle
    {
        Front,
        Rear
    }

    // ======================================
    // UI CONTROLS
    // ======================================
    [Header("UI Controls")]
    public UnityEngine.UI.Button brakeButton;
    public UnityEngine.UI.Slider throttleSlider;

    private bool brakePressed = false;
    private bool sliderHeld = false;

    private float targetSpeed = 0f;       // in m/s
    public float maxSpeedKPH = 360f;      // top speed in km/h
    public float speedHoldAssist = 10f;   // speed regulation strength
    public float reverseEntryKPH = 10f;   // if braking below this, engage reverse
    public float brakeSliderDecayPerSecond = 2f; // how fast the slider eases down while braking


    // ======================================
    // WHEELS
    // ======================================
    [System.Serializable]
    public class Wheel
    {
        public string wheelName;
        public Axle axle;

        [Header("Physics")]
        public WheelCollider collider;

        [Header("Visual")]
        public Transform visual;
    }

    [Header("Wheels")]
    public Wheel[] wheels;


    // ======================================
    // CAR SETTINGS
    // ======================================
    [Header("Car settings")]
    public float maxMotorTorque = 2000f;
    public float maxSteerAngle = 30f;
    public float brakeTorque = 4000f;


    [Header("Stability")]
    public float downforceMultiplier = 50f;
    public float antiRollStrength = 6000f;


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


    // ======================================
    // AWAKE
    // ======================================
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Lower center of mass
        rb.centerOfMass = new Vector3(0f, -0.3f, 0f);
        maxSpeedMS = maxSpeedKPH / 3.6f;

        if (inputActions != null)
        {
            playerMap = inputActions.FindActionMap("Player", throwIfNotFound: true);
            moveAction = playerMap.FindAction("Move", throwIfNotFound: true);
        }
    }


    // ======================================
    // UI EVENTS
    // ======================================
    private void OnEnable()
    {
        // Bind slider
        if (throttleSlider != null)
            throttleSlider.onValueChanged.AddListener(OnThrottleChanged);

        BindBrakeButtonHold();

        // Bind input actions
        if (playerMap == null || moveAction == null) return;

        playerMap.Enable();
        moveAction.performed += OnMove;
        moveAction.canceled += OnMove;
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
    }

    private void OnMove(InputAction.CallbackContext ctx)
    {
        moveInput = ctx.ReadValue<Vector2>();
    }


    private void BrakeDown() => brakePressed = true;
    private void BrakeUp() => brakePressed = false;


    private void OnThrottleChanged(float value)
    {
        sliderHeld = true;

        targetSpeed = value * maxSpeedMS;
    }

    public void BeginSliderDrag()
    {
        sliderHeld = true;
    }

    public void EndSliderDrag()
    {
        sliderHeld = false;
    }


    // ======================================
    // UPDATE → SLIDER RETURN
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
        // 0. Read steering
        float steerInput = moveInput.x;

        // 1. Current speed
        float currentSpeed = rb.linearVelocity.magnitude;
        float throttleTorque = 0f;
        float brakeInput = 0f;
        float reverseEntryMS = reverseEntryKPH / 3.6f;

        // ======================================
        // 2. BRAKE OVERRIDES EVERYTHING
        // ======================================
        if (brakePressed)
        {
            // If still moving forward, apply brakes. Once slow enough, drive reverse.
            if (currentSpeed > reverseEntryMS)
            {
                throttleTorque = 0f;
                brakeInput = 1f;
            }
            else
            {
                throttleTorque = -1f; // engage reverse while holding brake
                brakeInput = 0f;
            }
        }
        else
        {
            // ======================================
            // 3. TARGET SPEED LOGIC
            // ======================================
            if (currentSpeed < targetSpeed - 0.5f)
            {
                throttleTorque = 1f;   // accelerate
            }
            else if (currentSpeed > targetSpeed + 0.5f)
            {
                brakeInput = Mathf.Clamp(
                    (currentSpeed - targetSpeed) / speedHoldAssist,
                    0f, 1f
                );
            }
            else
            {
                throttleTorque = 0f;
                brakeInput = 0f;
            }
        }

        // 4. DOWNFORCE
        rb.AddForce(-transform.up * rb.linearVelocity.magnitude * downforceMultiplier);

        // 5. ANTI-ROLL BARS
        ApplyAntiRollBars();

        // 6. APPLY WHEEL FORCES
        foreach (var w in wheels)
        {
            if (w.collider == null) continue;

            if (w.axle == Axle.Front)
                w.collider.steerAngle = steerInput * maxSteerAngle;

            w.collider.motorTorque = throttleTorque * maxMotorTorque;
            w.collider.brakeTorque = brakeInput * brakeTorque;

            UpdateWheelVisual(w);
        }
    }


    // ======================================
    // WHEEL VISUAL UPDATE
    // ======================================
    private void UpdateWheelVisual(Wheel w)
    {
        if (w.visual == null || w.collider == null) return;

        w.collider.GetWorldPose(out Vector3 pos, out Quaternion rot);
        w.visual.position = pos;
        w.visual.rotation = rot;
    }


    // ======================================
    // ANTI-ROLL STABILIZER
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
        AddBrakeTrigger(trigger, EventTriggerType.PointerExit, BrakeUp); // release if pointer leaves

        brakeEventsBound = true;
    }

    private void AddBrakeTrigger(EventTrigger trigger, EventTriggerType type, System.Action action)
    {
        var entry = new EventTrigger.Entry { eventID = type, callback = new EventTrigger.TriggerEvent() };
        entry.callback.AddListener(_ => action());
        trigger.triggers.Add(entry);
    }

    private void HandleSliderDecay()
    {
        if (throttleSlider == null) return;

        // While braking, silence throttle and ease slider toward zero without snapping.
        if (brakePressed)
        {
            float newValue = Mathf.MoveTowards(throttleSlider.value, 0f, brakeSliderDecayPerSecond * Time.deltaTime);
            throttleSlider.SetValueWithoutNotify(newValue);
            targetSpeed = newValue * maxSpeedMS;
            return;
        }

        // Smoothly reset slider to 0 when released
        if (!sliderHeld)
        {
            float newValue = Mathf.Lerp(throttleSlider.value, 0f, Time.deltaTime * 4f);
            throttleSlider.SetValueWithoutNotify(newValue);
            targetSpeed = newValue * maxSpeedMS;
        }
    }
}
