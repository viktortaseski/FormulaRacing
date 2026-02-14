using UnityEngine;
using UnityEngine.UI;      // For Button
using TMPro;               // Only if you use TMP for the label (optional)
using System.Collections;
using UnityEngine.Serialization;

public class DRScontroler : MonoBehaviour
{
    [Header("References")]
    [Tooltip("The flap/wing mesh that visually opens/closes for DRS.")]
    public Transform drsFlap;

    [Tooltip("Optional SimpleCarController handling the car physics.")]
    [FormerlySerializedAs("carPhysics")]
    public SimpleCarController carController;

    [Tooltip("Optional Rigidbody for drag control during DRS.")]
    public Rigidbody carRigidbody;

    [Tooltip("Optional VehicleStability for downforce control during DRS.")]
    public VehicleStability vehicleStability;

    [Tooltip("The UI Button used to trigger DRS.")]
    public Button drsButton;

    [Tooltip("Optional: Text label on the DRS button (TMP).")]
    public TMP_Text drsLabel;

    [Header("DRS Input")]
    [SerializeField] private bool enableKeyboardInput = true;
    [SerializeField] private KeyCode drsKey = KeyCode.LeftShift;

    [Header("DRS Visual")]
    [Tooltip("Local X rotation when DRS is closed.")]
    public float closedAngleX = 0f;

    [Tooltip("Local X rotation when DRS is open.")]
    public float openAngleX = -40f;

    [Header("DRS Timing")]
    [Tooltip("How long DRS stays active (seconds).")]
    public float drsDurationSeconds = 3f;

    [Tooltip("Cooldown after DRS use (seconds). Set to 0 for no cooldown (for testing).")]
    public float drsCooldownSeconds = 40f;

    [Header("DRS Boost")]
    [Tooltip("How much extra wheel torque to give during DRS. 1 = no change, 1.5 = +50%, 2 = double.")]
    [FormerlySerializedAs("drsEngineTorqueMultiplier")]
    public float torqueBoostMultiplier = 1.5f;

    [Tooltip("Multiplier applied to Rigidbody drag while DRS is active. 0.5 = half drag.")]
    [Range(0f, 1f)] public float dragMultiplier = 0.5f;

    [Tooltip("Multiplier applied to downforce while DRS is active. 1 = no change, 0.6 = 40% less downforce.")]
    [FormerlySerializedAs("drsDownforceMultiplier")]
    public float downforceMultiplier = 0.8f;

    [Tooltip("Multiplier applied to car max speed while DRS is active.")]
    public float speedLimitMultiplier = 1.13f;

    // Internal state
    private bool _drsActive = false;
    private bool _cooldownActive = false;

    private float _originalTorqueMultiplier = 1f;
    private bool _cachedOriginalTorque = false;
    private float _originalDrag;
    private bool _cachedOriginalDrag = false;
    private float _originalDownforceMultiplier = 1f;
    private bool _cachedOriginalDownforce = false;
    private float _originalSpeedLimitMultiplier = 1f;
    private bool _cachedOriginalSpeedLimit = false;

    private void Awake()
    {
        if (carController == null)
            carController = GetComponentInParent<SimpleCarController>();

        if (carRigidbody == null)
            carRigidbody = carController != null ? carController.Rigidbody : GetComponentInParent<Rigidbody>();

        if (vehicleStability == null)
            vehicleStability = GetComponentInParent<VehicleStability>();
    }

    private void Start()
    {
        // Make sure button visuals start correctly
        UpdateButtonVisual();
    }

    private void Update()
    {
        HandleKeyboardInput();
    }

    // This is the function you hook up in the Button OnClick
    public void enableDRS()
    {
        if (drsFlap == null)
        {
            Debug.LogWarning("DRScontroler: Missing drsFlap reference.");
            return;
        }

        // If DRS is active or in cooldown, do nothing
        if (_drsActive || _cooldownActive)
            return;

        StartCoroutine(DRSRoutine());
    }

    private IEnumerator DRSRoutine()
    {
        _drsActive = true;
        UpdateButtonVisual();   // disable button visually

        // Cache original torque once
        if (carController != null)
        {
            if (!_cachedOriginalTorque)
            {
                _originalTorqueMultiplier = carController.MotorTorqueMultiplier;
                _cachedOriginalTorque = true;
            }

            carController.MotorTorqueMultiplier = _originalTorqueMultiplier * torqueBoostMultiplier;
        }

        if (carRigidbody != null)
        {
            if (!_cachedOriginalDrag)
            {
                _originalDrag = carRigidbody.linearDamping;
                _cachedOriginalDrag = true;
            }

            carRigidbody.linearDamping = _originalDrag * dragMultiplier;
        }

        if (vehicleStability != null)
        {
            if (!_cachedOriginalDownforce)
            {
                _originalDownforceMultiplier = vehicleStability.DownforceMultiplier;
                _cachedOriginalDownforce = true;
            }

            vehicleStability.DownforceMultiplier = _originalDownforceMultiplier * downforceMultiplier;
        }

        if (carController != null)
        {
            if (!_cachedOriginalSpeedLimit)
            {
                _originalSpeedLimitMultiplier = carController.MaxSpeedKphMultiplier;
                _cachedOriginalSpeedLimit = true;
            }

            carController.MaxSpeedKphMultiplier = _originalSpeedLimitMultiplier * speedLimitMultiplier;
        }


        // === 1. Open DRS flap ===
        Vector3 euler = drsFlap.localEulerAngles;
        euler.x = openAngleX;
        drsFlap.localEulerAngles = euler;

        // === 2. Apply boost (torque only for now) ===
        // Keep DRS active
        yield return new WaitForSeconds(drsDurationSeconds);

        // === 3. Close DRS flap ===
        euler = drsFlap.localEulerAngles;
        euler.x = closedAngleX;
        drsFlap.localEulerAngles = euler;

        // === 4. Restore torque ===
        if (carController != null && _cachedOriginalTorque)
            carController.MotorTorqueMultiplier = _originalTorqueMultiplier;

        if (carRigidbody != null && _cachedOriginalDrag)
            carRigidbody.linearDamping = _originalDrag;

        if (vehicleStability != null && _cachedOriginalDownforce)
            vehicleStability.DownforceMultiplier = _originalDownforceMultiplier;

        if (carController != null && _cachedOriginalSpeedLimit)
            carController.MaxSpeedKphMultiplier = _originalSpeedLimitMultiplier;


        _drsActive = false;

        // === 5. Start cooldown if > 0 ===
        if (drsCooldownSeconds > 0f)
        {
            _cooldownActive = true;
            UpdateButtonVisual();    // still disabled, maybe change text

            yield return new WaitForSeconds(drsCooldownSeconds);

            _cooldownActive = false;
        }

        // Ready again
        UpdateButtonVisual();
    }

    private void HandleKeyboardInput()
    {
        if (!enableKeyboardInput)
            return;

        if (Input.GetKeyDown(drsKey))
            enableDRS();
    }

    private void UpdateButtonVisual()
    {
        bool canUse = !_drsActive && !_cooldownActive;

        if (drsButton != null)
        {
            // This will switch between Normal / Disabled colors configured on the Button
            drsButton.interactable = canUse;
        }

        if (drsLabel != null)
        {
            if (_drsActive)
                drsLabel.text = "DRS ACTIVE";
            else if (_cooldownActive)
                drsLabel.text = "DRS (CD)";
            else
                drsLabel.text = "DRS";
        }
    }
}
