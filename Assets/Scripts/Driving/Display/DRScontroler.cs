using UnityEngine;
using UnityEngine.UI;      // For Button
using TMPro;               // Only if you use TMP for the label (optional)
using System.Collections;

public class DRScontroler : MonoBehaviour
{
    [Header("References")]
    [Tooltip("The flap/wing mesh that visually opens/closes for DRS.")]
    public Transform drsFlap;

    [Tooltip("The FormulaPhysics component handling the car physics.")]
    public FormulaPhysics carPhysics;

    [Tooltip("The UI Button used to trigger DRS.")]
    public Button drsButton;

    [Tooltip("Optional: Text label on the DRS button (TMP).")]
    public TMP_Text drsLabel;

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

    [Header("DRS Aero")]
    [Range(0.1f, 1f)] public float drsDownforceMultiplier = 0.55f; // 45% less downforce
    [Range(1f, 2f)] public float drsEngineTorqueMultiplier = 1.10f; // +10% torque

    private float _origEngineMul = 1f;
    private float _origDownMul = 1f;
    private bool _cachedOriginals = false;



    [Header("DRS Boost")]
    [Tooltip("How much extra wheel torque to give during DRS. 1 = no change, 1.5 = +50%, 2 = double.")]
    public float torqueBoostMultiplier = 1.5f;

    // Internal state
    private bool _drsActive = false;
    private bool _cooldownActive = false;

    private float _originalMaxWheelTorque;
    private bool _cachedOriginalTorque = false;
    private float _originalDrag;
    private bool _cachedOriginalDrag = false;

    private void Awake()
    {
        if (carPhysics == null)
            carPhysics = GetComponent<FormulaPhysics>();
    }

    private void Start()
    {
        // Make sure button visuals start correctly
        UpdateButtonVisual();
    }

    // This is the function you hook up in the Button OnClick
    public void enableDRS()
    {
        if (carPhysics == null || drsFlap == null)
        {
            Debug.LogWarning("DRScontroler: Missing carPhysics or drsFlap reference.");
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
        if (!_cachedOriginals)
        {
            _origEngineMul = carPhysics.engineTorqueMultiplier;
            _origDownMul = carPhysics.downforceMultiplier;
            _cachedOriginals = true;
        }

        if (!_cachedOriginalDrag && carPhysics.rb != null)
        {
            _originalDrag = carPhysics.rb.linearDamping;
            _cachedOriginalDrag = true;
        }

        if (!_cachedOriginalTorque)
        {
            _originalMaxWheelTorque = carPhysics.maxWheelTorque;
            _cachedOriginalTorque = true;
        }


        carPhysics.engineTorqueMultiplier = _origEngineMul * drsEngineTorqueMultiplier;
        carPhysics.downforceMultiplier = _origDownMul * drsDownforceMultiplier;
        if (carPhysics.rb != null && _cachedOriginalDrag)
            carPhysics.rb.linearDamping = 0f;


        // === 1. Open DRS flap ===
        Vector3 euler = drsFlap.localEulerAngles;
        euler.x = openAngleX;
        drsFlap.localEulerAngles = euler;

        // === 2. Apply boost (torque only for now) ===
        carPhysics.maxWheelTorque = _originalMaxWheelTorque * torqueBoostMultiplier;

        // Keep DRS active
        yield return new WaitForSeconds(drsDurationSeconds);

        // === 3. Close DRS flap ===
        euler = drsFlap.localEulerAngles;
        euler.x = closedAngleX;
        drsFlap.localEulerAngles = euler;

        // === 4. Restore torque ===
        carPhysics.maxWheelTorque = _originalMaxWheelTorque;
        carPhysics.engineTorqueMultiplier = _origEngineMul;
        carPhysics.downforceMultiplier = _origDownMul;
        if (carPhysics.rb != null && _cachedOriginalDrag)
            carPhysics.rb.linearDamping = _originalDrag;


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
