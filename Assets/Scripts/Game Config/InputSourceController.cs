using UnityEngine;
using TMPro;

/// <summary>
/// Persists the player's input source choice (Keyboard vs Mobile) and
/// enables/disables MobileCarControls accordingly.
///
/// Wire-up in Unity:
///   1. Add this component to your Settings GameObject (same one that has SettingsManager).
///   2. Assign the TMP_Dropdown for input source to the "Input Source Dropdown" field.
///      The dropdown must have exactly two options in order: "Keyboard", "Mobile".
///   3. Assign all MobileCarControls components in the scene to the "Mobile Controls" array.
///      (Each car that can be driven on mobile needs its MobileCarControls listed here.)
///   4. In every gameplay scene, add a GameObject with only this component (no dropdown needed)
///      and populate the "Mobile Controls" array — it will read the saved setting on Awake
///      and enable/disable the scripts automatically.
/// </summary>
public class InputSourceController : MonoBehaviour
{
    public const string InputSourceKey = "settings.inputSource";

    public enum InputSource { Keyboard = 0, Mobile = 1 }

    [Header("UI (Settings screen only — leave empty in gameplay scenes)")]
    [SerializeField] private TMP_Dropdown inputSourceDropdown;

    [Header("Targets")]
    [SerializeField] private MobileCarControls[] mobileControls;

    private void Awake()
    {
        Apply(GetSavedInputSource());
        LoadToUI();
        HookUIEvents();
    }

    private void OnEnable()  => HookUIEvents();
    private void OnDisable() => UnhookUIEvents();

    // ── UI ──────────────────────────────────────────────────────────────────

    private void LoadToUI()
    {
        if (inputSourceDropdown == null) return;
        inputSourceDropdown.SetValueWithoutNotify((int)GetSavedInputSource());
    }

    private void HookUIEvents()
    {
        if (inputSourceDropdown != null)
            inputSourceDropdown.onValueChanged.AddListener(OnDropdownChanged);
    }

    private void UnhookUIEvents()
    {
        if (inputSourceDropdown != null)
            inputSourceDropdown.onValueChanged.RemoveListener(OnDropdownChanged);
    }

    private void OnDropdownChanged(int index)
    {
        var source = (InputSource)Mathf.Clamp(index, 0, 1);
        PlayerPrefs.SetInt(InputSourceKey, (int)source);
        PlayerPrefs.Save();
        Apply(source);
    }

    // ── Apply ────────────────────────────────────────────────────────────────

    private void Apply(InputSource source)
    {
        bool mobileActive = source == InputSource.Mobile;

        if (mobileControls == null) return;

        foreach (var ctrl in mobileControls)
        {
            if (ctrl != null)
                ctrl.enabled = mobileActive;
        }
    }

    // ── Static helpers (read from anywhere) ─────────────────────────────────

    public static InputSource GetSavedInputSource()
    {
        int raw = PlayerPrefs.GetInt(InputSourceKey, (int)InputSource.Keyboard);
        return (InputSource)Mathf.Clamp(raw, 0, 1);
    }

    public static bool IsMobile() => GetSavedInputSource() == InputSource.Mobile;
}
