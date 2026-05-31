using System;
using UnityEngine;
using TMPro;

/// <summary>
/// Persists the player's input source choice (Keyboard vs Mobile) and
/// enables exactly one set of input controllers accordingly: KeyboardCarControls
/// for Keyboard, MobileCarControls for Mobile. Keyboard is the default.
///
/// The dropdown that writes the setting and the cars that react to it usually
/// live in different scenes (Settings vs gameplay). To keep them in sync, a
/// change broadcasts the static <see cref="Changed"/> event, and every instance
/// re-applies it — so switching the dropdown updates already-spawned cars even
/// when the Settings screen is an additive overlay.
///
/// Wire-up in Unity:
///   1. Add this component to your Settings GameObject (same one that has SettingsManager).
///   2. Assign the TMP_Dropdown for input source to the "Input Source Dropdown" field.
///      The dropdown must have exactly two options in order: "Keyboard", "Mobile".
///   3. Assign every car's KeyboardCarControls to "Keyboard Controls" and every car's
///      MobileCarControls to "Mobile Controls".
///   4. In every gameplay scene, add a GameObject with only this component (no dropdown needed)
///      and populate both arrays — it applies the saved setting on enable and re-applies
///      whenever the setting changes.
/// </summary>
public class InputSourceController : MonoBehaviour
{
    public const string InputSourceKey = "settings.inputSource";

    public enum InputSource { Keyboard = 0, Mobile = 1 }

    /// <summary>Raised whenever the saved input source changes (from any instance/scene).</summary>
    public static event Action<InputSource> Changed;

    [Header("UI (Settings screen only — leave empty in gameplay scenes)")]
    [SerializeField] private TMP_Dropdown inputSourceDropdown;

    [Header("Targets")]
    [SerializeField] private KeyboardCarControls[] keyboardControls;
    [SerializeField] private MobileCarControls[] mobileControls;

    private void OnEnable()
    {
        Changed += OnInputSourceChanged;
        Apply(GetSavedInputSource());
        LoadToUI();
        HookUIEvents();
    }

    private void OnDisable()
    {
        Changed -= OnInputSourceChanged;
        UnhookUIEvents();
    }

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
        Changed?.Invoke(source);   // notify every instance, including gameplay-scene cars
    }

    private void OnInputSourceChanged(InputSource source)
    {
        Apply(source);
        LoadToUI();
    }

    // ── Apply ────────────────────────────────────────────────────────────────

    private void Apply(InputSource source)
    {
        bool mobileActive = source == InputSource.Mobile;

        SetEnabled(mobileControls, mobileActive);
        SetEnabled(keyboardControls, !mobileActive);
    }

    private static void SetEnabled(MonoBehaviour[] controls, bool enabled)
    {
        if (controls == null) return;

        foreach (var ctrl in controls)
        {
            if (ctrl != null)
                ctrl.enabled = enabled;
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
