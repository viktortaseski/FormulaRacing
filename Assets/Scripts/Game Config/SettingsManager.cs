using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class SettingsManager : MonoBehaviour
{
    private const string SteeringSensitivityKey = "settings.steerSensitivity";
    private const string MusicEnabledKey = "settings.musicEnabled";
    private const string BrakeAssistKey = "settings.brakeAssist";
    private const string StabilityAssistKey = "settings.stabilityAssist";
    public const float DefaultSteerSensitivityNormalized = 0.5f;
    public const float SteeringSensitivityMinMultiplier = 0.3f;
    public const float SteeringSensitivityMaxMultiplier = 1f;

    private enum AssistLevel { Off = 0, Low = 1, Medium = 2, High = 3 }

    [Header("UI")]
    [SerializeField] private Slider steeringSensitivitySlider;
    [SerializeField] private Toggle musicToggle;
    [SerializeField] private TMP_Dropdown brakeAssistDropdownTmp;
    [SerializeField] private TMP_Dropdown stabilityAssistDropdownTmp;

    [Header("Defaults")]
    [SerializeField][Range(0f, 1f)] private float defaultSteeringSensitivity = DefaultSteerSensitivityNormalized;
    [SerializeField] private bool defaultMusicEnabled = true;
    [SerializeField] private AssistLevel defaultBrakeAssist = AssistLevel.Medium;
    [SerializeField] private AssistLevel defaultStabilityAssist = AssistLevel.Medium;

    private bool listenersHooked;

    private void Awake()
    {
        EnsureAssistOptions();
        LoadToUI();
        HookUIEvents();
        AudioManager.SetMusicEnabled(AudioManager.GetMusicEnabled(defaultMusicEnabled));
    }

    private void OnEnable() => HookUIEvents();
    private void OnDisable() => UnhookUIEvents();

    private void EnsureAssistOptions()
    {
        string[] labels = { "Off", "Low", "Medium", "High" };
        if (brakeAssistDropdownTmp != null && brakeAssistDropdownTmp.options.Count == 0)
            AddOptions(brakeAssistDropdownTmp, labels);
        if (stabilityAssistDropdownTmp != null && stabilityAssistDropdownTmp.options.Count == 0)
            AddOptions(stabilityAssistDropdownTmp, labels);
    }

    private void AddOptions(TMP_Dropdown dropdown, string[] labels)
    {
        dropdown.options.Clear();
        foreach (var label in labels)
            dropdown.options.Add(new TMP_Dropdown.OptionData(label));
    }

    private void HookUIEvents()
    {
        if (listenersHooked) return;
        steeringSensitivitySlider?.onValueChanged.AddListener(OnSteeringSensitivityChanged);
        musicToggle?.onValueChanged.AddListener(OnMusicToggled);
        brakeAssistDropdownTmp?.onValueChanged.AddListener(OnBrakeAssistChanged);
        stabilityAssistDropdownTmp?.onValueChanged.AddListener(OnStabilityAssistChanged);
        listenersHooked = true;
    }

    private void UnhookUIEvents()
    {
        if (!listenersHooked) return;
        steeringSensitivitySlider?.onValueChanged.RemoveListener(OnSteeringSensitivityChanged);
        musicToggle?.onValueChanged.RemoveListener(OnMusicToggled);
        brakeAssistDropdownTmp?.onValueChanged.RemoveListener(OnBrakeAssistChanged);
        stabilityAssistDropdownTmp?.onValueChanged.RemoveListener(OnStabilityAssistChanged);
        listenersHooked = false;
    }

    private void LoadToUI()
    {
        steeringSensitivitySlider?.SetValueWithoutNotify(GetSteeringSensitivityNormalized(defaultSteeringSensitivity));
        musicToggle?.SetIsOnWithoutNotify(AudioManager.GetMusicEnabled(defaultMusicEnabled));
        brakeAssistDropdownTmp?.SetValueWithoutNotify(Mathf.Clamp(PlayerPrefs.GetInt(BrakeAssistKey, (int)defaultBrakeAssist), 0, 3));
        stabilityAssistDropdownTmp?.SetValueWithoutNotify(Mathf.Clamp(PlayerPrefs.GetInt(StabilityAssistKey, (int)defaultStabilityAssist), 0, 3));
    }

    private void OnSteeringSensitivityChanged(float value)
    {
        PlayerPrefs.SetFloat(SteeringSensitivityKey, Mathf.Clamp01(value));
        PlayerPrefs.Save();
    }

    private void OnMusicToggled(bool isOn) => AudioManager.SetMusicEnabled(isOn);

    private void OnBrakeAssistChanged(int index) => SaveAssist(BrakeAssistKey, index);
    private void OnStabilityAssistChanged(int index) => SaveAssist(StabilityAssistKey, index);

    private void SaveAssist(string key, int index)
    {
        PlayerPrefs.SetInt(key, Mathf.Clamp(index, 0, 3));
        PlayerPrefs.Save();
    }

    public static float GetSteeringSensitivityNormalized(float fallback = DefaultSteerSensitivityNormalized)
    {
        float raw = PlayerPrefs.GetFloat(SteeringSensitivityKey, fallback);
        if (raw < 0f || raw > 1f)
            return Mathf.Clamp01(Mathf.InverseLerp(SteeringSensitivityMinMultiplier, SteeringSensitivityMaxMultiplier, raw));
        return Mathf.Clamp01(raw);
    }

    public static float GetSteeringSensitivityMultiplier(float fallback = DefaultSteerSensitivityNormalized)
        => Mathf.Lerp(SteeringSensitivityMinMultiplier, SteeringSensitivityMaxMultiplier, GetSteeringSensitivityNormalized(fallback));

    public static int GetBrakeAssistLevelIndex(int fallback = (int)AssistLevel.High)
        => Mathf.Clamp(PlayerPrefs.GetInt(BrakeAssistKey, Mathf.Clamp(fallback, 0, 3)), 0, 3);

    public static float GetBrakeAssistTurningMultiplier(int fallback = (int)AssistLevel.High)
    {
        float[] multipliers = { 0f, 0.4f, 0.7f, 1f };
        return multipliers[GetTurningAssistLevelIndex(fallback)];
    }

    public static int GetTurningAssistLevelIndex(int fallback = (int)AssistLevel.High)
    {
        int safe = Mathf.Clamp(fallback, 0, 3);
        return Mathf.Min(
            Mathf.Clamp(PlayerPrefs.GetInt(BrakeAssistKey, safe), 0, 3),
            Mathf.Clamp(PlayerPrefs.GetInt(StabilityAssistKey, safe), 0, 3)
        );
    }
}
