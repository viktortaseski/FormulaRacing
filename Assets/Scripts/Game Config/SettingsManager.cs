using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class SettingsManager : MonoBehaviour
{
    private const string SteeringSensitivityKey = "settings.steerSensitivity";
    private const string MusicEnabledKey = "settings.musicEnabled";
    private const string BrakeAssistKey = "settings.brakeAssist";
    private const string StabilityAssistKey = "settings.stabilityAssist";

    private enum AssistLevel
    {
        Off = 0,
        Low = 1,
        Medium = 2,
        High = 3
    }

    [Header("UI")]
    [SerializeField] private Slider steeringSensitivitySlider;
    [SerializeField] private Toggle musicToggle;
    [SerializeField] private TMP_Dropdown brakeAssistDropdownTmp;
    [SerializeField] private TMP_Dropdown stabilityAssistDropdownTmp;

    [Header("Defaults")]
    [SerializeField][Range(0.1f, 2f)] private float defaultSteeringSensitivity = 1f;
    [SerializeField] private bool defaultMusicEnabled = true;
    [SerializeField] private AssistLevel defaultBrakeAssist = AssistLevel.Medium;
    [SerializeField] private AssistLevel defaultStabilityAssist = AssistLevel.Medium;

    private bool listenersHooked;

    private void Awake()
    {
        EnsureAssistOptions();
        LoadToUI();
        HookUIEvents();
        ApplyAll();
    }

    private void OnEnable()
    {
        HookUIEvents();
    }

    private void OnDisable()
    {
        UnhookUIEvents();
    }

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
        for (int i = 0; i < labels.Length; i++)
            dropdown.options.Add(new TMP_Dropdown.OptionData(labels[i]));
    }

    private void HookUIEvents()
    {
        if (listenersHooked)
            return;

        if (steeringSensitivitySlider != null)
            steeringSensitivitySlider.onValueChanged.AddListener(OnSteeringSensitivityChanged);

        if (musicToggle != null)
            musicToggle.onValueChanged.AddListener(OnMusicToggled);

        if (brakeAssistDropdownTmp != null)
            brakeAssistDropdownTmp.onValueChanged.AddListener(OnBrakeAssistChanged);

        if (stabilityAssistDropdownTmp != null)
            stabilityAssistDropdownTmp.onValueChanged.AddListener(OnStabilityAssistChanged);

        listenersHooked = true;
    }

    private void UnhookUIEvents()
    {
        if (!listenersHooked)
            return;

        if (steeringSensitivitySlider != null)
            steeringSensitivitySlider.onValueChanged.RemoveListener(OnSteeringSensitivityChanged);

        if (musicToggle != null)
            musicToggle.onValueChanged.RemoveListener(OnMusicToggled);

        if (brakeAssistDropdownTmp != null)
            brakeAssistDropdownTmp.onValueChanged.RemoveListener(OnBrakeAssistChanged);

        if (stabilityAssistDropdownTmp != null)
            stabilityAssistDropdownTmp.onValueChanged.RemoveListener(OnStabilityAssistChanged);

        listenersHooked = false;
    }

    private void LoadToUI()
    {
        float steering = PlayerPrefs.GetFloat(SteeringSensitivityKey, defaultSteeringSensitivity);
        bool musicOn = PlayerPrefs.GetInt(MusicEnabledKey, defaultMusicEnabled ? 1 : 0) == 1;
        int brakeAssist = Mathf.Clamp(PlayerPrefs.GetInt(BrakeAssistKey, (int)defaultBrakeAssist), 0, 3);
        int stabilityAssist = Mathf.Clamp(PlayerPrefs.GetInt(StabilityAssistKey, (int)defaultStabilityAssist), 0, 3);

        if (steeringSensitivitySlider != null)
            steeringSensitivitySlider.SetValueWithoutNotify(steering);

        if (musicToggle != null)
            musicToggle.SetIsOnWithoutNotify(musicOn);

        if (brakeAssistDropdownTmp != null)
            brakeAssistDropdownTmp.SetValueWithoutNotify(brakeAssist);

        if (stabilityAssistDropdownTmp != null)
            stabilityAssistDropdownTmp.SetValueWithoutNotify(stabilityAssist);
    }

    private void ApplyAll()
    {
        ApplyMusicState(GetMusicEnabled());
    }

    private void OnSteeringSensitivityChanged(float value)
    {
        PlayerPrefs.SetFloat(SteeringSensitivityKey, value);
        PlayerPrefs.Save();
    }

    private void OnMusicToggled(bool isOn)
    {
        ApplyMusicState(isOn);
    }

    private void OnBrakeAssistChanged(int index)
    {
        int level = Mathf.Clamp(index, 0, 3);
        PlayerPrefs.SetInt(BrakeAssistKey, level);
        PlayerPrefs.Save();
    }

    private void OnStabilityAssistChanged(int index)
    {
        int level = Mathf.Clamp(index, 0, 3);
        PlayerPrefs.SetInt(StabilityAssistKey, level);
        PlayerPrefs.Save();
    }

    private bool GetMusicEnabled()
    {
        return PlayerPrefs.GetInt(MusicEnabledKey, defaultMusicEnabled ? 1 : 0) == 1;
    }

    private void ApplyMusicState(bool enabled)
    {
        AudioManager.SetMusicEnabled(enabled);
    }
}
