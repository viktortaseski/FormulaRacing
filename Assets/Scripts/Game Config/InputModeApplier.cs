using UnityEngine;

public class InputModeApplier : MonoBehaviour
{
    [SerializeField] private SimpleCarController carController;
    [SerializeField] private MobileCarControls mobileControls;
    [SerializeField] private SettingsManager.InputMode defaultInputMode = SettingsManager.InputMode.Keyboard;

    private void Awake()
    {
        var mode = SettingsManager.GetInputMode(defaultInputMode);
        SettingsManager.ApplyInputMode(mode, carController, mobileControls);
    }
}
