using UnityEngine;
using TMPro;

// Lives in the MainMenu scene. Lets the player pick how many laps the race
// lasts and saves it via LapSelectionState so the race scene can read it.
public class LapCountSelector : MonoBehaviour
{
    [Header("UI")]
    [SerializeField] private TMP_InputField lapsInputField;   // "InputLaps"
    [SerializeField] private TMP_Text lapsNumberLabel;        // "Laps Number"

    private void Start()
    {
        int laps = LapSelectionState.GetSelectedLaps();

        if (lapsInputField != null)
        {
            lapsInputField.text = laps.ToString();
            lapsInputField.onValueChanged.AddListener(OnLapsInputChanged);
        }
        UpdateLabel(laps);
    }

    private void OnDestroy()
    {
        if (lapsInputField != null)
            lapsInputField.onValueChanged.RemoveListener(OnLapsInputChanged);
    }

    private void OnLapsInputChanged(string value)
    {
        if (!int.TryParse(value, out int laps))
            return;

        int clamped = Mathf.Clamp(laps, LapSelectionState.MinLaps, LapSelectionState.MaxLaps);
        LapSelectionState.SetSelectedLaps(clamped);
        UpdateLabel(clamped);
    }

    private void UpdateLabel(int laps)
    {
        if (lapsNumberLabel != null)
            lapsNumberLabel.text = laps.ToString();
    }
}
