using UnityEngine;

public static class LapSelectionState
{
    private const string SelectedLapsKey = "settings.totalLaps";

    public const int MinLaps = 1;
    public const int MaxLaps = 99;
    public const int DefaultLaps = 3;

    public static int GetSelectedLaps()
    {
        int raw = PlayerPrefs.GetInt(SelectedLapsKey, DefaultLaps);
        return Mathf.Clamp(raw, MinLaps, MaxLaps);
    }

    public static void SetSelectedLaps(int laps)
    {
        int clamped = Mathf.Clamp(laps, MinLaps, MaxLaps);
        PlayerPrefs.SetInt(SelectedLapsKey, clamped);
        PlayerPrefs.Save();
    }
}
