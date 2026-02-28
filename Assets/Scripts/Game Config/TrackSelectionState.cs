using UnityEngine;

public enum TrackId
{
    Monza = 0,
    Singapore = 1
}

public static class TrackSelectionState
{
    private const string SelectedTrackKey = "settings.selectedTrack";
    private const TrackId DefaultTrack = TrackId.Monza;

    public static TrackId GetSelectedTrack()
    {
        int raw = PlayerPrefs.GetInt(SelectedTrackKey, (int)DefaultTrack);
        if (!System.Enum.IsDefined(typeof(TrackId), raw))
            return DefaultTrack;

        return (TrackId)raw;
    }

    public static void SetSelectedTrack(TrackId track)
    {
        PlayerPrefs.SetInt(SelectedTrackKey, (int)track);
        PlayerPrefs.Save();
    }
}
