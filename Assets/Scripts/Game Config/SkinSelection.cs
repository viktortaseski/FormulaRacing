using UnityEngine;

public static class SkinSelection
{
    private const string SelectedSkinIdKey = "SelectedSkinId";
    private static string cachedId;

    public static void SetSelectedId(string id)
    {
        if (string.IsNullOrWhiteSpace(id))
        {
            return;
        }

        cachedId = id;
        PlayerPrefs.SetString(SelectedSkinIdKey, id);
        PlayerPrefs.Save();
    }

    public static string GetSelectedId(string fallbackId)
    {
        if (!string.IsNullOrEmpty(cachedId))
        {
            return cachedId;
        }

        cachedId = PlayerPrefs.GetString(SelectedSkinIdKey, fallbackId);
        return cachedId;
    }
}
