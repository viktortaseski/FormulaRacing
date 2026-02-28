using UnityEngine;
using UnityEngine.SceneManagement;

public class TrackMapSelector : MonoBehaviour
{
    [Header("Map Roots")]
    [SerializeField] private GameObject monzaRoot;
    [SerializeField] private GameObject singaporeRoot;
    [SerializeField] private bool autoFindByNameIfMissing = true;

    private void Awake()
    {
        ApplySelectedTrack();
    }

    [ContextMenu("Apply Selected Track")]
    public void ApplySelectedTrack()
    {
        EnsureReferences();
        ApplyTrack(TrackSelectionState.GetSelectedTrack());
    }

    public void ApplyTrackByIndex(int trackIndex)
    {
        if (!System.Enum.IsDefined(typeof(TrackId), trackIndex))
        {
            Debug.LogWarning($"Unknown track index {trackIndex}. Falling back to Monza.");
            ApplyTrack(TrackId.Monza);
            return;
        }

        ApplyTrack((TrackId)trackIndex);
    }

    public void ApplyTrack(TrackId track)
    {
        bool monzaEnabled = track == TrackId.Monza;
        bool singaporeEnabled = track == TrackId.Singapore;

        if (monzaRoot != null)
            monzaRoot.SetActive(monzaEnabled);

        if (singaporeRoot != null)
            singaporeRoot.SetActive(singaporeEnabled);

        if (monzaRoot == null || singaporeRoot == null)
        {
            Debug.LogWarning("TrackMapSelector is missing one or more map references. Assign Monza and Singapore roots.");
        }
    }

    private void EnsureReferences()
    {
        if (!autoFindByNameIfMissing)
            return;

        if (monzaRoot == null)
            monzaRoot = FindInSceneIncludingInactive("Monza");

        if (singaporeRoot == null)
            singaporeRoot = FindInSceneIncludingInactive("Singapore");
    }

    private GameObject FindInSceneIncludingInactive(string objectName)
    {
        Scene scene = gameObject.scene;
        var roots = scene.GetRootGameObjects();
        for (int i = 0; i < roots.Length; i++)
        {
            Transform found = FindChildRecursive(roots[i].transform, objectName);
            if (found != null)
                return found.gameObject;
        }

        return null;
    }

    private Transform FindChildRecursive(Transform current, string targetName)
    {
        if (current.name == targetName)
            return current;

        for (int i = 0; i < current.childCount; i++)
        {
            Transform found = FindChildRecursive(current.GetChild(i), targetName);
            if (found != null)
                return found;
        }

        return null;
    }
}
