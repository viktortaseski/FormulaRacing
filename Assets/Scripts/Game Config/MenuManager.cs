using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuManager : MonoBehaviour
{
    [SerializeField] private GameObject mainMenuPanel;
    [SerializeField] private GameObject trackSelectionPanel;
    [SerializeField] private string drivingSceneName = "Multiplayer";

    public void StartFreePractice()
    {
        OpenTrackSelection();
    }

    public void OpenTrackSelection() => SetTrackSelectionVisible(true);
    public void BackFromTrackSelection() => SetTrackSelectionVisible(false);

    private void SetTrackSelectionVisible(bool visible)
    {
        if (mainMenuPanel != null) mainMenuPanel.SetActive(!visible);
        if (trackSelectionPanel != null) trackSelectionPanel.SetActive(visible);
    }

    public void StartFreePracticeMonza()
    {
        StartFreePracticeWithTrack(TrackId.Monza);
    }

    public void StartFreePracticeSingapore()
    {
        StartFreePracticeWithTrack(TrackId.Singapore);
    }

    public void StartFreePracticeWithTrackIndex(int trackIndex)
    {
        if (!System.Enum.IsDefined(typeof(TrackId), trackIndex))
            trackIndex = (int)TrackId.Monza;

        StartFreePracticeWithTrack((TrackId)trackIndex);
    }

    private void StartFreePracticeWithTrack(TrackId track)
    {
        TrackSelectionState.SetSelectedTrack(track);
        Time.timeScale = 1f;
        SceneManager.LoadScene(drivingSceneName);
    }

    public void OpenMultiplayer()
    {
        // Placeholder until implemented
        Debug.Log("Multiplayer coming soon!");
    }


    public void QuitGame()
    {
        Application.Quit();
    }
}
