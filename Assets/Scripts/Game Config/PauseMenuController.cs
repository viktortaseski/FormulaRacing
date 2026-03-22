using UnityEngine;

public class PauseMenuController : MonoBehaviour
{
    [SerializeField] private GameObject menuPanel;
    [SerializeField] private bool startPaused = false;
    [SerializeField] private bool pauseTime = true;
    private bool isMenuVisible;

    private void Awake()
    {
        if (menuPanel != null) menuPanel.SetActive(startPaused);
        SyncPauseState(force: true);
    }

    private void LateUpdate() => SyncPauseState();

    public void OpenMenu() => SetMenuVisible(true);
    public void CloseMenu() => SetMenuVisible(false);
    public void ToggleMenu() => SetMenuVisible(menuPanel != null && !menuPanel.activeSelf);

    private void SetMenuVisible(bool visible)
    {
        if (menuPanel != null) menuPanel.SetActive(visible);
        SetTimeScale(visible);
    }

    public static bool HasVisibleMenuPanel()
    {
        foreach (var c in Resources.FindObjectsOfTypeAll<PauseMenuController>())
            if (c != null && c.pauseTime && c.menuPanel != null && c.menuPanel.activeInHierarchy)
                return true;
        return false;
    }

    private void SetTimeScale(bool paused) { if (pauseTime) Time.timeScale = paused ? 0f : 1f; }

    private void SyncPauseState(bool force = false)
    {
        bool visible = menuPanel != null && menuPanel.activeInHierarchy;
        if (!force && visible == isMenuVisible) return;
        isMenuVisible = visible;
        SetTimeScale(isMenuVisible);
    }
}
