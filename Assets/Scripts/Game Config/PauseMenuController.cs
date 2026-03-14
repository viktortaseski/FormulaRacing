using UnityEngine;

public class PauseMenuController : MonoBehaviour
{
    [SerializeField] private GameObject menuPanel;
    [SerializeField] private bool startPaused = false;
    [SerializeField] private bool pauseTime = true;
    private bool isMenuVisible;

    private void Awake()
    {
        if (menuPanel != null)
        {
            menuPanel.SetActive(startPaused);
        }

        SyncPauseState(force: true);
    }

    private void LateUpdate()
    {
        SyncPauseState();
    }

    public void OpenMenu()
    {
        if (menuPanel != null)
        {
            menuPanel.SetActive(true);
        }

        Pause();
    }

    public void CloseMenu()
    {
        if (menuPanel != null)
        {
            menuPanel.SetActive(false);
        }

        Resume();
    }

    public void ToggleMenu()
    {
        if (menuPanel == null)
        {
            return;
        }

        if (menuPanel.activeSelf)
        {
            CloseMenu();
        }
        else
        {
            OpenMenu();
        }
    }

    public static bool HasVisibleMenuPanel()
    {
        var controllers = Resources.FindObjectsOfTypeAll<PauseMenuController>();
        for (int i = 0; i < controllers.Length; i++)
        {
            var controller = controllers[i];
            if (controller == null || !controller.pauseTime)
                continue;

            if (controller.menuPanel != null && controller.menuPanel.activeInHierarchy)
                return true;
        }

        return false;
    }

    private void Pause()
    {
        if (pauseTime)
        {
            Time.timeScale = 0f;
        }
    }

    private void Resume()
    {
        if (pauseTime)
        {
            Time.timeScale = 1f;
        }
    }

    private void SyncPauseState(bool force = false)
    {
        bool menuIsVisible = menuPanel != null && menuPanel.activeInHierarchy;
        if (!force && menuIsVisible == isMenuVisible)
            return;

        isMenuVisible = menuIsVisible;

        if (isMenuVisible)
            Pause();
        else
            Resume();
    }
}
