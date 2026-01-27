using UnityEngine;

public class PauseMenuController : MonoBehaviour
{
    [SerializeField] private GameObject menuPanel;
    [SerializeField] private bool startPaused = false;
    [SerializeField] private bool pauseTime = true;

    private void Awake()
    {
        if (menuPanel != null)
        {
            menuPanel.SetActive(startPaused);
        }

        if (startPaused)
        {
            Pause();
        }
        else
        {
            Resume();
        }
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
}
