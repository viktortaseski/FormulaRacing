using UnityEngine;
using UnityEngine.SceneManagement;

public class RestartManager : MonoBehaviour
{
    [SerializeField] private bool enableKeyboardInput = true;
    [SerializeField] private KeyCode restartKey = KeyCode.R;
    [SerializeField] private string mainMenuSceneName = "MainMenu";
    [SerializeField] private string settingsSceneName = "Settings";

    private void Update()
    {
        HandleKeyboardInput();
    }

    public void RestartScene()
    {
        Time.timeScale = 1f;
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    public void LoadMainMenu()
    {
        Time.timeScale = 1f;
        if (!string.IsNullOrEmpty(mainMenuSceneName))
        {
            SceneManager.LoadScene(mainMenuSceneName);
        }
    }

    public void LoadSettings()
    {
        if (!string.IsNullOrEmpty(settingsSceneName))
        {
            SceneManager.LoadScene(settingsSceneName);
        }
    }

    private void HandleKeyboardInput()
    {
        if (!enableKeyboardInput)
            return;

        if (Input.GetKeyDown(restartKey))
            RestartScene();
    }
}
