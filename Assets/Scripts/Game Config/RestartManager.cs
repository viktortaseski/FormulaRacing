using UnityEngine;
using UnityEngine.SceneManagement;

public class RestartManager : MonoBehaviour
{
    [SerializeField] private bool enableKeyboardInput = true;
    [SerializeField] private KeyCode restartKey = KeyCode.R;
    [SerializeField] private string mainMenuSceneName = "MainMenu";

    private void Update()
    {
        HandleKeyboardInput();
    }

    public void RestartScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    public void LoadMainMenu()
    {
        if (!string.IsNullOrEmpty(mainMenuSceneName))
            SceneManager.LoadScene(mainMenuSceneName);
    }

    private void HandleKeyboardInput()
    {
        if (!enableKeyboardInput)
            return;

        if (Input.GetKeyDown(restartKey))
            RestartScene();
    }
}
